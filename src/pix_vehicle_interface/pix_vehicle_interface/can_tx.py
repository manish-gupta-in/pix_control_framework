#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd
import can
import os
import sys
from ament_index_python.packages import get_package_share_directory
from pix_vehicle_interface.dbc_encoder import DBCEncoder

class PixCanTxNode(Node):
    def __init__(self):
        super().__init__('pix_can_tx')
        
        # Declare parameters
        self.declare_parameter('can_interface', 'can4')  # Default from existing shuttle config
        self.declare_parameter('dbc_file', '')
        self.declare_parameter('loop_rate', 50.0)  # 50 Hz
        self.declare_parameter('enable_can_tx', True)
        
        self.can_interface = self.get_parameter('can_interface').value
        self.loop_rate = self.get_parameter('loop_rate').value
        self.enable_can_tx = self.get_parameter('enable_can_tx').value
        
        # Resolve DBC path
        dbc_file = self.get_parameter('dbc_file').value
        if not dbc_file:
            try:
                share_dir = get_package_share_directory('pix_vehicle_interface')
                dbc_file = os.path.join(share_dir, 'config', 'hook2_AD.dbc')
            except Exception:
                # Fallback to local workspace search if not fully installed
                dbc_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/hook2_AD.dbc'))
            
        self.get_logger().info(f"Loading DBC from: {dbc_file}")
        self.encoder = DBCEncoder(dbc_file)
        
        # Initialize CAN Bus
        self.bus = None
        if self.enable_can_tx:
            try:
                self.bus = can.interface.Bus(channel=self.can_interface, interface='socketcan')
                self.get_logger().info(f"Opened SocketCAN interface: {self.can_interface}")
            except Exception as e:
                self.get_logger().error(f"Failed to open SocketCAN interface '{self.can_interface}': {e}. CAN TX will be mocked.")
                self.bus = None
                
        # Command state
        self.latest_cmd = PixControlCmd()
        self.latest_cmd_time = 0.0
        self.cmd_timeout = 0.5  # 500 ms watchdog timeout
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            PixControlCmd,
            '/pix/control_cmd',
            self.cmd_callback,
            10
        )
        
        # Timer for cyclical CAN sending (50Hz)
        self.timer = self.create_timer(1.0 / self.loop_rate, self.timer_callback)
        self.get_logger().info("CAN TX Node Initialized.")
        
    def cmd_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now().nanoseconds / 1e9
        
    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        
        # Watchdog check: if no command received for too long, apply safe fallback
        is_timeout = (self.latest_cmd_time > 0) and ((now - self.latest_cmd_time) > self.cmd_timeout)
        
        if is_timeout:
            self.get_logger().warning("Watchdog timeout in can_tx: No commands from arbitration/safety manager! Safe fallback activated.", throttle_duration_sec=2.0)
            cmd = PixControlCmd()
            cmd.emergency_stop = True
            cmd.steer_en = True
            cmd.steer_target = 0.0
            cmd.steer_speed = 150.0
            cmd.drive_en = True
            cmd.speed_target = 0.0
            cmd.brake_en = True
            cmd.brake_target = 50.0  # Apply moderate braking
            cmd.gear_en = True
            cmd.gear_target = PixControlCmd.GEAR_TARGET_NEUTRAL
            cmd.park_en = True
            cmd.park_target = PixControlCmd.PARK_TARGET_RELEASE
        else:
            cmd = self.latest_cmd
            
        # If emergency stop is active, command full braking and zero speed
        if cmd.emergency_stop:
            cmd.steer_target = 0.0
            cmd.speed_target = 0.0
            cmd.brake_en = True
            cmd.brake_target = 100.0  # Full brake
            cmd.drive_en = True
            
        # Prepare signals for each CAN command message
        
        # 1. Vehicle_Mode_Command (0x105 / 261)
        vm_signals = {
            'Auto_Professional': 1 if cmd.steer_en or cmd.drive_en or cmd.brake_en else 0,
            'Headlight_Ctrl': 1 if cmd.headlight_ctrl else 0,
            'TurnLight_Ctrl': cmd.turn_light_ctrl,
            'Vehicle_VIN_Req': 0,
            'Drive_ModeCtrl': 1,  # Speed Drive Mode
            'Steer_ModeCtrl': 0   # Standard Steer Mode
        }
        
        # 2. Steering_Command (0x102 / 258)
        steer_signals = {
            'Steer_EnCtrl': 1 if cmd.steer_en else 0,
            'Steer_AngleTarget': int(cmd.steer_target),
            'Steer_AngleSpeed': min(int(cmd.steer_speed), 250) if cmd.steer_speed > 0 else 150
        }
        
        # 3. Throttle_Command (0x100 / 256)
        throttle_signals = {
            'Dirve_EnCtrl': 1 if cmd.drive_en else 0,
            'Dirve_SpeedTarget': float(cmd.speed_target),
            'Dirve_Acc': float(cmd.accel_target) if cmd.accel_target > 0 else 1.0,
            'Dirve_ThrottlePedalTarget': 0.0  # Used only in Throttle Paddle Drive mode
        }
        
        # 4. Brake_Command (0x101 / 257)
        brake_signals = {
            'Brake_EnCtrl': 1 if cmd.brake_en else 0,
            'Brake_Pedal_Target': float(cmd.brake_target),
            'Brake_Dec': 0.0,
            'AEB_EnCtrl': 0
        }
        
        # 5. Gear_Command (0x103 / 259)
        gear_signals = {
            'Gear_EnCtrl': 1 if cmd.gear_en else 0,
            'Gear_Target': cmd.gear_target
        }
        
        # 6. Park_Command (0x104 / 260)
        park_signals = {
            'Park_EnCtrl': 1 if cmd.park_en else 0,
            'Park_Target': cmd.park_target
        }
        
        # Encode and send messages
        commands = [
            ('Vehicle_Mode_Command', vm_signals),
            ('Steering_Command', steer_signals),
            ('Throttle_Command', throttle_signals),
            ('Brake_Command', brake_signals),
            ('Gear_Command', gear_signals),
            ('Park_Command', park_signals)
        ]
        
        for name, signals in commands:
            frame_id, payload = self.encoder.encode_message(name, signals)
            if frame_id is not None:
                self.send_can_frame(frame_id, payload)
                
    def send_can_frame(self, frame_id, payload):
        if self.bus is not None:
            try:
                msg = can.Message(
                    arbitration_id=frame_id,
                    data=payload,
                    is_extended_id=False
                )
                self.bus.send(msg)
            except Exception as e:
                self.get_logger().error(f"Error sending CAN frame 0x{frame_id:03X}: {e}")
                
    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
                self.get_logger().info("SocketCAN interface closed.")
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PixCanTxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

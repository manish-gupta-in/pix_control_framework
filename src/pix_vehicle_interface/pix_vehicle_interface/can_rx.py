#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixVehicleStatus
import can
import os
import sys
import threading
from ament_index_python.packages import get_package_share_directory
from pix_vehicle_interface.dbc_decoder import DBCDecoder

class PixCanRxNode(Node):
    def __init__(self):
        super().__init__('pix_can_rx')
        
        # Declare parameters
        self.declare_parameter('can_interface', 'can4')
        self.declare_parameter('dbc_file', '')
        self.declare_parameter('loop_rate', 50.0)  # 50 Hz
        self.declare_parameter('enable_can_rx', True)
        
        self.can_interface = self.get_parameter('can_interface').value
        self.loop_rate = self.get_parameter('loop_rate').value
        self.enable_can_rx = self.get_parameter('enable_can_rx').value
        
        # Resolve DBC path
        dbc_file = self.get_parameter('dbc_file').value
        if not dbc_file:
            try:
                share_dir = get_package_share_directory('pix_vehicle_interface')
                dbc_file = os.path.join(share_dir, 'config', 'hook2_AD.dbc')
            except Exception:
                dbc_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/hook2_AD.dbc'))
                
        self.get_logger().info(f"Loading DBC from: {dbc_file}")
        self.decoder = DBCDecoder(dbc_file)
        
        # Status message state
        self.status_msg = PixVehicleStatus()
        self.status_lock = threading.Lock()
        
        # Publisher
        self.status_pub = self.create_publisher(
            PixVehicleStatus,
            '/pix/vehicle_status',
            10
        )
        
        # SocketCAN Thread
        self.bus = None
        self.rx_thread = None
        self.running = False
        
        if self.enable_can_rx:
            try:
                self.bus = can.interface.Bus(channel=self.can_interface, interface='socketcan')
                self.get_logger().info(f"Opened SocketCAN interface: {self.can_interface}")
                self.running = True
                self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
                self.rx_thread.start()
            except Exception as e:
                self.get_logger().error(f"Failed to open SocketCAN interface '{self.can_interface}': {e}. CAN RX will be mocked.")
                self.bus = None
                
        # Status publisher timer (50Hz)
        self.timer = self.create_timer(1.0 / self.loop_rate, self.timer_callback)
        self.get_logger().info("CAN RX Node Initialized.")
        
    def rx_loop(self):
        while self.running and rclpy.ok():
            try:
                # Non-blocking read with timeout to allow thread shutdown
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                self.process_can_frame(msg.arbitration_id, msg.data)
            except Exception as e:
                self.get_logger().error(f"Error in CAN RX loop: {e}")
                
    def process_can_frame(self, frame_id, data):
        name, decoded = self.decoder.decode_message(frame_id, data)
        if name is None or decoded is None:
            return
            
        with self.status_lock:
            # Map DBC signals to PixVehicleStatus fields
            if name == 'Steering_Report':
                self.status_msg.steer_angle = float(decoded.get('Steer_AngleActual', 0.0))
                self.status_msg.steer_speed = float(decoded.get('Steer_AngleSpeedActual', 0.0))
                self.status_msg.steer_en_state = int(decoded.get('Steer_EnState', 0))
                self.status_msg.steer_flt1 = int(decoded.get('Steer_Flt1', 0))
                self.status_msg.steer_flt2 = int(decoded.get('Steer_Flt2', 0))
                
            elif name == 'Throttle_Report':
                self.status_msg.throttle_pedal = float(decoded.get('Dirve_ThrottlePedalActual', 0.0))
                self.status_msg.drive_en_state = int(decoded.get('Dirve_EnState', 0))
                self.status_msg.drive_flt1 = int(decoded.get('Dirve_Flt1', 0))
                self.status_msg.drive_flt2 = int(decoded.get('Dirve_Flt2', 0))
                
            elif name == 'Brake_Report':
                self.status_msg.brake_pedal = float(decoded.get('Brake_PedalActual', 0.0))
                self.status_msg.brake_en_state = int(decoded.get('Brake_EnState', 0))
                self.status_msg.brake_flt1 = int(decoded.get('Brake_Flt1', 0))
                self.status_msg.brake_flt2 = int(decoded.get('Brake_Flt2', 0))
                
            elif name == 'Gear_Report':
                self.status_msg.gear_actual = int(decoded.get('Gear_Actual', 0))
                self.status_msg.gear_flt = int(decoded.get('Gear_Flt', 0))
                
            elif name == 'Park_Report':
                self.status_msg.park_actual = int(decoded.get('Parking_Actual', 0))
                self.status_msg.park_flt = int(decoded.get('Park_Flt', 0))
                # Park_EnState not in DBC — infer from park_actual signal presence
                
            elif name == 'VCU_Report':
                self.status_msg.vehicle_speed = float(decoded.get('Vehicle_Speed', 0.0))
                self.status_msg.vehicle_accel = float(decoded.get('Vehicle_Acc', 0.0))
                self.status_msg.vehicle_mode = int(decoded.get('Vehicle_ModeState', 0))
                self.status_msg.drive_mode_status = int(decoded.get('Drive_ModeStatus', 0))
                self.status_msg.steer_mode_status = int(decoded.get('Steer_ModeStatus', 0))
                self.status_msg.front_crash = bool(decoded.get('Vehicle_FrontCrashState', 0))
                self.status_msg.back_crash = bool(decoded.get('BackCrash_State', 0))
                # DBC uses AEB_Trigger_State (not AEB_BrakeState)
                self.status_msg.aeb_active = bool(decoded.get('AEB_Trigger_State', 0))
                self.status_msg.brake_light_actual = bool(decoded.get('Brake_LightActual', 0))
                self.status_msg.turn_light_actual = int(decoded.get('TurnLight_Actual', 0))
                
            elif name == 'BMS_Report':
                self.status_msg.battery_voltage = float(decoded.get('Battery_Voltage', 0.0))
                self.status_msg.battery_current = float(decoded.get('Battery_Current', 0.0))
                self.status_msg.battery_soc = float(decoded.get('Battery_Soc', 0.0))
                
    def timer_callback(self):
        with self.status_lock:
            # Create a shallow copy to publish
            msg = PixVehicleStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            # Copy all fields
            msg.steer_angle = self.status_msg.steer_angle
            msg.steer_speed = self.status_msg.steer_speed
            msg.steer_en_state = self.status_msg.steer_en_state
            msg.vehicle_speed = self.status_msg.vehicle_speed
            msg.vehicle_accel = self.status_msg.vehicle_accel
            msg.throttle_pedal = self.status_msg.throttle_pedal
            msg.brake_pedal = self.status_msg.brake_pedal
            msg.drive_en_state = self.status_msg.drive_en_state
            msg.brake_en_state = self.status_msg.brake_en_state
            msg.gear_actual = self.status_msg.gear_actual
            msg.park_actual = self.status_msg.park_actual
            msg.vehicle_mode = self.status_msg.vehicle_mode
            msg.drive_mode_status = self.status_msg.drive_mode_status
            msg.steer_mode_status = self.status_msg.steer_mode_status
            msg.front_crash = self.status_msg.front_crash
            msg.back_crash = self.status_msg.back_crash
            msg.aeb_active = self.status_msg.aeb_active
            msg.brake_light_actual = self.status_msg.brake_light_actual
            msg.turn_light_actual = self.status_msg.turn_light_actual
            msg.steer_flt1 = self.status_msg.steer_flt1
            msg.steer_flt2 = self.status_msg.steer_flt2
            msg.drive_flt1 = self.status_msg.drive_flt1
            msg.drive_flt2 = self.status_msg.drive_flt2
            msg.brake_flt1 = self.status_msg.brake_flt1
            msg.brake_flt2 = self.status_msg.brake_flt2
            msg.park_flt = self.status_msg.park_flt
            msg.gear_flt = self.status_msg.gear_flt
            msg.battery_voltage = self.status_msg.battery_voltage
            msg.battery_current = self.status_msg.battery_current
            msg.battery_soc = self.status_msg.battery_soc
            
            self.status_pub.publish(msg)
            
    def destroy_node(self):
        self.running = False
        if self.rx_thread is not None:
            self.rx_thread.join(timeout=1.0)
        if self.bus is not None:
            try:
                self.bus.shutdown()
                self.get_logger().info("SocketCAN interface closed.")
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PixCanRxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

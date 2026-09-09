#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd, PixVehicleStatus
from std_msgs.msg import Bool
import time

from pix_safety_manager.safety_clamp_logic import SafetyClampLogic

class PixSafetyManagerNode(Node):
    def __init__(self):
        super().__init__('pix_safety_manager')
        
        # Declare parameters for safety limits
        self.declare_parameter('max_steer_angle', 350.0)      # deg (chassis limit is 500)
        self.declare_parameter('max_steer_rate', 150.0)       # deg/s
        self.declare_parameter('max_speed', 5.0)             # m/s (approx 18 km/h for campus shuttle)
        self.declare_parameter('max_accel', 2.0)             # m/s^2
        self.declare_parameter('watchdog_timeout', 0.3)       # seconds
        
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.max_steer_rate = self.get_parameter('max_steer_rate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_accel = self.get_parameter('max_accel').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        # Pure-logic safety clamp class (no ROS deps — unit-testable)
        self.clamp = SafetyClampLogic(
            max_steer_angle=self.max_steer_angle,
            max_steer_rate=self.max_steer_rate,
            max_speed=self.max_speed,
            max_accel=self.max_accel,
        )
        
        # Subscriptions
        self.raw_cmd_sub = self.create_subscription(
            PixControlCmd,
            '/pix/raw_control_cmd',
            self.raw_cmd_callback,
            10
        )
        self.status_sub = self.create_subscription(
            PixVehicleStatus,
            '/pix/vehicle_status',
            self.status_callback,
            10
        )
        self.estop_sub = self.create_subscription(
            Bool,
            '/pix/estop_trigger',
            self.estop_callback,
            10
        )
        self.estop_clear_sub = self.create_subscription(
            Bool,
            '/pix/estop_clear',
            self.estop_clear_callback,
            10
        )
        
        # Publisher
        self.safe_cmd_pub = self.create_publisher(
            PixControlCmd,
            '/pix/control_cmd',
            10
        )
        
        # Internal states
        self.latest_raw_cmd = None
        self.latest_raw_cmd_time = 0.0
        self.latest_status = None
        self.latest_status_time = 0.0
        
        # Safety states
        self.estop_triggered = False
        self.estop_reason = "None"
        self.last_steer_cmd = 0.0
        self.last_steer_time = 0.0
        # Grace period: do NOT monitor chassis faults for the first N seconds.
        # The VCU reports transient communication faults (flt2=1) while it boots.
        # Triggering E-stop on startup faults locks the vehicle before it is ready.
        self.startup_grace_period = 3.0   # seconds
        self.node_start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Safety monitoring timer (runs at 50Hz)
        self.timer = self.create_timer(0.02, self.safety_loop)
        self.get_logger().info("Safety Manager Node Initialized.")
        
    def raw_cmd_callback(self, msg):
        self.latest_raw_cmd = msg
        self.latest_raw_cmd_time = self.get_clock().now().nanoseconds / 1e9
        
    def estop_callback(self, msg):
        if msg.data:
            self.trigger_estop("External software E-stop trigger")

    def estop_clear_callback(self, msg):
        """Clear the E-stop latch. Operator must manually confirm area is safe."""
        if msg.data and self.estop_triggered:
            self.estop_triggered = False
            self.estop_reason = "None"
            self.last_steer_cmd = 0.0
            self.last_steer_time = 0.0
            self.get_logger().warn("E-stop CLEARED via /pix/estop_clear. Returning to normal operation.")

            
    def status_callback(self, msg):
        self.latest_status = msg
        self.latest_status_time = self.get_clock().now().nanoseconds / 1e9
        self.check_chassis_faults(msg)
        
    def check_chassis_faults(self, status):
        # Skip fault monitoring during startup grace period.
        # The VCU sends transient communication faults (e.g. steer_flt2=1)
        # for the first 1-2 seconds while it boots — acting on them would
        # permanently latch an E-stop before operation even begins.
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.node_start_time) < self.startup_grace_period:
            return
        # Check all fault indicators from decoded CAN reports
        if status.steer_flt1 or status.steer_flt2:
            self.trigger_estop(f"Chassis Steering Fault! flt1: {status.steer_flt1}, flt2: {status.steer_flt2}")
        elif status.drive_flt1 or status.drive_flt2:
            self.trigger_estop(f"Chassis Drive Fault! flt1: {status.drive_flt1}, flt2: {status.drive_flt2}")
        elif status.brake_flt1 or status.brake_flt2:
            self.trigger_estop(f"Chassis Brake Fault! flt1: {status.brake_flt1}, flt2: {status.brake_flt2}")
        elif status.park_flt:
            self.trigger_estop(f"Chassis Parking Fault! flt: {status.park_flt}")
        elif status.gear_flt:
            self.trigger_estop(f"Chassis Gear Fault! flt: {status.gear_flt}")
        elif status.front_crash:
            self.trigger_estop("Chassis Collision Front Crash Sensor Triggered!")
        elif status.back_crash:
            self.trigger_estop("Chassis Collision Back Crash Sensor Triggered!")
            
    def trigger_estop(self, reason):
        if not self.estop_triggered:
            self.estop_triggered = True
            self.estop_reason = reason
            self.get_logger().error(f"EMERGENCY STOP TRIGGERED! Reason: {reason}")
            
    def safety_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        safe_cmd = PixControlCmd()
        safe_cmd.header.stamp = self.get_clock().now().to_msg()
        safe_cmd.header.frame_id = 'base_link'
        
        # 1. Watchdog Checks
        # Verify if raw control command has been received recently
        if self.latest_raw_cmd_time > 0 and (now - self.latest_raw_cmd_time) > self.watchdog_timeout:
            self.trigger_estop(f"Watchdog Timeout: No raw control commands for {(now - self.latest_raw_cmd_time)*1000:.1f}ms")
            
        # 2. Compile output based on safety state
        if self.estop_triggered:
            # Safe Fallback: Stop vehicle immediately
            safe_cmd.emergency_stop = True
            safe_cmd.steer_en = True
            safe_cmd.steer_target = 0.0  # Return wheels to center
            safe_cmd.steer_speed = 150.0
            
            safe_cmd.drive_en = True
            safe_cmd.speed_target = 0.0
            safe_cmd.accel_target = 1.5
            
            safe_cmd.brake_en = True
            safe_cmd.brake_target = 100.0  # Apply full brakes
            
            safe_cmd.gear_en = True
            safe_cmd.gear_target = PixControlCmd.GEAR_TARGET_NEUTRAL
            safe_cmd.park_en = True
            safe_cmd.park_target = PixControlCmd.PARK_TARGET_RELEASE
            
            # Publish E-stop command
            self.safe_cmd_pub.publish(safe_cmd)
            return
            
        # If no commands received yet, send standby commands
        if self.latest_raw_cmd is None:
            safe_cmd.steer_en = False
            safe_cmd.drive_en = False
            safe_cmd.brake_en = False
            safe_cmd.gear_en = False
            safe_cmd.park_en = False
            self.safe_cmd_pub.publish(safe_cmd)
            return
            
        # 3. Command Validation & Rate Limiting
        raw = self.latest_raw_cmd
        
        # Validate E-Stop request from input command
        if raw.emergency_stop:
            self.trigger_estop("Emergency stop requested in input command")
            return
            
        # Delegated to SafetyClampLogic (pure, unit-testable)
        dt = (now - self.last_steer_time) if self.last_steer_time > 0 else 0.02
        validated = self.clamp.validate_command(
            steer_target=raw.steer_target,
            steer_speed=raw.steer_speed,
            speed_target=raw.speed_target,
            accel_target=raw.accel_target,
            brake_target=raw.brake_target,
            previous_steer=self.last_steer_cmd,
            dt=dt,
        )

        self.last_steer_cmd = validated['steer_target']
        self.last_steer_time = now

        # Construct Safe Command
        safe_cmd.steer_en = raw.steer_en
        safe_cmd.steer_target = validated['steer_target']
        safe_cmd.steer_speed = validated['steer_speed']
        
        safe_cmd.drive_en = raw.drive_en
        safe_cmd.speed_target = validated['speed_target']
        safe_cmd.accel_target = validated['accel_target']
        
        safe_cmd.brake_en = raw.brake_en
        safe_cmd.brake_target = validated['brake_target']
        
        safe_cmd.gear_en = raw.gear_en
        safe_cmd.gear_target = raw.gear_target
        safe_cmd.park_en = raw.park_en
        safe_cmd.park_target = raw.park_target
        
        safe_cmd.turn_light_ctrl = raw.turn_light_ctrl
        safe_cmd.headlight_ctrl = raw.headlight_ctrl
        safe_cmd.emergency_stop = False
        
        self.safe_cmd_pub.publish(safe_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PixSafetyManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

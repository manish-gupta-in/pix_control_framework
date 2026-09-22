#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface
import math
import time

class LateralAvoidancePlanner(BaseAlgorithmInterface):
    """
    Industry-Standard Lateral Avoidance Planner.
    Maintains a set driving speed and steers away from obstacles detected by YOLO.
    Uses a proportional controller with ramping logic.
    """
    def __init__(self):
        super().__init__('lateral_avoidance_planner', '/pix/commands/human_avoidance')
        
        # Adjustable parameters
        self.declare_parameter('driving_speed', 2.0)      # m/s
        self.declare_parameter('avoidance_gain', 35.0)    # Degrees of steering per unit of offset
        self.declare_parameter('max_avoidance_angle', 150.0) # Maximum steering angle (degrees)
        self.declare_parameter('deadband', 0.1)           # Ignore small offsets
        self.declare_parameter('hold_time', 1.0)          # Seconds to hold avoidance after object leaves view
        
        self.driving_speed = self.get_parameter('driving_speed').value
        self.gain = self.get_parameter('avoidance_gain').value
        self.max_angle = self.get_parameter('max_avoidance_angle').value
        self.deadband = self.get_parameter('deadband').value
        self.hold_time = self.get_parameter('hold_time').value
        
        # Subscribe to YOLO
        self.yolo_sub = self.create_subscription(
            Float32MultiArray,
            '/perception/obstacles',
            self.perception_callback,
            10
        )
        
        # State variables
        self.last_offset = 0.0
        self.last_detect_time = 0.0
        self.current_steer = 0.0
        
        self.state = 'WAKE'
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Control Loop at 50Hz (Standard for vehicles)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Lateral Avoidance Planner starting...")

    def perception_callback(self, msg):
        # msg.data = [distance, norm_offset]
        if msg.data and len(msg.data) >= 2:
            distance = msg.data[0]
            offset = msg.data[1]
            
            # If distance is not 999 (meaning an object is detected)
            if distance < 990.0:
                self.last_offset = offset
                self.last_detect_time = time.time()

    def control_loop(self):
        now = time.time()
        target_steer = 0.0
        
        # Check if we should avoid
        if (now - self.last_detect_time) <= self.hold_time:
            # Person is visible (or recently visible)
            if abs(self.last_offset) > self.deadband:
                # Proportional calculation (Negative because if person is on right (+), we steer left (-))
                # Note: The reference code did target = offset * gain, but handled inversion later.
                # In standard ROS, positive = left. If offset > 0 (right), we steer left (+). 
                # Let's match the old logic exactly:
                
                # Person on right (+offset), steer away (left). Left is typically positive in ROS, but we use degrees.
                # Let's say: offset > 0 -> steer away (left = negative degrees for your VCU convention)
                target_steer = -self.last_offset * self.gain
                
                # Clamp to max angle
                target_steer = max(-self.max_angle, min(self.max_angle, target_steer))
        else:
            # No person detected for 'hold_time' seconds -> steer back to center
            target_steer = 0.0
            
        # Smooth ramping logic (max 100 degrees per second)
        step = 100.0 * 0.02 # 50Hz
        if target_steer > self.current_steer:
            self.current_steer = min(self.current_steer + step, target_steer)
        elif target_steer < self.current_steer:
            self.current_steer = max(self.current_steer - step, target_steer)
            
        # ── State Machine for Safe Shifting ──
        elapsed = now - self.state_start_time
        
        if self.state == 'WAKE':
            self.publish_control_cmd(
                drive_en=True, speed_target=0.0, accel_target=1.0,
                steer_en=True, steer_target=0.0, steer_speed=150.0,
                brake_en=True, brake_target=100.0, 
                gear_en=True, gear_target=3, # NEUTRAL
                park_en=True, park_target=0
            )
            if elapsed > 5.0:
                self.state = 'SHIFT'
                self.state_start_time = now
                
        elif self.state == 'SHIFT':
            self.publish_control_cmd(
                drive_en=True, speed_target=0.0, accel_target=1.0,
                steer_en=True, steer_target=0.0, steer_speed=150.0,
                brake_en=True, brake_target=100.0, 
                gear_en=True, gear_target=4, # DRIVE
                park_en=True, park_target=0
            )
            if elapsed > 3.0:
                self.state = 'RELEASE_BRAKE'
                self.state_start_time = now
                
        elif self.state == 'RELEASE_BRAKE':
            self.publish_control_cmd(
                drive_en=True, speed_target=0.0, accel_target=1.0,
                steer_en=True, steer_target=0.0, steer_speed=150.0,
                brake_en=True, brake_target=0.0, 
                gear_en=True, gear_target=4,
                park_en=True, park_target=0
            )
            if elapsed > 1.0:
                self.state = 'DRIVE'
                self.state_start_time = now
                self.get_logger().info("Driving and avoiding obstacles...")
                
        elif self.state == 'DRIVE':
            # Publish the command!
            self.publish_control_cmd(
                drive_en=True, speed_target=self.driving_speed, accel_target=1.0,
                steer_en=True, steer_target=self.current_steer, steer_speed=150.0,
                brake_en=False, brake_target=0.0, 
                gear_en=True, gear_target=4, # 4 = DRIVE
                park_en=True, park_target=0
            )

def main(args=None):
    rclpy.init(args=args)
    node = LateralAvoidancePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

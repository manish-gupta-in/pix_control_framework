#!/usr/bin/env python3
import rclpy
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface

class StraightDriveNode(BaseAlgorithmInterface):
    """
    Simple straight-line driving node for AEB testing.
    Commands a constant speed and 0 steering.
    """
    def __init__(self):
        super().__init__('straight_drive_planner', '/pix/commands/lane_following')
        
        self.declare_parameter('speed', 1.5) # m/s
        self.speed = self.get_parameter('speed').value
        
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.state = 'WAKE'
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"Straight Drive Node starting. Target speed: {self.speed} m/s")

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time
        
        if self.state == 'WAKE':
            # Hold 100% brake, neutral, for 2 seconds to wake up VCU safely
            self.publish_control_cmd(
                drive_en=True, speed_target=0.0, accel_target=1.0,
                steer_en=True, steer_target=0.0, steer_speed=150.0,
                brake_en=True, brake_target=100.0, 
                gear_en=True, gear_target=3, # 3 = NEUTRAL
                park_en=True, park_target=0
            )
            if elapsed > 5.0:
                self.state = 'SHIFT'
                self.state_start_time = now
                self.get_logger().info("Shifting to DRIVE (holding brake)...")
                
        elif self.state == 'SHIFT':
            # Shift to DRIVE while holding brake, wait 3 seconds
            self.publish_control_cmd(
                drive_en=True, speed_target=0.0, accel_target=1.0,
                steer_en=True, steer_target=0.0, steer_speed=150.0,
                brake_en=True, brake_target=100.0, 
                gear_en=True, gear_target=4, # 4 = DRIVE
                park_en=True, park_target=0
            )
            if elapsed > 3.0:
                self.state = 'RELEASE_BRAKE'
                self.state_start_time = now
                self.get_logger().info("Releasing brake...")
                
        elif self.state == 'RELEASE_BRAKE':
            # Release brake to 0, wait 1 second before applying speed
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
                self.get_logger().info(f"Driving straight at {self.speed} m/s...")
                
        elif self.state == 'DRIVE':
            # Always command straight forward
            self.publish_control_cmd(
                drive_en=True, speed_target=self.speed, accel_target=1.0,
                steer_en=True, steer_target=0.0, steer_speed=150.0,
                brake_en=False, brake_target=0.0, 
                gear_en=True, gear_target=4, # 4 = DRIVE
                park_en=True, park_target=0  # 0 = RELEASE
            )

def main(args=None):
    rclpy.init(args=args)
    node = StraightDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

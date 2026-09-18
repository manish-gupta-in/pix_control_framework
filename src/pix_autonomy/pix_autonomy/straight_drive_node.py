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
        self.get_logger().info(f"Straight Drive Node running. Constant speed: {self.speed} m/s")

    def control_loop(self):
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

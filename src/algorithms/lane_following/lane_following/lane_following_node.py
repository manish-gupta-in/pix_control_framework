#!/usr/bin/env python3
import rclpy
from pix_vehicle_msgs.msg import PixControlCmd
from pix_algorithm_api import BaseAlgorithmInterface
import math

class LaneFollowingNode(BaseAlgorithmInterface):
    def __init__(self):
        super().__init__('lane_following', '/pix/commands/lane_following')
        
        # Declare parameters
        self.declare_parameter('enable_control', True)
        self.declare_parameter('speed_target', 3.0)  # 3.0 m/s (10.8 km/h)
        self.declare_parameter('steer_amplitude', 150.0)  # degrees
        self.declare_parameter('steer_frequency', 0.1)  # Hz
        
        self.enable_control = self.get_parameter('enable_control').value
        self.speed_target = self.get_parameter('speed_target').value
        self.steer_amplitude = self.get_parameter('steer_amplitude').value
        self.steer_frequency = self.get_parameter('steer_frequency').value
        
        # Command publishing timer (runs at 50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
    def control_loop(self):
        if not self.enable_control:
            return
            
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time
        
        # Generate dummy sinus steering command for simulation test
        steer_target = self.steer_amplitude * math.sin(2.0 * math.pi * self.steer_frequency * elapsed)
        
        # Publish control commands
        self.publish_control_cmd(
            steer_target=steer_target,
            steer_speed=120.0,
            steer_en=True,
            speed_target=self.speed_target,
            accel_target=1.0,
            drive_en=True,
            brake_en=False,
            brake_target=0.0,
            gear_target=PixControlCmd.GEAR_TARGET_DRIVE,
            gear_en=True,
            park_target=PixControlCmd.PARK_TARGET_RELEASE,
            park_en=True
        )

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

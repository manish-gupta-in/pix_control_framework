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
        
        # Convert degrees to radians for standard path
        steer_target_rad = steer_target * math.pi / 180.0
        steer_rate_rad = 120.0 * math.pi / 180.0
        
        # Publish control commands using the standard standard control path
        self.publish_standard_control(
            steering_tire_angle=steer_target_rad,
            steering_tire_rotation_rate=steer_rate_rad,
            velocity=self.speed_target,
            acceleration=1.0,
            gear_command=2  # 2=DRIVE (pix_control_msgs/GearCommand)
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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

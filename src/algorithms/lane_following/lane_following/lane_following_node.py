#!/usr/bin/env python3
import rclpy
import math
from pix_algorithm_api import BaseAlgorithmInterface

class LaneFollowingNode(BaseAlgorithmInterface):
    def __init__(self):
        super().__init__('lane_following', '/pix/commands/lane_following')

        # Declare parameters
        self.declare_parameter('enable_control', True)
        self.declare_parameter('speed_target', 3.0)      # m/s
        self.declare_parameter('steer_amplitude', 150.0) # degrees wheel angle
        self.declare_parameter('steer_frequency', 0.1)   # Hz

        self.enable_control  = self.get_parameter('enable_control').value
        self.speed_target    = self.get_parameter('speed_target').value
        self.steer_amplitude = self.get_parameter('steer_amplitude').value
        self.steer_frequency = self.get_parameter('steer_frequency').value

        self.timer      = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def control_loop(self):
        if not self.enable_control:
            return

        now     = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        # Sinusoidal steering sweep for simulation/bench testing
        steer_target = self.steer_amplitude * math.sin(
            2.0 * math.pi * self.steer_frequency * elapsed)

        self.publish_control_cmd(
            drive_en=True, speed_target=self.speed_target, accel_target=1.0,
            steer_en=True, steer_target=steer_target, steer_speed=120.0,
            brake_en=False, brake_target=0.0,
            gear_en=True, gear_target=4,   # 4 = DRIVE
            park_en=True, park_target=0,   # 0 = RELEASE
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

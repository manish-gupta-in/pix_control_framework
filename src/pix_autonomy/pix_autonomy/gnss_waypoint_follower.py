#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import os
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface

class GNSSWaypointFollower(BaseAlgorithmInterface):
    """
    A simple GNSS-based waypoint follower using Pure Pursuit logic.
    Reads waypoints from a TXT file and pushes commands.
    """
    def __init__(self):
        super().__init__('gnss_waypoint_follower', '/pix/commands/lane_following')
        
        # Load Waypoints
        self.declare_parameter('waypoint_file', 'waypoints.txt')
        self.declare_parameter('lookahead_distance', 5.0)
        
        wp_file = self.get_parameter('waypoint_file').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        
        self.waypoints = []
        if os.path.exists(wp_file):
            with open(wp_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        self.waypoints.append((float(parts[0]), float(parts[1])))
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        else:
            self.get_logger().warn(f"Waypoint file {wp_file} not found. Running idle.")
            
        self.current_wp_idx = 0
        
        # In a real system, you'd subscribe to /odometry or /gnss for current X/Y/Yaw.
        # Here we mock it with a timer to show the structure.
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def control_loop(self):
        if not self.waypoints or self.current_wp_idx >= len(self.waypoints):
            # Stop vehicle if no waypoints
            self.publish_control_cmd(
                drive_en=True, speed_target=0.0, accel_target=2.0,
                steer_en=True, steer_target=0.0, steer_speed=100.0,
                brake_en=True, brake_target=50.0, gear_en=True, gear_target=1 # 1 = Park
            )
            return

        # MOCK POSITION: Assume we get current x, y, yaw from GNSS topic.
        current_x = 0.0
        current_y = 0.0
        current_yaw = 0.0
        
        target_x, target_y = self.waypoints[self.current_wp_idx]
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        
        if distance < self.lookahead:
            self.current_wp_idx += 1
            self.get_logger().info(f"Reached waypoint. Moving to next: {self.current_wp_idx}")
            return
            
        # Calculate Pure Pursuit steering angle
        alpha = math.atan2(dy, dx) - current_yaw
        wheelbase = 2.8 # meters
        steer_rad = math.atan2(2.0 * wheelbase * math.sin(alpha), self.lookahead)
        steer_deg = math.degrees(steer_rad) * 16.0 # assuming steering ratio
        
        # Publish Command to arbitrator!
        self.publish_control_cmd(
            drive_en=True, speed_target=3.0, accel_target=1.0, # 3 m/s
            steer_en=True, steer_target=steer_deg, steer_speed=200.0,
            brake_en=False, brake_target=0.0,
            gear_en=True, gear_target=4 # 4 = Drive
        )

def main(args=None):
    rclpy.init(args=args)
    node = GNSSWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

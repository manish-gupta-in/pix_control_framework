#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd
import time

class PixCommandArbitrator(Node):
    def __init__(self):
        super().__init__('pix_command_arbitrator')
        
        # Declare parameter for timeout after which an algorithm command is considered inactive
        self.declare_parameter('active_timeout', 0.4)  # seconds
        self.active_timeout = self.get_parameter('active_timeout').value
        
        # Topic definitions and priority order (highest to lowest)
        self.priorities = [
            ('EMERGENCY_STOP',      '/pix/commands/emergency_stop'),
            ('COLLISION_AVOIDANCE', '/pix/commands/collision_avoidance'),
            ('HUMAN_AVOIDANCE',     '/pix/commands/human_avoidance'),
            ('LANE_FOLLOWING',      '/pix/commands/lane_following'),
            ('CRUISE_CONTROL',      '/pix/commands/cruise_control')
        ]
        
        # Store for latest commands and their timestamps
        self.cmd_storage = {name: {'msg': None, 'time': 0.0} for name, _ in self.priorities}
        
        # Subscriptions
        self.subs = []
        for name, topic in self.priorities:
            # We use a default argument in lambda to capture the current name correctly
            sub = self.create_subscription(
                PixControlCmd,
                topic,
                lambda msg, n=name: self.command_callback(msg, n),
                10
            )
            self.subs.append(sub)
            
        # Publisher for the arbitrated command to be routed to safety manager
        self.raw_cmd_pub = self.create_publisher(
            PixControlCmd,
            '/pix/raw_control_cmd',
            10
        )
        
        # Track selected source for log throttling
        self.last_active_source = "NONE"
        
        # Arbitration timer (runs at 50Hz)
        self.timer = self.create_timer(0.02, self.arbitrate_and_publish)
        self.get_logger().info("Command Arbitrator Node Initialized.")
        
    def command_callback(self, msg, source_name):
        self.cmd_storage[source_name]['msg'] = msg
        self.cmd_storage[source_name]['time'] = self.get_clock().now().nanoseconds / 1e9
        
    def arbitrate_and_publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        selected_source = None
        selected_cmd = None
        
        # Scan priorities from highest to lowest
        for name, _ in self.priorities:
            data = self.cmd_storage[name]
            if data['msg'] is not None:
                elapsed = now - data['time']
                if elapsed < self.active_timeout:
                    selected_source = name
                    selected_cmd = data['msg']
                    break

        # Use a stable sentinel so "no active source" compares correctly
        current_label = selected_source if selected_source is not None else 'STANDBY/NONE'

        # Log only on actual source change (not every tick)
        if current_label != self.last_active_source:
            self.get_logger().info(f"Arbitration State Change: [{self.last_active_source}] -> [{current_label}]")
            self.last_active_source = current_label
            
        if selected_cmd is not None:
            # Publish the active command to safety layer
            self.raw_cmd_pub.publish(selected_cmd)
        else:
            # If no algorithm is publishing, send default standby command
            standby_cmd = PixControlCmd()
            standby_cmd.header.stamp = self.get_clock().now().to_msg()
            standby_cmd.header.frame_id = 'base_link'
            standby_cmd.steer_en = False
            standby_cmd.drive_en = False
            standby_cmd.brake_en = False
            standby_cmd.gear_en = False
            standby_cmd.park_en = False
            self.raw_cmd_pub.publish(standby_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PixCommandArbitrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd
from pix_control_msgs.msg import Control, GearCommand
import math

class PixCommandArbitrator(Node):
    def __init__(self):
        super().__init__('pix_command_arbitrator')
        
        self.declare_parameter('active_timeout', 0.4)
        self.declare_parameter('steering_ratio', 16.6)
        self.declare_parameter('priority_sources', ['STANDARD_CONTROL', 'COLLISION_AVOIDANCE', 'HUMAN_AVOIDANCE', 'LANE_FOLLOWING', 'CRUISE_CONTROL'])
        self.declare_parameter('priority_topics', ['/pix_framework/control_cmd', '/pix/commands/collision_avoidance', '/pix/commands/human_avoidance', '/pix/commands/lane_following', '/pix/commands/cruise_control'])
        
        self.active_timeout = self.get_parameter('active_timeout').value
        self.steering_ratio = self.get_parameter('steering_ratio').value
        sources = self.get_parameter('priority_sources').value
        topics = self.get_parameter('priority_topics').value
        
        if len(sources) != len(topics):
            self.get_logger().error("priority_sources and priority_topics length mismatch!")
            
        self.priorities = list(zip(sources, topics))
        
        # Store for latest commands and their timestamps
        self.cmd_storage = {name: {'msg': None, 'time': 0.0} for name, _ in self.priorities}
        
        # Explicit E-Stop tracking (Unconditional override)
        self.estop_active = False
        self.estop_time = 0.0
        self.create_subscription(PixControlCmd, '/pix/commands/emergency_stop', self.estop_callback, 10)
        
        # Subscriptions based on priority list
        self.subs = []
        for name, topic in self.priorities:
            if topic == '/pix_framework/control_cmd':
                # Setup standard message path
                sub_c = self.create_subscription(Control, '/pix_framework/control_cmd',
                                                lambda msg, n=name: self.standard_control_callback(msg, n), 10)
                sub_g = self.create_subscription(GearCommand, '/pix_framework/gear_cmd',
                                                lambda msg, n=name: self.standard_gear_callback(msg, n), 10)
                self.subs.extend([sub_c, sub_g])
                # Track gear separately for the standard source
                self.cmd_storage[name]['gear'] = None
            else:
                # Setup legacy path
                sub = self.create_subscription(PixControlCmd, topic,
                                              lambda msg, n=name: self.command_callback(msg, n), 10)
                self.subs.append(sub)
                
        self.raw_cmd_pub = self.create_publisher(PixControlCmd, '/pix/raw_control_cmd', 10)
        self.last_active_source = "NONE"
        self.timer = self.create_timer(0.02, self.arbitrate_and_publish)
        self.get_logger().info("Command Arbitrator Node Initialized with Config Priorities.")
        
    def estop_callback(self, msg):
        self.estop_active = msg.emergency_stop
        self.estop_time = self.get_clock().now().nanoseconds / 1e9

    def command_callback(self, msg, source_name):
        self.cmd_storage[source_name]['msg'] = msg
        self.cmd_storage[source_name]['time'] = self.get_clock().now().nanoseconds / 1e9
        
    def standard_control_callback(self, msg, source_name):
        # Normalize into PixControlCmd
        cmd = PixControlCmd()
        cmd.header.stamp = msg.stamp
        cmd.header.frame_id = 'base_link'
        
        cmd.steer_en = True
        cmd.steer_target = (msg.steering_tire_angle * 180.0 / math.pi) * self.steering_ratio
        steer_spd = abs(msg.steering_tire_rotation_rate * 180.0 / math.pi * self.steering_ratio)
        cmd.steer_speed = steer_spd if steer_spd >= 1.0 else 1.0
        
        cmd.drive_en = True
        cmd.speed_target = max(0.0, msg.velocity)
        
        if msg.acceleration >= 0.0:
            cmd.accel_target = msg.acceleration
            cmd.brake_en = False
            cmd.brake_target = 0.0
        else:
            cmd.accel_target = abs(msg.acceleration)
            cmd.brake_en = True
            cmd.brake_target = 0.0 # Uses decel_ms2 in interface
            
        cmd.gear_en = True
        gear = self.cmd_storage[source_name].get('gear')
        cmd.gear_target = gear if gear is not None else PixControlCmd.GEAR_TARGET_DRIVE
        cmd.park_en = True
        cmd.park_target = PixControlCmd.PARK_TARGET_RELEASE if cmd.gear_target != 1 else PixControlCmd.PARK_TARGET_ENGAGE
        
        self.cmd_storage[source_name]['msg'] = cmd
        self.cmd_storage[source_name]['time'] = self.get_clock().now().nanoseconds / 1e9

    def standard_gear_callback(self, msg, source_name):
        self.cmd_storage[source_name]['gear'] = msg.command
        
    def arbitrate_and_publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Check Unconditional E-Stop Override
        if self.estop_active and (now - self.estop_time) < self.active_timeout:
            selected_source = 'EMERGENCY_STOP'
            selected_cmd = PixControlCmd()
            selected_cmd.header.stamp = self.get_clock().now().to_msg()
            selected_cmd.emergency_stop = True
        else:
            selected_source = None
            selected_cmd = None
            
            # 2. Scan priorities from highest to lowest
            for name, _ in self.priorities:
                data = self.cmd_storage[name]
                if data['msg'] is not None:
                    elapsed = now - data['time']
                    if elapsed < self.active_timeout:
                        selected_source = name
                        selected_cmd = data['msg']
                        break

        current_label = selected_source if selected_source is not None else 'STANDBY/NONE'

        # Preemption logging
        if current_label != self.last_active_source:
            if self.last_active_source != 'STANDBY/NONE' and current_label != 'STANDBY/NONE':
                self.get_logger().warn(f"PREEMPTION EVENT: [{self.last_active_source}] overridden by [{current_label}]")
            else:
                self.get_logger().info(f"Arbitration State Change: [{self.last_active_source}] -> [{current_label}]")
            self.last_active_source = current_label
            
        if selected_cmd is not None:
            self.raw_cmd_pub.publish(selected_cmd)
        else:
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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()


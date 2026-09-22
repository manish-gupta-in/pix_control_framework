#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd

class PixCommandArbitrator(Node):
    """
    Priority-based command multiplexer.

    All algorithm nodes publish PixControlCmd to a registered /pix/commands/<name> topic.
    This node scans the priority list top-to-bottom and publishes the freshest active command
    to /pix/raw_control_cmd. Only one message format is used: PixControlCmd (degrees, percent).

    Priority order (highest first) comes from arbitrator_params.yaml.
    EMERGENCY_STOP is an unconditional override handled separately — it is NOT in the list.

    To add a new algorithm: publish to /pix/commands/<name> and register in arbitrator_params.yaml.
    No changes to this file are ever needed.
    """
    def __init__(self):
        super().__init__('pix_command_arbitrator')

        self.declare_parameter('active_timeout', 0.4)
        self.declare_parameter('priority_sources', [
            'COLLISION_AVOIDANCE', 'HUMAN_AVOIDANCE', 'LANE_FOLLOWING', 'CRUISE_CONTROL'
        ])
        self.declare_parameter('priority_topics', [
            '/pix/commands/collision_avoidance',
            '/pix/commands/human_avoidance',
            '/pix/commands/lane_following',
            '/pix/commands/cruise_control',
        ])

        self.active_timeout = self.get_parameter('active_timeout').value
        sources = self.get_parameter('priority_sources').value
        topics  = self.get_parameter('priority_topics').value

        if len(sources) != len(topics):
            self.get_logger().error('priority_sources and priority_topics length mismatch!')

        self.priorities = list(zip(sources, topics))

        # Command storage keyed by source name
        self.cmd_storage = {name: {'msg': None, 'time': 0.0} for name, _ in self.priorities}

        # Unconditional E-Stop override (not in priority list)
        self.estop_active = False
        self.estop_time   = 0.0
        self.create_subscription(
            PixControlCmd, '/pix/commands/emergency_stop', self.estop_callback, 10)

        # One subscription per priority topic — all use PixControlCmd
        self.subs = []
        for name, topic in self.priorities:
            sub = self.create_subscription(
                PixControlCmd, topic,
                lambda msg, n=name: self.command_callback(msg, n), 10)
            self.subs.append(sub)

        self.raw_cmd_pub = self.create_publisher(PixControlCmd, '/pix/raw_control_cmd', 10)
        self.last_active_source = 'STANDBY/NONE'
        self.timer = self.create_timer(0.02, self.arbitrate_and_publish)
        self.get_logger().info('Command Arbitrator initialized. Priorities: '
                               + str([s for s, _ in self.priorities]))

    # ─── Callbacks ─────────────────────────────────────────────────────────────

    def estop_callback(self, msg: PixControlCmd):
        self.estop_active = msg.emergency_stop
        self.estop_time   = self.get_clock().now().nanoseconds / 1e9

    def command_callback(self, msg: PixControlCmd, source_name: str):
        self.cmd_storage[source_name]['msg']  = msg
        self.cmd_storage[source_name]['time'] = self.get_clock().now().nanoseconds / 1e9

    # ─── Arbitration loop (50 Hz) ───────────────────────────────────────────────

    def arbitrate_and_publish(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # 1. Unconditional E-Stop override
        if self.estop_active and (now - self.estop_time) < self.active_timeout:
            selected_source = 'EMERGENCY_STOP'
            selected_cmd = PixControlCmd()
            selected_cmd.header.stamp = self.get_clock().now().to_msg()
            selected_cmd.emergency_stop = True

        else:
            selected_source = None
            selected_cmd   = None

            # 2. Scan priorities highest → lowest
            for name, _ in self.priorities:
                data = self.cmd_storage[name]
                if data['msg'] is not None and (now - data['time']) < self.active_timeout:
                    selected_source = name
                    selected_cmd    = data['msg']
                    break

        current_label = selected_source if selected_source else 'STANDBY/NONE'

        # Preemption / state-change logging
        if current_label != self.last_active_source:
            if (self.last_active_source != 'STANDBY/NONE'
                    and current_label != 'STANDBY/NONE'):
                self.get_logger().warn(
                    f'PREEMPTION EVENT: [{self.last_active_source}] overridden by [{current_label}]')
            else:
                self.get_logger().info(
                    f'Arbitration State Change: [{self.last_active_source}] -> [{current_label}]')
            self.last_active_source = current_label

        # Publish winner or STANDBY no-op
        if selected_cmd is not None:
            self.raw_cmd_pub.publish(selected_cmd)
        else:
            standby = PixControlCmd()
            standby.header.stamp    = self.get_clock().now().to_msg()
            standby.header.frame_id = 'base_link'
            standby.steer_en  = False
            standby.drive_en  = False
            standby.brake_en  = False
            standby.gear_en   = False
            standby.park_en   = False
            self.raw_cmd_pub.publish(standby)


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

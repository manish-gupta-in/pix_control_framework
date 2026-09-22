#!/usr/bin/env python3
"""
Unit tests for PixCommandArbitrator.

All tests use PixControlCmd exclusively — no pix_control_msgs / Control / GearCommand.
"""
import time
import pytest
from unittest.mock import MagicMock
from pix_command_manager.command_arbitrator import PixCommandArbitrator
from pix_vehicle_msgs.msg import PixControlCmd
import rclpy


class TestPixCommandArbitrator:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def test_priority_order_config(self):
        """Priority list loads from YAML and has the correct order."""
        node = PixCommandArbitrator()
        names = [s for s, _ in node.priorities]
        assert names[0] == 'COLLISION_AVOIDANCE'
        assert names[1] == 'HUMAN_AVOIDANCE'
        assert names[2] == 'LANE_FOLLOWING'
        assert names[3] == 'CRUISE_CONTROL'
        node.destroy_node()

    def test_emergency_stop_unconditional_override(self):
        """E-stop overrides all active algorithm sources."""
        node = PixCommandArbitrator()

        # Fill every priority slot with fresh commands
        for name, _ in node.priorities:
            cmd = PixControlCmd()
            cmd.steer_target = 50.0
            node.cmd_storage[name]['msg']  = cmd
            node.cmd_storage[name]['time'] = node.get_clock().now().nanoseconds / 1e9

        # Trigger e-stop
        estop_msg = PixControlCmd()
        estop_msg.emergency_stop = True
        node.estop_callback(estop_msg)

        node.raw_cmd_pub = MagicMock()
        node.arbitrate_and_publish()

        published = node.raw_cmd_pub.publish.call_args[0][0]
        assert published.emergency_stop == True
        assert node.last_active_source == 'EMERGENCY_STOP'

        node.destroy_node()

    def test_higher_priority_wins(self):
        """COLLISION_AVOIDANCE beats LANE_FOLLOWING when both are fresh."""
        node = PixCommandArbitrator()
        now = node.get_clock().now().nanoseconds / 1e9

        lane_cmd = PixControlCmd()
        lane_cmd.speed_target = 1.5
        node.cmd_storage['LANE_FOLLOWING']['msg']  = lane_cmd
        node.cmd_storage['LANE_FOLLOWING']['time'] = now

        collision_cmd = PixControlCmd()
        collision_cmd.brake_target = 100.0
        node.cmd_storage['COLLISION_AVOIDANCE']['msg']  = collision_cmd
        node.cmd_storage['COLLISION_AVOIDANCE']['time'] = now

        node.raw_cmd_pub = MagicMock()
        node.arbitrate_and_publish()

        assert node.last_active_source == 'COLLISION_AVOIDANCE'

        node.destroy_node()

    def test_preemption_logging(self):
        """Switching from lower to higher priority emits a PREEMPTION EVENT log."""
        node = PixCommandArbitrator()
        now = node.get_clock().now().nanoseconds / 1e9

        # Only LANE_FOLLOWING is active
        cmd1 = PixControlCmd()
        node.cmd_storage['LANE_FOLLOWING']['msg']  = cmd1
        node.cmd_storage['LANE_FOLLOWING']['time'] = now

        node.raw_cmd_pub = MagicMock()
        node.arbitrate_and_publish()
        assert node.last_active_source == 'LANE_FOLLOWING'

        # COLLISION_AVOIDANCE activates
        cmd2 = PixControlCmd()
        node.cmd_storage['COLLISION_AVOIDANCE']['msg']  = cmd2
        node.cmd_storage['COLLISION_AVOIDANCE']['time'] = now

        node.get_logger().warn = MagicMock()
        node.arbitrate_and_publish()

        assert node.last_active_source == 'COLLISION_AVOIDANCE'
        node.get_logger().warn.assert_called_with(
            'PREEMPTION EVENT: [LANE_FOLLOWING] overridden by [COLLISION_AVOIDANCE]')

        node.destroy_node()

    def test_stale_source_falls_through(self):
        """A source whose last message is older than active_timeout is skipped."""
        node = PixCommandArbitrator()
        now = node.get_clock().now().nanoseconds / 1e9

        stale_cmd = PixControlCmd()
        node.cmd_storage['COLLISION_AVOIDANCE']['msg']  = stale_cmd
        node.cmd_storage['COLLISION_AVOIDANCE']['time'] = now - 10.0  # 10s ago — stale

        fresh_cmd = PixControlCmd()
        node.cmd_storage['LANE_FOLLOWING']['msg']  = fresh_cmd
        node.cmd_storage['LANE_FOLLOWING']['time'] = now

        node.raw_cmd_pub = MagicMock()
        node.arbitrate_and_publish()

        assert node.last_active_source == 'LANE_FOLLOWING'
        node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

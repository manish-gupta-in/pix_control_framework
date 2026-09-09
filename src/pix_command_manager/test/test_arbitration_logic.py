#!/usr/bin/env python3
import time
import pytest
import math
from unittest.mock import MagicMock
from pix_command_manager.command_arbitrator import PixCommandArbitrator
from pix_control_msgs.msg import Control
from pix_vehicle_msgs.msg import PixControlCmd
import rclpy

# We can run these with a mock rclpy context to instantiate the actual class
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

    def test_standard_message_normalization(self):
        node = PixCommandArbitrator()
        # Publish a standard control message
        control_msg = Control()
        control_msg.stamp = node.get_clock().now().to_msg()
        control_msg.steering_tire_angle = math.pi / 4  # 45 deg tire angle
        control_msg.steering_tire_rotation_rate = math.pi / 18 # 10 deg/s
        control_msg.velocity = 2.5
        control_msg.acceleration = -1.0 # Should map to braking

        node.standard_control_callback(control_msg, 'STANDARD_CONTROL')
        
        # Verify normalization
        stored = node.cmd_storage['STANDARD_CONTROL']['msg']
        assert stored is not None
        assert stored.steer_en == True
        # 45 deg * 16.6 = 747 deg wheel angle
        assert math.isclose(stored.steer_target, 747.0, abs_tol=0.1)
        # 10 deg/s * 16.6 = 166 deg/s
        assert math.isclose(stored.steer_speed, 166.0, abs_tol=0.1)
        assert stored.drive_en == True
        assert stored.speed_target == 2.5
        assert stored.brake_en == True
        assert stored.accel_target == 1.0 # abs(acceleration)
        assert stored.brake_target == 0.0 # Handled in CPP node
        
        node.destroy_node()

    def test_emergency_stop_unconditional_override(self):
        node = PixCommandArbitrator()
        
        # Fill all priorities with fresh messages
        for name, _ in node.priorities:
            cmd = PixControlCmd()
            cmd.steer_target = 100.0
            node.cmd_storage[name]['msg'] = cmd
            node.cmd_storage[name]['time'] = node.get_clock().now().nanoseconds / 1e9

        # Trigger estop
        estop_msg = PixControlCmd()
        estop_msg.emergency_stop = True
        node.estop_callback(estop_msg)

        # Mock publisher to capture output
        node.raw_cmd_pub = MagicMock()
        node.arbitrate_and_publish()

        published = node.raw_cmd_pub.publish.call_args[0][0]
        assert published.emergency_stop == True
        # E-stop is the unconditional source
        assert node.last_active_source == 'EMERGENCY_STOP'
        
        node.destroy_node()

    def test_priority_order_config(self):
        node = PixCommandArbitrator()
        # By default the config is loaded
        assert node.priorities[0][0] == 'STANDARD_CONTROL'
        assert node.priorities[1][0] == 'COLLISION_AVOIDANCE'
        
        node.destroy_node()

    def test_preemption_logging(self, capsys):
        node = PixCommandArbitrator()
        now = node.get_clock().now().nanoseconds / 1e9
        
        # Lower priority active
        cmd1 = PixControlCmd()
        node.cmd_storage['LANE_FOLLOWING']['msg'] = cmd1
        node.cmd_storage['LANE_FOLLOWING']['time'] = now
        
        # First tick
        node.arbitrate_and_publish()
        assert node.last_active_source == 'LANE_FOLLOWING'
        
        # Higher priority activates
        cmd2 = PixControlCmd()
        node.cmd_storage['COLLISION_AVOIDANCE']['msg'] = cmd2
        node.cmd_storage['COLLISION_AVOIDANCE']['time'] = now
        
        node.get_logger().warn = MagicMock()
        node.arbitrate_and_publish()
        
        assert node.last_active_source == 'COLLISION_AVOIDANCE'
        node.get_logger().warn.assert_called_with("PREEMPTION EVENT: [LANE_FOLLOWING] overridden by [COLLISION_AVOIDANCE]")
        
        node.destroy_node()

if __name__ == '__main__':
    pytest.main([__file__, '-v'])

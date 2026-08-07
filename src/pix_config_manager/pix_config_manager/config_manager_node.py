#!/usr/bin/env python3
"""
Configuration Profile Manager for PIXKIT Control Framework
===========================================================
Loads named YAML profiles at startup and broadcasts them as ROS2 parameters
via /pix/config/active_profile topic. Profiles are stored in:
  src/pix_config_manager/profiles/

Active profile is selected by parameter 'profile' (default: 'simulation').

Available profiles:
  simulation  — high gains, loose limits for sim testing
  hardware    — conservative gains, strict limits for real vehicle
  tuning      — intermediate settings for field tuning sessions
"""
import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class ConfigManagerNode(Node):
    """
    Loads and re-publishes active profile parameters.
    Other nodes can subscribe to /pix/config/active_profile to get the JSON-encoded config.
    """

    def __init__(self):
        super().__init__('pix_config_manager')
        self.declare_parameter('profile', 'simulation')
        self.declare_parameter('publish_rate', 1.0)

        profile_name = self.get_parameter('profile').value
        rate         = self.get_parameter('publish_rate').value

        # Locate profiles directory
        try:
            share = get_package_share_directory('pix_config_manager')
            profile_dir = os.path.join(share, 'profiles')
        except Exception:
            profile_dir = os.path.join(os.path.dirname(__file__), '../profiles')

        profile_path = os.path.join(profile_dir, f'{profile_name}.yaml')

        self._profile_name = profile_name
        self._profile_data = {}

        if os.path.exists(profile_path):
            with open(profile_path) as f:
                self._profile_data = yaml.safe_load(f) or {}
            self.get_logger().info(f"Loaded profile '{profile_name}' from {profile_path}")
            self._log_profile()
        else:
            self.get_logger().error(f"Profile '{profile_name}' not found at: {profile_path}")

        self._pub = self.create_publisher(String, '/pix/config/active_profile', 10)
        self.create_timer(1.0 / rate, self._publish)

    def _log_profile(self):
        """Log all profile parameters at startup."""
        self.get_logger().info(f'=== Profile: {self._profile_name} ===')
        for section, params in self._profile_data.items():
            self.get_logger().info(f'  [{section}]')
            if isinstance(params, dict):
                for k, v in params.items():
                    self.get_logger().info(f'    {k}: {v}')

    def _publish(self):
        import json
        msg = String()
        msg.data = json.dumps({
            'profile': self._profile_name,
            'params':  self._profile_data,
        })
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConfigManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from pix_vehicle_msgs.msg import PixControlCmd
from pix_algorithm_api import BaseAlgorithmInterface

class ObjectTrackingNode(BaseAlgorithmInterface):
    def __init__(self):
        super().__init__('object_tracking', '/pix/commands/collision_avoidance')
        
        self.declare_parameter('enable_control', True)
        self.declare_parameter('target_speed', 1.5)  # 1.5 m/s slow tracking speed
        
        self.enable_control = self.get_parameter('enable_control').value
        self.target_speed = self.get_parameter('target_speed').value
        
        # Command publishing timer (runs at 50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
    def control_loop(self):
        if not self.enable_control:
            return
            
        # Publish straight path and tracking speed
        self.publish_control_cmd(
            steer_target=0.0,  # Straight
            steer_speed=100.0,
            steer_en=True,
            speed_target=self.target_speed,
            accel_target=0.8,
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
    node = ObjectTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface

class AEBSystemNode(BaseAlgorithmInterface):
    """
    Autonomous Emergency Braking (AEB).
    Reads obstacle distances from YOLO perception and overrides control if collision is imminent.
    """
    def __init__(self):
        super().__init__('aeb_system', '/pix/commands/collision_avoidance')
        
        self.declare_parameter('ttc_threshold', 2.0) # Time to collision (seconds)
        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        
        self.yolo_sub = self.create_subscription(
            Float32MultiArray,
            '/perception/obstacles',
            self.perception_callback,
            10
        )
        
        self.min_obstacle_distance = 999.0
        self.timer = self.create_timer(0.05, self.aeb_logic_loop)
        
    def perception_callback(self, msg):
        # Assume msg.data contains distances to detected obstacles in front of the vehicle
        if msg.data:
            self.min_obstacle_distance = min(msg.data)
        else:
            self.min_obstacle_distance = 999.0
            
    def aeb_logic_loop(self):
        status = self.get_vehicle_status()
        current_speed = status.vehicle_speed if hasattr(status, 'vehicle_speed') else 0.0
        
        if current_speed > 0.5 and self.min_obstacle_distance < 999.0:
            ttc = self.min_obstacle_distance / current_speed
            
            if ttc < self.ttc_threshold:
                self.get_logger().error(f"AEB ENGAGED! TTC: {ttc:.2f}s, Dist: {self.min_obstacle_distance:.2f}m")
                # Command harsh braking!
                self.publish_control_cmd(
                    drive_en=False, speed_target=0.0, accel_target=0.0,
                    steer_en=True, steer_target=0.0, steer_speed=150.0, # Optional: keep steering straight
                    brake_en=True, brake_target=100.0, # Full brake
                    emergency_stop=True 
                )
            else:
                # No danger, output nothing or output an "OK" status
                pass

def main(args=None):
    rclpy.init(args=args)
    node = AEBSystemNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

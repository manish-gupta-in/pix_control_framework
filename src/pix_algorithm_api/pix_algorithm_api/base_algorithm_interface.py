import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd, PixVehicleStatus
from pix_control_msgs.msg import Control, GearCommand
import threading

class BaseAlgorithmInterface(Node):
    """
    Standard Base class for all modular control and perception algorithms in the PIX Control Framework.
    Provides easy access to vehicle state feedback and standardizes the command publishing pipeline.
    """
    def __init__(self, node_name, algorithm_topic):
        super().__init__(node_name)
        
        # Command publisher to arbitrator (Legacy)
        self.cmd_pub = self.create_publisher(
            PixControlCmd,
            algorithm_topic,
            10
        )
        
        # Standard publisher
        self.std_cmd_pub = self.create_publisher(
            Control,
            '/pix_framework/control_cmd',
            10
        )
        
        self.std_gear_pub = self.create_publisher(
            GearCommand,
            '/pix_framework/gear_cmd',
            10
        )
        
        # Vehicle status subscription
        self.vehicle_status = PixVehicleStatus()
        self.status_lock = threading.Lock()
        
        self.status_sub = self.create_subscription(
            PixVehicleStatus,
            '/pix/vehicle_status',
            self.status_callback,
            10
        )
        self.get_logger().info(f"Algorithm node '{node_name}' initialized. Publishing to: {algorithm_topic}")
        
    def status_callback(self, msg):
        with self.status_lock:
            self.vehicle_status = msg
            
    def get_vehicle_status(self):
        """
        Safely retrieve the latest vehicle status report.
        """
        with self.status_lock:
            return self.vehicle_status
            
    def publish_control_cmd(self, 
                            steer_target=0.0, 
                            steer_speed=150.0, 
                            steer_en=False,
                            speed_target=0.0, 
                            accel_target=1.0, 
                            drive_en=False,
                            brake_target=0.0, 
                            brake_en=False,
                            gear_target=PixControlCmd.GEAR_TARGET_INVALID, 
                            gear_en=False,
                            park_target=PixControlCmd.PARK_TARGET_RELEASE,
                            park_en=False,
                            turn_light_ctrl=0,
                            headlight_ctrl=False,
                            emergency_stop=False):
        """
        Helper method to construct and publish a standardized command message.
        """
        cmd = PixControlCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        # Steering
        cmd.steer_target = float(steer_target)
        cmd.steer_speed = float(steer_speed)
        cmd.steer_en = bool(steer_en)
        
        # Longitudinal
        cmd.speed_target = float(speed_target)
        cmd.accel_target = float(accel_target)
        cmd.drive_en = bool(drive_en)
        cmd.brake_target = float(brake_target)
        cmd.brake_en = bool(brake_en)
        
        # Gear and Park
        cmd.gear_target = int(gear_target)
        cmd.gear_en = bool(gear_en)
        cmd.park_target = int(park_target)
        cmd.park_en = bool(park_en)
        
        # Aux & Safety
        cmd.turn_light_ctrl = int(turn_light_ctrl)
        cmd.headlight_ctrl = bool(headlight_ctrl)
        cmd.emergency_stop = bool(emergency_stop)
        
        self.cmd_pub.publish(cmd)

    def publish_standard_control(self,
                                 steering_tire_angle=0.0,
                                 steering_tire_rotation_rate=0.0,
                                 velocity=0.0,
                                 acceleration=0.0,
                                 gear_command=None):
        """
        Publish standard control message on /pix_framework/control_cmd.
        """
        cmd = Control()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.steering_tire_angle = float(steering_tire_angle)
        cmd.steering_tire_rotation_rate = float(steering_tire_rotation_rate)
        cmd.velocity = float(velocity)
        cmd.acceleration = float(acceleration)
        self.std_cmd_pub.publish(cmd)
        
        if gear_command is not None:
            gear = GearCommand()
            gear.stamp = cmd.stamp
            gear.command = int(gear_command)
            self.std_gear_pub.publish(gear)

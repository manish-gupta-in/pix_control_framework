#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd, PixVehicleStatus
from geometry_msgs.msg import TransformStamped, Pose, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros
import math
import numpy as np

class PixVehicleSimulator(Node):
    def __init__(self):
        super().__init__('pix_vehicle_simulator')
        
        # Declare parameters
        self.declare_parameter('wheelbase', 2.0)           # meters
        self.declare_parameter('max_steer_limit', 500.0)    # VCU degrees
        self.declare_parameter('max_brake_decel', 4.0)     # m/s^2
        self.declare_parameter('friction_decel', 0.5)      # m/s^2
        
        self.L = self.get_parameter('wheelbase').value
        self.max_steer_limit = self.get_parameter('max_steer_limit').value
        self.max_brake_decel = self.get_parameter('max_brake_decel').value
        self.friction_decel = self.get_parameter('friction_decel').value
        
        # Simulated states
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0         # signed (m/s)
        self.steer_angle = 0.0    # degrees
        self.gear = PixControlCmd.GEAR_TARGET_NEUTRAL
        self.park = PixControlCmd.PARK_TARGET_RELEASE
        
        # Battery mock state
        self.battery_soc = 95.0
        self.battery_voltage = 55.0
        
        # Latest received command
        self.latest_cmd = None
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            PixControlCmd,
            '/pix/control_cmd',
            self.cmd_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(
            PixVehicleStatus,
            '/pix/vehicle_status',
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            '/pix/odom',
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            '/pix/visualization_marker',
            10
        )
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Simulation loop timer (50 Hz -> dt = 0.02s)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.simulation_step)
        
        self.get_logger().info("Vehicle Simulator Node Initialized.")
        
    def cmd_callback(self, msg):
        self.latest_cmd = msg
        
    def simulation_step(self):
        # 1. Integrate Dynamics
        if self.latest_cmd is not None:
            cmd = self.latest_cmd
            
            # Update gear state if enabled
            if cmd.gear_en:
                self.gear = cmd.gear_target
            if cmd.park_en:
                self.park = cmd.park_target
                
            # Decelerate / Accelerate speed
            target_spd = 0.0
            accel_lim = 1.0
            
            if cmd.drive_en:
                if self.gear == PixControlCmd.GEAR_TARGET_DRIVE:
                    target_spd = cmd.speed_target
                elif self.gear == PixControlCmd.GEAR_TARGET_REVERSE:
                    target_spd = -cmd.speed_target
                accel_lim = cmd.accel_target if cmd.accel_target > 0 else 1.0
                
            # If braking is enabled
            decel_val = self.friction_decel
            if cmd.brake_en and cmd.brake_target > 0:
                decel_val += (cmd.brake_target / 100.0) * self.max_brake_decel
                
            if cmd.emergency_stop:
                decel_val = self.max_brake_decel * 2.0  # hard e-stop deceleration
                target_spd = 0.0
                
            # Speed updates
            if self.speed < target_spd:
                self.speed = min(target_spd, self.speed + accel_lim * self.dt)
            elif self.speed > target_spd:
                self.speed = max(target_spd, self.speed - decel_val * self.dt)
                
            # Steering updates
            if cmd.steer_en:
                steer_diff = cmd.steer_target - self.steer_angle
                max_change = cmd.steer_speed * self.dt
                if abs(steer_diff) <= max_change:
                    self.steer_angle = cmd.steer_target
                else:
                    self.steer_angle += np.sign(steer_diff) * max_change
                    
                self.steer_angle = np.clip(self.steer_angle, -self.max_steer_limit, self.max_steer_limit)
        else:
            # Passive deceleration (friction)
            if self.speed > 0:
                self.speed = max(0.0, self.speed - self.friction_decel * self.dt)
            elif self.speed < 0:
                self.speed = min(0.0, self.speed + self.friction_decel * self.dt)
                
        # 2. Integrate Kinematic equations (Ackermann)
        # yaw rate = v/L * tan(delta_rad)
        # delta here is steering wheel angle. Let's assume ratio of steering wheel to wheel angle is 15.0
        wheel_angle_rad = math.radians(self.steer_angle / 15.0)
        
        self.x += self.speed * math.cos(self.yaw) * self.dt
        self.y += self.speed * math.sin(self.yaw) * self.dt
        
        yaw_rate = (self.speed / self.L) * math.tan(wheel_angle_rad)
        self.yaw += yaw_rate * self.dt
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        
        # 3. Discharge Battery
        self.battery_soc = max(10.0, self.battery_soc - 0.0001 * (1.0 + abs(self.speed)))
        self.battery_voltage = 54.0 - (100.0 - self.battery_soc) * 0.05 - abs(self.speed) * 0.2
        
        # 4. Publish ROS2 Status Message
        self.publish_status_msg()
        
        # 5. Publish TF and Odom
        self.publish_odom_and_tf(yaw_rate)
        
        # 6. Publish RViz Visualization Marker
        self.publish_rviz_marker()
        
    def publish_status_msg(self):
        status = PixVehicleStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
        
        status.steer_angle = float(self.steer_angle)
        status.steer_speed = 120.0
        status.steer_en_state = 1 if (self.latest_cmd and self.latest_cmd.steer_en) else 0
        
        status.vehicle_speed = float(self.speed)
        status.vehicle_accel = 0.0
        status.throttle_pedal = 20.0 if (self.speed != 0.0 and self.latest_cmd and self.latest_cmd.drive_en) else 0.0
        status.brake_pedal = float(self.latest_cmd.brake_target) if (self.latest_cmd and self.latest_cmd.brake_en) else 0.0
        
        status.drive_en_state = 1 if (self.latest_cmd and self.latest_cmd.drive_en) else 0
        status.brake_en_state = 1 if (self.latest_cmd and self.latest_cmd.brake_en) else 0
        
        status.gear_actual = int(self.gear)
        status.park_actual = int(self.park)
        
        status.vehicle_mode = 1 if (status.steer_en_state or status.drive_en_state) else 0  # 1 = Auto, 0 = Manual
        status.drive_mode_status = 1  # Speed drive
        status.steer_mode_status = 0  # Standard steer
        
        status.battery_voltage = float(self.battery_voltage)
        status.battery_current = float(abs(self.speed) * 4.5)
        status.battery_soc = float(self.battery_soc)
        
        self.status_pub.publish(status)
        
    def publish_odom_and_tf(self, yaw_rate):
        now = self.get_clock().now().to_msg()
        
        # Euler to Quaternion conversion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = 1.0 # cos(0)
        sp = 0.0 # sin(0)
        cr = 1.0 # cos(0)
        sr = 0.0 # sin(0)
        
        qx = 0.0
        qy = 0.0
        qz = sy
        qw = cy
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Twist
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = yaw_rate
        
        self.odom_pub.publish(odom)
        
    def publish_rviz_marker(self):
        # Publish vehicle marker (Box) representing the PIXKIT shuttle
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        marker.ns = 'shuttle_model'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Center of the shuttle relative to base_link (rear axle center)
        # Shuttle is 3m long, rear axle is 0.5m from rear, so center is 1.0m forward of rear axle
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.9  # half height
        marker.pose.orientation.w = 1.0
        
        # Size: Length 3.0m, Width 1.6m, Height 1.8m
        marker.scale.x = 3.0
        marker.scale.y = 1.6
        marker.scale.z = 1.8
        
        # Color: sleek dark green with transparency
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.2
        marker.color.a = 0.75
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PixVehicleSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

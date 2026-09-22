#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32MultiArray
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface


class AEBSystemNode(BaseAlgorithmInterface):
    """
    Autonomous Emergency Braking (AEB).

    Subscribes to /perception/obstacles (Float32MultiArray):
        msg.data[0] = estimated distance to closest person in metres
        msg.data[1] = lateral offset (unused by AEB — handled by avoidance node)

    When TTC (distance / current_speed) < ttc_threshold:
        - Publishes full brake on /pix/commands/collision_avoidance (Priority 1)
        - drive_en=False, brake_en=True, brake_target=100%
        - gear_en=True, gear_target=4 (DRIVE — do NOT drop to Neutral while braking)
        - emergency_stop flag set so safety manager latches brakes unconditionally

    When no danger: publishes nothing (COLLISION_AVOIDANCE source goes stale after
    active_timeout, allowing the lower-priority LANE_FOLLOWING source to drive).
    """

    def __init__(self):
        super().__init__('aeb_system', '/pix/commands/collision_avoidance')

        self.declare_parameter('ttc_threshold', 2.0)   # seconds
        self.declare_parameter('min_trigger_speed', 0.3)  # m/s
        self.declare_parameter('min_distance', 2.5)       # meters — trigger if closer than this even at standstill
        self.ttc_threshold   = self.get_parameter('ttc_threshold').value
        self.min_trigger_spd = self.get_parameter('min_trigger_speed').value
        self.min_distance    = self.get_parameter('min_distance').value

        # /perception/obstacles from yolo_perception_node
        # data[0] = distance (m), data[1] = lateral_offset (normalised, ignored here)
        self.obstacle_distance = 999.0
        self.yolo_sub = self.create_subscription(
            Float32MultiArray, '/perception/obstacles',
            self.perception_callback, 10)

        # AEB loop at 20 Hz (same as control loop — fast enough for 5 m/s max speed)
        self.timer = self.create_timer(0.05, self.aeb_logic_loop)
        self.aeb_engaged = False
        self.get_logger().info(
            f'AEB node ready. TTC threshold={self.ttc_threshold}s, '
            f'min trigger speed={self.min_trigger_spd} m/s')

    def perception_callback(self, msg: Float32MultiArray):
        """Extract obstacle distance from perception array (index 0 = distance)."""
        if msg.data and len(msg.data) >= 1:
            self.obstacle_distance = float(msg.data[0])   # metres
        else:
            self.obstacle_distance = 999.0

    def aeb_logic_loop(self):
        status = self.get_vehicle_status()
        # PixVehicleStatus field: vehicle_speed (m/s)
        current_speed = float(getattr(status, 'vehicle_speed', 0.0))

        # Engage if TTC is critical OR if the obstacle is absolutely very close
        is_moving_fast_enough = current_speed > self.min_trigger_spd
        ttc = self.obstacle_distance / max(current_speed, 0.01)
        
        trigger_ttc = is_moving_fast_enough and (ttc < self.ttc_threshold)
        trigger_dist = self.obstacle_distance < self.min_distance

        if (trigger_ttc or trigger_dist) and self.obstacle_distance < 900.0:
            if not self.aeb_engaged:
                self.get_logger().error(
                    f'AEB ENGAGED! dist={self.obstacle_distance:.2f}m '
                    f'speed={current_speed:.2f}m/s TTC={ttc:.2f}s')
                self.aeb_engaged = True

                # Priority 1 (COLLISION_AVOIDANCE) beats LANE_FOLLOWING.
                # emergency_stop=True: safety manager applies full brakes regardless.
                # Keep gear_target=4 (DRIVE) — dropping to Neutral while moving is unsafe.
                self.publish_control_cmd(
                    drive_en=False,  speed_target=0.0,   accel_target=0.0,
                    steer_en=True,   steer_target=0.0,   steer_speed=150.0,
                    brake_en=True,   brake_target=100.0,
                    gear_en=True,    gear_target=4,        # stay in DRIVE
                    park_en=False,   park_target=0,        # do not engage park at speed
                    emergency_stop=True
                )
                return   # Do not fall through — keep COLLISION_AVOIDANCE alive

        # No danger (or vehicle stopped): stop publishing so the source goes stale.
        # After active_timeout (0.4s), the arbitrator drops COLLISION_AVOIDANCE and
        # LANE_FOLLOWING resumes driving.
        if self.aeb_engaged:
            self.get_logger().info('AEB cleared — resuming normal operation.')
            self.aeb_engaged = False
        # (publish nothing — intentional)


def main(args=None):
    rclpy.init(args=args)
    node = AEBSystemNode()
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

#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32MultiArray
from pix_vehicle_msgs.msg import PixControlCmd
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface


class AEBSystemNode(BaseAlgorithmInterface):
    """
    Autonomous Emergency Braking (AEB) — v20.

    Subscribes to /perception/obstacles (Float32MultiArray):
        msg.data[0] = estimated distance to closest obstacle in metres
        msg.data[1] = lateral offset (unused by AEB — handled by avoidance node)

    AEB braking is a PRIORITY-1 override via the arbitrator — it does NOT use
    emergency_stop=True. Reason: emergency_stop latches the safety manager forever,
    preventing automatic resume when the obstacle clears. AEB must be self-recovering.

    Trigger conditions (either):
        1. TTC (distance / speed) < ttc_threshold  AND  speed > min_trigger_speed
        2. distance < min_distance  (even if vehicle is stationary)

    When triggered:
        - Continuously publishes full-brake command to /pix/commands/collision_avoidance
        - Priority 1 in arbitrator → overrides ALL driving commands
        - gear stays at DRIVE (4) — dropping to Neutral at speed is unsafe
        - park NOT engaged — park brake while moving would cause wheel lock

    When cleared (obstacle > min_distance AND TTC > threshold):
        - Stops publishing → COLLISION_AVOIDANCE source goes stale (0.4 s)
        - Arbitrator falls back to LANE_FOLLOWING → vehicle resumes

    For a TRUE hard E-stop (sensor fail, crash contact): publish to /pix/estop_trigger.
    That is a separate, operator-cleared latch. AEB must NOT use it.
    """

    def __init__(self):
        super().__init__('aeb_system', '/pix/commands/collision_avoidance')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('ttc_threshold',    2.0)   # seconds
        self.declare_parameter('min_trigger_speed', 0.3)  # m/s — ignore TTC if nearly stopped
        self.declare_parameter('min_distance',     2.5)   # m — absolute trigger even at standstill
        self.declare_parameter('resume_hysteresis', 0.5)  # extra margin before resume (m and s)

        self.ttc_threshold    = self.get_parameter('ttc_threshold').value
        self.min_trigger_spd  = self.get_parameter('min_trigger_speed').value
        self.min_distance     = self.get_parameter('min_distance').value
        self.resume_hysteresis = self.get_parameter('resume_hysteresis').value

        # ── Perception Input ─────────────────────────────────────────────────
        self.obstacle_distance = 999.0
        self.yolo_sub = self.create_subscription(
            Float32MultiArray, '/perception/obstacles',
            self.perception_callback, 10)

        # ── State ────────────────────────────────────────────────────────────
        self.aeb_engaged = False

        # ── Loop at 20 Hz ────────────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.aeb_logic_loop)
        self.get_logger().info(
            f'AEB ready. TTC={self.ttc_threshold}s | '
            f'min_dist={self.min_distance}m | '
            f'min_spd={self.min_trigger_spd}m/s')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def perception_callback(self, msg: Float32MultiArray):
        """data[0]=distance (m), data[1]=lateral offset (ignored by AEB)."""
        if msg.data and len(msg.data) >= 1:
            self.obstacle_distance = float(msg.data[0])
        else:
            self.obstacle_distance = 999.0

    # ── Control Loop ──────────────────────────────────────────────────────────

    def aeb_logic_loop(self):
        status = self.get_vehicle_status()
        current_speed = float(getattr(status, 'vehicle_speed', 0.0))
        dist = self.obstacle_distance

        # ── Engage condition ──────────────────────────────────────────────────
        ttc = dist / max(current_speed, 0.01)
        trigger_ttc  = (current_speed > self.min_trigger_spd) and (ttc < self.ttc_threshold)
        trigger_dist = dist < self.min_distance

        should_engage = (trigger_ttc or trigger_dist) and dist < 900.0

        # ── Clear condition (hysteresis prevents rapid toggle) ────────────────
        resume_clear = (
            dist > (self.min_distance + self.resume_hysteresis) and
            (not trigger_ttc)
        )

        if should_engage:
            if not self.aeb_engaged:
                self.get_logger().error(
                    f'[AEB ENGAGED] dist={dist:.2f}m speed={current_speed:.2f}m/s TTC={ttc:.2f}s')
                self.aeb_engaged = True

            # ── Publish braking command — NO emergency_stop flag ──────────────
            # Using the priority-1 arbitration slot is sufficient to override
            # all driving commands without latching the safety manager.
            self.publish_control_cmd(
                drive_en=True,   speed_target=0.0,     accel_target=0.0,
                steer_en=True,   steer_target=0.0,     steer_speed=150.0,
                brake_en=True,   brake_target=100.0,
                gear_en=True,    gear_target=4,   # stay in DRIVE — do NOT drop to Neutral
                park_en=True,    park_target=0,   # do NOT engage park brake at speed
                emergency_stop=False              # NEVER latch — AEB must be self-recovering
            )

        elif self.aeb_engaged and resume_clear:
            # ── Stop publishing → source goes stale → arbitrator resumes LANE_FOLLOWING ──
            self.get_logger().info(
                f'[AEB CLEARED] dist={dist:.2f}m — resuming normal operation.')
            self.aeb_engaged = False
            # Intentionally publish nothing here


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

#!/usr/bin/env python3
"""
straight_drive_node.py — Integrated Lane-Following + Avoidance + AEB Node (v20)

Full autonomy sequence:
  1. STARTUP (safe gear-shift interlock, ~9s):
       WAKE → SHIFT → RELEASE_BRAKE → DRIVE
  2. LANE_FOLLOWING:
       Straight cruise at target speed, 0 steer.
       Publishes to /pix/commands/lane_following (Priority 4).
  3. SLOWING_AVOIDANCE (person detected, > aeb_min_distance):
       Slow down + steer away (proportional to person lateral offset).
       Still publishes to /pix/commands/lane_following.
  4. AEB (person inside aeb_min_distance or TTC < threshold):
       aeb_node takes over via /pix/commands/collision_avoidance (Priority 1).
       This node detects AEB is active and holds state.
  5. RESUME (person gone, AEB cleared):
       aeb_node stops publishing → arbitrator falls back to lane_following.
       This node resumes straight cruise.

Perception input: /perception/obstacles (Float32MultiArray)
  data[0] = distance to closest person (m)
  data[1] = lateral offset (normalized: -1=left, 0=center, +1=right)
"""
import rclpy
from std_msgs.msg import Float32MultiArray
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface


class StraightDriveNode(BaseAlgorithmInterface):
    """
    Integrated straight-line driver with person-aware speed control and steering avoidance.
    Publishes to /pix/commands/lane_following (Priority 4 in arbitrator).
    AEB node (Priority 1) overrides when obstacle is too close.
    """

    def __init__(self):
        super().__init__('straight_drive_planner', '/pix/commands/lane_following')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('speed', 1.5)              # m/s cruise speed
        self.declare_parameter('steer_speed_dps', 250.0)  # deg/s — Whale: 4.36 rad/s

        # Avoidance: starts slowing/steering when person detected
        self.declare_parameter('avoidance_start_dist', 5.0)   # m — start slowing
        self.declare_parameter('avoidance_steer_gain', 40.0)  # deg per unit offset
        self.declare_parameter('avoidance_max_steer', 200.0)  # deg — max avoidance steer
        self.declare_parameter('avoidance_min_speed', 0.4)    # m/s — slow to this while avoiding

        # AEB handoff thresholds (must match aeb_node params)
        self.declare_parameter('aeb_min_distance', 2.5)       # m — AEB takes over below this
        self.declare_parameter('aeb_ttc_threshold', 2.0)      # s

        self.speed            = self.get_parameter('speed').value
        self.steer_speed_dps  = self.get_parameter('steer_speed_dps').value
        self.avoid_start_dist = self.get_parameter('avoidance_start_dist').value
        self.avoid_steer_gain = self.get_parameter('avoidance_steer_gain').value
        self.avoid_max_steer  = self.get_parameter('avoidance_max_steer').value
        self.avoid_min_speed  = self.get_parameter('avoidance_min_speed').value
        self.aeb_min_dist     = self.get_parameter('aeb_min_distance').value
        self.aeb_ttc_thresh   = self.get_parameter('aeb_ttc_threshold').value

        # ── Perception subscriber ──────────────────────────────────────────
        self.obstacle_distance = 999.0
        self.obstacle_offset   = 0.0   # lateral offset: -1=left, 0=center, +1=right
        self.create_subscription(
            Float32MultiArray, '/perception/obstacles',
            self.perception_callback, 10)

        # ── Startup state machine ──────────────────────────────────────────
        # Required VCU gear interlock sequence (verified against actuator_test.py):
        #   WAKE (5s):          Neutral + 100% brake → VCU wakes up cleanly
        #   SHIFT (3s):         DRIVE + 100% brake → VCU accepts gear without fault
        #   RELEASE_BRAKE (1s): DRIVE + 0% brake  → smooth ramp to motion
        #   DRIVING:            Normal operation
        self.state = 'WAKE'
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info(
            f'StraightDriveNode ready. Speed={self.speed}m/s | '
            f'AvoidStart={self.avoid_start_dist}m | AEB_dist={self.aeb_min_dist}m')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def perception_callback(self, msg: Float32MultiArray):
        if msg.data and len(msg.data) >= 2:
            self.obstacle_distance = float(msg.data[0])
            self.obstacle_offset   = float(msg.data[1])
        elif msg.data and len(msg.data) >= 1:
            self.obstacle_distance = float(msg.data[0])
        else:
            self.obstacle_distance = 999.0
            self.obstacle_offset   = 0.0

    # ── Control loop ───────────────────────────────────────────────────────────

    def control_loop(self):
        now     = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time
        dist    = self.obstacle_distance
        offset  = self.obstacle_offset

        # ══════════════════════════════════════════════════════
        # PHASE 1: Safe startup gear-shift interlock sequence
        # ══════════════════════════════════════════════════════

        if self.state == 'WAKE':
            # Neutral + 100% brake: VCU safe wake
            self._cmd(speed=0.0, gear=3, brake=100.0, steer=0.0)
            if elapsed > 5.0:
                self.state = 'SHIFT'
                self.state_start_time = now
                self.get_logger().info('WAKE done → SHIFT (DRIVE + 100% brake)')
            return

        if self.state == 'SHIFT':
            # DRIVE + 100% brake: VCU accepts gear change
            self._cmd(speed=0.0, gear=4, brake=100.0, steer=0.0)
            if elapsed > 3.0:
                self.state = 'RELEASE_BRAKE'
                self.state_start_time = now
                self.get_logger().info('SHIFT done → RELEASE_BRAKE')
            return

        if self.state == 'RELEASE_BRAKE':
            # DRIVE + 0% brake: smooth handoff to motion
            self._cmd(speed=0.0, gear=4, brake=0.0, steer=0.0)
            if elapsed > 1.0:
                self.state = 'DRIVING'
                self.state_start_time = now
                self.get_logger().info(f'RELEASE_BRAKE done → DRIVING at {self.speed} m/s')
            return

        # ══════════════════════════════════════════════════════
        # PHASE 2: DRIVING — Sense → Plan → Act
        # ══════════════════════════════════════════════════════

        # AEB zone: person is too close — stop publishing (let AEB node take over)
        # aeb_node publishes to /pix/commands/collision_avoidance (Priority 1)
        # which overrides our /pix/commands/lane_following (Priority 4).
        # We simply stop publishing — the arbitrator will select AEB's command.
        # When AEB clears (person moves away), we resume publishing and cruise resumes.
        status = self.get_vehicle_status()
        current_speed = float(getattr(status, 'vehicle_speed', 0.0))
        ttc = dist / max(current_speed, 0.01)
        in_aeb_zone = (dist < self.aeb_min_dist) or (
            current_speed > 0.3 and ttc < self.aeb_ttc_thresh
        )

        if in_aeb_zone:
            # AEB node handles it — we publish nothing this cycle.
            # The arbitrator's active_timeout (0.4s) means if we stop for >0.4s,
            # LANE_FOLLOWING goes stale and AEB (collision_avoidance) wins.
            # This is safe — do NOT publish a command that fights AEB.
            if self.state != 'AEB_HOLD':
                self.get_logger().warn(
                    f'[LANE] AEB zone entered (dist={dist:.2f}m) — deferring to AEB node')
                self.state = 'AEB_HOLD'
            return

        # Back from AEB
        if self.state == 'AEB_HOLD':
            self.get_logger().info(
                f'[LANE] AEB cleared (dist={dist:.2f}m) — resuming lane following')
            self.state = 'DRIVING'

        # ── Avoidance zone: person visible but not in AEB range ──────────────
        if dist < self.avoid_start_dist and dist < 900.0:
            # Scale down speed proportionally: closer → slower
            # At avoid_start_dist → full speed; at aeb_min_dist → avoid_min_speed
            avoidance_range = max(self.avoid_start_dist - self.aeb_min_dist, 0.1)
            proximity = max(0.0, self.avoid_start_dist - dist) / avoidance_range
            target_speed = self.speed - proximity * (self.speed - self.avoid_min_speed)
            target_speed = max(self.avoid_min_speed, target_speed)

            # Steer away: person on left (offset < 0) → steer right (+); vice versa
            steer_cmd = -offset * self.avoid_steer_gain * (1.0 + proximity)
            steer_cmd = max(-self.avoid_max_steer, min(self.avoid_max_steer, steer_cmd))

            if self.state != 'AVOIDING':
                self.get_logger().warn(
                    f'[LANE] AVOIDING: dist={dist:.2f}m offset={offset:+.2f} '
                    f'speed={target_speed:.2f}m/s steer={steer_cmd:+.1f}deg')
                self.state = 'AVOIDING'

            self._cmd(speed=target_speed, gear=4, brake=0.0, steer=steer_cmd)

        else:
            # ── Clear: drive straight at cruise speed ─────────────────────────
            if self.state == 'AVOIDING':
                self.get_logger().info('[LANE] Obstacle clear — resuming straight cruise')
                self.state = 'DRIVING'

            self._cmd(speed=self.speed, gear=4, brake=0.0, steer=0.0)

    # ── Helper ─────────────────────────────────────────────────────────────────

    def _cmd(self, speed: float, gear: int, brake: float, steer: float):
        """Publish a PixControlCmd with the given params."""
        self.publish_control_cmd(
            drive_en=True,   speed_target=speed,     accel_target=1.5,
            steer_en=True,   steer_target=steer,     steer_speed=self.steer_speed_dps,
            brake_en=(brake > 0.0), brake_target=brake,
            gear_en=True,    gear_target=gear,
            park_en=True,    park_target=0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = StraightDriveNode()
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

#!/usr/bin/env python3
"""
System State Manager for PIXKIT Control Framework
==================================================
Manages the high-level operational state machine:

  MANUAL (0)    — Human control, DBW off. Default power-on state.
  STANDBY (1)   — DBW armed, no autonomous algorithm active.
  AUTONOMOUS (2)— Algorithm actively commanding the vehicle.
  FAULT (3)     — Recoverable fault detected. Safe fallback active.
                  Attempts auto-recovery after fault clears.
  ESTOP (4)     — Emergency stop latched. Requires explicit /pix/estop_clear.

State transitions are validated based on:
  - VCU feedback (steer_en_state, drive_en_state, faults)
  - Algorithm activity (monitoring /pix/raw_control_cmd)
  - External triggers (/pix/estop_trigger, /pix/estop_clear)
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from pix_vehicle_msgs.msg import PixSystemState, PixVehicleStatus, PixControlCmd


# ─── State constants ────────────────────────────────────────────────────────
STATE_MANUAL     = PixSystemState.STATE_MANUAL
STATE_STANDBY    = PixSystemState.STATE_STANDBY
STATE_AUTONOMOUS = PixSystemState.STATE_AUTONOMOUS
STATE_FAULT      = PixSystemState.STATE_FAULT
STATE_ESTOP      = PixSystemState.STATE_ESTOP

STATE_NAMES = {
    STATE_MANUAL:     'MANUAL',
    STATE_STANDBY:    'STANDBY',
    STATE_AUTONOMOUS: 'AUTONOMOUS',
    STATE_FAULT:      'FAULT',
    STATE_ESTOP:      'ESTOP',
}


class SystemStateManager(Node):
    """
    Publishes /pix/system_state at 10 Hz.
    Transitions state based on VCU feedback and external triggers.
    """

    def __init__(self):
        super().__init__('pix_system_state_manager')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('algorithm_timeout', 0.5)    # seconds before algo considered inactive
        self.declare_parameter('fault_clear_delay', 2.0)    # seconds to wait before recovering from fault
        self.declare_parameter('dbw_detect_timeout', 1.0)   # seconds to confirm DBW engagement
        rate = self.get_parameter('publish_rate').value
        self.algo_timeout    = self.get_parameter('algorithm_timeout').value
        self.fault_clear_del = self.get_parameter('fault_clear_delay').value

        # State machine
        self._state          = STATE_MANUAL
        self._state_reason   = 'Power-on default'
        self._state_entry_ts = time.monotonic()
        self._estop_latched  = False
        self._fault_latched  = False
        self._fault_count    = 0
        self._fault_clear_ts = None          # when faults last cleared
        self._active_algo    = ''

        # Latest data
        self._latest_status       = None
        self._latest_raw_cmd      = None
        self._latest_raw_cmd_ts   = 0.0
        self._dbw_enabled         = False
        # Grace period: same 3s window as safety_manager — ignore startup faults
        self._startup_grace = 3.0
        self._node_start_ts = time.monotonic()

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(PixVehicleStatus, '/pix/vehicle_status',
                                 self._status_cb, 10)
        self.create_subscription(PixControlCmd, '/pix/raw_control_cmd',
                                 self._raw_cmd_cb, 10)
        self.create_subscription(Bool, '/pix/estop_trigger',
                                 self._estop_trigger_cb, 10)
        self.create_subscription(Bool, '/pix/estop_clear',
                                 self._estop_clear_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.state_pub = self.create_publisher(PixSystemState, '/pix/system_state', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._loop)
        self.get_logger().info('System State Manager initialized. State: MANUAL')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _status_cb(self, msg: PixVehicleStatus):
        self._latest_status = msg
        # Determine DBW engagement from steer_en_state (1=Auto, 2=Takeover, 3=Standby all mean DBW-capable)
        # Manual (0) means DBW off
        self._dbw_enabled = (msg.steer_en_state != 0 or
                             msg.drive_en_state != 0 or
                             msg.brake_en_state != 0)

    def _raw_cmd_cb(self, msg: PixControlCmd):
        self._latest_raw_cmd    = msg
        self._latest_raw_cmd_ts = time.monotonic()
        # Extract source algorithm name if available (from header frame_id convention)
        if msg.header.frame_id and msg.header.frame_id != 'base_link':
            self._active_algo = msg.header.frame_id
        elif msg.steer_en or msg.drive_en or msg.brake_en:
            self._active_algo = 'algorithm'
        else:
            self._active_algo = ''

    def _estop_trigger_cb(self, msg: Bool):
        if msg.data:
            self._transition(STATE_ESTOP, 'External /pix/estop_trigger received')
            self._estop_latched = True

    def _estop_clear_cb(self, msg: Bool):
        if msg.data and self._state == STATE_ESTOP:
            self._estop_latched = False
            self._fault_latched = False
            self._transition(STATE_MANUAL, 'E-stop cleared via /pix/estop_clear')
            self.get_logger().warn('E-stop cleared. Returning to MANUAL.')

    # ── State Machine ─────────────────────────────────────────────────────────

    def _transition(self, new_state: int, reason: str):
        old_name = STATE_NAMES.get(self._state, '?')
        new_name = STATE_NAMES.get(new_state, '?')
        if new_state != self._state:
            if new_state == STATE_FAULT:
                self._fault_count += 1
                self._fault_latched = True
                self._fault_clear_ts = None
            self.get_logger().info(f'State: {old_name} → {new_name}  ({reason})')
            self._state          = new_state
            self._state_reason   = reason
            self._state_entry_ts = time.monotonic()

    def _check_chassis_faults(self) -> str:
        """Returns fault description string if any fault is active, else empty."""
        # Skip during startup grace period — VCU sends transient faults while booting
        if (time.monotonic() - self._node_start_ts) < self._startup_grace:
            return ''
        if self._latest_status is None:
            return ''
        s = self._latest_status
        if s.steer_flt1 or s.steer_flt2:
            return f'Steering fault (flt1={s.steer_flt1} flt2={s.steer_flt2})'
        if s.drive_flt1 or s.drive_flt2:
            return f'Drive fault (flt1={s.drive_flt1} flt2={s.drive_flt2})'
        if s.brake_flt1 or s.brake_flt2:
            return f'Brake fault (flt1={s.brake_flt1} flt2={s.brake_flt2})'
        if s.park_flt:
            return f'Park fault (flt={s.park_flt})'
        if s.gear_flt:
            return f'Gear fault (flt={s.gear_flt})'
        if s.front_crash:
            return 'Front crash sensor triggered'
        if s.back_crash:
            return 'Back crash sensor triggered'
        return ''

    def _algo_active(self) -> bool:
        """True if an algorithm published a command recently."""
        if self._latest_raw_cmd is None:
            return False
        age = time.monotonic() - self._latest_raw_cmd_ts
        if age > self.algo_timeout:
            return False
        cmd = self._latest_raw_cmd
        return (cmd.steer_en or cmd.drive_en or cmd.brake_en or cmd.gear_en)

    def _update_state(self):
        now = time.monotonic()

        # ESTOP is latched — only /pix/estop_clear can exit
        if self._state == STATE_ESTOP:
            return

        fault_desc = self._check_chassis_faults()

        # Any active fault triggers FAULT state from any non-ESTOP state
        if fault_desc:
            if self._state != STATE_FAULT:
                self._transition(STATE_FAULT, fault_desc)
            self._fault_latched = True
            self._fault_clear_ts = None
            return

        # Fault cleared — start recovery countdown
        if self._state == STATE_FAULT:
            if self._fault_clear_ts is None:
                self._fault_clear_ts = now
            elif (now - self._fault_clear_ts) >= self.fault_clear_del:
                self._fault_latched = False
                self._transition(STATE_STANDBY, 'Fault cleared, recovery complete')
            return

        # Normal state transitions
        if self._state == STATE_MANUAL:
            if self._dbw_enabled:
                self._transition(STATE_STANDBY, 'DBW engaged by VCU')

        elif self._state == STATE_STANDBY:
            if not self._dbw_enabled:
                self._transition(STATE_MANUAL, 'DBW disengaged')
            elif self._algo_active():
                self._transition(STATE_AUTONOMOUS, f'Algorithm active: {self._active_algo}')

        elif self._state == STATE_AUTONOMOUS:
            if not self._dbw_enabled:
                self._transition(STATE_MANUAL, 'DBW disengaged during autonomous')
            elif not self._algo_active():
                self._transition(STATE_STANDBY, 'No active algorithm commands')

    def _loop(self):
        self._update_state()
        self._publish()

    def _publish(self):
        msg = PixSystemState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.state           = self._state
        msg.reason          = self._state_reason
        msg.state_duration_secs = time.monotonic() - self._state_entry_ts
        msg.estop_latched   = self._estop_latched
        msg.fault_latched   = self._fault_latched
        msg.active_algorithm = self._active_algo
        msg.fault_count     = self._fault_count
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

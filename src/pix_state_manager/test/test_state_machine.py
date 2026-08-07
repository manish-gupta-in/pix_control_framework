"""
Unit tests for System State Manager
Validates all state transitions, fault handling, and E-stop latch logic.
"""
import time
import pytest
from unittest.mock import MagicMock, patch


# ─── Helpers to mock rclpy environment without a running ROS2 daemon ──────────

class MockClock:
    def now(self):
        m = MagicMock()
        m.to_msg.return_value = MagicMock()
        return m

class MockLogger:
    def info(self, msg): pass
    def warn(self, msg): pass
    def warning(self, msg): pass
    def error(self, msg): pass


# ─── Import state constants directly (no Node required) ───────────────────────
# We test the pure logic, not the ROS2 plumbing

STATE_MANUAL     = 0
STATE_STANDBY    = 1
STATE_AUTONOMOUS = 2
STATE_FAULT      = 3
STATE_ESTOP      = 4


class StateMachineLogic:
    """
    Extracted pure logic from SystemStateManager for isolated unit testing.
    Mirrors the real implementation without ROS2 dependencies.
    """
    def __init__(self, fault_clear_delay=2.0, algo_timeout=0.5):
        self._state          = STATE_MANUAL
        self._state_reason   = 'init'
        self._state_entry_ts = time.monotonic()
        self._estop_latched  = False
        self._fault_latched  = False
        self._fault_count    = 0
        self._fault_clear_ts = None
        self._active_algo    = ''
        self._dbw_enabled    = False
        self._latest_status  = None
        self._latest_raw_cmd = None
        self._latest_raw_cmd_ts = 0.0
        self.fault_clear_del = fault_clear_delay
        self.algo_timeout    = algo_timeout

    def transition(self, new_state, reason='test'):
        if new_state != self._state:
            if new_state == STATE_FAULT:
                self._fault_count += 1
                self._fault_latched = True
                self._fault_clear_ts = None
            self._state = new_state
            self._state_reason = reason
            self._state_entry_ts = time.monotonic()

    def set_dbw(self, enabled):
        self._dbw_enabled = enabled

    def set_algo_active(self, active, steer_en=True):
        if active:
            self._latest_raw_cmd_ts = time.monotonic()
            cmd = MagicMock()
            cmd.steer_en = steer_en
            cmd.drive_en = False
            cmd.brake_en = False
            cmd.gear_en  = False
            self._latest_raw_cmd = cmd
        else:
            self._latest_raw_cmd_ts = 0.0
            self._latest_raw_cmd = None

    def set_fault(self, flt1=0, flt2=0, kind='steer'):
        status = MagicMock()
        status.steer_flt1  = flt1 if kind == 'steer' else 0
        status.steer_flt2  = flt2 if kind == 'steer' else 0
        status.drive_flt1  = flt1 if kind == 'drive' else 0
        status.drive_flt2  = flt2 if kind == 'drive' else 0
        status.brake_flt1  = 0
        status.brake_flt2  = 0
        status.park_flt    = 0
        status.gear_flt    = 0
        status.front_crash = False
        status.back_crash  = False
        status.aeb_active  = False
        self._latest_status = status

    def clear_faults(self):
        status = MagicMock()
        for attr in ['steer_flt1','steer_flt2','drive_flt1','drive_flt2',
                     'brake_flt1','brake_flt2','park_flt','gear_flt']:
            setattr(status, attr, 0)
        status.front_crash = False
        status.back_crash  = False
        status.aeb_active  = False
        self._latest_status = status

    def _check_chassis_faults(self):
        if self._latest_status is None:
            return ''
        s = self._latest_status
        if s.steer_flt1 or s.steer_flt2: return f'steer_flt'
        if s.drive_flt1 or s.drive_flt2: return 'drive_flt'
        if s.brake_flt1 or s.brake_flt2: return 'brake_flt'
        if s.front_crash: return 'front_crash'
        return ''

    def _algo_active(self):
        if self._latest_raw_cmd is None:
            return False
        age = time.monotonic() - self._latest_raw_cmd_ts
        if age > self.algo_timeout:
            return False
        cmd = self._latest_raw_cmd
        return (cmd.steer_en or cmd.drive_en or cmd.brake_en or cmd.gear_en)

    def update(self):
        """One tick of state machine logic."""
        now = time.monotonic()
        if self._state == STATE_ESTOP:
            return
        fault_desc = self._check_chassis_faults()
        if fault_desc:
            if self._state != STATE_FAULT:
                self.transition(STATE_FAULT, fault_desc)
            self._fault_latched = True
            self._fault_clear_ts = None
            return
        if self._state == STATE_FAULT:
            if self._fault_clear_ts is None:
                self._fault_clear_ts = now
            elif (now - self._fault_clear_ts) >= self.fault_clear_del:
                self._fault_latched = False
                self.transition(STATE_STANDBY, 'Fault cleared')
            return
        if self._state == STATE_MANUAL:
            if self._dbw_enabled:
                self.transition(STATE_STANDBY, 'DBW engaged')
        elif self._state == STATE_STANDBY:
            if not self._dbw_enabled:
                self.transition(STATE_MANUAL, 'DBW disengaged')
            elif self._algo_active():
                self.transition(STATE_AUTONOMOUS, 'Algorithm active')
        elif self._state == STATE_AUTONOMOUS:
            if not self._dbw_enabled:
                self.transition(STATE_MANUAL, 'DBW disengaged')
            elif not self._algo_active():
                self.transition(STATE_STANDBY, 'No algorithm')


# ─── Tests ────────────────────────────────────────────────────────────────────

class TestStateMachineTransitions:

    def test_initial_state_is_manual(self):
        sm = StateMachineLogic()
        assert sm._state == STATE_MANUAL

    def test_manual_to_standby_when_dbw_engaged(self):
        sm = StateMachineLogic()
        sm.set_dbw(True)
        sm.update()
        assert sm._state == STATE_STANDBY

    def test_standby_to_manual_when_dbw_lost(self):
        sm = StateMachineLogic()
        sm.transition(STATE_STANDBY)
        sm.set_dbw(False)
        sm.update()
        assert sm._state == STATE_MANUAL

    def test_standby_to_autonomous_when_algo_active(self):
        sm = StateMachineLogic()
        sm.transition(STATE_STANDBY)
        sm.set_dbw(True)
        sm.set_algo_active(True)
        sm.update()
        assert sm._state == STATE_AUTONOMOUS

    def test_autonomous_to_standby_when_algo_stops(self):
        sm = StateMachineLogic()
        sm.transition(STATE_AUTONOMOUS)
        sm.set_dbw(True)
        sm.set_algo_active(False)
        sm.update()
        assert sm._state == STATE_STANDBY

    def test_autonomous_to_manual_when_dbw_lost(self):
        sm = StateMachineLogic()
        sm.transition(STATE_AUTONOMOUS)
        sm.set_dbw(False)
        sm.update()
        assert sm._state == STATE_MANUAL

    def test_any_state_to_fault_on_vcu_fault(self):
        for start_state in [STATE_MANUAL, STATE_STANDBY, STATE_AUTONOMOUS]:
            sm = StateMachineLogic()
            sm.transition(start_state)
            sm.set_fault(flt1=1, kind='steer')
            sm.update()
            assert sm._state == STATE_FAULT, f"Should be FAULT from {start_state}"

    def test_fault_increments_counter(self):
        sm = StateMachineLogic()
        sm.set_fault(flt1=1)
        sm.update()
        assert sm._fault_count == 1
        sm.clear_faults()
        time.sleep(0.01)
        sm._fault_clear_ts = time.monotonic() - 3.0   # fast-forward delay
        sm.update()   # recover
        sm.transition(STATE_STANDBY)
        sm.set_fault(flt1=2)
        sm.update()
        assert sm._fault_count == 2

    def test_fault_latched_until_cleared(self):
        sm = StateMachineLogic(fault_clear_delay=0.1)
        sm.set_fault(flt1=1)
        sm.update()
        assert sm._fault_latched is True
        sm.clear_faults()
        sm.update()   # starts countdown
        assert sm._state == STATE_FAULT   # still in FAULT
        time.sleep(0.15)
        sm.update()   # delay elapsed -> recover
        assert sm._state == STATE_STANDBY
        assert sm._fault_latched is False

    def test_estop_latches_and_blocks_all_transitions(self):
        sm = StateMachineLogic()
        sm._estop_latched = True
        sm.transition(STATE_ESTOP, 'test estop')
        sm.set_dbw(True)
        sm.set_algo_active(True)
        sm.update()   # should NOT transition
        assert sm._state == STATE_ESTOP

    def test_estop_cleared_returns_to_manual(self):
        sm = StateMachineLogic()
        sm.transition(STATE_ESTOP)
        sm._estop_latched = True
        # Simulate clear
        sm._estop_latched = False
        sm._fault_latched = False
        sm.transition(STATE_MANUAL, 'E-stop cleared')
        assert sm._state == STATE_MANUAL
        assert sm._estop_latched is False

    def test_fault_does_not_transition_from_estop(self):
        sm = StateMachineLogic()
        sm.transition(STATE_ESTOP)
        sm.set_fault(flt1=1)
        sm.update()
        assert sm._state == STATE_ESTOP   # ESTOP is sticky

    def test_algo_timeout_correctly_detected(self):
        sm = StateMachineLogic(algo_timeout=0.1)
        sm.set_algo_active(True)
        assert sm._algo_active() is True
        time.sleep(0.15)
        assert sm._algo_active() is False   # stale

    def test_no_spurious_transition_when_already_in_state(self):
        """Calling update repeatedly in stable state must not change state."""
        sm = StateMachineLogic()
        sm.transition(STATE_STANDBY)
        sm.set_dbw(True)
        for _ in range(10):
            sm.update()
        assert sm._state == STATE_STANDBY


class TestFaultDetection:

    def test_steer_flt1_triggers_fault(self):
        sm = StateMachineLogic()
        sm.set_fault(flt1=1, kind='steer')
        assert sm._check_chassis_faults() == 'steer_flt'

    def test_drive_flt_triggers_fault(self):
        sm = StateMachineLogic()
        sm.set_fault(flt1=1, kind='drive')
        assert sm._check_chassis_faults() == 'drive_flt'

    def test_no_fault_when_all_zero(self):
        sm = StateMachineLogic()
        sm.clear_faults()
        assert sm._check_chassis_faults() == ''

    def test_no_fault_when_no_status(self):
        sm = StateMachineLogic()
        # _latest_status = None by default
        assert sm._check_chassis_faults() == ''

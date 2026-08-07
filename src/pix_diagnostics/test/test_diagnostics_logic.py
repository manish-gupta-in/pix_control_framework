"""
Unit tests for Diagnostics Framework
Tests diagnostic status level logic for each channel.
"""
import time
import pytest
from unittest.mock import MagicMock


def make_status_msg(voltage=52.0, soc=85.0,
                    steer_flt1=0, steer_flt2=0,
                    drive_flt1=0, drive_flt2=0,
                    brake_flt1=0, brake_flt2=0,
                    park_flt=0, gear_flt=0,
                    front_crash=False, back_crash=False,
                    aeb_active=False,
                    steer_en_state=3, drive_en_state=3, brake_en_state=3,
                    vehicle_speed=0.0):
    m = MagicMock()
    m.battery_voltage = voltage
    m.battery_soc     = soc
    m.battery_current = 5.0
    m.steer_flt1  = steer_flt1
    m.steer_flt2  = steer_flt2
    m.drive_flt1  = drive_flt1
    m.drive_flt2  = drive_flt2
    m.brake_flt1  = brake_flt1
    m.brake_flt2  = brake_flt2
    m.park_flt    = park_flt
    m.gear_flt    = gear_flt
    m.front_crash = front_crash
    m.back_crash  = back_crash
    m.aeb_active  = aeb_active
    m.steer_en_state  = steer_en_state
    m.drive_en_state  = drive_en_state
    m.brake_en_state  = brake_en_state
    m.vehicle_speed   = vehicle_speed
    return m


# Inline the diagnostic logic (mirrors diagnostics_node.py)
DIAG_OK    = 0
DIAG_WARN  = 1
DIAG_ERROR = 2


def check_can_rx(status_ts, now, timeout=0.5):
    age = now - status_ts if status_ts > 0 else 9999.0
    if status_ts == 0:     return DIAG_ERROR
    if age > timeout:      return DIAG_WARN
    return DIAG_OK


def check_vcu_faults(status):
    if status is None:
        return DIAG_WARN
    if (status.steer_flt1 or status.steer_flt2 or
        status.drive_flt1 or status.drive_flt2 or
        status.brake_flt1 or status.brake_flt2 or
        status.park_flt or status.gear_flt or
        status.front_crash or status.back_crash):
        return DIAG_ERROR
    return DIAG_OK


def check_battery(status, warn_v=46.0, error_v=42.0, warn_soc=20.0, error_soc=10.0):
    if status is None:
        return DIAG_WARN
    if status.battery_voltage < error_v or status.battery_soc < error_soc:
        return DIAG_ERROR
    if status.battery_voltage < warn_v or status.battery_soc < warn_soc:
        return DIAG_WARN
    return DIAG_OK


def check_watchdog(raw_cmd_ts, now, timeout=1.0):
    if raw_cmd_ts == 0:
        return DIAG_WARN
    age = now - raw_cmd_ts
    if age > timeout:
        return DIAG_WARN
    return DIAG_OK


# ─── Tests ────────────────────────────────────────────────────────────────────

class TestCANRxDiagnostics:

    def test_no_data_is_error(self):
        assert check_can_rx(0, time.monotonic()) == DIAG_ERROR

    def test_fresh_data_is_ok(self):
        now = time.monotonic()
        assert check_can_rx(now - 0.1, now) == DIAG_OK

    def test_stale_data_is_warn(self):
        now = time.monotonic()
        assert check_can_rx(now - 1.0, now, timeout=0.5) == DIAG_WARN

    def test_data_slightly_past_timeout_is_warn(self):
        """age > timeout (strict) so we use 0.501 to guarantee warn."""
        now = time.monotonic()
        assert check_can_rx(now - 0.501, now, timeout=0.5) == DIAG_WARN

    def test_data_just_under_timeout_is_ok(self):
        now = time.monotonic()
        assert check_can_rx(now - 0.49, now, timeout=0.5) == DIAG_OK


class TestVCUFaultDiagnostics:

    def test_no_faults_is_ok(self):
        status = make_status_msg()
        assert check_vcu_faults(status) == DIAG_OK

    def test_steer_flt1_is_error(self):
        status = make_status_msg(steer_flt1=1)
        assert check_vcu_faults(status) == DIAG_ERROR

    def test_steer_flt2_is_error(self):
        status = make_status_msg(steer_flt2=1)
        assert check_vcu_faults(status) == DIAG_ERROR

    def test_drive_flt_is_error(self):
        status = make_status_msg(drive_flt1=1)
        assert check_vcu_faults(status) == DIAG_ERROR

    def test_brake_flt_is_error(self):
        status = make_status_msg(brake_flt1=1)
        assert check_vcu_faults(status) == DIAG_ERROR

    def test_front_crash_is_error(self):
        status = make_status_msg(front_crash=True)
        assert check_vcu_faults(status) == DIAG_ERROR

    def test_back_crash_is_error(self):
        status = make_status_msg(back_crash=True)
        assert check_vcu_faults(status) == DIAG_ERROR

    def test_none_status_is_warn(self):
        assert check_vcu_faults(None) == DIAG_WARN

    def test_aeb_active_does_not_cause_error(self):
        """AEB is a safety feature, not a fault."""
        status = make_status_msg(aeb_active=True)
        assert check_vcu_faults(status) == DIAG_OK


class TestBatteryDiagnostics:

    def test_good_battery_is_ok(self):
        status = make_status_msg(voltage=52.0, soc=85.0)
        assert check_battery(status) == DIAG_OK

    def test_low_voltage_warn(self):
        status = make_status_msg(voltage=45.0, soc=50.0)
        assert check_battery(status) == DIAG_WARN

    def test_critical_voltage_error(self):
        status = make_status_msg(voltage=40.0, soc=50.0)
        assert check_battery(status) == DIAG_ERROR

    def test_low_soc_warn(self):
        status = make_status_msg(voltage=50.0, soc=15.0)
        assert check_battery(status) == DIAG_WARN

    def test_critical_soc_error(self):
        status = make_status_msg(voltage=50.0, soc=5.0)
        assert check_battery(status) == DIAG_ERROR

    def test_none_status_is_warn(self):
        assert check_battery(None) == DIAG_WARN

    def test_borderline_warn_voltage(self):
        status = make_status_msg(voltage=46.0, soc=85.0)
        # Exactly at warn threshold — below 46 triggers warn
        assert check_battery(status) == DIAG_OK

    def test_just_below_warn_voltage(self):
        status = make_status_msg(voltage=45.9, soc=85.0)
        assert check_battery(status) == DIAG_WARN


class TestWatchdogDiagnostics:

    def test_no_algo_cmd_is_warn(self):
        assert check_watchdog(0, time.monotonic()) == DIAG_WARN

    def test_fresh_cmd_is_ok(self):
        now = time.monotonic()
        assert check_watchdog(now - 0.1, now, timeout=1.0) == DIAG_OK

    def test_stale_cmd_is_warn(self):
        now = time.monotonic()
        assert check_watchdog(now - 2.0, now, timeout=1.0) == DIAG_WARN

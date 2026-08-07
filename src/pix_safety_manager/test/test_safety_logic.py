#!/usr/bin/env python3
"""
Unit tests for pix_safety_manager logic.

Tests safety envelope validation, steering rate limiting, and E-stop logic
WITHOUT requiring a running ROS2 system (pure Python logic tests).

Run with:
    cd pix_control_framework
    python3 -m pytest src/pix_safety_manager/test/test_safety_logic.py -v
"""
import sys
import math
import pytest

# ---------------------------------------------------------------------------
# Pure-Python re-implementations of the safety logic (mirror of node logic)
# so we can test them without ROS2 context.
# ---------------------------------------------------------------------------

class SafetyEnvelope:
    """Mirrors the validation logic from PixSafetyManagerNode."""
    def __init__(self,
                 max_steer_angle=350.0,
                 max_steer_rate=150.0,
                 max_speed=5.0,
                 max_accel=2.0):
        self.max_steer_angle = max_steer_angle
        self.max_steer_rate  = max_steer_rate
        self.max_speed       = max_speed
        self.max_accel       = max_accel
        self._last_steer     = 0.0
        self._last_steer_t   = None

    def validate_steer(self, steer_target, now):
        """Clamp angle and apply rate limiting."""
        # Clamp to max angle
        clamped = max(-self.max_steer_angle, min(self.max_steer_angle, steer_target))
        # Apply rate limit
        if self._last_steer_t is not None:
            dt = now - self._last_steer_t
            if dt > 0.001:
                max_change = self.max_steer_rate * dt
                diff = clamped - self._last_steer
                if abs(diff) > max_change:
                    clamped = self._last_steer + (max_change if diff > 0 else -max_change)
        self._last_steer   = clamped
        self._last_steer_t = now
        return clamped

    def validate_speed(self, speed):
        return max(0.0, min(self.max_speed, speed))

    def validate_accel(self, accel):
        return max(0.0, min(self.max_accel, accel))

    def validate_brake(self, brake):
        return max(0.0, min(100.0, brake))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSteeringAngleClamping:
    def test_within_limit_passes_through(self):
        env = SafetyEnvelope(max_steer_angle=350.0)
        assert env.validate_steer(100.0, 0.0) == pytest.approx(100.0)

    def test_positive_over_limit_clamped(self):
        env = SafetyEnvelope(max_steer_angle=350.0)
        result = env.validate_steer(450.0, 0.0)
        assert result == pytest.approx(350.0)

    def test_negative_over_limit_clamped(self):
        env = SafetyEnvelope(max_steer_angle=350.0)
        result = env.validate_steer(-500.0, 0.0)
        assert result == pytest.approx(-350.0)

    def test_zero_passes(self):
        env = SafetyEnvelope(max_steer_angle=350.0)
        assert env.validate_steer(0.0, 0.0) == pytest.approx(0.0)

    def test_exact_limit_passes(self):
        env = SafetyEnvelope(max_steer_angle=350.0)
        assert env.validate_steer(350.0, 0.0) == pytest.approx(350.0)


class TestSteeringRateLimiting:
    def test_small_step_passes_through(self):
        """A step smaller than max_rate * dt is not limited."""
        env = SafetyEnvelope(max_steer_rate=150.0)
        env.validate_steer(0.0, 0.0)        # seed initial position
        # After 1.0s, max change = 150 deg. Step of 100 deg is within limit.
        result = env.validate_steer(100.0, 1.0)
        assert result == pytest.approx(100.0)

    def test_large_step_is_rate_limited(self):
        """A step larger than max_rate * dt is rate-limited."""
        env = SafetyEnvelope(max_steer_rate=150.0)
        env.validate_steer(0.0, 0.0)        # seed at t=0
        # dt=0.02s (50Hz tick), max_change = 150 * 0.02 = 3.0 deg
        result = env.validate_steer(100.0, 0.02)
        assert result == pytest.approx(3.0, abs=0.01)

    def test_rate_limit_negative_direction(self):
        """Rate limiting works in the negative direction too."""
        env = SafetyEnvelope(max_steer_rate=150.0)
        env.validate_steer(0.0, 0.0)
        result = env.validate_steer(-100.0, 0.02)
        assert result == pytest.approx(-3.0, abs=0.01)

    def test_gradual_increase_reaches_target(self):
        """Multiple ticks eventually reach the commanded angle."""
        env = SafetyEnvelope(max_steer_rate=150.0)
        steer = 0.0
        t = 0.0
        steer = env.validate_steer(steer, t)
        for i in range(1, 100):
            t += 0.02
            steer = env.validate_steer(100.0, t)
        assert steer == pytest.approx(100.0, abs=1.0)


class TestSpeedValidation:
    def test_within_limit(self):
        env = SafetyEnvelope(max_speed=5.0)
        assert env.validate_speed(3.0) == pytest.approx(3.0)

    def test_over_limit_clamped(self):
        env = SafetyEnvelope(max_speed=5.0)
        assert env.validate_speed(10.0) == pytest.approx(5.0)

    def test_negative_clamped_to_zero(self):
        env = SafetyEnvelope(max_speed=5.0)
        assert env.validate_speed(-1.0) == pytest.approx(0.0)

    def test_zero_passes(self):
        env = SafetyEnvelope(max_speed=5.0)
        assert env.validate_speed(0.0) == pytest.approx(0.0)


class TestAccelValidation:
    def test_within_limit(self):
        env = SafetyEnvelope(max_accel=2.0)
        assert env.validate_accel(1.5) == pytest.approx(1.5)

    def test_over_limit_clamped(self):
        env = SafetyEnvelope(max_accel=2.0)
        assert env.validate_accel(5.0) == pytest.approx(2.0)

    def test_negative_clamped_to_zero(self):
        env = SafetyEnvelope(max_accel=2.0)
        assert env.validate_accel(-1.0) == pytest.approx(0.0)


class TestBrakeValidation:
    def test_within_range(self):
        env = SafetyEnvelope()
        assert env.validate_brake(50.0) == pytest.approx(50.0)

    def test_over_100_clamped(self):
        env = SafetyEnvelope()
        assert env.validate_brake(150.0) == pytest.approx(100.0)

    def test_negative_clamped_to_zero(self):
        env = SafetyEnvelope()
        assert env.validate_brake(-10.0) == pytest.approx(0.0)

    def test_full_brake(self):
        env = SafetyEnvelope()
        assert env.validate_brake(100.0) == pytest.approx(100.0)


class TestEStopConditions:
    """Test that E-stop conditions are correctly detected."""

    def _check_faults(self, status_dict):
        """Mirror of check_chassis_faults logic."""
        if status_dict.get('steer_flt1') or status_dict.get('steer_flt2'):
            return True, "steer_fault"
        if status_dict.get('drive_flt1') or status_dict.get('drive_flt2'):
            return True, "drive_fault"
        if status_dict.get('brake_flt1') or status_dict.get('brake_flt2'):
            return True, "brake_fault"
        if status_dict.get('park_flt'):
            return True, "park_fault"
        if status_dict.get('gear_flt'):
            return True, "gear_fault"
        if status_dict.get('front_crash'):
            return True, "front_crash"
        if status_dict.get('back_crash'):
            return True, "back_crash"
        return False, None

    def test_no_faults_no_estop(self):
        ok, reason = self._check_faults({})
        assert ok is False

    def test_steer_fault1_triggers_estop(self):
        ok, reason = self._check_faults({'steer_flt1': 1})
        assert ok is True
        assert reason == "steer_fault"

    def test_drive_fault2_triggers_estop(self):
        ok, reason = self._check_faults({'drive_flt2': 2})
        assert ok is True
        assert reason == "drive_fault"

    def test_front_crash_triggers_estop(self):
        ok, reason = self._check_faults({'front_crash': True})
        assert ok is True
        assert reason == "front_crash"

    def test_back_crash_triggers_estop(self):
        ok, reason = self._check_faults({'back_crash': True})
        assert ok is True
        assert reason == "back_crash"

    def test_brake_fault_triggers_estop(self):
        ok, reason = self._check_faults({'brake_flt1': 3})
        assert ok is True
        assert reason == "brake_fault"

    def test_gear_fault_triggers_estop(self):
        ok, reason = self._check_faults({'gear_flt': 1})
        assert ok is True
        assert reason == "gear_fault"

    def test_park_fault_triggers_estop(self):
        ok, reason = self._check_faults({'park_flt': 1})
        assert ok is True
        assert reason == "park_fault"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

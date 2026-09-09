#!/usr/bin/env python3
"""
test_safety_and_arbitration.py — Unit tests for pure safety clamp logic
and arbitration priority selection.

These tests are independent of rclpy — they run fast without a ROS2 environment.

Run with:
    cd pix_control_framework
    python3 -m pytest src/pix_safety_manager/test/test_safety_and_arbitration.py -v
"""
import sys
import os
import pytest

# Add paths so modules can be imported without ROS install
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pix_safety_manager.safety_clamp_logic import SafetyClampLogic


# ═══════════════════════════════════════════════════════════════════════════
# SafetyClampLogic Tests
# ═══════════════════════════════════════════════════════════════════════════

class TestSteeringClamp:
    """Test steering angle clamping."""

    def setup_method(self):
        self.logic = SafetyClampLogic(max_steer_angle=350.0)

    def test_within_range(self):
        assert self.logic.clamp_steering(200.0) == 200.0

    def test_positive_over(self):
        assert self.logic.clamp_steering(500.0) == 350.0

    def test_negative_over(self):
        assert self.logic.clamp_steering(-500.0) == -350.0

    def test_zero(self):
        assert self.logic.clamp_steering(0.0) == 0.0

    def test_at_boundary(self):
        assert self.logic.clamp_steering(350.0) == 350.0
        assert self.logic.clamp_steering(-350.0) == -350.0


class TestSpeedClamp:
    """Test speed clamping."""

    def setup_method(self):
        self.logic = SafetyClampLogic(max_speed=5.0)

    def test_within_range(self):
        assert self.logic.clamp_speed(3.0) == 3.0

    def test_over_max(self):
        assert self.logic.clamp_speed(10.0) == 5.0

    def test_negative_clamped_to_zero(self):
        assert self.logic.clamp_speed(-1.0) == 0.0


class TestAccelClamp:
    """Test acceleration clamping."""

    def setup_method(self):
        self.logic = SafetyClampLogic(max_accel=2.0)

    def test_within_range(self):
        assert self.logic.clamp_accel(1.5) == 1.5

    def test_over_max(self):
        assert self.logic.clamp_accel(5.0) == 2.0

    def test_negative_clamped_to_zero(self):
        assert self.logic.clamp_accel(-1.0) == 0.0


class TestBrakeClamp:
    """Test brake clamping."""

    def setup_method(self):
        self.logic = SafetyClampLogic(max_brake=100.0)

    def test_within_range(self):
        assert self.logic.clamp_brake(50.0) == 50.0

    def test_over_100(self):
        assert self.logic.clamp_brake(150.0) == 100.0

    def test_negative(self):
        assert self.logic.clamp_brake(-10.0) == 0.0


class TestSteerRateLimit:
    """Test steering rate limiting."""

    def setup_method(self):
        self.logic = SafetyClampLogic(max_steer_rate=150.0)

    def test_small_change_passes(self):
        # At 150 deg/s, in 0.02s, max change = 3 deg
        result = self.logic.rate_limit_steer(2.0, 0.0, 0.02)
        assert result == 2.0  # within limit

    def test_large_change_limited(self):
        # At 150 deg/s, in 0.02s, max change = 3 deg
        result = self.logic.rate_limit_steer(100.0, 0.0, 0.02)
        assert result == pytest.approx(3.0, abs=0.1)

    def test_negative_direction(self):
        result = self.logic.rate_limit_steer(-100.0, 0.0, 0.02)
        assert result == pytest.approx(-3.0, abs=0.1)

    def test_tiny_dt_passes_through(self):
        # dt <= 0.001 → no rate limiting (avoid division issues)
        result = self.logic.rate_limit_steer(100.0, 0.0, 0.0005)
        assert result == 100.0


class TestValidateCommand:
    """Test full command validation pipeline."""

    def setup_method(self):
        self.logic = SafetyClampLogic(
            max_steer_angle=350.0,
            max_steer_rate=150.0,
            max_speed=5.0,
            max_accel=2.0,
        )

    def test_all_within_limits(self):
        result = self.logic.validate_command(
            steer_target=100.0,
            steer_speed=100.0,
            speed_target=3.0,
            accel_target=1.0,
            brake_target=30.0,
            dt=1.0,  # large dt to avoid rate limiting
        )
        assert result['steer_target'] == pytest.approx(100.0, abs=0.1)
        assert result['speed_target'] == 3.0
        assert result['accel_target'] == 1.0
        assert result['brake_target'] == 30.0

    def test_all_over_limits(self):
        result = self.logic.validate_command(
            steer_target=500.0,
            steer_speed=300.0,
            speed_target=20.0,
            accel_target=10.0,
            brake_target=200.0,
            dt=10.0,  # very large dt to not hit rate limit
        )
        assert result['steer_target'] == 350.0  # clamped
        assert result['steer_speed'] == 250.0    # clamped
        assert result['speed_target'] == 5.0     # clamped
        assert result['accel_target'] == 2.0     # clamped
        assert result['brake_target'] == 100.0   # clamped


class TestEstopCommand:
    """Test E-stop safe fallback generation."""

    def test_estop_values(self):
        logic = SafetyClampLogic()
        cmd = logic.generate_estop_command()
        assert cmd['steer_target'] == 0.0
        assert cmd['speed_target'] == 0.0
        assert cmd['brake_target'] == 100.0
        assert cmd['brake_en'] is True
        assert cmd['drive_en'] is False
        assert cmd['emergency_stop'] is True


# ═══════════════════════════════════════════════════════════════════════════
# Arbitration Priority Logic Tests (pure logic, no rclpy)
# ═══════════════════════════════════════════════════════════════════════════

class ArbitrationLogic:
    """
    Pure-logic arbitration test helper.
    Mirrors the priority selection in command_arbitrator.py without ROS deps.
    """

    def __init__(self, priorities, timeout=0.4):
        self.priorities = priorities
        self.timeout = timeout
        self.storage = {name: {'msg': None, 'time': 0.0} for name in priorities}

    def update(self, source, msg, timestamp):
        self.storage[source]['msg'] = msg
        self.storage[source]['time'] = timestamp

    def select(self, now):
        for name in self.priorities:
            data = self.storage[name]
            if data['msg'] is not None:
                if (now - data['time']) < self.timeout:
                    return name, data['msg']
        return None, None


class TestArbitrationPriority:
    """Test priority-based arbitration selection."""

    PRIORITIES = [
        'EMERGENCY_STOP',
        'COLLISION_AVOIDANCE',
        'HUMAN_AVOIDANCE',
        'LANE_FOLLOWING',
        'CRUISE_CONTROL',
    ]

    def setup_method(self):
        self.arb = ArbitrationLogic(self.PRIORITIES, timeout=0.4)

    def test_no_commands_returns_none(self):
        source, msg = self.arb.select(now=1.0)
        assert source is None
        assert msg is None

    def test_single_source(self):
        self.arb.update('CRUISE_CONTROL', {'speed': 1.0}, timestamp=1.0)
        source, msg = self.arb.select(now=1.1)
        assert source == 'CRUISE_CONTROL'
        assert msg['speed'] == 1.0

    def test_higher_priority_wins(self):
        self.arb.update('CRUISE_CONTROL', {'speed': 1.0}, timestamp=1.0)
        self.arb.update('COLLISION_AVOIDANCE', {'brake': 100.0}, timestamp=1.0)
        source, _ = self.arb.select(now=1.1)
        assert source == 'COLLISION_AVOIDANCE'

    def test_emergency_stop_highest(self):
        for name in self.PRIORITIES:
            self.arb.update(name, {'cmd': name}, timestamp=1.0)
        source, _ = self.arb.select(now=1.1)
        assert source == 'EMERGENCY_STOP'

    def test_timeout_falls_through(self):
        self.arb.update('COLLISION_AVOIDANCE', {'brake': 50.0}, timestamp=0.5)
        self.arb.update('CRUISE_CONTROL', {'speed': 2.0}, timestamp=1.0)
        # At t=1.2, COLLISION_AVOIDANCE is 0.7s old (> 0.4 timeout), CRUISE is 0.2s
        source, _ = self.arb.select(now=1.2)
        assert source == 'CRUISE_CONTROL'

    def test_all_timed_out(self):
        self.arb.update('CRUISE_CONTROL', {'speed': 1.0}, timestamp=0.0)
        source, _ = self.arb.select(now=1.0)
        assert source is None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

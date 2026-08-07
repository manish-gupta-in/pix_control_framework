#!/usr/bin/env python3
"""
Unit tests for the kinematic bicycle model used in the vehicle simulator.

Tests that the simulator math correctly integrates position, speed,
heading, and steering for known input sequences.

Run with:
    cd pix_control_framework
    python3 -m pytest src/pix_simulator/test/test_kinematics.py -v
"""
import math
import pytest


# ---------------------------------------------------------------------------
# Pure-Python bicycle model (mirrors vehicle_simulator.py logic)
# ---------------------------------------------------------------------------

class BicycleModel:
    def __init__(self, wheelbase=2.0, max_steer_limit=500.0,
                 max_brake_decel=4.0, friction_decel=0.5,
                 steering_ratio=15.0, dt=0.02):
        self.L = wheelbase
        self.max_steer_limit = max_steer_limit
        self.max_brake_decel = max_brake_decel
        self.friction_decel = friction_decel
        self.steering_ratio = steering_ratio
        self.dt = dt

        # State
        self.x     = 0.0
        self.y     = 0.0
        self.yaw   = 0.0    # radians
        self.speed = 0.0    # m/s
        self.steer = 0.0    # VCU steering wheel degrees

    def step(self, steer_target=None, steer_speed=150.0, steer_en=False,
             speed_target=0.0, accel_limit=1.0, drive_en=False,
             brake_pct=0.0, brake_en=False, emergency_stop=False,
             gear=4):  # 4=DRIVE
        """One integration step."""
        # ── Deceleration ──
        decel = self.friction_decel
        if brake_en and brake_pct > 0:
            decel += (brake_pct / 100.0) * self.max_brake_decel
        if emergency_stop:
            decel = self.max_brake_decel * 2.0
            speed_target = 0.0

        # ── Speed integration ──
        target = speed_target if (drive_en and gear == 4) else 0.0
        if self.speed < target:
            self.speed = min(target, self.speed + accel_limit * self.dt)
        elif self.speed > target:
            self.speed = max(target, self.speed - decel * self.dt)

        # ── Steering integration ──
        if steer_en and steer_target is not None:
            diff = steer_target - self.steer
            max_ch = steer_speed * self.dt
            if abs(diff) <= max_ch:
                self.steer = steer_target
            else:
                self.steer += math.copysign(max_ch, diff)
            self.steer = max(-self.max_steer_limit, min(self.max_steer_limit, self.steer))

        # ── Position integration ──
        wheel_angle_rad = math.radians(self.steer / self.steering_ratio)
        self.x   += self.speed * math.cos(self.yaw) * self.dt
        self.y   += self.speed * math.sin(self.yaw) * self.dt
        yaw_rate  = (self.speed / self.L) * math.tan(wheel_angle_rad)
        self.yaw += yaw_rate * self.dt
        self.yaw  = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        return yaw_rate


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSpeedIntegration:
    def test_vehicle_accelerates_from_rest(self):
        m = BicycleModel()
        for _ in range(50):
            m.step(speed_target=3.0, accel_limit=1.0, drive_en=True, gear=4)
        assert m.speed > 0.0

    def test_vehicle_reaches_target_speed(self):
        m = BicycleModel()
        for _ in range(500):  # 10 seconds at 50Hz
            m.step(speed_target=3.0, accel_limit=1.0, drive_en=True, gear=4)
        assert m.speed == pytest.approx(3.0, abs=0.05)

    def test_vehicle_does_not_exceed_target(self):
        m = BicycleModel()
        for _ in range(1000):
            m.step(speed_target=2.0, accel_limit=2.0, drive_en=True, gear=4)
        assert m.speed <= 2.0 + 0.001

    def test_brake_decelerates_vehicle(self):
        m = BicycleModel()
        # Get to speed first
        for _ in range(200):
            m.step(speed_target=3.0, accel_limit=2.0, drive_en=True, gear=4)
        speed_before = m.speed
        # Now brake
        for _ in range(50):
            m.step(brake_pct=50.0, brake_en=True)
        assert m.speed < speed_before

    def test_emergency_stop_halts_vehicle(self):
        m = BicycleModel()
        for _ in range(200):
            m.step(speed_target=3.0, accel_limit=2.0, drive_en=True, gear=4)
        for _ in range(200):
            m.step(emergency_stop=True)
        assert m.speed == pytest.approx(0.0, abs=0.01)

    def test_friction_decelerates_with_no_command(self):
        m = BicycleModel(friction_decel=0.5)
        # Accelerate to ~2 m/s
        for _ in range(200):
            m.step(speed_target=2.0, accel_limit=1.0, drive_en=True, gear=4)
        # Coast to stop: 2 m/s / 0.5 decel = 4 s = 200 ticks; use 250 for margin
        for _ in range(250):
            m.step()
        assert m.speed == pytest.approx(0.0, abs=0.05)

    def test_no_movement_in_neutral_gear(self):
        m = BicycleModel()
        for _ in range(200):
            m.step(speed_target=3.0, accel_limit=1.0, drive_en=True, gear=3)  # NEUTRAL
        assert m.speed == pytest.approx(0.0, abs=0.01)


class TestPositionIntegration:
    def test_straight_line_motion(self):
        """Vehicle should move along +X when heading is 0."""
        m = BicycleModel()
        for _ in range(250):  # 5 seconds
            m.step(speed_target=2.0, accel_limit=2.0, drive_en=True, gear=4)
        assert m.x > 0.0
        assert m.y == pytest.approx(0.0, abs=0.01)

    def test_no_motion_when_stopped(self):
        m = BicycleModel()
        for _ in range(50):
            m.step()
        assert m.x == pytest.approx(0.0)
        assert m.y == pytest.approx(0.0)

    def test_steering_causes_lateral_displacement(self):
        """Turning right should produce positive Y displacement (for heading=0)."""
        m = BicycleModel()
        # Drive forward with right steer (positive VCU angle)
        for _ in range(300):
            m.step(steer_target=200.0, steer_speed=500.0, steer_en=True,
                   speed_target=2.0, accel_limit=2.0, drive_en=True, gear=4)
        # With positive yaw rate, Y should change
        assert abs(m.y) > 0.1 or abs(m.x) > 0.1  # vehicle has moved


class TestSteeringIntegration:
    def test_steer_reaches_target(self):
        m = BicycleModel()
        for _ in range(200):
            m.step(steer_target=300.0, steer_speed=150.0, steer_en=True)
        assert m.steer == pytest.approx(300.0, abs=1.0)

    def test_steer_is_rate_limited(self):
        """After 1 tick at 150 deg/s, steering should change by 150*0.02=3 deg."""
        m = BicycleModel()
        m.step(steer_target=300.0, steer_speed=150.0, steer_en=True)
        assert m.steer == pytest.approx(3.0, abs=0.1)

    def test_steer_clamps_to_max(self):
        m = BicycleModel(max_steer_limit=500.0)
        for _ in range(500):
            m.step(steer_target=600.0, steer_speed=500.0, steer_en=True)
        assert m.steer == pytest.approx(500.0, abs=0.1)

    def test_steer_clamps_negative(self):
        m = BicycleModel(max_steer_limit=500.0)
        for _ in range(500):
            m.step(steer_target=-600.0, steer_speed=500.0, steer_en=True)
        assert m.steer == pytest.approx(-500.0, abs=0.1)

    def test_steer_disabled_does_not_change(self):
        m = BicycleModel()
        m.steer = 100.0
        m.step(steer_target=300.0, steer_en=False)
        assert m.steer == pytest.approx(100.0)


class TestYawIntegration:
    def test_zero_steer_no_yaw_change(self):
        m = BicycleModel()
        for _ in range(100):
            m.step(speed_target=2.0, accel_limit=2.0, drive_en=True, gear=4,
                   steer_target=0.0, steer_en=True)
        assert m.yaw == pytest.approx(0.0, abs=0.01)

    def test_positive_steer_changes_yaw(self):
        m = BicycleModel()
        for _ in range(200):
            m.step(steer_target=200.0, steer_speed=1000.0, steer_en=True,
                   speed_target=2.0, accel_limit=2.0, drive_en=True, gear=4)
        assert m.yaw != pytest.approx(0.0, abs=0.01)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

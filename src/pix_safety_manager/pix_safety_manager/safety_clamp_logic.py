"""
safety_clamp_logic.py — Pure, dependency-free safety clamp/validate class.

This class is deliberately free of any ROS2 (rclpy) dependency so it can be
unit-tested exhaustively without a ROS2 environment.

If profiling later shows this is tight against the loop budget, this isolated
class is a five-minute port to C++ without touching ROS glue.
"""


class SafetyClampLogic:
    """
    Pure-logic safety clamping and validation.
    No ROS, no threading, no I/O — just math.
    """

    def __init__(
        self,
        max_steer_angle: float = 450.0,   # deg — Whale: 8.72 rad = 499.6° (we use 90%)
        max_steer_rate: float = 250.0,    # deg/s — Whale: 4.36 rad/s = 249.8°/s
        max_speed: float = 1.5,           # m/s — Whale prod: 5.0; test: 1.5
        max_accel: float = 1.5,           # m/s² — Whale sim: 7.0; conservative: 1.5
        max_brake: float = 100.0,
        max_steer_speed: float = 250.0,   # deg/s — matches max_steer_rate
        min_steer_speed: float = 1.0,
    ):
        self.max_steer_angle = max_steer_angle
        self.max_steer_rate = max_steer_rate
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.max_brake = max_brake
        self.max_steer_speed = max_steer_speed
        self.min_steer_speed = min_steer_speed

    def clamp_steering(self, angle: float) -> float:
        """Clamp steering angle to [-max_steer_angle, max_steer_angle]."""
        return max(-self.max_steer_angle, min(self.max_steer_angle, angle))

    def clamp_speed(self, speed: float) -> float:
        """Clamp speed to [0, max_speed]."""
        return max(0.0, min(self.max_speed, speed))

    def clamp_accel(self, accel: float) -> float:
        """Clamp acceleration to [0, max_accel]."""
        return max(0.0, min(self.max_accel, accel))

    def clamp_brake(self, brake: float) -> float:
        """Clamp brake to [0, max_brake]."""
        return max(0.0, min(self.max_brake, brake))

    def clamp_steer_speed(self, steer_speed: float) -> float:
        """Clamp steer speed to [min_steer_speed, max_steer_speed]."""
        return max(self.min_steer_speed, min(self.max_steer_speed, steer_speed))

    def rate_limit_steer(
        self, requested: float, previous: float, dt: float
    ) -> float:
        """
        Apply steering rate limiting.

        Args:
            requested: Desired steering angle (already clamped)
            previous: Previous commanded steering angle
            dt: Time delta in seconds

        Returns:
            Rate-limited steering angle
        """
        if dt <= 0.001:
            return requested

        max_change = self.max_steer_rate * dt
        diff = requested - previous
        if abs(diff) > max_change:
            direction = 1.0 if diff > 0 else -1.0
            return previous + direction * max_change
        return requested

    def validate_command(
        self,
        steer_target: float,
        steer_speed: float,
        speed_target: float,
        accel_target: float,
        brake_target: float,
        previous_steer: float = 0.0,
        dt: float = 0.02,
    ) -> dict:
        """
        Validate and clamp all command fields.

        Returns:
            dict with validated values for each field.
        """
        clamped_steer = self.clamp_steering(steer_target)
        rate_limited_steer = self.rate_limit_steer(clamped_steer, previous_steer, dt)

        return {
            'steer_target': rate_limited_steer,
            'steer_speed': self.clamp_steer_speed(steer_speed),
            'speed_target': self.clamp_speed(speed_target),
            'accel_target': self.clamp_accel(accel_target),
            'brake_target': self.clamp_brake(brake_target),
        }

    def generate_estop_command(self) -> dict:
        """
        Generate a safe emergency stop command.
        Returns a dict of safe-fallback values.
        """
        return {
            'steer_target': 0.0,
            'steer_speed': 150.0,
            'speed_target': 0.0,
            'accel_target': 0.0,
            'brake_target': 100.0,
            'drive_en': False,
            'brake_en': True,
            'steer_en': True,
            'emergency_stop': True,
        }

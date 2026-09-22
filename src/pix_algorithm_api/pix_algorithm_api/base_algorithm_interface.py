import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd, PixVehicleStatus
import threading

class BaseAlgorithmInterface(Node):
    """
    Base class for all modular control and perception algorithms in the PIX Control Framework.

    Provides:
      - Thread-safe access to the latest PixVehicleStatus via get_vehicle_status()
      - publish_control_cmd() — the single command publishing method

    There is exactly ONE message format: PixControlCmd (pix_vehicle_msgs).
    Units: steering in degrees (wheel angle), speed in m/s, brake 0–100%.

    Usage:
        class MyAlgorithm(BaseAlgorithmInterface):
            def __init__(self):
                super().__init__('my_algorithm', '/pix/commands/lane_following')
                self.timer = self.create_timer(0.02, self.control_loop)

            def control_loop(self):
                self.publish_control_cmd(
                    drive_en=True, speed_target=2.0, accel_target=1.0,
                    steer_en=True, steer_target=0.0, steer_speed=150.0,
                    gear_en=True, gear_target=4,   # 4 = DRIVE
                    park_en=True, park_target=0,   # 0 = RELEASE
                )
    """

    def __init__(self, node_name: str, algorithm_topic: str):
        super().__init__(node_name)

        # Publisher to the algorithm's registered priority topic
        self.cmd_pub = self.create_publisher(PixControlCmd, algorithm_topic, 10)

        # Thread-safe vehicle status
        self.vehicle_status = PixVehicleStatus()
        self.status_lock = threading.Lock()
        self.status_sub = self.create_subscription(
            PixVehicleStatus, '/pix/vehicle_status', self.status_callback, 10)

        self.get_logger().info(
            f"Algorithm node '{node_name}' initialized. Publishing to: {algorithm_topic}")

    # ─── Vehicle Status ─────────────────────────────────────────────────────────

    def status_callback(self, msg: PixVehicleStatus):
        with self.status_lock:
            self.vehicle_status = msg

    def get_vehicle_status(self) -> PixVehicleStatus:
        """Return the latest vehicle status snapshot (thread-safe)."""
        with self.status_lock:
            return self.vehicle_status

    # ─── Command Publishing ──────────────────────────────────────────────────────

    def publish_control_cmd(self,
                            steer_target: float = 0.0,
                            steer_speed: float  = 150.0,
                            steer_en: bool      = False,
                            speed_target: float = 0.0,
                            accel_target: float = 1.0,
                            drive_en: bool      = False,
                            brake_target: float = 0.0,
                            brake_en: bool      = False,
                            gear_target: int    = PixControlCmd.GEAR_TARGET_INVALID,
                            gear_en: bool       = False,
                            park_target: int    = PixControlCmd.PARK_TARGET_RELEASE,
                            park_en: bool       = False,
                            turn_light_ctrl: int = 0,
                            headlight_ctrl: bool = False,
                            emergency_stop: bool = False):
        """
        Build and publish a PixControlCmd to this algorithm's registered topic.

        Args:
            steer_target   : wheel angle in degrees (±500° max, enforced by safety manager)
            steer_speed    : steering rate in deg/s
            steer_en       : enable steering control
            speed_target   : desired speed in m/s
            accel_target   : acceleration in m/s²
            drive_en       : enable drive (throttle)
            brake_target   : brake pressure 0–100 %
            brake_en       : enable braking
            gear_target    : 1=Park, 2=Reverse, 3=Neutral, 4=Drive
            gear_en        : enable gear change
            park_target    : 0=Release, 1=Engage
            park_en        : enable park brake control
            turn_light_ctrl: 0=off, 1=left, 2=right, 3=hazard
            headlight_ctrl : True=on
            emergency_stop : True triggers unconditional full stop in safety manager
        """
        cmd = PixControlCmd()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.steer_target = float(steer_target)
        cmd.steer_speed  = float(steer_speed)
        cmd.steer_en     = bool(steer_en)

        cmd.speed_target = float(speed_target)
        cmd.accel_target = float(accel_target)
        cmd.drive_en     = bool(drive_en)
        cmd.brake_target = float(brake_target)
        cmd.brake_en     = bool(brake_en)

        cmd.gear_target = int(gear_target)
        cmd.gear_en     = bool(gear_en)
        cmd.park_target = int(park_target)
        cmd.park_en     = bool(park_en)

        cmd.turn_light_ctrl = int(turn_light_ctrl)
        cmd.headlight_ctrl  = bool(headlight_ctrl)
        cmd.emergency_stop  = bool(emergency_stop)

        self.cmd_pub.publish(cmd)

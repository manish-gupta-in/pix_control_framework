#!/usr/bin/env python3
"""
pix_can_tx.py — PIXKIT CAN TX Node
=====================================
Encodes /pix/control_cmd → 6 raw CAN frames at 50 Hz.

CHECKSUM NOTE (derived from working gear.py — verified on vehicle):
  Each message uses a different checksum algorithm:
    0x100 Throttle  : SUM of bytes 0-6 (= XOR for typical payloads, SUM is safe)
    0x101 Brake     : XOR of bytes 0-6  (confirmed by gear.py _xor + candump)
    0x102 Steer     : XOR of bytes 0-6  (confirmed by candump: F4=XOR not SUM)
    0x103 Gear      : SUM of bytes 0-6  (gear.py confirmed, SUM=04 vs XOR=02)
    0x104 Park      : SUM of bytes 0-6  (gear.py confirmed)
    0x105 VCU Mode  : SUM of bytes 0-6  (gear.py confirmed)

  CRITICAL: Using XOR for 0x103 (Gear) sends checksum 0x02 instead of SUM 0x04.
  The VCU silently rejects the frame → gear never changes → gear stays at 4 (DRIVE).
  This was the primary reason gear control did not work via the framework.

FRAME LAYOUT (from gear.py CAN matrix — verified on vehicle):
  0x100 Throttle:
    byte0 bit0    : Dirve_EnCtrl
    byte1 bits[7:0]: Steer_AngleSpeed (not throttle acc — throttle acc uses bytes 1-2)
    bytes 3-4     : Dirve_ThrottlePedalTarget  (16-bit, res=0.1%)
    byte7         : SUM checksum

  0x101 Brake:
    byte0 bit0    : Brake_EnCtrl
    byte0 bit1    : AEB_EnCtrl (always 0)
    bytes 1-2     : Brake_Dec  (10-bit, res=0.01 m/s²)
    bytes 3-4     : Brake_Pedal_Target (16-bit, res=0.1%)
    byte7         : XOR checksum

  0x102 Steering:
    byte0 bit0    : Steer_EnCtrl
    byte1         : Steer_AngleSpeed (0-250 deg/s)
    bytes 2-3     : Steer_AngleTarget (raw = angle_deg + 500, 0-1000)
    byte7         : XOR checksum

  0x103 Gear:
    byte0 bit0    : Gear_EnCtrl
    byte1 bits[2:0]: Gear_Target (1=PARK, 2=REVERSE, 3=NEUTRAL, 4=DRIVE)
    byte7         : SUM checksum  ← CRITICAL: SUM not XOR

  0x104 Park:
    byte0 bit0    : Park_EnCtrl
    byte1         : Park_Target (0=RELEASE, 1=ENGAGE)
    byte7         : SUM checksum

  0x105 VCU Mode:
    byte0 bit7    : Auto_Professional (always 1)
    byte1 bits[2:0]: Drive_ModeCtrl (1=SPEED_DRIVE)
    byte7         : SUM checksum
"""

import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd
import can


# ── Per-message checksum helpers ─────────────────────────────────────────────

def _xor7(d: bytearray) -> int:
    """XOR of bytes 0-6 (used by 0x101 Brake, 0x102 Steer)."""
    cs = 0
    for b in d[:7]:
        cs ^= b
    return cs & 0xFF


def _sum7(d: bytearray) -> int:
    """SUM of bytes 0-6 (used by 0x100 Throttle, 0x103 Gear, 0x104 Park, 0x105 Mode)."""
    return sum(d[:7]) & 0xFF


# ── Frame builders (byte-exact, verified against gear.py working implementation) ─

def build_vehicle_mode(headlight: bool = False, turnlight: int = 0) -> bytes:
    """
    0x105 Vehicle_Mode_Command — SUM checksum
    Auto_Professional=1 (bit7 of byte0), Drive_ModeCtrl=1 (byte1 bits[2:0])
    Keeps VCU in autonomous mode continuously.
    Also controls Headlight (bit 2 of byte 2) and Turnlights (bits 0-1 of byte 2).
    """
    d = bytearray(8)
    d[0] = 0x80  # Auto_Professional=1 (bit7)
    d[1] = 0x01  # Drive_ModeCtrl=1 (SPEED_DRIVE)
    d[2] = ((1 if headlight else 0) << 2) | (turnlight & 0x03)
    d[7] = _sum7(d)
    return bytes(d)


def build_brake(enable: bool, pedal_pct: float, decel_ms2: float = 0.0) -> bytes:
    """
    0x101 Brake_Command — XOR checksum
    pedal_pct: 0.0–100.0%
    decel_ms2: 0.0–10.0 m/s² (usually 0)
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    # Brake_Dec: 10-bit at start_bit=15 Motorola → upper 8 bits in byte1, lower 2 in byte2 bits[7:6]
    dec_raw = min(int(round(max(0.0, min(10.0, decel_ms2)) / 0.01)), 0x3FF)
    d[1] = (dec_raw >> 2) & 0xFF
    d[2] = (dec_raw & 0x3) << 6
    # Brake_Pedal_Target: 16-bit at start_bit=31 Motorola → bytes 3-4
    pedal_raw = min(int(round(max(0.0, min(100.0, pedal_pct)) / 0.1)), 0xFFFF)
    d[3] = (pedal_raw >> 8) & 0xFF
    d[4] = pedal_raw & 0xFF
    d[7] = _xor7(d)
    return bytes(d)


def build_steer(enable: bool, angle_deg: float, angle_speed: int = 100) -> bytes:
    """
    0x102 Steering_Command — XOR checksum
    angle_deg: −500 to +500° (raw = angle + 500, clamped 0–1000)
    angle_speed: 0–250 deg/s
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    d[1] = max(0, min(250, int(angle_speed)))
    raw = max(0, min(1000, int(round(angle_deg)) + 500))
    d[3] = (raw >> 8) & 0xFF  # Motorola start_bit=31 len=16 -> msb in byte 3
    d[4] = raw & 0xFF         # lsb in byte 4
    d[7] = _xor7(d)
    return bytes(d)


def build_gear(enable: bool, gear_id: int) -> bytes:
    """
    0x103 Gear_Command — SUM checksum  ← CRITICAL: must be SUM not XOR
    gear_id: 1=PARK, 2=REVERSE, 3=NEUTRAL, 4=DRIVE
    Gear_EnCtrl: byte0 bit0
    Gear_Target:  byte1 bits[2:0]
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    d[1] = int(gear_id) & 0x07
    d[7] = _sum7(d)    # SUM — not XOR. XOR=0x02 is rejected by VCU for gear NEUTRAL.
    return bytes(d)


def build_park(enable: bool, engage: bool) -> bytes:
    """
    0x104 Park_Command — SUM checksum
    engage: True=ENGAGE, False=RELEASE
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    d[1] = 0x01 if engage else 0x00
    d[7] = _sum7(d)
    return bytes(d)


def build_throttle(enable: bool, speed_ms: float, accel_ms2: float = 1.0) -> bytes:
    """
    0x100 Throttle_Command (Speed Drive Mode) — SUM checksum
    speed_ms: target speed in m/s (0.0–15.0)
    accel_ms2: target acceleration in m/s² (0.0–10.0)

    Speed encoding (DBC: Dirve_SpeedTarget, start_bit=47, len=12, scale=0.01, max=40.95 m/s):
      raw = speed_ms / 0.01 → placed in bytes 5-6 (upper nibble of byte6 + byte5)
      byte5 = (raw >> 4) & 0xFF
      byte6 upper nibble = raw & 0x0F  → byte6 |= (raw & 0x0F) << 4

    Accel encoding (DBC: Dirve_Acc, start_bit=15, len=10, scale=0.01):
      raw = accel_ms2 / 0.01 → 10-bit Motorola at start_bit=15
      byte1 upper 8 bits, byte2 bits[7:6] lower 2 bits
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00

    # Accel: 10-bit at start_bit=15 Motorola → byte1 = upper 8 bits, byte2[7:6] = lower 2 bits
    accel_clamped = max(0.0, min(10.0, accel_ms2))
    acc_raw = min(int(round(accel_clamped / 0.01)), 0x3FF)
    d[1] = (acc_raw >> 2) & 0xFF
    d[2] = (acc_raw & 0x3) << 6

    # Speed: 12-bit at start_bit=47 Motorola (len=12)
    # Bit 47 in Motorola: byte5 bits[7:0], byte6 bits[7:4]
    speed_clamped = max(0.0, min(15.0, speed_ms))  # software cap at 15 m/s
    spd_raw = min(int(round(speed_clamped / 0.01)), 0xFFF)
    d[5] = (spd_raw >> 4) & 0xFF        # upper 8 bits
    d[6] = (spd_raw & 0x0F) << 4        # lower 4 bits → upper nibble of byte6

    d[7] = _sum7(d)
    return bytes(d)


# ── CAN TX Node ───────────────────────────────────────────────────────────────

class PixCanTxNode(Node):
    def __init__(self):
        super().__init__('pix_can_tx')

        self.declare_parameter('can_interface', 'can4')
        self.declare_parameter('loop_rate', 50.0)
        self.declare_parameter('enable_can_tx', True)

        self.can_interface  = self.get_parameter('can_interface').value
        self.loop_rate      = self.get_parameter('loop_rate').value
        self.enable_can_tx  = self.get_parameter('enable_can_tx').value

        # CAN bus
        self.bus = None
        if self.enable_can_tx:
            try:
                self.bus = can.interface.Bus(
                    channel=self.can_interface, interface='socketcan')
                self.get_logger().info(
                    f'Opened SocketCAN interface: {self.can_interface}')
            except Exception as e:
                self.get_logger().error(
                    f"Failed to open '{self.can_interface}': {e}. TX mocked.")

        # Command state
        self.latest_cmd      = PixControlCmd()
        self.latest_cmd_time = 0.0
        self.cmd_timeout     = 0.5   # 500 ms watchdog

        self.cmd_sub = self.create_subscription(
            PixControlCmd, '/pix/control_cmd', self._cmd_cb, 10)

        self.timer = self.create_timer(1.0 / self.loop_rate, self._tx_tick)
        self.get_logger().info('CAN TX Node Initialized.')

    def _cmd_cb(self, msg: PixControlCmd):
        self.latest_cmd      = msg
        self.latest_cmd_time = self.get_clock().now().nanoseconds / 1e9

    def _tx_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Watchdog: no command for >500 ms → full brake safe fallback
        is_timeout = (self.latest_cmd_time > 0) and \
                     ((now - self.latest_cmd_time) > self.cmd_timeout)

        if is_timeout:
            self.get_logger().warning(
                'Watchdog timeout: no commands → safe fallback.',
                throttle_duration_sec=2.0)
            cmd = PixControlCmd()
            cmd.emergency_stop = True
            cmd.steer_en       = True
            cmd.steer_target   = 0.0
            cmd.steer_speed    = 150.0
            cmd.drive_en       = False
            cmd.speed_target   = 0.0
            cmd.brake_en       = True
            cmd.brake_target   = 50.0
            cmd.gear_en        = True
            cmd.gear_target    = PixControlCmd.GEAR_TARGET_NEUTRAL
            cmd.park_en        = False
        else:
            cmd = self.latest_cmd

        # Emergency stop override
        if cmd.emergency_stop:
            cmd.steer_target  = 0.0
            cmd.speed_target  = 0.0
            cmd.brake_en      = True
            cmd.brake_target  = 100.0
            cmd.drive_en      = False

        # ── Build all 6 frames ──────────────────────────────────────────────

        # 0x105 — always send AP=1 to keep VCU in autonomous mode, plus lights
        f105 = build_vehicle_mode(
            headlight = bool(cmd.headlight_ctrl),
            turnlight = int(cmd.turn_light_ctrl)
        )

        # 0x102 — Steering
        f102 = build_steer(
            enable      = bool(cmd.steer_en),
            angle_deg   = float(cmd.steer_target),
            angle_speed = max(1, min(250, int(cmd.steer_speed))) if cmd.steer_speed > 0 else 100
        )

        # 0x101 — Brake
        f101 = build_brake(
            enable    = bool(cmd.brake_en),
            pedal_pct = float(cmd.brake_target)
        )

        # 0x100 — Throttle (Speed Drive mode)
        f100 = build_throttle(
            enable    = bool(cmd.drive_en),
            speed_ms  = float(cmd.speed_target),
            accel_ms2 = float(cmd.accel_target) if cmd.accel_target > 0 else 1.0
        )

        # 0x103 — Gear (SUM checksum — critical!)
        f103 = build_gear(
            enable  = bool(cmd.gear_en),
            gear_id = int(cmd.gear_target)
        )

        # 0x104 — Park
        f104 = build_park(
            enable = bool(cmd.park_en),
            engage = (int(cmd.park_target) == PixControlCmd.PARK_TARGET_PARKING_TRIGGER)
        )

        # ── Send all frames ─────────────────────────────────────────────────
        for frame_id, payload in [
            (0x105, f105),
            (0x102, f102),
            (0x101, f101),
            (0x100, f100),
            (0x103, f103),
            (0x104, f104),
        ]:
            self._send(frame_id, payload)

    def _send(self, frame_id: int, payload: bytes):
        if self.bus is None:
            return
        try:
            self.bus.send(can.Message(
                arbitration_id=frame_id,
                data=payload,
                is_extended_id=False
            ))
        except OSError as e:
            if e.errno == 105:   # ENOBUFS
                self.get_logger().warning(
                    f'CAN TX buffer full, 0x{frame_id:03X} dropped (errno 105)',
                    throttle_duration_sec=2.0)
            else:
                self.get_logger().error(
                    f'Error sending 0x{frame_id:03X}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error sending 0x{frame_id:03X}: {e}')

    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
                self.get_logger().info('SocketCAN interface closed.')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PixCanTxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
PIXKIT Control Framework — Manual Actuator Test Script
=======================================================
Sends individual, time-limited commands to each VCU subsystem for safe,
step-by-step commissioning of the physical shuttle or simulation.

Usage
-----
  # In simulation (safe, no vehicle):
  ros2 launch launch/sim_framework.launch.py
  # Then in a SEPARATE terminal:
  python3 scripts/actuator_test.py --mode sim --test steering

  # On real vehicle (requires hw_framework running):
  ros2 launch launch/hw_framework.launch.py
  # Then:
  python3 scripts/actuator_test.py --mode hw --test brake

Available tests:
  steering   — Ramp steer right, hold, ramp back to center
  brake      — Apply brake, hold, release
  throttle   — Ramp speed up, hold, coast to stop
  gear       — Cycle Park → Neutral → Drive → Neutral → Park
  park       — Release and re-engage parking brake
  estop      — Publish emergency stop and verify
  full       — Run all tests in sequence (interactive, waits for confirm)

All tests publish to /pix/commands/cruise_control (lowest priority) so higher
priority safety systems (e.g. an operator's E-stop) always override.
"""

import argparse
import sys
import time
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd, PixVehicleStatus

# ── Test configuration (safe defaults) ─────────────────────────────────────
STEER_RIGHT_TARGET  =  200.0   # deg (positive = right)
STEER_LEFT_TARGET   = -200.0   # deg
STEER_CENTER        =    0.0
STEER_SPEED         =  100.0   # deg/s — slow and safe

BRAKE_LEVEL         =   30.0   # % (light braking for commission test)
THROTTLE_TARGET     =    1.5   # m/s (very slow creep)
THROTTLE_ACCEL      =    0.5   # m/s^2

HOLD_SECONDS        =    3.0   # How long to hold each command

TOPIC               = '/pix/commands/cruise_control'
STATUS_TOPIC        = '/pix/vehicle_status'


# ── Helper to build a PixControlCmd with safe defaults ──────────────────────

def make_cmd(node,
             steer_en=False, steer_target=0.0, steer_speed=100.0,
             drive_en=False, speed_target=0.0, accel_target=0.5,
             brake_en=False, brake_target=0.0,
             gear_en=False, gear_target=3,    # 3 = NEUTRAL
             park_en=False, park_target=0,    # 0 = RELEASE
             estop=False):
    cmd = PixControlCmd()
    cmd.header.stamp    = node.get_clock().now().to_msg()
    cmd.header.frame_id = 'base_link'
    cmd.steer_en        = steer_en
    cmd.steer_target    = float(steer_target)
    cmd.steer_speed     = float(steer_speed)
    cmd.drive_en        = drive_en
    cmd.speed_target    = float(speed_target)
    cmd.accel_target    = float(accel_target)
    cmd.brake_en        = brake_en
    cmd.brake_target    = float(brake_target)
    cmd.gear_en         = gear_en
    cmd.gear_target     = int(gear_target)
    cmd.park_en         = park_en
    cmd.park_target     = int(park_target)
    cmd.emergency_stop  = estop
    return cmd


# ── Base test node ──────────────────────────────────────────────────────────

class ActuatorTestNode(Node):
    def __init__(self, mode='sim'):
        super().__init__('pix_actuator_test')
        self.mode = mode
        self.pub  = self.create_publisher(PixControlCmd, TOPIC, 10)
        self.latest_status = None
        self.status_sub = self.create_subscription(
            PixVehicleStatus, STATUS_TOPIC, self._status_cb, 10
        )
        self.get_logger().info(
            f'Actuator Test Node ready. Mode={mode}. Topic={TOPIC}'
        )

    def _status_cb(self, msg):
        self.latest_status = msg

    def send(self, **kwargs):
        self.pub.publish(make_cmd(self, **kwargs))
        rclpy.spin_once(self, timeout_sec=0.0)

    def publish_for(self, duration_s, rate_hz=50, **kwargs):
        """Publish the same command at rate_hz for duration_s seconds."""
        period   = 1.0 / rate_hz
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self.send(**kwargs)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def center_and_stop(self, hold=2.0):
        """Safe neutral state: center steer, zero speed, 100% brake, NEUTRAL gear.
        100% brake during gear shift satisfies VCU interlock (same as gear.py).
        """
        self.get_logger().info('  → Returning to center / NEUTRAL / 100% brake …')
        self.publish_for(hold,
                         steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                         drive_en=False, speed_target=0.0,   accel_target=1.0,
                         brake_en=True,  brake_target=100.0,  # 100% like gear.py
                         gear_en=True,   gear_target=3,        # NEUTRAL
                         park_en=True,   park_target=0)

    def print_status(self):
        if self.latest_status is not None:
            s = self.latest_status
            # vehicle_mode: 0=MANUAL, 1=AUTO(gear works!), 2=EMERGENCY, 3=STANDBY(gear blocked)
            mode_names = {0: 'MANUAL', 1: 'AUTO✓', 2: 'EMERG', 3: 'STANDBY⚠'}
            mode_str = mode_names.get(s.vehicle_mode, f'?{s.vehicle_mode}')
            print(f'    VCU → speed={s.vehicle_speed:.2f}m/s | steer={s.steer_angle:.1f}° | '
                  f'gear={s.gear_actual} | park={s.park_actual} | '
                  f'brake={s.brake_pedal:.1f}% | mode={mode_str}({s.vehicle_mode})')
            if s.vehicle_mode == 3:
                print(f'    ⚠  VCU in STANDBY — gear changes BLOCKED by hardware interlock.')
                print(f'    ⚠  Set physical VCU remote/key to AUTO position to enable gear control.')
            elif s.vehicle_mode == 1:
                print(f'    ✓  VCU in AUTO mode — gear commands will be accepted.')
        else:
            print('    VCU Status → (no feedback received yet)')


# ── Individual test functions ────────────────────────────────────────────────

def test_steering(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ STEERING TEST ═══')
    log.info('Step 1/4: Enable steering, ramp to center …')
    node.publish_for(2.0, steer_en=True, steer_target=STEER_CENTER, steer_speed=STEER_SPEED)

    log.info(f'Step 2/4: Ramp RIGHT to {STEER_RIGHT_TARGET}° …')
    node.publish_for(HOLD_SECONDS, steer_en=True, steer_target=STEER_RIGHT_TARGET, steer_speed=STEER_SPEED)
    node.print_status()

    log.info(f'Step 3/4: Ramp LEFT to {STEER_LEFT_TARGET}° …')
    node.publish_for(HOLD_SECONDS, steer_en=True, steer_target=STEER_LEFT_TARGET, steer_speed=STEER_SPEED)
    node.print_status()

    log.info('Step 4/4: Return to center …')
    node.publish_for(HOLD_SECONDS, steer_en=True, steer_target=STEER_CENTER, steer_speed=STEER_SPEED)
    node.print_status()
    log.info('Steering test COMPLETE ✓')


def test_brake(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ BRAKE TEST ═══')
    log.info(f'Step 1/3: Apply {BRAKE_LEVEL}% brake …')
    node.publish_for(HOLD_SECONDS,
                     brake_en=True, brake_target=BRAKE_LEVEL,
                     gear_en=True,  gear_target=3)
    node.print_status()

    log.info('Step 2/3: Apply 0% brake (release) …')
    node.publish_for(HOLD_SECONDS,
                     brake_en=True, brake_target=0.0,
                     gear_en=True, gear_target=3)
    node.print_status()

    log.info('Step 3/3: Verify brake pedal is 0% …')
    rclpy.spin_once(node, timeout_sec=0.5)
    node.print_status()
    log.info('Brake test COMPLETE ✓')


def test_throttle(node: ActuatorTestNode, safe_speed: float = None):
    """
    Throttle test — mirrors gear.py mode_throttle (the proven sequence).

    Sequence (same as gear.py):
      Step 0: 100% brake + steer + throttle + park OFF → wait all EnState=Auto
      Step 1: Gear → DRIVE (3 s, still 100% brake)
      Step 2: Release brake to 0%
      Step 3: Ramp speed  0 → target m/s
      Step 4: Hold speed for HOLD_SECONDS
      Step 5: Speed → 0, 100% brake until stopped
      Step 6: Gear → NEUTRAL, park engage

    ⚠  VEHICLE MOVES. Clear 15 m+ path before running.
    """
    log = node.get_logger()
    target = safe_speed if safe_speed is not None else THROTTLE_TARGET

    log.info('═══ THROTTLE / SPEED TEST ═══')
    log.info(f'⚠  VEHICLE WILL MOVE — target speed = {target} m/s  ({target*3.6:.1f} km/h)')
    log.info('⚠  Ensure 15 m+ clear flat path ahead and wheels chocked until Step 2.')

    # ── Step 0: Wake all subsystems (mirrors gear.py Step 0) ──────────────────
    # 100% brake + steer + throttle_en + park OFF → waits for all EnState=Auto
    log.info('Step 0/6: Wake all subsystems (100% brake + steer + drive, 5 s) …')
    node.publish_for(5.0,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=0.0,   accel_target=1.0,
                     brake_en=True,  brake_target=100.0,
                     gear_en=True,   gear_target=3,       # NEUTRAL
                     park_en=True,   park_target=0)       # Park RELEASED
    node.print_status()
    log.info('  → Check: brake_en_state=1(Auto), drive_en_state=1(Auto), steer_en_state=1(Auto)')

    # ── Step 1: Engage DRIVE gear (100% brake) ────────────────────────────────
    log.info('Step 1/6: Shift to DRIVE gear (100% brake held, 3 s) …')
    node.publish_for(3.0,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=0.0,   accel_target=1.0,
                     brake_en=True,  brake_target=100.0,
                     gear_en=True,   gear_target=4,       # DRIVE
                     park_en=True,   park_target=0)
    node.print_status()
    log.info('  → Check: gear_actual=4 (DRIVE). If not, STOP — do not proceed to Step 2.')

    # ── Step 2: Release brake ─────────────────────────────────────────────────
    log.info('Step 2/6: Release brake to 0% (1 s) — VEHICLE MAY START MOVING …')
    log.info('  ⚠  REMOVE CHOCKS NOW if stationary test confirmed above.')
    node.publish_for(1.0,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=0.0,   accel_target=1.0,
                     brake_en=True,  brake_target=0.0,
                     gear_en=True,   gear_target=4,
                     park_en=True,   park_target=0)

    # ── Step 3: Ramp speed to target ─────────────────────────────────────────
    log.info(f'Step 3/6: Ramp speed → {target} m/s …')
    node.publish_for(HOLD_SECONDS,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=target,  accel_target=THROTTLE_ACCEL,
                     brake_en=False, brake_target=0.0,
                     gear_en=True,   gear_target=4,
                     park_en=True,   park_target=0)
    node.print_status()

    # ── Step 4: Hold speed ────────────────────────────────────────────────────
    log.info(f'Step 4/6: Hold {target} m/s for {HOLD_SECONDS} s …')
    node.publish_for(HOLD_SECONDS,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=target,  accel_target=THROTTLE_ACCEL,
                     brake_en=False, brake_target=0.0,
                     gear_en=True,   gear_target=4,
                     park_en=True,   park_target=0)
    node.print_status()

    # ── Step 5: SMOOTH STOP (2-phase) ─────────────────────────────────────────
    # July 3 field data: 100% brake from 2 m/s → 3.38 m/s² decel (0.6s stop) = HARSH
    # Fix: Phase 5a lets SpeedTarget=0 trigger VCU motor braking (~1 m/s² natural)
    #      Phase 5b ramps brake 0→15→30% once nearly stopped (final hold only)
    #      100% brake is reserved for Ctrl+C emergency path ONLY.
    log.info('Step 5/6: SMOOTH stop — motor brake then gentle ramp (NOT 100% jump) …')

    # Phase 5a: SpeedTarget=0, no brake — VCU motor braking decelerates naturally
    log.info('  5a: SpeedTarget=0, brake OFF — motor braking (3 s) …')
    node.publish_for(3.0,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=0.0,   accel_target=1.0,
                     brake_en=False, brake_target=0.0,
                     gear_en=True,   gear_target=4,
                     park_en=True,   park_target=0)
    node.print_status()

    # Phase 5b: gentle ramp 0 → 15 → 30% brake (2 s each step)
    log.info('  5b: Brake ramp 0→15→30% gentle hold (4 s) …')
    for b_pct in [15.0, 30.0]:
        node.publish_for(2.0,
                         steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                         drive_en=True,  speed_target=0.0,   accel_target=1.0,
                         brake_en=True,  brake_target=b_pct,
                         gear_en=True,   gear_target=4,
                         park_en=True,   park_target=0)
    node.print_status()

    # ── Step 6: NEUTRAL + park engage ─────────────────────────────────────────
    log.info('Step 6/6: Shift to NEUTRAL + engage park brake …')
    node.publish_for(2.0,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=0.0,   accel_target=1.0,
                     brake_en=True,  brake_target=50.0,  # 50% enough once stopped
                     gear_en=True,   gear_target=3,       # NEUTRAL
                     park_en=True,   park_target=0)
    node.publish_for(2.0,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=False, speed_target=0.0,   accel_target=1.0,
                     brake_en=True,  brake_target=50.0,
                     gear_en=True,   gear_target=3,
                     park_en=True,   park_target=1)       # Park ENGAGE via 0x104
    node.print_status()
    log.info('Throttle test COMPLETE ✓')
    log.info('  → Smooth stop: ~1 m/s² decel vs old 3.38 m/s² harsh stop')
    log.info('  → Check: vehicle_speed=0, park_actual=1')


def test_gear(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ GEAR CYCLE TEST ═══')
    log.info('Mirrors gear.py proven sequence: 100% brake + all EnState=Auto before shift.')

    # ── Step 0: Wake all subsystems (100% brake + steer + throttle + park OFF) ──
    # gear.py waits until brake_en=Auto AND throttle_en=Auto AND steer_en=Auto.
    # The VCU will not accept gear commands until all three subsystems are in Auto.
    # This is the critical interlock — 100% brake (not 30%) is required.
    log.info('Step 0: Wake all subsystems — 100% brake + steer + throttle (5 s) …')
    log.info('         Waiting for brake_en=Auto, throttle_en=Auto, steer_en=Auto')
    node.publish_for(5.0,
                     steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                     drive_en=True,  speed_target=0.0,   accel_target=1.0,
                     brake_en=True,  brake_target=100.0,  # 100% brake (gear.py value)
                     gear_en=True,   gear_target=3,        # NEUTRAL
                     park_en=True,   park_target=0)        # Park RELEASED
    node.print_status()
    log.info('  → Check above: brake_en_state=1(Auto), drive_en_state=1(Auto)')
    log.info('  → If en_states are still 3(Standby), wait and retry Step 0')

    # ── Gear cycle: same sequence as gear.py (no PARK gear via 0x103) ──────────
    # IMPORTANT: gear.py confirmed PARK gear (id=1) via 0x103 is NOT supported
    # on this VCU. Park is done via 0x104 (Park_Command) separately.
    # Cycle: NEUTRAL → DRIVE → NEUTRAL → REVERSE → NEUTRAL
    sequence = [
        (3, 'NEUTRAL',  4.0),
        (4, 'DRIVE',    4.0),
        (3, 'NEUTRAL',  4.0),
        (2, 'REVERSE',  4.0),
        (3, 'NEUTRAL',  4.0),
    ]
    for gear_val, name, dur in sequence:
        log.info(f'  → Gear → {name} ({gear_val}) … (0x103: EnCtrl=1 Target={gear_val})')
        # Keep ALL enablers active + 100% brake during every shift (exact gear.py logic)
        node.publish_for(dur,
                         steer_en=True,  steer_target=0.0,   steer_speed=100.0,
                         drive_en=True,  speed_target=0.0,   accel_target=1.0,
                         brake_en=True,  brake_target=100.0,
                         gear_en=True,   gear_target=gear_val,
                         park_en=True,   park_target=0)
        node.print_status()

    log.info('Gear test COMPLETE ✓')
    log.info('NOTE: To test PARK gear, use: python3 scripts/actuator_test.py --test park')


def test_park(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ PARKING BRAKE TEST ═══')

    # --- Extended VCU Warm-up with brake ---
    # VCU safety lock: gear must NOT be in DRIVE to engage park brake.
    # Must have brake engaged. Transition to NEUTRAL first, then PARK gear.
    log.info('  → VCU warm-up: brake + steer + gear=NEUTRAL (5 s) …')
    node.publish_for(5.0,
                     steer_en=True,  steer_target=0.0, steer_speed=100.0,
                     brake_en=True,  brake_target=30.0,
                     gear_en=True,   gear_target=3,    # NEUTRAL first
                     park_en=True,   park_target=0)
    node.print_status()

    log.info('  → Shifting to PARK gear …')
    node.publish_for(3.0,
                     steer_en=True,  steer_target=0.0, steer_speed=100.0,
                     brake_en=True,  brake_target=30.0,
                     gear_en=True,   gear_target=1,    # PARK gear
                     park_en=True,   park_target=0)
    node.print_status()

    log.info('Step 1/2: Release parking brake …')
    node.publish_for(HOLD_SECONDS,
                     steer_en=True,  steer_target=0.0, steer_speed=100.0,
                     brake_en=True,  brake_target=20.0,
                     park_en=True,   park_target=0,   # RELEASE
                     gear_en=True,   gear_target=1)   # PARK gear
    node.print_status()

    log.info('Step 2/2: Engage parking brake …')
    node.publish_for(HOLD_SECONDS,
                     steer_en=True,  steer_target=0.0, steer_speed=100.0,
                     brake_en=True,  brake_target=20.0,
                     park_en=True,   park_target=1,   # PARKING_TRIGGER
                     gear_en=True,   gear_target=1)
    node.print_status()
    log.info('Park test COMPLETE ✓')


def test_estop(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ EMERGENCY STOP TEST ═══')
    log.info('Sending E-stop command (emergency_stop=True) for 3 seconds …')
    node.publish_for(3.0, estop=True)
    node.print_status()
    log.info('IMPORTANT: Once E-stop is triggered, the safety_manager latches it.')
    log.info('           Restart the safety_manager node to clear the latch.')
    log.info('E-stop test COMPLETE ✓')


def test_full(node: ActuatorTestNode):
    tests = [
        ('Steering',  test_steering),
        ('Brake',     test_brake),
        ('Gear',      test_gear),
        ('Park',      test_park),
        ('Throttle',  test_throttle),
    ]
    for name, fn in tests:
        print(f'\n{"="*60}')
        print(f'  Ready to run: {name} Test')
        print(f'{"="*60}')
        resp = input('Press ENTER to continue, or type "skip" to skip: ').strip()
        if resp.lower() == 'skip':
            print(f'  Skipped {name}.')
            continue
        fn(node)
        node.center_and_stop()
        time.sleep(1.0)
    print('\nAll actuator tests COMPLETE.')


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='PIXKIT Actuator Test Script')
    parser.add_argument('--mode', choices=['sim', 'hw'], default='sim',
                        help='sim = simulator, hw = real vehicle')
    parser.add_argument('--test', choices=['steering', 'brake', 'throttle',
                                           'gear', 'park', 'estop', 'full'],
                        default='full', help='Which test to run')
    args = parser.parse_args()

    rclpy.init()
    node = ActuatorTestNode(mode=args.mode)

    # Warn user on hardware mode
    if args.mode == 'hw':
        print('\n' + '!'*60)
        print('  HARDWARE MODE — Commands will be sent to the real vehicle!')
        print('  Ensure vehicle is in a SAFE, CLEAR area before proceeding.')
        print('!'*60)
        resp = input('Type YES to confirm and continue: ')
        if resp.strip() != 'YES':
            print('Aborted.')
            rclpy.shutdown()
            sys.exit(0)

    TEST_MAP = {
        'steering': test_steering,
        'brake':    test_brake,
        'throttle': test_throttle,
        'gear':     test_gear,
        'park':     test_park,
        'estop':    test_estop,
        'full':     test_full,
    }

    try:
        TEST_MAP[args.test](node)
    except KeyboardInterrupt:
        node.get_logger().warn('Test interrupted by user. Sending stop command …')
        node.center_and_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

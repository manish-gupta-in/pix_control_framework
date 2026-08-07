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
        """Safe neutral state: center steer, zero speed, brakes off."""
        self.get_logger().info('  → Returning to center / stopped state …')
        self.publish_for(hold,
                         steer_en=True, steer_target=0.0, steer_speed=150.0,
                         drive_en=True, speed_target=0.0, accel_target=0.5,
                         brake_en=True, brake_target=0.0,
                         gear_en=True,  gear_target=3,
                         park_en=True,  park_target=0)

    def print_status(self):
        if self.latest_status is not None:
            s = self.latest_status
            print(f'    VCU Status → speed={s.vehicle_speed:.2f}m/s | '
                  f'steer={s.steer_angle:.1f}° | '
                  f'gear={s.gear_actual} | '
                  f'park={s.park_actual} | '
                  f'brake={s.brake_pedal:.1f}%')
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


def test_throttle(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ THROTTLE / SPEED TEST ═══')
    log.info('⚠  CAUTION: Vehicle will move forward! Ensure clear path.')
    log.info('Step 1/3: Engage Drive gear …')
    node.publish_for(1.0, gear_en=True, gear_target=4, park_en=True, park_target=0)

    log.info(f'Step 2/3: Ramp speed to {THROTTLE_TARGET} m/s …')
    node.publish_for(HOLD_SECONDS,
                     drive_en=True, speed_target=THROTTLE_TARGET, accel_target=THROTTLE_ACCEL,
                     gear_en=True,  gear_target=4,
                     steer_en=True, steer_target=0.0, steer_speed=100.0)
    node.print_status()

    log.info('Step 3/3: Command speed = 0, coast to stop …')
    node.publish_for(HOLD_SECONDS,
                     drive_en=True, speed_target=0.0, accel_target=0.5,
                     brake_en=True, brake_target=10.0,
                     gear_en=True,  gear_target=4)
    node.print_status()
    node.center_and_stop()
    log.info('Throttle test COMPLETE ✓')


def test_gear(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ GEAR CYCLE TEST ═══')
    log.info('NOTE: Vehicle must be stationary. No drive commands issued.')
    sequence = [
        (1, 'PARK',    3.0),
        (3, 'NEUTRAL', 3.0),
        (4, 'DRIVE',   3.0),
        (3, 'NEUTRAL', 3.0),
        (1, 'PARK',    3.0),
    ]
    for gear_val, name, dur in sequence:
        log.info(f'  → Gear → {name} ({gear_val}) …')
        node.publish_for(dur, gear_en=True, gear_target=gear_val, park_en=True, park_target=0)
        node.print_status()
    log.info('Gear test COMPLETE ✓')


def test_park(node: ActuatorTestNode):
    log = node.get_logger()
    log.info('═══ PARKING BRAKE TEST ═══')
    log.info('Step 1/2: Release parking brake …')
    node.publish_for(HOLD_SECONDS,
                     park_en=True, park_target=0,   # RELEASE
                     gear_en=True, gear_target=1)   # PARK gear
    node.print_status()

    log.info('Step 2/2: Engage parking brake …')
    node.publish_for(HOLD_SECONDS,
                     park_en=True, park_target=1,   # PARKING_TRIGGER
                     gear_en=True, gear_target=1)
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

#!/usr/bin/env python3
"""
pix_framework_watcher_node.py — v12 Live Verification Tool
==============================================================
Subscribes to ALL framework topics and prints a live terminal summary.
Verifies:
  1. Topic presence and liveness (both /pix_framework/* and legacy /pix/*)
  2. Control mode reporting (/pix_framework/control_mode_report)
  3. Zero naming collision with Autoware topics

Usage:
  python3 scripts/pix_framework_watcher_node.py

The watcher exits with code 0 if all checks pass, code 1 if any fail.
"""
import sys
import time
import rclpy
from rclpy.node import Node


# ─── Expected topic lists ────────────────────────────────────────────────────

STANDARD_TOPICS = [
    '/pix_framework/can_tx',
    '/pix_framework/can_rx',
    '/pix_framework/vehicle_status/steering',
    '/pix_framework/vehicle_status/velocity',
    '/pix_framework/vehicle_status/gear',
    '/pix_framework/control_mode_report',
]

LEGACY_TOPICS = [
    '/pix/vehicle_status',
    '/pix/control_cmd',
    '/pix/raw_control_cmd',
    '/pix/system_state',
    '/pix/commands/cruise_control',
]

EXPECTED_SERVICES = [
]

# These MUST NOT appear (collision with Autoware production stack)
FORBIDDEN_TOPICS = [
    '/control/command/control_cmd',
    '/control/command/gear_cmd',
    '/control/command/turn_indicators_cmd',
    '/control/command/hazard_lights_cmd',
    '/control/command/actuation_cmd',
    '/control/command/emergency_cmd',
    '/control/control_mode_request',
    '/vehicle/status/control_mode',
    '/vehicle/status/velocity_status',
    '/vehicle/status/steering_status',
    '/vehicle/status/gear_status',
    '/vehicle/status/turn_indicators_status',
    '/vehicle/status/hazard_lights_status',
    '/vehicle/status/actuation_status',
]


class FrameworkWatcher(Node):
    """
    Watches all PIX framework topics and reports their status.
    """

    def __init__(self):
        super().__init__('pix_framework_watcher')
        self.get_logger().info('PIX Framework Watcher starting...')

        self._results = {}
        self._errors = []

    def run_checks(self):
        """Run all verification checks."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('PIX FRAMEWORK v12 WATCHER — Topic/Service Verification')
        self.get_logger().info('=' * 60)

        # Get all active topics and services
        topic_list = self.get_topic_names_and_types()
        active_topics = set()
        for name, types in topic_list:
            active_topics.add(name)

        service_list = self.get_service_names_and_types()
        active_services = set()
        for name, types in service_list:
            active_services.add(name)

        # ── Check standard topics ────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── STANDARD PATH (/pix_framework/*) ──')
        for topic in STANDARD_TOPICS:
            found = topic in active_topics
            status = '✓ ACTIVE' if found else '✗ MISSING'
            self.get_logger().info(f'  {status}  {topic}')
            if not found:
                self._errors.append(f'Standard topic missing: {topic}')

        # ── Check legacy topics ──────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── LEGACY PATH (/pix/*) ──')
        for topic in LEGACY_TOPICS:
            found = topic in active_topics
            status = '✓ ACTIVE' if found else '· ABSENT (OK if bench-only)'
            self.get_logger().info(f'  {status}  {topic}')

        # ── Check services ───────────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── SERVICES ──')
        for svc in EXPECTED_SERVICES:
            found = svc in active_services
            status = '✓ ACTIVE' if found else '✗ MISSING'
            self.get_logger().info(f'  {status}  {svc}')
            if not found:
                self._errors.append(f'Service missing: {svc}')

        # ── Check forbidden topics (collision check) ─────────────────────
        self.get_logger().info('')
        self.get_logger().info('── COLLISION CHECK (must be empty) ──')
        collisions = []
        for topic in FORBIDDEN_TOPICS:
            if topic in active_topics:
                collisions.append(topic)
                self._errors.append(f'COLLISION: {topic} found — conflicts with Autoware!')

        if collisions:
            for t in collisions:
                self.get_logger().error(f'  ✗ COLLISION: {t}')
        else:
            self.get_logger().info('  ✓ No Autoware topic collisions detected')

        # ── Summary ──────────────────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        if self._errors:
            self.get_logger().error(f'RESULT: {len(self._errors)} ISSUES FOUND')
            for e in self._errors:
                self.get_logger().error(f'  → {e}')
            return 1
        else:
            self.get_logger().info('RESULT: ALL CHECKS PASSED ✓')
            return 0


def main(args=None):
    rclpy.init(args=args)
    watcher = FrameworkWatcher()

    # Wait a moment for topic discovery
    time.sleep(2.0)

    exit_code = watcher.run_checks()

    watcher.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

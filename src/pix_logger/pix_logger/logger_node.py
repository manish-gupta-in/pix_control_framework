#!/usr/bin/env python3
"""
Logging & Replay Framework for PIXKIT Control Framework
========================================================
Logs all key topics to timestamped CSV files for:
  - Post-run analysis
  - Debugging algorithm behavior
  - Replay via /pix/vehicle_status simulation

CSV files are written to ~/pix_logs/<session_timestamp>/
One file per topic category.

Topics logged:
  /pix/vehicle_status   → vehicle_state.csv
  /pix/control_cmd      → control_cmd.csv
  /pix/raw_control_cmd  → raw_cmd.csv
  /pix/system_state     → system_state.csv
"""
import os
import csv
import time
import datetime
import threading
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixVehicleStatus, PixControlCmd, PixSystemState


class PixLoggerNode(Node):
    """
    Subscribes to key PIX topics and writes CSV logs with wall-clock timestamps.
    Flushes files every 5 seconds for safety.
    """

    def __init__(self):
        super().__init__('pix_logger')

        # Parameters
        self.declare_parameter('log_dir',      os.path.expanduser('~/pix_logs'))
        self.declare_parameter('flush_interval', 5.0)   # seconds
        self.declare_parameter('log_vehicle_status', True)
        self.declare_parameter('log_control_cmd',    True)
        self.declare_parameter('log_raw_cmd',        True)
        self.declare_parameter('log_system_state',   True)

        log_base        = self.get_parameter('log_dir').value
        self.flush_int  = self.get_parameter('flush_interval').value
        log_vs          = self.get_parameter('log_vehicle_status').value
        log_cc          = self.get_parameter('log_control_cmd').value
        log_rc          = self.get_parameter('log_raw_cmd').value
        log_ss          = self.get_parameter('log_system_state').value

        # Create session directory
        session = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_dir = os.path.join(log_base, session)
        os.makedirs(self._log_dir, exist_ok=True)
        self.get_logger().info(f'Logging to: {self._log_dir}')

        self._lock = threading.Lock()
        self._files = {}
        self._writers = {}

        # Open CSV files and write headers
        if log_vs:
            self._open('vehicle_state', [
                'wall_time', 'ros_time',
                'steer_angle', 'steer_speed', 'steer_en_state',
                'vehicle_speed', 'vehicle_accel',
                'throttle_pedal', 'brake_pedal',
                'drive_en_state', 'brake_en_state',
                'gear_actual', 'park_actual',
                'vehicle_mode', 'battery_voltage', 'battery_soc',
                'steer_flt1', 'steer_flt2', 'drive_flt1', 'drive_flt2',
                'brake_flt1', 'brake_flt2', 'front_crash', 'back_crash',
            ])
            self.create_subscription(PixVehicleStatus, '/pix/vehicle_status',
                                     self._status_cb, 10)

        if log_cc:
            self._open_cmd_file('control_cmd')
            self.create_subscription(PixControlCmd, '/pix/control_cmd',
                                     lambda m: self._cmd_cb(m, 'control_cmd'), 10)

        if log_rc:
            self._open_cmd_file('raw_cmd')
            self.create_subscription(PixControlCmd, '/pix/raw_control_cmd',
                                     lambda m: self._cmd_cb(m, 'raw_cmd'), 10)

        if log_ss:
            state_names = ['MANUAL', 'STANDBY', 'AUTONOMOUS', 'FAULT', 'ESTOP']
            self._open('system_state', [
                'wall_time', 'ros_time', 'state', 'state_name',
                'reason', 'duration_s', 'estop_latched',
                'fault_latched', 'active_algorithm', 'fault_count',
            ])
            self.create_subscription(PixSystemState, '/pix/system_state',
                                     self._sysstate_cb, 10)

        self._state_names = {0: 'MANUAL', 1: 'STANDBY', 2: 'AUTONOMOUS',
                             3: 'FAULT', 4: 'ESTOP'}

        # Flush timer
        self.create_timer(self.flush_int, self._flush_all)
        self.get_logger().info('Logger node running.')

    def _open(self, name: str, headers: list):
        path = os.path.join(self._log_dir, f'{name}.csv')
        f = open(path, 'w', newline='')
        w = csv.writer(f)
        w.writerow(headers)
        self._files[name]   = f
        self._writers[name] = w

    def _open_cmd_file(self, name: str):
        self._open(name, [
            'wall_time', 'ros_time',
            'steer_target', 'steer_speed', 'steer_en',
            'speed_target', 'accel_target', 'drive_en',
            'brake_target', 'brake_en',
            'gear_target', 'gear_en',
            'park_target', 'park_en',
            'emergency_stop',
        ])

    def _status_cb(self, msg: PixVehicleStatus):
        rt = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._writers['vehicle_state'].writerow([
                time.time(), rt,
                msg.steer_angle, msg.steer_speed, msg.steer_en_state,
                msg.vehicle_speed, msg.vehicle_accel,
                msg.throttle_pedal, msg.brake_pedal,
                msg.drive_en_state, msg.brake_en_state,
                msg.gear_actual, msg.park_actual,
                msg.vehicle_mode, msg.battery_voltage, msg.battery_soc,
                msg.steer_flt1, msg.steer_flt2, msg.drive_flt1, msg.drive_flt2,
                msg.brake_flt1, msg.brake_flt2,
                int(msg.front_crash), int(msg.back_crash),
            ])

    def _cmd_cb(self, msg: PixControlCmd, name: str):
        rt = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._writers[name].writerow([
                time.time(), rt,
                msg.steer_target, msg.steer_speed, int(msg.steer_en),
                msg.speed_target, msg.accel_target, int(msg.drive_en),
                msg.brake_target, int(msg.brake_en),
                msg.gear_target, int(msg.gear_en),
                msg.park_target, int(msg.park_en),
                int(msg.emergency_stop),
            ])

    def _sysstate_cb(self, msg: PixSystemState):
        rt = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._writers['system_state'].writerow([
                time.time(), rt,
                msg.state, self._state_names.get(msg.state, '?'),
                msg.reason, f'{msg.state_duration_secs:.2f}',
                int(msg.estop_latched), int(msg.fault_latched),
                msg.active_algorithm, msg.fault_count,
            ])

    def _flush_all(self):
        with self._lock:
            for f in self._files.values():
                try:
                    f.flush()
                except Exception:
                    pass

    def destroy_node(self):
        self._flush_all()
        with self._lock:
            for f in self._files.values():
                try:
                    f.close()
                except Exception:
                    pass
        self.get_logger().info(f'Logs saved to: {self._log_dir}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PixLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

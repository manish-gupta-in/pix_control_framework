#!/usr/bin/env python3
"""
Diagnostics Framework for PIXKIT Control Framework
====================================================
Uses ROS2 diagnostic_updater to publish /diagnostics.
Monitors:
  1. CAN RX health     — /pix/vehicle_status update rate
  2. CAN TX health     — /pix/control_cmd update rate
  3. VCU faults        — all fault bits from vehicle_status
  4. Watchdog status   — raw_control_cmd freshness
  5. System state      — current state from state manager
  6. Battery           — voltage and SOC thresholds

View with:
  ros2 topic echo /diagnostics
  ros2 run rqt_runtime_monitor rqt_runtime_monitor  (optional GUI)
"""
import time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pix_vehicle_msgs.msg import PixVehicleStatus, PixControlCmd, PixSystemState


def _kv(key: str, value) -> KeyValue:
    kv = KeyValue()
    kv.key = str(key)
    kv.value = str(value)
    return kv


class PixDiagnosticsNode(Node):
    """
    Publishes /diagnostics at 2 Hz covering CAN, VCU, watchdog, battery, and state.
    """

    def __init__(self):
        super().__init__('pix_diagnostics')

        # Parameters
        self.declare_parameter('publish_rate',       2.0)    # Hz
        self.declare_parameter('can_rx_timeout',     0.5)    # s  — warn if no vehicle_status
        self.declare_parameter('can_tx_timeout',     0.5)    # s  — warn if no control_cmd
        self.declare_parameter('wdog_timeout',       1.0)    # s  — warn if no raw cmd
        self.declare_parameter('battery_warn_v',    46.0)    # V  — low voltage warning
        self.declare_parameter('battery_error_v',   42.0)    # V  — critical low
        self.declare_parameter('battery_warn_soc',  20.0)    # %  — low SOC warning
        self.declare_parameter('battery_error_soc', 10.0)    # %  — critical low SOC

        rate              = self.get_parameter('publish_rate').value
        self.can_rx_to    = self.get_parameter('can_rx_timeout').value
        self.can_tx_to    = self.get_parameter('can_tx_timeout').value
        self.wdog_to      = self.get_parameter('wdog_timeout').value
        self.bat_warn_v   = self.get_parameter('battery_warn_v').value
        self.bat_error_v  = self.get_parameter('battery_error_v').value
        self.bat_warn_soc = self.get_parameter('battery_warn_soc').value
        self.bat_err_soc  = self.get_parameter('battery_error_soc').value

        # Timestamps & data
        self._status_ts   = 0.0
        self._ctrl_ts     = 0.0
        self._raw_cmd_ts  = 0.0
        self._latest_status  = None
        self._latest_ctrl    = None
        self._latest_state   = None

        # Subscriptions
        self.create_subscription(PixVehicleStatus, '/pix/vehicle_status',
                                 self._status_cb, 10)
        self.create_subscription(PixControlCmd,    '/pix/control_cmd',
                                 self._ctrl_cb,    10)
        self.create_subscription(PixControlCmd,    '/pix/raw_control_cmd',
                                 self._raw_cmd_cb, 10)
        self.create_subscription(PixSystemState,   '/pix/system_state',
                                 self._state_cb,   10)

        # Publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_timer(1.0 / rate, self._publish_diagnostics)
        self.get_logger().info('Diagnostics node initialized. Publishing /diagnostics')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _status_cb(self, msg):
        self._latest_status = msg
        self._status_ts = time.monotonic()

    def _ctrl_cb(self, msg):
        self._latest_ctrl = msg
        self._ctrl_ts = time.monotonic()

    def _raw_cmd_cb(self, msg):
        self._raw_cmd_ts = time.monotonic()

    def _state_cb(self, msg):
        self._latest_state = msg

    # ── Diagnostics checks ────────────────────────────────────────────────────

    def _check_can_rx(self, now: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'CAN RX — vehicle_status'
        s.hardware_id = 'can4_rx'
        age = now - self._status_ts if self._status_ts > 0 else 9999.0
        s.values = [_kv('last_received_age_ms', f'{age*1000:.1f}')]

        if self._status_ts == 0:
            s.level = DiagnosticStatus.ERROR
            s.message = 'No vehicle_status received — CAN RX may not be running'
        elif age > self.can_rx_to:
            s.level = DiagnosticStatus.WARN
            s.message = f'vehicle_status stale ({age*1000:.0f} ms)'
        else:
            s.level = DiagnosticStatus.OK
            s.message = f'OK ({age*1000:.0f} ms)'

        if self._latest_status is not None:
            v = self._latest_status
            s.values += [
                _kv('steer_en_state',  v.steer_en_state),
                _kv('drive_en_state',  v.drive_en_state),
                _kv('brake_en_state',  v.brake_en_state),
                _kv('vehicle_speed_mps', f'{v.vehicle_speed:.3f}'),
            ]
        return s

    def _check_can_tx(self, now: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'CAN TX — control_cmd'
        s.hardware_id = 'can4_tx'
        age = now - self._ctrl_ts if self._ctrl_ts > 0 else 9999.0
        s.values = [_kv('last_sent_age_ms', f'{age*1000:.1f}')]

        if self._ctrl_ts == 0:
            s.level = DiagnosticStatus.ERROR
            s.message = 'No control_cmd sent — CAN TX may not be running'
        elif age > self.can_tx_to:
            s.level = DiagnosticStatus.WARN
            s.message = f'control_cmd stale ({age*1000:.0f} ms)'
        else:
            s.level = DiagnosticStatus.OK
            s.message = f'OK ({age*1000:.0f} ms)'
        return s

    def _check_vcu_faults(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'VCU Faults'
        s.hardware_id = 'vcu'

        if self._latest_status is None:
            s.level = DiagnosticStatus.WARN
            s.message = 'No vehicle_status — cannot check faults'
            return s

        v = self._latest_status
        faults = []
        if v.steer_flt1: faults.append(f'steer_flt1={v.steer_flt1}')
        if v.steer_flt2: faults.append(f'steer_flt2={v.steer_flt2}')
        if v.drive_flt1: faults.append(f'drive_flt1={v.drive_flt1}')
        if v.drive_flt2: faults.append(f'drive_flt2={v.drive_flt2}')
        if v.brake_flt1: faults.append(f'brake_flt1={v.brake_flt1}')
        if v.brake_flt2: faults.append(f'brake_flt2={v.brake_flt2}')
        if v.park_flt:   faults.append(f'park_flt={v.park_flt}')
        if v.gear_flt:   faults.append(f'gear_flt={v.gear_flt}')
        if v.front_crash: faults.append('FRONT_CRASH')
        if v.back_crash:  faults.append('BACK_CRASH')
        if v.aeb_active:  faults.append('AEB_ACTIVE')

        s.values = [_kv('fault_list', ', '.join(faults) or 'none')]
        if faults:
            s.level = DiagnosticStatus.ERROR
            s.message = 'VCU FAULTS: ' + ', '.join(faults)
        else:
            s.level = DiagnosticStatus.OK
            s.message = 'No VCU faults'
        return s

    def _check_watchdog(self, now: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'Algorithm Watchdog'
        s.hardware_id = 'watchdog'
        age = now - self._raw_cmd_ts if self._raw_cmd_ts > 0 else 9999.0
        s.values = [_kv('raw_cmd_age_ms', f'{age*1000:.1f}')]

        if self._raw_cmd_ts == 0:
            s.level = DiagnosticStatus.WARN
            s.message = 'No algorithm commands received yet (normal in STANDBY)'
        elif age > self.wdog_to:
            s.level = DiagnosticStatus.WARN
            s.message = f'No algorithm command for {age*1000:.0f} ms'
        else:
            s.level = DiagnosticStatus.OK
            s.message = f'Algorithm active ({age*1000:.0f} ms ago)'
        return s

    def _check_battery(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'Battery (BMS)'
        s.hardware_id = 'bms'

        if self._latest_status is None:
            s.level = DiagnosticStatus.WARN
            s.message = 'No BMS data'
            return s

        v = self._latest_status
        s.values = [
            _kv('voltage_V',  f'{v.battery_voltage:.2f}'),
            _kv('current_A',  f'{v.battery_current:.2f}'),
            _kv('soc_pct',    f'{v.battery_soc:.1f}'),
        ]

        if v.battery_voltage < self.bat_error_v or v.battery_soc < self.bat_err_soc:
            s.level = DiagnosticStatus.ERROR
            s.message = f'CRITICAL: V={v.battery_voltage:.1f}V SOC={v.battery_soc:.0f}%'
        elif v.battery_voltage < self.bat_warn_v or v.battery_soc < self.bat_warn_soc:
            s.level = DiagnosticStatus.WARN
            s.message = f'Low battery: V={v.battery_voltage:.1f}V SOC={v.battery_soc:.0f}%'
        else:
            s.level = DiagnosticStatus.OK
            s.message = f'OK: V={v.battery_voltage:.1f}V SOC={v.battery_soc:.0f}%'
        return s

    def _check_system_state(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'System State'
        s.hardware_id = 'state_manager'

        state_names = {0: 'MANUAL', 1: 'STANDBY', 2: 'AUTONOMOUS', 3: 'FAULT', 4: 'ESTOP'}

        if self._latest_state is None:
            s.level = DiagnosticStatus.WARN
            s.message = 'System State Manager not running'
            return s

        st = self._latest_state
        name = state_names.get(st.state, f'UNKNOWN({st.state})')
        s.values = [
            _kv('state',              name),
            _kv('reason',             st.reason),
            _kv('duration_s',         f'{st.state_duration_secs:.1f}'),
            _kv('estop_latched',      st.estop_latched),
            _kv('fault_latched',      st.fault_latched),
            _kv('active_algorithm',   st.active_algorithm),
            _kv('total_fault_count',  st.fault_count),
        ]

        if st.state == 4:   # ESTOP
            s.level = DiagnosticStatus.ERROR
            s.message = f'ESTOP LATCHED — {st.reason}'
        elif st.state == 3:  # FAULT
            s.level = DiagnosticStatus.ERROR
            s.message = f'FAULT — {st.reason}'
        elif st.state == 2:  # AUTONOMOUS
            s.level = DiagnosticStatus.OK
            s.message = f'AUTONOMOUS: {st.active_algorithm}'
        elif st.state == 1:  # STANDBY
            s.level = DiagnosticStatus.OK
            s.message = 'STANDBY — waiting for algorithm'
        else:                # MANUAL
            s.level = DiagnosticStatus.OK
            s.message = 'MANUAL — DBW off'
        return s

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish_diagnostics(self):
        now = time.monotonic()
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            self._check_can_rx(now),
            self._check_can_tx(now),
            self._check_vcu_faults(),
            self._check_watchdog(now),
            self._check_battery(),
            self._check_system_state(),
        ]
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PixDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

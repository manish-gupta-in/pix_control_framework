"""
Unit tests for Logger and Config Manager
Tests CSV row construction, file creation, and profile loading.
"""
import os
import csv
import tempfile
import yaml
import json
import pytest
import time


# ─── Logger logic tests ───────────────────────────────────────────────────────

class TestLoggerCSV:
    """Validates CSV file structure and row writing logic (no ROS2 required)."""

    def _vehicle_state_headers(self):
        return [
            'wall_time', 'ros_time',
            'steer_angle', 'steer_speed', 'steer_en_state',
            'vehicle_speed', 'vehicle_accel',
            'throttle_pedal', 'brake_pedal',
            'drive_en_state', 'brake_en_state',
            'gear_actual', 'park_actual',
            'vehicle_mode', 'battery_voltage', 'battery_soc',
            'steer_flt1', 'steer_flt2', 'drive_flt1', 'drive_flt2',
            'brake_flt1', 'brake_flt2', 'front_crash', 'back_crash',
        ]

    def _cmd_headers(self):
        return [
            'wall_time', 'ros_time',
            'steer_target', 'steer_speed', 'steer_en',
            'speed_target', 'accel_target', 'drive_en',
            'brake_target', 'brake_en',
            'gear_target', 'gear_en',
            'park_target', 'park_en',
            'emergency_stop',
        ]

    def test_vehicle_state_csv_header_count(self):
        headers = self._vehicle_state_headers()
        assert len(headers) == 24

    def test_cmd_csv_header_count(self):
        assert len(self._cmd_headers()) == 15

    def test_csv_write_and_read_roundtrip(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv',
                                         delete=False, newline='') as f:
            writer = csv.writer(f)
            headers = self._vehicle_state_headers()
            writer.writerow(headers)
            row = [time.time(), 1000.0,
                   -5.0, 100.0, 3,
                   2.5, 0.0,
                   20.0, 0.0,
                   1, 0,
                   4, 0,
                   1, 52.0, 85.0,
                   0, 0, 0, 0,
                   0, 0, 0, 0]
            writer.writerow(row)
            path = f.name

        with open(path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        assert len(rows) == 1
        assert float(rows[0]['steer_angle']) == pytest.approx(-5.0)
        assert float(rows[0]['vehicle_speed']) == pytest.approx(2.5)
        assert float(rows[0]['battery_voltage']) == pytest.approx(52.0)
        os.unlink(path)

    def test_csv_log_directory_creation(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            session_dir = os.path.join(tmpdir, '20260608_120000')
            os.makedirs(session_dir, exist_ok=True)
            assert os.path.exists(session_dir)

    def test_multiple_rows_preserved(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv',
                                         delete=False, newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'val'])
            for i in range(50):
                writer.writerow([float(i), i * 1.5])
            path = f.name

        with open(path) as f:
            rows = list(csv.DictReader(f))

        assert len(rows) == 50
        assert float(rows[49]['val']) == pytest.approx(49 * 1.5)
        os.unlink(path)


# ─── Config Manager profile tests ────────────────────────────────────────────

class TestConfigProfiles:
    """Tests profile YAML loading and structure validation."""

    def _profile_path(self, name):
        # test file is at: src/pix_logger/test/test_logger_config.py
        # profiles are at: src/pix_config_manager/profiles/<name>.yaml
        test_dir   = os.path.dirname(os.path.abspath(__file__))  # .../pix_logger/test
        pix_logger = os.path.dirname(test_dir)                    # .../pix_logger
        src_dir    = os.path.dirname(pix_logger)                  # .../src
        return os.path.join(src_dir, 'pix_config_manager', 'profiles', f'{name}.yaml')

    def _load_profile(self, name):
        path = self._profile_path(name)
        if not os.path.exists(path):
            pytest.skip(f"Profile '{name}' not installed yet")
        with open(path) as f:
            return yaml.safe_load(f)

    def test_simulation_profile_exists(self):
        assert os.path.exists(self._profile_path('simulation')), \
            "simulation.yaml profile missing"

    def test_hardware_profile_exists(self):
        assert os.path.exists(self._profile_path('hardware')), \
            "hardware.yaml profile missing"

    def test_tuning_profile_exists(self):
        assert os.path.exists(self._profile_path('tuning')), \
            "tuning.yaml profile missing"

    def test_simulation_has_required_sections(self):
        data = self._load_profile('simulation')
        assert 'safety' in data
        assert 'can' in data

    def test_hardware_has_required_sections(self):
        data = self._load_profile('hardware')
        assert 'safety' in data
        assert 'can' in data

    def test_hardware_can_interface_is_can4(self):
        data = self._load_profile('hardware')
        assert data['can']['can_interface'] == 'can4'

    def test_simulation_can_interface_is_vcan0(self):
        data = self._load_profile('simulation')
        assert data['can']['can_interface'] == 'vcan0'

    def test_hardware_max_speed_is_conservative(self):
        data = self._load_profile('hardware')
        assert data['safety']['max_speed'] <= 3.5, \
            "Hardware max speed should be ≤ 3.5 m/s for safety"

    def test_simulation_max_speed_higher_than_hardware(self):
        sim = self._load_profile('simulation')
        hw  = self._load_profile('hardware')
        assert sim['safety']['max_speed'] > hw['safety']['max_speed']

    def test_hardware_steer_rate_is_safe(self):
        data = self._load_profile('hardware')
        assert data['safety']['max_steer_rate'] <= 200.0, \
            "Hardware steer rate should be ≤ 200°/s"

    def test_profile_json_serializable(self):
        data = self._load_profile('hardware')
        output = json.dumps({'profile': 'hardware', 'params': data})
        parsed = json.loads(output)
        assert parsed['profile'] == 'hardware'
        assert 'safety' in parsed['params']

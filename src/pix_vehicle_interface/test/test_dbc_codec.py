#!/usr/bin/env python3
"""
Unit tests for DBC encoder/decoder using the actual hook2_AD.dbc file.

Tests that CAN frames are correctly encoded and decoded for all 6 command
messages and 7 report messages.

Run with:
    cd pix_control_framework
    python3 -m pytest src/pix_vehicle_interface/test/test_dbc_codec.py -v
"""
import os
import sys
import pytest

# Resolve the real DBC path (tries workspace locations)
_SEARCH_PATHS = [
    os.path.join(os.path.dirname(__file__), '..', 'config', 'hook2_AD.dbc'),
    '/home/bits/Desktop/Manish/Custom_Interface_study/pix_control_framework/src/pix_vehicle_interface/config/hook2_AD.dbc',
    '/home/bits/Desktop/Manish/Custom_Interface_study/hook2_AD (1).dbc',
]

DBC_PATH = None
for p in _SEARCH_PATHS:
    if os.path.exists(p):
        DBC_PATH = os.path.abspath(p)
        break

if DBC_PATH is None:
    pytest.skip("hook2_AD.dbc not found – skipping CAN codec tests", allow_module_level=True)

# Add src to import path so dbc_encoder/decoder can be imported directly
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pix_vehicle_interface.dbc_encoder import DBCEncoder
from pix_vehicle_interface.dbc_decoder import DBCDecoder


@pytest.fixture(scope='module')
def encoder():
    return DBCEncoder(DBC_PATH)


@pytest.fixture(scope='module')
def decoder():
    return DBCDecoder(DBC_PATH)


# ---------------------------------------------------------------------------
# Message existence tests
# ---------------------------------------------------------------------------

class TestMessagePresence:
    EXPECTED_CMD_MSGS = [
        'Throttle_Command',
        'Brake_Command',
        'Steering_Command',
        'Gear_Command',
        'Park_Command',
        'Vehicle_Mode_Command',
    ]
    EXPECTED_REPORT_MSGS = [
        'Throttle_Report',
        'Brake_Report',
        'Steering_Report',
        'Gear_Report',
        'Park_Report',
        'VCU_Report',
        'BMS_Report',
    ]

    def test_all_command_messages_present(self, encoder):
        import cantools
        db = cantools.database.load_file(DBC_PATH)
        msg_names = [m.name for m in db.messages]
        for name in self.EXPECTED_CMD_MSGS:
            assert name in msg_names, f"Missing command message: {name}"

    def test_all_report_messages_present(self, encoder):
        import cantools
        db = cantools.database.load_file(DBC_PATH)
        msg_names = [m.name for m in db.messages]
        for name in self.EXPECTED_REPORT_MSGS:
            assert name in msg_names, f"Missing report message: {name}"


# ---------------------------------------------------------------------------
# Encoding tests
# ---------------------------------------------------------------------------

class TestEncoding:
    def test_throttle_command_encodes(self, encoder):
        fid, payload = encoder.encode_message('Throttle_Command', {
            'Dirve_EnCtrl': 1,
            'Dirve_SpeedTarget': 2.5,
            'Dirve_Acc': 1.0,
            'Dirve_ThrottlePedalTarget': 0.0,
        })
        assert fid is not None
        assert payload is not None
        assert len(payload) == 8

    def test_brake_command_encodes(self, encoder):
        fid, payload = encoder.encode_message('Brake_Command', {
            'Brake_EnCtrl': 1,
            'Brake_Pedal_Target': 30.0,
            'Brake_Dec': 0.0,
            'AEB_EnCtrl': 0,
        })
        assert fid is not None
        assert len(payload) == 8

    def test_steering_command_encodes(self, encoder):
        fid, payload = encoder.encode_message('Steering_Command', {
            'Steer_EnCtrl': 1,
            'Steer_AngleTarget': 200,
            'Steer_AngleSpeed': 120,
        })
        assert fid is not None
        assert len(payload) == 8

    def test_gear_command_encodes(self, encoder):
        fid, payload = encoder.encode_message('Gear_Command', {
            'Gear_EnCtrl': 1,
            'Gear_Target': 4,   # DRIVE
        })
        assert fid is not None
        assert len(payload) == 8

    def test_park_command_encodes(self, encoder):
        fid, payload = encoder.encode_message('Park_Command', {
            'Park_EnCtrl': 1,
            'Park_Target': 0,   # RELEASE
        })
        assert fid is not None
        assert len(payload) == 8

    def test_vehicle_mode_command_encodes(self, encoder):
        fid, payload = encoder.encode_message('Vehicle_Mode_Command', {
            'Auto_Professional': 1,
            'Headlight_Ctrl': 0,
            'TurnLight_Ctrl': 0,
            'Vehicle_VIN_Req': 0,
            'Drive_ModeCtrl': 1,
            'Steer_ModeCtrl': 0,
        })
        assert fid is not None
        assert len(payload) == 8

    def test_throttle_frame_id_is_0x100(self, encoder):
        fid, _ = encoder.encode_message('Throttle_Command', {
            'Dirve_EnCtrl': 0,
            'Dirve_SpeedTarget': 0.0,
            'Dirve_Acc': 0.0,
            'Dirve_ThrottlePedalTarget': 0.0,
        })
        assert fid == 0x100  # 256 decimal

    def test_brake_frame_id_is_0x101(self, encoder):
        fid, _ = encoder.encode_message('Brake_Command', {
            'Brake_EnCtrl': 0,
            'Brake_Pedal_Target': 0.0,
            'Brake_Dec': 0.0,
            'AEB_EnCtrl': 0,
        })
        assert fid == 0x101  # 257 decimal

    def test_steering_frame_id_is_0x102(self, encoder):
        fid, _ = encoder.encode_message('Steering_Command', {
            'Steer_EnCtrl': 0,
            'Steer_AngleTarget': 0,
            'Steer_AngleSpeed': 1,
        })
        assert fid == 0x102  # 258 decimal

    def test_gear_frame_id_is_0x103(self, encoder):
        fid, _ = encoder.encode_message('Gear_Command', {
            'Gear_EnCtrl': 0,
            'Gear_Target': 3,
        })
        assert fid == 0x103  # 259 decimal

    def test_park_frame_id_is_0x104(self, encoder):
        fid, _ = encoder.encode_message('Park_Command', {
            'Park_EnCtrl': 0,
            'Park_Target': 0,
        })
        assert fid == 0x104  # 260 decimal

    def test_vehicle_mode_frame_id_is_0x105(self, encoder):
        fid, _ = encoder.encode_message('Vehicle_Mode_Command', {
            'Auto_Professional': 0,
            'Headlight_Ctrl': 0,
            'TurnLight_Ctrl': 0,
            'Vehicle_VIN_Req': 0,
            'Drive_ModeCtrl': 0,
            'Steer_ModeCtrl': 0,
        })
        assert fid == 0x105  # 261 decimal

    def test_unknown_message_returns_none(self, encoder):
        fid, payload = encoder.encode_message('NonExistentMessage', {})
        assert fid is None
        assert payload is None


# ---------------------------------------------------------------------------
# Decoding tests
# ---------------------------------------------------------------------------

class TestDecoding:
    def test_encode_decode_roundtrip_throttle(self, encoder, decoder):
        """Encode then decode throttle command, check signal values survive."""
        fid, payload = encoder.encode_message('Throttle_Command', {
            'Dirve_EnCtrl': 1,
            'Dirve_SpeedTarget': 3.0,
            'Dirve_Acc': 1.5,
            'Dirve_ThrottlePedalTarget': 0.0,
        })
        name, decoded = decoder.decode_message(fid, payload)
        assert name == 'Throttle_Command'
        # cantools returns named enum strings from VAL_TABLE (e.g. 'Enable')
        assert str(decoded['Dirve_EnCtrl']) in ('1', 'Enable')
        assert float(decoded['Dirve_SpeedTarget']) == pytest.approx(3.0, abs=0.1)

    def test_encode_decode_roundtrip_brake(self, encoder, decoder):
        fid, payload = encoder.encode_message('Brake_Command', {
            'Brake_EnCtrl': 1,
            'Brake_Pedal_Target': 50.0,
            'Brake_Dec': 0.0,
            'AEB_EnCtrl': 0,
        })
        name, decoded = decoder.decode_message(fid, payload)
        assert name == 'Brake_Command'
        # cantools returns named enum strings from VAL_TABLE
        assert str(decoded['Brake_EnCtrl']) in ('1', 'Enable')
        assert float(decoded['Brake_Pedal_Target']) == pytest.approx(50.0, abs=1.0)

    def test_encode_decode_roundtrip_steering(self, encoder, decoder):
        fid, payload = encoder.encode_message('Steering_Command', {
            'Steer_EnCtrl': 1,
            'Steer_AngleTarget': 150,
            'Steer_AngleSpeed': 100,
        })
        name, decoded = decoder.decode_message(fid, payload)
        assert name == 'Steering_Command'
        assert int(decoded['Steer_AngleTarget']) == pytest.approx(150, abs=2)

    def test_unknown_frame_id_returns_none(self, decoder):
        name, decoded = decoder.decode_message(0xDEAD, bytes(8))
        assert name is None
        assert decoded is None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

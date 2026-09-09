#pragma once
/**
 * @file can_codec.hpp
 * @brief Hand-written CAN encode/decode for Hooke2 VCU protocol.
 *
 * Replaces the cantools/DBC runtime dependency. Each function implements
 * the exact byte layout and checksum verified on hardware.
 *
 * Reference: hooke2_interface/src/common/ command files + gear.py verified payloads
 */

#include <cstdint>
#include <cstring>
#include <algorithm>

#include "pix_can_codec/can_ids.hpp"
#include "pix_can_codec/checksum.hpp"

namespace pix_can_codec {

// ─── Encode structs ─────────────────────────────────────────────────────────

struct ThrottleCmd {
  bool enable = false;
  double speed_ms = 0.0;        // 0.0–15.0 m/s
  double accel_ms2 = 1.0;       // 0.0–10.0 m/s²
  double pedal_pct = 0.0;       // 0.0–100.0 %
};

struct BrakeCmd {
  bool enable = false;
  double pedal_pct = 0.0;       // 0.0–100.0 %
  double decel_ms2 = 0.0;       // 0.0–10.0 m/s²
  bool aeb_enable = false;
};

struct SteeringCmd {
  bool enable = false;
  double angle_deg = 0.0;       // -500 to +500 deg
  int angle_speed = 100;        // 0–250 deg/s
};

struct GearCmd {
  bool enable = false;
  uint8_t gear_target = GEAR_INVALID;  // 0–4
};

struct ParkCmd {
  bool enable = false;
  bool engage = false;           // true=ENGAGE, false=RELEASE
};

struct VcuModeCmd {
  bool auto_professional = true; // ALWAYS true per v6 fix
  uint8_t drive_mode = 1;       // 1=SPEED_DRIVE
  uint8_t steer_mode = 0;       // 0=STANDARD
  uint8_t turn_light = 0;       // 0=OFF, 1=LEFT, 2=RIGHT, 3=HAZARD
  bool vin_req = false;
};

// ─── Decode structs ─────────────────────────────────────────────────────────

struct ThrottleReport {
  uint8_t en_state = 0;
  double throttle_pedal_actual = 0.0;
  double speed_actual = 0.0;
  uint8_t flt1 = 0;
  uint8_t flt2 = 0;
};

struct BrakeReport {
  uint8_t en_state = 0;
  double brake_pedal_actual = 0.0;
  uint8_t flt1 = 0;
  uint8_t flt2 = 0;
};

struct SteeringReport {
  uint8_t en_state = 0;
  double steer_angle_actual = 0.0;   // deg (raw - 500)
  double steer_speed_actual = 0.0;
  uint8_t flt1 = 0;
  uint8_t flt2 = 0;
};

struct GearReport {
  uint8_t gear_actual = 0;
  uint8_t gear_flt = 0;
};

struct ParkReport {
  uint8_t park_actual = 0;
  uint8_t park_flt = 0;
};

struct VcuReport {
  double vehicle_speed = 0.0;
  double vehicle_accel = 0.0;
  uint8_t vehicle_mode_state = 0;
  uint8_t drive_mode_status = 0;
  uint8_t steer_mode_status = 0;
  bool front_crash = false;
  bool back_crash = false;
  bool aeb_active = false;
  bool brake_light = false;
  uint8_t turn_light_actual = 0;
};

struct BmsReport {
  double battery_voltage = 0.0;
  double battery_current = 0.0;
  double battery_soc = 0.0;
};

// ═══════════════════════════════════════════════════════════════════════════
// ENCODE FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Encode 0x100 Throttle_Command — SUM checksum
 *
 * Layout (from hooke2_interface throttle_command_100.cpp):
 *   byte0 bit0: Dirve_EnCtrl
 *   bytes 1-2:  Dirve_Acc (10-bit, scale 0.01 m/s², Motorola at bit 15)
 *   bytes 3-4:  Dirve_ThrottlePedalTarget (16-bit, scale 0.1%, Motorola at bit 31)
 *   bytes 5-6:  Vel_Target (10-bit, scale 0.01 m/s, Motorola at bit 47) — value/4.0
 *   byte7:      SUM checksum
 */
void encode_throttle(const ThrottleCmd& cmd, uint8_t* data);

/**
 * @brief Encode 0x101 Brake_Command — XOR checksum
 *
 * Layout (from hooke2_interface brake_command_101.cpp):
 *   byte0 bit0: Brake_EnCtrl
 *   byte0 bit1: AEB_EnCtrl
 *   bytes 1-2:  Brake_Dec (10-bit, scale 0.01, Motorola at bit 15)
 *   bytes 3-4:  Brake_Pedal_Target (16-bit, scale 0.1%, Motorola at bit 31)
 *   byte7:      XOR checksum
 */
void encode_brake(const BrakeCmd& cmd, uint8_t* data);

/**
 * @brief Encode 0x102 Steering_Command — XOR checksum
 *
 * Layout (from hooke2_interface steering_command_102.cpp):
 *   byte0 bit0: Steer_EnCtrl
 *   byte1:      Steer_AngleSpeed (0-250 deg/s)
 *   bytes 3-4:  Steer_AngleTarget (raw = angle + 500, Motorola at bit 31)
 *   byte7:      XOR checksum
 */
void encode_steering(const SteeringCmd& cmd, uint8_t* data);

/**
 * @brief Encode 0x103 Gear_Command — SUM checksum (CRITICAL: must be SUM, not XOR)
 *
 * Layout (from hooke2_interface gear_command_103.cpp):
 *   byte0 bit0:    Gear_EnCtrl
 *   byte1 bits[2:0]: Gear_Target (1=PARK, 2=REVERSE, 3=NEUTRAL, 4=DRIVE)
 *   byte7:         SUM checksum
 */
void encode_gear(const GearCmd& cmd, uint8_t* data);

/**
 * @brief Encode 0x104 Park_Command — SUM checksum
 *
 * Layout (from hooke2_interface park_command_104.cpp):
 *   byte0 bit0: Park_EnCtrl
 *   byte1 bit0: Park_Target (0=RELEASE, 1=ENGAGE)
 *   byte7:      SUM checksum
 */
void encode_park(const ParkCmd& cmd, uint8_t* data);

/**
 * @brief Encode 0x105 Vehicle_Mode_Command — SUM checksum
 *
 * Layout (from hooke2_interface vehicle_mode_command_105.cpp):
 *   byte0 bits[2:0]: Steer_ModeCtrl
 *   byte0 bit7:      Auto_Professional (ALWAYS 1 — v6 critical fix)
 *   byte1 bits[2:0]: Drive_ModeCtrl
 *   byte2 bits[1:0]: TurnLight_Ctrl
 *   byte3 bit0:      VIN_Req
 *   byte7:           SUM checksum
 */
void encode_vcu_mode(const VcuModeCmd& cmd, uint8_t* data);

// ═══════════════════════════════════════════════════════════════════════════
// DECODE FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void decode_throttle_report(const uint8_t* data, ThrottleReport& rpt);
void decode_brake_report(const uint8_t* data, BrakeReport& rpt);
void decode_steering_report(const uint8_t* data, SteeringReport& rpt);
void decode_gear_report(const uint8_t* data, GearReport& rpt);
void decode_park_report(const uint8_t* data, ParkReport& rpt);
void decode_vcu_report(const uint8_t* data, VcuReport& rpt);
void decode_bms_report(const uint8_t* data, BmsReport& rpt);

}  // namespace pix_can_codec

/**
 * @file encoder.cpp
 * @brief Hand-written CAN frame encoders for Hooke2 VCU protocol.
 *
 * Each function implements the exact byte layout verified on hardware.
 * Reference: hooke2_interface/src/common/ command files
 *
 * No cantools, no DBC parsing — pure byte manipulation.
 */

#include "pix_can_codec/can_codec.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>

namespace pix_can_codec {

// Helper: clamp value to [lo, hi]
template<typename T>
static T clamp(T val, T lo, T hi) {
  return std::max(lo, std::min(hi, val));
}

// ═══════════════════════════════════════════════════════════════════════════
// 0x100 Throttle_Command — SUM checksum
// ═══════════════════════════════════════════════════════════════════════════
void encode_throttle(const ThrottleCmd& cmd, uint8_t* data)
{
  std::memset(data, 0, CAN_FRAME_LEN);

  // byte0 bit0: Dirve_EnCtrl
  if (cmd.enable) {
    data[0] |= 0x01;
  }

  // Dirve_Acc: 10-bit, scale 0.01, Motorola at bit 15
  // byte1 = upper 8 bits, byte2 bits[7:6] = lower 2 bits
  double acc = clamp(cmd.accel_ms2, 0.0, 10.0);
  int acc_raw = static_cast<int>(acc / 0.01);
  acc_raw = clamp(acc_raw, 0, 0x3FF);
  data[1] = (acc_raw >> 2) & 0xFF;
  data[2] = (acc_raw & 0x03) << 6;

  // Dirve_ThrottlePedalTarget: 16-bit, scale 0.1, Motorola at bit 31
  // byte3 = upper 8 bits, byte4 = lower 8 bits
  double pedal = clamp(cmd.pedal_pct, 0.0, 100.0);
  int pedal_raw = static_cast<int>(pedal / 0.1);
  pedal_raw = clamp(pedal_raw, 0, 0xFFFF);
  data[3] = (pedal_raw >> 8) & 0xFF;
  data[4] = pedal_raw & 0xFF;

  // Vel_Target: 10-bit, scale 0.01, Motorola at bit 47
  // Reference: value / 4.0 before encoding (from throttle_command_100.cpp)
  // byte5 = upper 8 bits, byte6 bits[7:6] = lower 2 bits
  double speed = clamp(cmd.speed_ms / 4.0, 0.0, 10.23);
  int spd_raw = static_cast<int>(speed / 0.01);
  spd_raw = clamp(spd_raw, 0, 0x3FF);
  data[5] = (spd_raw >> 2) & 0xFF;
  data[6] = (spd_raw & 0x03) << 6;

  // SUM checksum
  data[7] = checksum_sum(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// 0x101 Brake_Command — XOR checksum
// ═══════════════════════════════════════════════════════════════════════════
void encode_brake(const BrakeCmd& cmd, uint8_t* data)
{
  std::memset(data, 0, CAN_FRAME_LEN);

  // byte0 bit0: Brake_EnCtrl
  if (cmd.enable) {
    data[0] |= 0x01;
  }
  // byte0 bit1: AEB_EnCtrl
  if (cmd.aeb_enable) {
    data[0] |= 0x02;
  }

  // Brake_Dec: 10-bit, scale 0.01, Motorola at bit 15
  // byte1 = upper 8 bits, byte2 bits[7:6] = lower 2 bits
  double dec = clamp(cmd.decel_ms2, 1.0, 10.0);
  int dec_raw = static_cast<int>(dec / 0.01);
  dec_raw = clamp(dec_raw, 0, 0x3FF);
  data[1] = (dec_raw >> 2) & 0xFF;
  data[2] = (dec_raw & 0x03) << 6;

  // Brake_Pedal_Target: 16-bit, scale 0.1, Motorola at bit 31
  // byte3 = upper 8 bits, byte4 = lower 8 bits
  double pedal = clamp(cmd.pedal_pct, 0.0, 100.0);
  int pedal_raw = static_cast<int>(pedal / 0.1);
  pedal_raw = clamp(pedal_raw, 0, 0xFFFF);
  data[3] = (pedal_raw >> 8) & 0xFF;
  data[4] = pedal_raw & 0xFF;

  // XOR checksum
  data[7] = checksum_xor(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// 0x102 Steering_Command — XOR checksum
// ═══════════════════════════════════════════════════════════════════════════
void encode_steering(const SteeringCmd& cmd, uint8_t* data)
{
  std::memset(data, 0, CAN_FRAME_LEN);

  // byte0 bit0: Steer_EnCtrl
  if (cmd.enable) {
    data[0] |= 0x01;
  }

  // Steer_AngleSpeed: 8-bit, byte1 (0-250 deg/s)
  int spd = clamp(cmd.angle_speed, 0, 250);
  data[1] = static_cast<uint8_t>(spd);

  // Steer_AngleTarget: 16-bit, offset -500, Motorola at bit 31
  // raw = angle + 500, clamped to [0, 1000] (physical range [-440, 440])
  int angle = clamp(static_cast<int>(std::round(cmd.angle_deg)), -440, 440);
  int raw = angle + 500;
  raw = clamp(raw, 0, 1000);
  // byte3 = upper 8 bits, byte4 = lower 8 bits (Motorola order)
  data[3] = (raw >> 8) & 0xFF;
  data[4] = raw & 0xFF;

  // XOR checksum
  data[7] = checksum_xor(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// 0x103 Gear_Command — SUM checksum (CRITICAL: must be SUM, not XOR)
// ═══════════════════════════════════════════════════════════════════════════
void encode_gear(const GearCmd& cmd, uint8_t* data)
{
  std::memset(data, 0, CAN_FRAME_LEN);

  // byte0 bit0: Gear_EnCtrl
  if (cmd.enable) {
    data[0] |= 0x01;
  }

  // byte1 bits[2:0]: Gear_Target
  data[1] = cmd.gear_target & 0x07;

  // SUM checksum — CRITICAL: not XOR
  // XOR would produce 0x02 for NEUTRAL, VCU silently rejects
  data[7] = checksum_sum(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// 0x104 Park_Command — SUM checksum
// ═══════════════════════════════════════════════════════════════════════════
void encode_park(const ParkCmd& cmd, uint8_t* data)
{
  std::memset(data, 0, CAN_FRAME_LEN);

  // byte0 bit0: Park_EnCtrl
  if (cmd.enable) {
    data[0] |= 0x01;
  }

  // byte1 bit0: Park_Target (0=RELEASE, 1=ENGAGE)
  if (cmd.engage) {
    data[1] = 0x01;
  }

  // SUM checksum
  data[7] = checksum_sum(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// 0x105 Vehicle_Mode_Command — SUM checksum
// ═══════════════════════════════════════════════════════════════════════════
void encode_vcu_mode(const VcuModeCmd& cmd, uint8_t* data)
{
  std::memset(data, 0, CAN_FRAME_LEN);

  // byte0: Steer_ModeCtrl bits[2:0] + Auto_Professional bit7
  data[0] = cmd.steer_mode & 0x07;
  if (cmd.auto_professional) {
    data[0] |= 0x80;  // bit7 = Auto_Professional = ALWAYS 1
  }

  // byte1 bits[2:0]: Drive_ModeCtrl
  data[1] = cmd.drive_mode & 0x07;

  // byte2 bits[1:0]: TurnLight_Ctrl
  data[2] = cmd.turn_light & 0x03;

  // byte3 bit0: VIN_Req
  if (cmd.vin_req) {
    data[3] |= 0x01;
  }

  // SUM checksum
  data[7] = checksum_sum(data);
}

}  // namespace pix_can_codec

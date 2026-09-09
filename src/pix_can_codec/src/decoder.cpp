/**
 * @file decoder.cpp
 * @brief Hand-written CAN frame decoders for Hooke2 VCU report messages.
 */

#include "pix_can_codec/can_codec.hpp"
#include <cstring>

namespace pix_can_codec {

// 0x500 Throttle_Report
void decode_throttle_report(const uint8_t* data, ThrottleReport& rpt)
{
  // Dirve_EnState: 1|2@0+ (byte0 bits[1:0])
  rpt.en_state = data[0] & 0x03;
  // Dirve_Flt1: 15|8@0+ (byte1)
  rpt.flt1 = data[1];
  // Dirve_Flt2: 23|8@0+ (byte2)
  rpt.flt2 = data[2];
  // Dirve_ThrottlePedalActual: 31|16@0+ (bytes 3-4), scale 0.1%
  int pedal_raw = (static_cast<int>(data[3]) << 8) | data[4];
  rpt.throttle_pedal_actual = pedal_raw * 0.1;
  // Speed is not present in 0x500
  rpt.speed_actual = 0.0;
}

// 0x501 Brake_Report
void decode_brake_report(const uint8_t* data, BrakeReport& rpt)
{
  // Brake_EnState: 1|2@0+ (byte0 bits[1:0])
  rpt.en_state = data[0] & 0x03;
  // Brake_Flt1: 15|8@0+ (byte1)
  rpt.flt1 = data[1];
  // Brake_Flt2: 23|8@0+ (byte2)
  rpt.flt2 = data[2];
  // Brake_PedalActual: 31|16@0+ (bytes 3-4), scale 0.1%
  int pedal_raw = (static_cast<int>(data[3]) << 8) | data[4];
  rpt.brake_pedal_actual = pedal_raw * 0.1;
}

// 0x502 Steering_Report
void decode_steering_report(const uint8_t* data, SteeringReport& rpt)
{
  // Steer_EnState: 1|2@0+ (byte0 bits[1:0])
  rpt.en_state = data[0] & 0x03;
  // Steer_Flt1: 15|8@0+ (byte1)
  rpt.flt1 = data[1];
  // Steer_Flt2: 23|8@0+ (byte2)
  rpt.flt2 = data[2];
  // Steer_AngleActual: 31|16@0+ (bytes 3-4), offset -500
  int raw = (static_cast<int>(data[3]) << 8) | data[4];
  rpt.steer_angle_actual = static_cast<double>(raw) - 500.0;
  // Steer_AngleSpeedActual: 63|8@0+ (byte7)
  rpt.steer_speed_actual = static_cast<double>(data[7]);
}

// 0x503 Gear_Report
void decode_gear_report(const uint8_t* data, GearReport& rpt)
{
  // Gear_Actual: 2|3@0+ (byte0 bits[2:0])
  rpt.gear_actual = data[0] & 0x07;
  // Gear_Flt: 15|8@0+ (byte1)
  rpt.gear_flt = data[1];
}

// 0x504 Park_Report
void decode_park_report(const uint8_t* data, ParkReport& rpt)
{
  // Parking_Actual: 0|1@0+ (byte0 bit0)
  rpt.park_actual = data[0] & 0x01;
  // Park_Flt: 15|8@0+ (byte1)
  rpt.park_flt = data[1];
}

// 0x505 VCU_Report
void decode_vcu_report(const uint8_t* data, VcuReport& rpt)
{
  // Vehicle_Acc: 7|12@0- (signed, bytes 0 and 1 upper nibble)
  // Scale 0.01
  int16_t acc_raw = static_cast<int16_t>((static_cast<int>(data[0]) << 8) | (data[1] & 0xF0));
  acc_raw = acc_raw >> 4; // Shift down the 4 bits of padding
  rpt.vehicle_accel = acc_raw * 0.01;

  // Vehicle_Speed: 23|16@0- (signed, bytes 2 and 3)
  // Scale 0.001
  int16_t spd_raw = static_cast<int16_t>((static_cast<int>(data[2]) << 8) | data[3]);
  rpt.vehicle_speed = spd_raw * 0.001;

  // Vehicle_ModeState: 36|2@0+ (byte4 bits[5:4])
  rpt.vehicle_mode_state = (data[4] >> 3) & 0x03;

  // Drive_ModeStatus: 39|3@0+ (byte4 bits[7:5]) - Wait! DBC says 39|3@0+. 
  // Bit 39 is byte4 bit7. Length 3 means bits 7, 6, 5!
  rpt.drive_mode_status = (data[4] >> 5) & 0x07;

  // Steer_ModeStatus: 10|3@0+ (byte1 bits[2:0])
  rpt.steer_mode_status = data[1] & 0x07;

  // Vehicle_FrontCrashState: 33|1@0+ (byte4 bit1)
  rpt.front_crash = (data[4] >> 1) & 0x01;

  // BackCrash_State: 34|1@0+ (byte4 bit2)
  rpt.back_crash = (data[4] >> 2) & 0x01;

  // AEB_BrakeState: 32|1@0+ (byte4 bit0)
  rpt.aeb_active = data[4] & 0x01;

  // Brake_LightActual: 11|1@0+ (byte1 bit3)
  rpt.brake_light = (data[1] >> 3) & 0x01;

  // TurnLight_Actual: 57|2@0+ (byte7 bits[1:0])
  rpt.turn_light_actual = data[7] & 0x03;
}

// 0x512 BMS_Report
void decode_bms_report(const uint8_t* data, BmsReport& rpt)
{
  // Battery_Voltage: 7|16@0+ (bytes 0-1) scale 0.01
  int v_raw = (static_cast<int>(data[0]) << 8) | data[1];
  rpt.battery_voltage = v_raw * 0.01;

  // Battery_Current: 23|16@0+ (bytes 2-3) scale 0.1, offset -3200
  int i_raw = (static_cast<int>(data[2]) << 8) | data[3];
  rpt.battery_current = (i_raw * 0.1) - 3200.0;

  // Battery_Soc: 39|8@0+ (byte4) scale 1
  rpt.battery_soc = data[4];
}

}  // namespace pix_can_codec

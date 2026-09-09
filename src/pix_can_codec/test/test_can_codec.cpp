/**
 * @file test_can_codec.cpp
 * @brief Unit tests for pix_can_codec — the hand-written CAN encode/decode module.
 *
 * Tests the exact byte sequences verified on hardware. Every bug in the
 * regression matrix (Auto_Professional always 1, gear SUM not XOR, byte offsets)
 * is a real assertion here.
 *
 * Run: colcon test --packages-select pix_can_codec
 */

#include <gtest/gtest.h>
#include "pix_can_codec/can_codec.hpp"

namespace pix = pix_can_codec;

// ═══════════════════════════════════════════════════════════════════════════
// Checksum Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(Checksum, SumOfZeros)
{
  uint8_t data[7] = {0};
  EXPECT_EQ(pix::checksum_sum(data), 0);
}

TEST(Checksum, SumBasic)
{
  uint8_t data[7] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
  // 0x01 + 0x04 = 0x05
  EXPECT_EQ(pix::checksum_sum(data), 0x05);
}

TEST(Checksum, XorBasic)
{
  uint8_t data[7] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
  // 0x01 ^ 0x04 = 0x05
  EXPECT_EQ(pix::checksum_xor(data), 0x05);
}

TEST(Checksum, SumVsXorDifference)
{
  // For gear NEUTRAL (enable=1, target=3):
  // data = [0x01, 0x03, 0, 0, 0, 0, 0]
  // SUM = 0x01 + 0x03 = 0x04
  // XOR = 0x01 ^ 0x03 = 0x02
  // CRITICAL: VCU expects SUM=0x04 for gear; XOR=0x02 is silently rejected
  uint8_t data[7] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(pix::checksum_sum(data), 0x04);
  EXPECT_EQ(pix::checksum_xor(data), 0x02);
  EXPECT_NE(pix::checksum_sum(data), pix::checksum_xor(data));
}

// ═══════════════════════════════════════════════════════════════════════════
// Throttle Encode Tests (0x100)
// ═══════════════════════════════════════════════════════════════════════════

TEST(EncodeThrottle, EnableBit)
{
  uint8_t data[8];
  pix::ThrottleCmd cmd;
  cmd.enable = true;
  pix::encode_throttle(cmd, data);
  EXPECT_EQ(data[0] & 0x01, 0x01);
}

TEST(EncodeThrottle, DisabledAllZeros)
{
  uint8_t data[8];
  pix::ThrottleCmd cmd;
  cmd.enable = false;
  cmd.speed_ms = 0.0;
  cmd.accel_ms2 = 0.0;
  cmd.pedal_pct = 0.0;
  pix::encode_throttle(cmd, data);
  // bytes 0-6 all zero → checksum SUM = 0
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(data[i], 0) << "byte " << i;
  }
  EXPECT_EQ(data[7], 0); // SUM of zeros
}

TEST(EncodeThrottle, ChecksumIsSUM)
{
  uint8_t data[8];
  pix::ThrottleCmd cmd;
  cmd.enable = true;
  cmd.accel_ms2 = 1.0;
  pix::encode_throttle(cmd, data);
  // Verify checksum is SUM, not XOR
  uint8_t expected_sum = 0;
  for (int i = 0; i < 7; i++) expected_sum += data[i];
  EXPECT_EQ(data[7], expected_sum);
}

// ═══════════════════════════════════════════════════════════════════════════
// Brake Encode Tests (0x101)
// ═══════════════════════════════════════════════════════════════════════════

TEST(EncodeBrake, EnableBit)
{
  uint8_t data[8];
  pix::BrakeCmd cmd;
  cmd.enable = true;
  pix::encode_brake(cmd, data);
  EXPECT_EQ(data[0] & 0x01, 0x01);
}

TEST(EncodeBrake, AEBBit)
{
  uint8_t data[8];
  pix::BrakeCmd cmd;
  cmd.aeb_enable = true;
  pix::encode_brake(cmd, data);
  EXPECT_EQ(data[0] & 0x02, 0x02);
}

TEST(EncodeBrake, ChecksumIsXOR)
{
  uint8_t data[8];
  pix::BrakeCmd cmd;
  cmd.enable = true;
  cmd.pedal_pct = 50.0;
  pix::encode_brake(cmd, data);
  // Verify checksum is XOR, not SUM
  uint8_t expected_xor = 0;
  for (int i = 0; i < 7; i++) expected_xor ^= data[i];
  EXPECT_EQ(data[7], expected_xor);
}

// ═══════════════════════════════════════════════════════════════════════════
// Steering Encode Tests (0x102)
// ═══════════════════════════════════════════════════════════════════════════

TEST(EncodeSteering, AngleZeroCentered)
{
  uint8_t data[8];
  pix::SteeringCmd cmd;
  cmd.enable = true;
  cmd.angle_deg = 0.0;
  cmd.angle_speed = 100;
  pix::encode_steering(cmd, data);
  // raw = 0 + 500 = 500 = 0x01F4
  // byte3 = 0x01, byte4 = 0xF4
  EXPECT_EQ(data[3], 0x01);
  EXPECT_EQ(data[4], 0xF4);
}

TEST(EncodeSteering, ChecksumIsXOR)
{
  uint8_t data[8];
  pix::SteeringCmd cmd;
  cmd.enable = true;
  cmd.angle_deg = 100.0;
  pix::encode_steering(cmd, data);
  uint8_t expected_xor = 0;
  for (int i = 0; i < 7; i++) expected_xor ^= data[i];
  EXPECT_EQ(data[7], expected_xor);
}

// ═══════════════════════════════════════════════════════════════════════════
// Gear Encode Tests (0x103) — CRITICAL BUG REGRESSION
// ═══════════════════════════════════════════════════════════════════════════

TEST(EncodeGear, NeutralPayload)
{
  uint8_t data[8];
  pix::GearCmd cmd;
  cmd.enable = true;
  cmd.gear_target = pix::GEAR_NEUTRAL;
  pix::encode_gear(cmd, data);

  // Expected: [0x01, 0x03, 0, 0, 0, 0, 0, 0x04]
  EXPECT_EQ(data[0], 0x01);
  EXPECT_EQ(data[1], 0x03);
  for (int i = 2; i < 7; i++) EXPECT_EQ(data[i], 0);
  EXPECT_EQ(data[7], 0x04); // SUM = 0x01 + 0x03 = 0x04
}

TEST(EncodeGear, DrivePayload)
{
  uint8_t data[8];
  pix::GearCmd cmd;
  cmd.enable = true;
  cmd.gear_target = pix::GEAR_DRIVE;
  pix::encode_gear(cmd, data);

  // Expected: [0x01, 0x04, 0, 0, 0, 0, 0, 0x05]
  EXPECT_EQ(data[0], 0x01);
  EXPECT_EQ(data[1], 0x04);
  EXPECT_EQ(data[7], 0x05); // SUM = 0x01 + 0x04 = 0x05
}

TEST(EncodeGear, ParkPayload)
{
  uint8_t data[8];
  pix::GearCmd cmd;
  cmd.enable = true;
  cmd.gear_target = pix::GEAR_PARK;
  pix::encode_gear(cmd, data);

  // Expected: [0x01, 0x01, 0, 0, 0, 0, 0, 0x02]
  EXPECT_EQ(data[0], 0x01);
  EXPECT_EQ(data[1], 0x01);
  EXPECT_EQ(data[7], 0x02); // SUM = 0x01 + 0x01 = 0x02
}

TEST(EncodeGear, ChecksumIsSUMNotXOR)
{
  // CRITICAL regression test: gear checksum must be SUM, not XOR
  // For NEUTRAL: SUM=0x04, XOR=0x02. VCU rejects XOR=0x02.
  uint8_t data[8];
  pix::GearCmd cmd;
  cmd.enable = true;
  cmd.gear_target = pix::GEAR_NEUTRAL;
  pix::encode_gear(cmd, data);

  uint8_t sum = pix::checksum_sum(data);
  uint8_t xor_val = pix::checksum_xor(data);

  EXPECT_EQ(data[7], sum);      // Must use SUM
  EXPECT_NE(data[7], xor_val);  // Must NOT use XOR (they differ for NEUTRAL)
}

// ═══════════════════════════════════════════════════════════════════════════
// Park Encode Tests (0x104)
// ═══════════════════════════════════════════════════════════════════════════

TEST(EncodePark, EngagePayload)
{
  uint8_t data[8];
  pix::ParkCmd cmd;
  cmd.enable = true;
  cmd.engage = true;
  pix::encode_park(cmd, data);

  EXPECT_EQ(data[0], 0x01);
  EXPECT_EQ(data[1], 0x01);
  EXPECT_EQ(data[7], 0x02); // SUM
}

TEST(EncodePark, ReleasePayload)
{
  uint8_t data[8];
  pix::ParkCmd cmd;
  cmd.enable = true;
  cmd.engage = false;
  pix::encode_park(cmd, data);

  EXPECT_EQ(data[0], 0x01);
  EXPECT_EQ(data[1], 0x00);
  EXPECT_EQ(data[7], 0x01); // SUM
}

// ═══════════════════════════════════════════════════════════════════════════
// VCU Mode Encode Tests (0x105) — Auto_Professional ALWAYS 1
// ═══════════════════════════════════════════════════════════════════════════

TEST(EncodeVcuMode, AutoProfessionalAlwaysSet)
{
  uint8_t data[8];
  pix::VcuModeCmd cmd;
  cmd.auto_professional = true; // MUST be true always
  cmd.drive_mode = 1;           // SPEED_DRIVE
  pix::encode_vcu_mode(cmd, data);

  // byte0 bit7 must be 1 (Auto_Professional)
  EXPECT_EQ(data[0] & 0x80, 0x80);
}

TEST(EncodeVcuMode, DefaultPayload)
{
  uint8_t data[8];
  pix::VcuModeCmd cmd;  // defaults: auto_professional=true, drive_mode=1
  pix::encode_vcu_mode(cmd, data);

  // Expected: byte0=0x80, byte1=0x01 → 
  // Matches verified frame: can4 105 [8] 80 01 00 00 00 00 00 81
  EXPECT_EQ(data[0], 0x80);
  EXPECT_EQ(data[1], 0x01);
  EXPECT_EQ(data[7], 0x81); // SUM = 0x80 + 0x01 = 0x81
}

TEST(EncodeVcuMode, ChecksumIsSUM)
{
  uint8_t data[8];
  pix::VcuModeCmd cmd;
  pix::encode_vcu_mode(cmd, data);
  uint8_t expected = pix::checksum_sum(data);
  EXPECT_EQ(data[7], expected);
}

// ═══════════════════════════════════════════════════════════════════════════
// Decode Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(DecodeGear, ParkReport)
{
  // Simulate 0x503 with Gear_Actual = 1 (PARK)
  uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  pix::GearReport rpt;
  pix::decode_gear_report(data, rpt);
  EXPECT_EQ(rpt.gear_actual, pix::GEAR_PARK);
}

TEST(DecodeGear, DriveReport)
{
  uint8_t data[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  pix::GearReport rpt;
  pix::decode_gear_report(data, rpt);
  EXPECT_EQ(rpt.gear_actual, pix::GEAR_DRIVE);
}

TEST(DecodePark, EngagedReport)
{
  uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  pix::ParkReport rpt;
  pix::decode_park_report(data, rpt);
  EXPECT_EQ(rpt.park_actual, pix::PARK_ENGAGE);
}

TEST(DecodePark, ReleasedReport)
{
  uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  pix::ParkReport rpt;
  pix::decode_park_report(data, rpt);
  EXPECT_EQ(rpt.park_actual, pix::PARK_RELEASE);
}

TEST(DecodeSteering, ZeroAngle)
{
  // raw = 500 → angle = 0
  // 500 = 0x01F4 → byte2=0x01, byte3=0xF4
  uint8_t data[8] = {0x01, 0x00, 0x01, 0xF4, 0x64, 0x00, 0x00, 0x00};
  pix::SteeringReport rpt;
  pix::decode_steering_report(data, rpt);
  EXPECT_DOUBLE_EQ(rpt.steer_angle_actual, 0.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// Encode-Decode Roundtrip Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(Roundtrip, GearAllValues)
{
  for (uint8_t gear = 0; gear <= 4; gear++) {
    uint8_t data[8];
    pix::GearCmd cmd;
    cmd.enable = true;
    cmd.gear_target = gear;
    pix::encode_gear(cmd, data);

    pix::GearReport rpt;
    // Simulate report with same gear value in byte0 bits[2:0]
    uint8_t report_data[8] = {gear, 0, 0, 0, 0, 0, 0, 0};
    pix::decode_gear_report(report_data, rpt);
    EXPECT_EQ(rpt.gear_actual, gear) << "gear value: " << static_cast<int>(gear);
  }
}

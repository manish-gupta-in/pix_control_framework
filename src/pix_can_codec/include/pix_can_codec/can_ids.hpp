#pragma once
/**
 * @file can_ids.hpp
 * @brief CAN message ID constants for the Hooke2 VCU protocol.
 *
 * Command IDs: 0x100–0x105 (Laptop → VCU)
 * Report IDs:  0x500–0x512 (VCU → Laptop)
 *
 * Reference: hooke2_interface MessageID enum + pix_framework_complete_reference.md
 */

#include <cstdint>

namespace pix_can_codec {

// ─── TX Command IDs ─────────────────────────────────────────────────────────
constexpr uint32_t CAN_ID_THROTTLE_CMD   = 0x100;
constexpr uint32_t CAN_ID_BRAKE_CMD      = 0x101;
constexpr uint32_t CAN_ID_STEERING_CMD   = 0x102;
constexpr uint32_t CAN_ID_GEAR_CMD       = 0x103;
constexpr uint32_t CAN_ID_PARK_CMD       = 0x104;
constexpr uint32_t CAN_ID_VCU_MODE_CMD   = 0x105;

// ─── RX Report IDs ──────────────────────────────────────────────────────────
constexpr uint32_t CAN_ID_THROTTLE_RPT   = 0x500;
constexpr uint32_t CAN_ID_BRAKE_RPT      = 0x501;
constexpr uint32_t CAN_ID_STEERING_RPT   = 0x502;
constexpr uint32_t CAN_ID_GEAR_RPT       = 0x503;
constexpr uint32_t CAN_ID_PARK_RPT       = 0x504;
constexpr uint32_t CAN_ID_VCU_RPT        = 0x505;
constexpr uint32_t CAN_ID_WHEELSPEED_RPT = 0x506;
constexpr uint32_t CAN_ID_BMS_RPT        = 0x512;

// ─── Gear enum ──────────────────────────────────────────────────────────────
constexpr uint8_t GEAR_INVALID  = 0;
constexpr uint8_t GEAR_PARK     = 1;
constexpr uint8_t GEAR_REVERSE  = 2;
constexpr uint8_t GEAR_NEUTRAL  = 3;
constexpr uint8_t GEAR_DRIVE    = 4;

// ─── Park enum ──────────────────────────────────────────────────────────────
constexpr uint8_t PARK_RELEASE  = 0;
constexpr uint8_t PARK_ENGAGE   = 1;

// ─── CAN frame constants ────────────────────────────────────────────────────
constexpr size_t CAN_FRAME_LEN = 8;

}  // namespace pix_can_codec

#pragma once
/**
 * @file checksum.hpp
 * @brief Checksum algorithms for Hooke2 CAN protocol.
 *
 * Two checksum types per the hardware-verified protocol:
 *   SUM: 0x100 Throttle, 0x103 Gear, 0x104 Park, 0x105 VCU Mode
 *   XOR: 0x101 Brake, 0x102 Steering
 *
 * CRITICAL: Using XOR for 0x103 (Gear) sends checksum 0x02 instead of SUM 0x04.
 * The VCU silently rejects the frame. This was a real bug (v6 fix).
 */

#include <cstdint>
#include <cstddef>

namespace pix_can_codec {

/**
 * @brief SUM checksum: sum of bytes[0..6] mod 256, placed in byte[7].
 * Used by: 0x100 Throttle, 0x103 Gear, 0x104 Park, 0x105 VCU Mode
 */
inline uint8_t checksum_sum(const uint8_t* data, size_t len = 7)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  return sum;
}

/**
 * @brief XOR checksum: XOR of bytes[0..6], placed in byte[7].
 * Used by: 0x101 Brake, 0x102 Steering
 */
inline uint8_t checksum_xor(const uint8_t* data, size_t len = 7)
{
  uint8_t cs = 0;
  for (size_t i = 0; i < len; ++i) {
    cs ^= data[i];
  }
  return cs;
}

}  // namespace pix_can_codec

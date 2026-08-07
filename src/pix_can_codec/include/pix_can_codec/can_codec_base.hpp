#ifndef PIX_CAN_CODEC__CAN_CODEC_BASE_HPP_
#define PIX_CAN_CODEC__CAN_CODEC_BASE_HPP_

#include <vector>
#include "can_msgs/msg/frame.hpp"
#include "pix_msgs/msg/vehicle_command.hpp"
#include "pix_msgs/msg/vehicle_state.hpp"

namespace pix_control_framework
{
namespace can_codec
{

class CanCodecBase
{
public:
  virtual ~CanCodecBase() = default;

  // Encode DBW command into CAN frames
  virtual std::vector<can_msgs::msg::Frame> encode(const pix_msgs::msg::VehicleCommand & command) = 0;

  // Decode CAN frames into Vehicle State
  virtual void decode(const can_msgs::msg::Frame & frame, pix_msgs::msg::VehicleState & state) = 0;
};

}  // namespace can_codec
}  // namespace pix_control_framework

#endif  // PIX_CAN_CODEC__CAN_CODEC_BASE_HPP_

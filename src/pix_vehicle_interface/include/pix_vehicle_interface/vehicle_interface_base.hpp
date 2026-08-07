#ifndef PIX_VEHICLE_INTERFACE__VEHICLE_INTERFACE_BASE_HPP_
#define PIX_VEHICLE_INTERFACE__VEHICLE_INTERFACE_BASE_HPP_

#include "pix_msgs/msg/vehicle_command.hpp"
#include "pix_msgs/msg/vehicle_state.hpp"

namespace pix_control_framework
{
namespace vehicle_interface
{

class VehicleInterfaceBase
{
public:
  virtual ~VehicleInterfaceBase() = default;

  virtual bool init() = 0;
  virtual void send_command(const pix_msgs::msg::VehicleCommand & command) = 0;
  virtual pix_msgs::msg::VehicleState get_state() = 0;
};

}  // namespace vehicle_interface
}  // namespace pix_control_framework

#endif  // PIX_VEHICLE_INTERFACE__VEHICLE_INTERFACE_BASE_HPP_

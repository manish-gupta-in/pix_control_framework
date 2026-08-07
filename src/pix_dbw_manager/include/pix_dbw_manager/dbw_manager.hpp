#ifndef PIX_DBW_MANAGER__DBW_MANAGER_HPP_
#define PIX_DBW_MANAGER__DBW_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pix_msgs/msg/vehicle_command.hpp"
#include "pix_msgs/msg/vehicle_state.hpp"

namespace pix_control_framework {
namespace dbw_manager {
class DbwManager {
public:
  DbwManager();
  ~DbwManager();
};
}
}
#endif

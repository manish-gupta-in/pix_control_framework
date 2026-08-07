#ifndef PIX_SAFETY_MANAGER__SAFETY_MANAGER_HPP_
#define PIX_SAFETY_MANAGER__SAFETY_MANAGER_HPP_
#include "rclcpp/rclcpp.hpp"
#include "pix_msgs/msg/algorithm_command.hpp"
#include "pix_msgs/msg/vehicle_command.hpp"
namespace pix_control_framework {
namespace safety_manager {
class SafetyManager {
public:
  SafetyManager();
  bool check_command(const pix_msgs::msg::AlgorithmCommand & cmd);
};
}
}
#endif

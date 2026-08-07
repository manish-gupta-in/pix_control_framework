#include "pix_safety_manager/safety_manager.hpp"
namespace pix_control_framework {
namespace safety_manager {
SafetyManager::SafetyManager() {}
bool SafetyManager::check_command(const pix_msgs::msg::AlgorithmCommand & cmd) { return true; }
}
}

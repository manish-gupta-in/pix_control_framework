#ifndef PIX_CAN_DRIVER__SOCKETCAN_DRIVER_HPP_
#define PIX_CAN_DRIVER__SOCKETCAN_DRIVER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

namespace pix_control_framework
{
namespace can_driver
{

class SocketCanDriver
{
public:
  explicit SocketCanDriver(const std::string & interface_name);
  ~SocketCanDriver();

  bool init();
  bool write_frame(const can_msgs::msg::Frame & frame);
  bool read_frame(can_msgs::msg::Frame & frame);
  void close_socket();

private:
  std::string interface_name_;
  int socket_fd_;
  bool is_initialized_;
};

}  // namespace can_driver
}  // namespace pix_control_framework

#endif  // PIX_CAN_DRIVER__SOCKETCAN_DRIVER_HPP_

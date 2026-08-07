#include "pix_can_driver/socketcan_driver.hpp"
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace pix_control_framework
{
namespace can_driver
{

SocketCanDriver::SocketCanDriver(const std::string & interface_name)
: interface_name_(interface_name), socket_fd_(-1), is_initialized_(false)
{
}

SocketCanDriver::~SocketCanDriver()
{
  close_socket();
}

bool SocketCanDriver::init()
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    close_socket();
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close_socket();
    return false;
  }

  is_initialized_ = true;
  return true;
}

bool SocketCanDriver::write_frame(const can_msgs::msg::Frame & frame)
{
  if (!is_initialized_) return false;

  struct can_frame tx_frame;
  tx_frame.can_id = frame.id;
  if (frame.is_extended) {
    tx_frame.can_id |= CAN_EFF_FLAG;
  }
  tx_frame.can_dlc = frame.dlc;
  std::memcpy(tx_frame.data, frame.data.data(), frame.dlc);

  int nbytes = write(socket_fd_, &tx_frame, sizeof(struct can_frame));
  return nbytes == sizeof(struct can_frame);
}

bool SocketCanDriver::read_frame(can_msgs::msg::Frame & frame)
{
  if (!is_initialized_) return false;

  struct can_frame rx_frame;
  int nbytes = read(socket_fd_, &rx_frame, sizeof(struct can_frame));

  if (nbytes < 0) {
    return false;
  }

  frame.id = rx_frame.can_id & CAN_ERR_MASK;
  frame.is_extended = (rx_frame.can_id & CAN_EFF_FLAG) != 0;
  frame.is_rtr = (rx_frame.can_id & CAN_RTR_FLAG) != 0;
  frame.is_error = (rx_frame.can_id & CAN_ERR_FLAG) != 0;
  frame.dlc = rx_frame.can_dlc;
  std::memcpy(frame.data.data(), rx_frame.data, rx_frame.can_dlc);

  return true;
}

void SocketCanDriver::close_socket()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  is_initialized_ = false;
}

}  // namespace can_driver
}  // namespace pix_control_framework

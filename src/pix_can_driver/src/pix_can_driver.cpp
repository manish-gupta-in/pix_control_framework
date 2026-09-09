/**
 * @file pix_can_driver.cpp
 * @brief SocketCAN ROS2 driver implementation for PIX framework.
 *
 * This is a simplified, focused version of the production can_driver.
 * It only supports SocketCAN (our vehicle hardware) and implements:
 *   - Raw socket open/close/read/write
 *   - Background RX thread at bus rate
 *   - ROS2 pub/sub for frame transport
 *
 * Topic convention (distinct from hooke2_interface):
 *   /pix_framework/can_tx  — frames FROM VCU → to pix_vehicle_interface_cpp
 *   /pix_framework/can_rx  — frames TO VCU ← from pix_vehicle_interface_cpp
 */

#include "pix_can_driver/pix_can_driver.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace pix_can_driver {

PixCanDriver::PixCanDriver()
: Node("pix_can_driver"),
  socket_fd_(-1),
  is_running_(false)
{
  // Parameters
  can_interface_ = declare_parameter("can_interface", "can4");

  RCLCPP_INFO(get_logger(), "PIX CAN Driver starting on interface: %s", can_interface_.c_str());

  // Open SocketCAN
  if (!open_socket()) {
    RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface: %s", can_interface_.c_str());
    return;
  }

  // ROS2 pub/sub — distinct /pix_framework/ namespace, no collision with other stacks
  can_tx_pub_ = create_publisher<can_msgs::msg::Frame>(
    "/pix_framework/can_tx", rclcpp::QoS{100});

  can_rx_sub_ = create_subscription<can_msgs::msg::Frame>(
    "/pix_framework/can_rx", rclcpp::QoS{100},
    std::bind(&PixCanDriver::on_can_rx, this, std::placeholders::_1));

  // Start RX thread
  is_running_ = true;
  rx_thread_ = std::thread(&PixCanDriver::rx_thread_func, this);

  RCLCPP_INFO(get_logger(), "PIX CAN Driver initialized successfully.");
}

PixCanDriver::~PixCanDriver()
{
  is_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  close_socket();
}

bool PixCanDriver::open_socket()
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Set receive timeout (100ms — allows clean thread shutdown)
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  return true;
}

void PixCanDriver::close_socket()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void PixCanDriver::rx_thread_func()
{
  struct can_frame frame;

  while (is_running_) {
    auto nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
      // Timeout or error — loop continues
      continue;
    }
    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      continue;
    }

    // Publish as ROS2 Frame
    can_msgs::msg::Frame msg;
    msg.header.stamp = now();
    msg.id = frame.can_id & CAN_SFF_MASK;
    msg.dlc = frame.can_dlc;
    msg.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    msg.is_rtr = (frame.can_id & CAN_RTR_FLAG) != 0;
    msg.is_error = (frame.can_id & CAN_ERR_FLAG) != 0;
    std::memcpy(msg.data.data(), frame.data, std::min(static_cast<int>(frame.can_dlc), 8));

    can_tx_pub_->publish(msg);
  }
}

void PixCanDriver::on_can_rx(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  if (socket_fd_ < 0) {
    return;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = msg->id;
  if (msg->is_extended) {
    frame.can_id |= CAN_EFF_FLAG;
  }
  frame.can_dlc = std::min(static_cast<uint8_t>(msg->dlc), static_cast<uint8_t>(8));
  std::memcpy(frame.data, msg->data.data(), frame.can_dlc);

  auto nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
  if (nbytes < 0) {
    if (errno == ENOBUFS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "CAN TX buffer full (errno 105) — consider increasing txqueuelen");
    }
  }
}

}  // namespace pix_can_driver

// ─── Main entry point ────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pix_can_driver::PixCanDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

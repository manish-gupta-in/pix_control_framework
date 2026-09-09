#pragma once
/**
 * @file pix_can_driver.hpp
 * @brief SocketCAN ROS2 driver for the PIX framework.
 *
 * Adapted from the production can_driver package (read-only reference).
 * Uses raw SocketCAN API — no Innodisk/Kvaser/ESD complexity needed.
 *
 * Design:
 *   - Subscribes to /pix_framework/can_rx (outgoing frames to VCU)
 *   - Publishes received frames on /pix_framework/can_tx (incoming from VCU)
 *   - Background thread for CAN RX at bus rate
 */

#ifndef PIX_CAN_DRIVER__PIX_CAN_DRIVER_HPP_
#define PIX_CAN_DRIVER__PIX_CAN_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace pix_can_driver {

class PixCanDriver : public rclcpp::Node
{
public:
  PixCanDriver();
  ~PixCanDriver();

private:
  // Parameters
  std::string can_interface_;   // e.g. "can4"
  int socket_fd_;

  // ROS2 interfaces
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;

  // Background RX thread
  std::thread rx_thread_;
  std::atomic<bool> is_running_;

  // Methods
  bool open_socket();
  void close_socket();
  void rx_thread_func();
  void on_can_rx(const can_msgs::msg::Frame::ConstSharedPtr msg);
};

}  // namespace pix_can_driver

#endif  // PIX_CAN_DRIVER__PIX_CAN_DRIVER_HPP_

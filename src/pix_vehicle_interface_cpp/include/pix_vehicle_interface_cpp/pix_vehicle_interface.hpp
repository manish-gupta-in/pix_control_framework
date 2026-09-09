#pragma once
/**
 * @file pix_vehicle_interface.hpp
 * @brief C++ Vehicle Interface Node for the PIX Control Framework v12.
 *
 * v12 Unified Pipeline: This node is a pure CAN encoder/decoder.
 * It does NOT perform any arbitration, clamping, or freshness comparison.
 * All commands arrive pre-arbitrated and pre-clamped from the single
 * upstream pipeline: arbitrator → safety_manager → /pix/control_cmd.
 *
 * Subscribes to:
 *   /pix/control_cmd         (pix_vehicle_msgs/PixControlCmd — single unified input)
 *   /pix_framework/can_tx    (can_msgs/Frame — from pix_can_driver)
 *
 * Publishes to:
 *   /pix_framework/can_rx    (can_msgs/Frame — to pix_can_driver)
 *   /pix_framework/vehicle_status/steering
 *   /pix_framework/vehicle_status/velocity
 *   /pix_framework/vehicle_status/gear
 *   /pix_framework/control_mode_report
 */

#ifndef PIX_VEHICLE_INTERFACE_CPP__PIX_VEHICLE_INTERFACE_HPP_
#define PIX_VEHICLE_INTERFACE_CPP__PIX_VEHICLE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

#include <pix_control_msgs/msg/gear_report.hpp>
#include <pix_control_msgs/msg/control_mode_report.hpp>
#include <pix_control_msgs/msg/steering_report.hpp>
#include <pix_control_msgs/msg/velocity_report.hpp>
#include <pix_vehicle_msgs/msg/pix_control_cmd.hpp>
#include <pix_vehicle_msgs/msg/pix_vehicle_status.hpp>

#include <pix_can_codec/can_codec.hpp>
#include <pix_can_codec/can_ids.hpp>

#include <mutex>
#include <string>

namespace pix_vehicle_interface_cpp {

class PixVehicleInterface : public rclcpp::Node
{
public:
  PixVehicleInterface();

private:
  // ── Parameters ────────────────────────────────────────────────────────────
  double loop_rate_;          // Hz (default: 50)
  double steering_ratio_;     // wheel-to-tire ratio (used only for report conversion)
  std::string can_interface_; // e.g. "can4"

  // ── Subscribers ───────────────────────────────────────────────────────────
  // Single arbitrated+clamped command topic (from safety_manager output)
  rclcpp::Subscription<pix_vehicle_msgs::msg::PixControlCmd>::SharedPtr control_cmd_sub_;

  // CAN frame input (from pix_can_driver)
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  rclcpp::Publisher<pix_control_msgs::msg::SteeringReport>::SharedPtr steering_rpt_pub_;
  rclcpp::Publisher<pix_control_msgs::msg::VelocityReport>::SharedPtr velocity_rpt_pub_;
  rclcpp::Publisher<pix_control_msgs::msg::GearReport>::SharedPtr gear_rpt_pub_;
  rclcpp::Publisher<pix_control_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<pix_vehicle_msgs::msg::PixVehicleStatus>::SharedPtr legacy_status_pub_;

  // ── Timer ─────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ── State ─────────────────────────────────────────────────────────────────
  std::mutex mutex_;

  pix_vehicle_msgs::msg::PixControlCmd::SharedPtr latest_control_;
  rclcpp::Time control_received_time_;

  // Decoded VCU report state
  pix_can_codec::ThrottleReport throttle_rpt_;
  pix_can_codec::BrakeReport brake_rpt_;
  pix_can_codec::SteeringReport steering_rpt_;
  pix_can_codec::GearReport gear_rpt_;
  pix_can_codec::ParkReport park_rpt_;
  pix_can_codec::VcuReport vcu_rpt_;
  pix_can_codec::BmsReport bms_rpt_;
  bool vcu_report_received_ = false;

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void on_control_cmd(const pix_vehicle_msgs::msg::PixControlCmd::SharedPtr msg);
  void on_can_frame(const can_msgs::msg::Frame::ConstSharedPtr msg);

  // ── Core logic ────────────────────────────────────────────────────────────
  void publish_loop();
  void publish_commands();
  void publish_reports();
  void send_can_frame(uint32_t id, const uint8_t* data);
};

}  // namespace pix_vehicle_interface_cpp

#endif  // PIX_VEHICLE_INTERFACE_CPP__PIX_VEHICLE_INTERFACE_HPP_

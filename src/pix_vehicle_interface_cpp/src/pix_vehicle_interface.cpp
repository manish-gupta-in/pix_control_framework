/**
 * @file pix_vehicle_interface.cpp
 * @brief C++ Vehicle Interface Node implementation.
 *
 * Core 50Hz loop:
 *   1. Read latest control/gear commands
 *   2. Encode to CAN frames via pix_can_codec
 *   3. Publish frames to pix_can_driver
 *
 * CAN RX callback:
 *   1. Decode report frames via pix_can_codec
 *   2. Publish standardized reports on /pix_framework/vehicle_status/ topics
 *
 * Auto_Professional=1 is ALWAYS set in 0x105 — v6 critical fix baked in.
 */

#include "pix_vehicle_interface_cpp/pix_vehicle_interface.hpp"

#include <algorithm>
#include <cmath>

using namespace std::placeholders;

namespace pix_vehicle_interface_cpp {

PixVehicleInterface::PixVehicleInterface()
: Node("pix_vehicle_interface_cpp")
{
  // ── Parameters ──────────────────────────────────────────────────────────
  loop_rate_ = declare_parameter("loop_rate", 50.0);
  steering_ratio_ = declare_parameter("steering_ratio", 16.6);
  can_interface_ = declare_parameter("can_interface", "can4");

  control_received_time_ = now();

  // ── Subscribers ─────────────────────────────────────────────────────────
  control_cmd_sub_ = create_subscription<pix_vehicle_msgs::msg::PixControlCmd>(
    "/pix/control_cmd", rclcpp::QoS{1},
    std::bind(&PixVehicleInterface::on_control_cmd, this, _1));

  can_frame_sub_ = create_subscription<can_msgs::msg::Frame>(
    "/pix_framework/can_tx", rclcpp::QoS{100},
    std::bind(&PixVehicleInterface::on_can_frame, this, _1));

  // ── Publishers ──────────────────────────────────────────────────────────
  can_frame_pub_ = create_publisher<can_msgs::msg::Frame>(
    "/pix_framework/can_rx", rclcpp::QoS{100});

  steering_rpt_pub_ = create_publisher<pix_control_msgs::msg::SteeringReport>(
    "/pix_framework/vehicle_status/steering", rclcpp::QoS{1});

  velocity_rpt_pub_ = create_publisher<pix_control_msgs::msg::VelocityReport>(
    "/pix_framework/vehicle_status/velocity", rclcpp::QoS{1});

  gear_rpt_pub_ = create_publisher<pix_control_msgs::msg::GearReport>(
    "/pix_framework/vehicle_status/gear", rclcpp::QoS{1});

  control_mode_pub_ = create_publisher<pix_control_msgs::msg::ControlModeReport>(
    "/pix_framework/control_mode_report", rclcpp::QoS{1});

  legacy_status_pub_ = create_publisher<pix_vehicle_msgs::msg::PixVehicleStatus>(
    "/pix/vehicle_status", rclcpp::QoS{1});

  // ── Timer ───────────────────────────────────────────────────────────────
  const auto period_ms = std::chrono::milliseconds(
    static_cast<int64_t>(1000.0 / loop_rate_));
  publish_timer_ = create_wall_timer(period_ms,
    std::bind(&PixVehicleInterface::publish_loop, this));

  RCLCPP_INFO(get_logger(), "PIX Vehicle Interface C++ initialized at %.0f Hz", loop_rate_);
}

// ═══════════════════════════════════════════════════════════════════════════
// Callbacks
// ═══════════════════════════════════════════════════════════════════════════

void PixVehicleInterface::on_control_cmd(
  const pix_vehicle_msgs::msg::PixControlCmd::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_control_ = msg;
  control_received_time_ = now();
}

void PixVehicleInterface::on_can_frame(
  const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  // Decode known report IDs
  const uint8_t* data = msg->data.data();

  switch (msg->id) {
    case pix_can_codec::CAN_ID_THROTTLE_RPT:
      pix_can_codec::decode_throttle_report(data, throttle_rpt_);
      break;
    case pix_can_codec::CAN_ID_BRAKE_RPT:
      pix_can_codec::decode_brake_report(data, brake_rpt_);
      break;
    case pix_can_codec::CAN_ID_STEERING_RPT:
      pix_can_codec::decode_steering_report(data, steering_rpt_);
      break;
    case pix_can_codec::CAN_ID_GEAR_RPT:
      pix_can_codec::decode_gear_report(data, gear_rpt_);
      break;
    case pix_can_codec::CAN_ID_PARK_RPT:
      pix_can_codec::decode_park_report(data, park_rpt_);
      break;
    case pix_can_codec::CAN_ID_VCU_RPT:
      pix_can_codec::decode_vcu_report(data, vcu_rpt_);
      vcu_report_received_ = true;
      break;
    case pix_can_codec::CAN_ID_BMS_RPT:
      pix_can_codec::decode_bms_report(data, bms_rpt_);
      break;
    default:
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Core Loop (50 Hz)
// ═══════════════════════════════════════════════════════════════════════════

void PixVehicleInterface::publish_loop()
{
  publish_commands();
  publish_reports();
}

void PixVehicleInterface::publish_commands()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Check command freshness (1s timeout)
  const double control_age = (now() - control_received_time_).seconds();
  const bool control_fresh = (latest_control_ != nullptr && control_age < 1.0);

  // ── 0x105 Vehicle Mode Command (ALWAYS sent) ──────────────────────────
  {
    uint8_t data[8];
    pix_can_codec::VcuModeCmd mode_cmd;
    mode_cmd.auto_professional = true;  // ALWAYS true (v6 fix)
    mode_cmd.drive_mode = 1;            // SPEED_DRIVE
    mode_cmd.steer_mode = 0;           // STANDARD
    pix_can_codec::encode_vcu_mode(mode_cmd, data);
    send_can_frame(pix_can_codec::CAN_ID_VCU_MODE_CMD, data);
  }

  if (!control_fresh) {
    // Not fresh, send safe-zero commands
    // Throttle: zero
    {
      uint8_t data[8];
      pix_can_codec::ThrottleCmd cmd;
      cmd.enable = false;
      pix_can_codec::encode_throttle(cmd, data);
      send_can_frame(pix_can_codec::CAN_ID_THROTTLE_CMD, data);
    }
    // Brake: zero
    {
      uint8_t data[8];
      pix_can_codec::BrakeCmd cmd;
      cmd.enable = false;
      pix_can_codec::encode_brake(cmd, data);
      send_can_frame(pix_can_codec::CAN_ID_BRAKE_CMD, data);
    }
    // Steering: zero
    {
      uint8_t data[8];
      pix_can_codec::SteeringCmd cmd;
      cmd.enable = false;
      pix_can_codec::encode_steering(cmd, data);
      send_can_frame(pix_can_codec::CAN_ID_STEERING_CMD, data);
    }
    return;
  }

  auto& ctrl = *latest_control_;

  // ── 0x102 Steering Command ────────────────────────────────────────────
  {
    uint8_t data[8];
    pix_can_codec::SteeringCmd steer_cmd;
    steer_cmd.enable = ctrl.steer_en;
    steer_cmd.angle_deg = ctrl.steer_target;
    steer_cmd.angle_speed = static_cast<int>(ctrl.steer_speed);
    pix_can_codec::encode_steering(steer_cmd, data);
    send_can_frame(pix_can_codec::CAN_ID_STEERING_CMD, data);
  }

  // ── 0x100 Throttle Command ────────────────────────────────────────────
  {
    uint8_t data[8];
    pix_can_codec::ThrottleCmd throttle_cmd;
    throttle_cmd.enable = ctrl.drive_en;
    throttle_cmd.speed_ms = ctrl.speed_target;
    throttle_cmd.accel_ms2 = ctrl.accel_target;
    pix_can_codec::encode_throttle(throttle_cmd, data);
    send_can_frame(pix_can_codec::CAN_ID_THROTTLE_CMD, data);
  }

  // ── 0x101 Brake Command ───────────────────────────────────────────────
  {
    uint8_t data[8];
    pix_can_codec::BrakeCmd brake_cmd;
    brake_cmd.enable = ctrl.brake_en;
    brake_cmd.decel_ms2 = ctrl.accel_target; // typically same as accel for API matching
    brake_cmd.pedal_pct = ctrl.brake_target;
    pix_can_codec::encode_brake(brake_cmd, data);
    send_can_frame(pix_can_codec::CAN_ID_BRAKE_CMD, data);
  }

  // ── 0x103 Gear Command ────────────────────────────────────────────────
  {
    uint8_t data[8];
    pix_can_codec::GearCmd gear_cmd;
    gear_cmd.enable = ctrl.gear_en;
    gear_cmd.gear_target = ctrl.gear_target;
    pix_can_codec::encode_gear(gear_cmd, data);
    send_can_frame(pix_can_codec::CAN_ID_GEAR_CMD, data);
  }
  
  // ── 0x104 Park Command ────────────────────────────────────────────────
  {
    uint8_t data[8];
    pix_can_codec::ParkCmd park_cmd;
    park_cmd.enable = ctrl.park_en;
    park_cmd.engage = (ctrl.park_target == 1);
    pix_can_codec::encode_park(park_cmd, data);
    send_can_frame(pix_can_codec::CAN_ID_PARK_CMD, data);
  }
}

void PixVehicleInterface::publish_reports()
{
  if (!vcu_report_received_) return;

  auto stamp = now();

  // Steering report
  {
    pix_control_msgs::msg::SteeringReport msg;
    msg.stamp = stamp;
    // Convert from wheel angle [deg] to tire angle [rad]
    msg.steering_tire_angle = (steering_rpt_.steer_angle_actual / steering_ratio_) * M_PI / 180.0;
    steering_rpt_pub_->publish(msg);
  }

  // Velocity report
  {
    pix_control_msgs::msg::VelocityReport msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.longitudinal_velocity = static_cast<float>(vcu_rpt_.vehicle_speed);
    msg.lateral_velocity = 0.0f;
    msg.heading_rate = 0.0f;
    velocity_rpt_pub_->publish(msg);
  }

  // Gear report
  {
    pix_control_msgs::msg::GearReport msg;
    msg.stamp = stamp;
    msg.report = gear_rpt_.gear_actual;
    gear_rpt_pub_->publish(msg);
  }

  // Control mode report
  {
    pix_control_msgs::msg::ControlModeReport msg;
    msg.stamp = stamp;
    bool is_auto = (vcu_rpt_.vehicle_mode_state == 1);  // 1 = Auto Mode
    msg.mode = is_auto
      ? pix_control_msgs::msg::ControlModeReport::AUTONOMOUS
      : pix_control_msgs::msg::ControlModeReport::MANUAL;
    control_mode_pub_->publish(msg);
  }

  // Legacy full status report (for state manager, diagnostics, etc.)
  {
    pix_vehicle_msgs::msg::PixVehicleStatus msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";

    msg.steer_angle = steering_rpt_.steer_angle_actual;
    msg.steer_speed = 0.0f;
    msg.steer_en_state = steering_rpt_.en_state;

    msg.vehicle_speed = static_cast<float>(vcu_rpt_.vehicle_speed);
    msg.vehicle_accel = 0.0f;
    msg.throttle_pedal = throttle_rpt_.throttle_pedal_actual;
    msg.brake_pedal = brake_rpt_.brake_pedal_actual;
    msg.drive_en_state = throttle_rpt_.en_state;
    msg.brake_en_state = brake_rpt_.en_state;

    msg.gear_actual = gear_rpt_.gear_actual;
    msg.park_actual = park_rpt_.park_actual;

    msg.vehicle_mode = vcu_rpt_.vehicle_mode_state;
    msg.drive_mode_status = vcu_rpt_.drive_mode_status;
    msg.steer_mode_status = vcu_rpt_.steer_mode_status;
    msg.front_crash = vcu_rpt_.front_crash;
    msg.back_crash = vcu_rpt_.back_crash;
    msg.aeb_active = vcu_rpt_.aeb_active;
    msg.brake_light_actual = false;
    msg.turn_light_actual = 0;

    msg.steer_flt1 = steering_rpt_.flt1;
    msg.steer_flt2 = steering_rpt_.flt2;
    msg.drive_flt1 = throttle_rpt_.flt1;
    msg.drive_flt2 = throttle_rpt_.flt2;
    msg.brake_flt1 = brake_rpt_.flt1;
    msg.brake_flt2 = brake_rpt_.flt2;
    msg.park_flt = park_rpt_.park_flt;
    msg.gear_flt = gear_rpt_.gear_flt;

    msg.battery_voltage = bms_rpt_.battery_voltage;
    msg.battery_current = bms_rpt_.battery_current;
    msg.battery_soc = bms_rpt_.battery_soc;

    legacy_status_pub_->publish(msg);
  }
}

void PixVehicleInterface::send_can_frame(uint32_t id, const uint8_t* data)
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = now();
  frame.id = id;
  frame.dlc = 8;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.is_error = false;
  std::memcpy(frame.data.data(), data, 8);
  can_frame_pub_->publish(frame);
}

}  // namespace pix_vehicle_interface_cpp

// ─── Main entry point ────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pix_vehicle_interface_cpp::PixVehicleInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

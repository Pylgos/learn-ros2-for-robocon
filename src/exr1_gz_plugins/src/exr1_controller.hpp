#pragma once
#include "quad_omni_drive_controller.hpp"
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

class Exr1Controller : public rclcpp::Node {
  std::optional<QuadOmniDriveController> omni_drive_controller_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Time prev_update_;
  std::array<float, 4> wheel_torques_;

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    omni_drive_controller_->set_target(
        Eigen::Vector2f{msg->linear.x, msg->linear.y}, msg->angular.z);
  }

public:
  Exr1Controller(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
      : Node("exr1_controller", node_options) {
    declare_parameter("wheel_separation", 0.3);
    declare_parameter("wheel_radius", 0.2);
    declare_parameter("mass", 30.0);
    declare_parameter("moment_of_inertia", 2.29);
    declare_parameter("linear_pid.kp", 0.0);
    declare_parameter("linear_pid.ki", 0.0);
    declare_parameter("linear_pid.kd", 0.0);
    declare_parameter("angular_pid.kp", 0.0);
    declare_parameter("angular_pid.ki", 0.0);
    declare_parameter("angular_pid.kd", 0.0);
    declare_parameter("max_torque", 10.0);

    omni_drive_controller_ = QuadOmniDriveController(
        PidGain{
            .kp = (float)get_parameter("linear_pid.kp").as_double(),
            .ki = (float)get_parameter("linear_pid.ki").as_double(),
            .kd = (float)get_parameter("linear_pid.kd").as_double(),
            .max = 10.0,
            .min = -10.0,
            .anti_windup = false,
        },
        PidGain{
            .kp = (float)get_parameter("angular_pid.kp").as_double(),
            .ki = (float)get_parameter("angular_pid.ki").as_double(),
            .kd = (float)get_parameter("angular_pid.kd").as_double(),
            .max = 10.0,
            .min = -10.0,
            .anti_windup = false,
        },
        (float)get_parameter("wheel_radius").as_double(),
        (float)get_parameter("wheel_separation").as_double(),
        (float)get_parameter("mass").as_double(),
        (float)get_parameter("moment_of_inertia").as_double());

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SensorDataQoS(),
        std::bind(&Exr1Controller::on_cmd_vel, this, std::placeholders::_1));

    prev_update_ = get_clock()->now();
  }

  void update(std::array<float, 4> current_wheel_angular_velocities) {
    auto now = get_clock()->now();
    float dt = (now - prev_update_).seconds();
    prev_update_ = now;

    auto [current_linear_velocity, current_angular_velocity] =
        omni_drive_controller_->compute_current_velocity(
            current_wheel_angular_velocities);

    wheel_torques_ = omni_drive_controller_->compute_torque(
        current_linear_velocity, current_angular_velocity, dt);
    float max_torque = get_parameter("max_torque").as_double();
    for (int i = 0; i < 4; i++) {
      wheel_torques_[i] =
          std::clamp(wheel_torques_[i], -max_torque, max_torque);
    }
  }

  std::array<float, 4> get_wheel_torques() { return wheel_torques_; }

private:
};

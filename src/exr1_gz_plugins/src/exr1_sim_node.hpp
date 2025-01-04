#pragma once
#include "exr1_node.hpp"

class Exr1SimNode : public Exr1Node {
  std::optional<QuadOmniDriveController> omni_drive_controller_;
  rclcpp::Time prev_update_;
  std::array<float, 4> wheel_torques_;

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) override {
    omni_drive_controller_->set_target(
        Eigen::Vector2f{msg->linear.x, msg->linear.y}, msg->angular.z);
  }

  void init() override {
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
    prev_update_ = get_clock()->now();
  }

public:
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
};

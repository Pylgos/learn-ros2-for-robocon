#pragma once
#include "exr1_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Vector3.h>

class Exr1SimNode : public Exr1Node {
  std::optional<QuadOmniDriveController> omni_drive_controller_;
  rclcpp::Time prev_update_;
  std::array<float, 4> wheel_torques_;
  Eigen::Vector2d position_;
  double orientation_;

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) override {
    omni_drive_controller_->set_target(
        Eigen::Vector2f{msg->linear.x, msg->linear.y}, msg->angular.z);
  }

public:
  Exr1SimNode(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
      : Exr1Node(node_options), wheel_torques_{0, 0, 0, 0}, position_{0, 0},
        orientation_{0} {
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

    double linear_variance = get_parameter("odom_linear_variance").as_double();
    double angular_variance =
        get_parameter("odom_angular_variance").as_double();

    position_ += Eigen::Rotation2Dd{orientation_} *
                 current_linear_velocity.cast<double>() * dt;
    orientation_ += current_angular_velocity * dt;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = current_linear_velocity.x();
    odom.twist.twist.linear.y = current_linear_velocity.y();
    odom.twist.twist.angular.z = current_angular_velocity;
    // clang-format off
    odom.twist.covariance = {
        linear_variance, 0,    0,    0,    0,    0,
        0,    linear_variance, 0,    0,    0,    0,
        0,    0,    0.0001, 0,    0,    0,
        0,    0,    0,    0.0001, 0,    0,
        0,    0,    0,    0,    0.0001, 0,
        0,    0,    0,    0,    0,    angular_variance,
    };
    // clang-format on
    publish_odometry(odom);

    Eigen::Quaterniond q{
        Eigen::AngleAxisd{orientation_, Eigen::Vector3d::UnitZ()}};
    geometry_msgs::msg::Transform tf;
    tf.translation.x = position_.x();
    tf.translation.y = position_.y();
    tf.rotation.w = q.w();
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    publish_odom_transform(now, tf);
  }

  std::array<float, 4> get_wheel_torques() { return wheel_torques_; }
};

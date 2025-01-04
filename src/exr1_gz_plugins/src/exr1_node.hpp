#pragma once
#include "quad_omni_drive_controller.hpp"
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>

class Exr1Node : public rclcpp::Node {
public:
  Exr1Node(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
      : Node("exr1_node", node_options) {
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

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SensorDataQoS{},
        std::bind(&Exr1Node::on_cmd_vel, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS{});
  }

protected:
  virtual void init() {}
  virtual void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

#pragma once
#include "quad_omni_drive_controller.hpp"
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

class Exr1Node : public rclcpp::Node {
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Time last_odom_publish_;

public:
  Exr1Node(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
      : Node("exr1_node", node_options), tf_broadcaster_(this) {
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
    declare_parameter("odom_publish_frequency", 100.0);
    declare_parameter("odom_linear_variance", 0.01);
    declare_parameter("odom_angular_variance", 0.01);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SensorDataQoS{},
        std::bind(&Exr1Node::on_cmd_vel, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS{});

    last_odom_publish_ = get_clock()->now();
  }

protected:
  virtual void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr) {};

  void publish_odometry(const nav_msgs::msg::Odometry &odom) {
    auto now = get_clock()->now();
    auto freq = get_parameter("odom_publish_frequency").as_double();
    auto interval = rclcpp::Duration::from_seconds(1.0 / freq);
    if (now - last_odom_publish_ < interval) {
      return;
    }
    last_odom_publish_ = now;
    odom_pub_->publish(odom);
  }

  void publish_odom_transform(rclcpp::Time stamp,
                              geometry_msgs::msg::Transform tf) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.header.stamp = stamp;
    transform.child_frame_id = "base_footprint";
    transform.transform = tf;
    tf_broadcaster_.sendTransform(transform);
  }
};

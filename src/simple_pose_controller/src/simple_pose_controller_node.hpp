#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace simple_pose_controller {

class SimplePoseControllerNode : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<geometry_msgs::msg::PoseStamped> goal_pose_;

  void timer_callback();
  void goal_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

public:
  SimplePoseControllerNode(
      const std::string name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  SimplePoseControllerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
};

} // namespace simple_pose_controller

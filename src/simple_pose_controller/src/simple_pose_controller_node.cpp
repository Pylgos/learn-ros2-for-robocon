#include "simple_pose_controller_node.hpp"
#include <chrono>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/time.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace simple_pose_controller {

SimplePoseControllerNode::SimplePoseControllerNode(
    const rclcpp::NodeOptions &options)
    : SimplePoseControllerNode("simple_pose_controller", options) {}

SimplePoseControllerNode::SimplePoseControllerNode(
    const std::string name, const rclcpp::NodeOptions &options)
    : Node(name, options) {
  declare_parameter("robot_frame", "base_link");
  declare_parameter("odom_frame", "odom");
  declare_parameter("transform_timeout", 0.2);
  declare_parameter("control_frequency", 100.0);
  declare_parameter("max_linear_speed", 1.0);
  declare_parameter("max_linear_deceleration", 1.0);
  declare_parameter("max_angular_speed", 1.0);
  declare_parameter("max_angular_deceleration", 1.0);

  tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);
  timer_ =
      create_timer(std::chrono::duration<double>(
                       1.0 / get_parameter("control_frequency").as_double()),
                   std::bind(&SimplePoseControllerNode::timer_callback, this));

  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::SystemDefaultsQoS{},
      std::bind(&SimplePoseControllerNode::goal_pose_callback, this,
                std::placeholders::_1));
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS{});
}

void SimplePoseControllerNode::timer_callback() {
  if (!goal_pose_.has_value()) {
    return;
  }
  const std::string robot_frame = get_parameter("robot_frame").as_string();
  const std::string odom_frame = get_parameter("odom_frame").as_string();
  const tf2::Duration transform_timeout =
      tf2::durationFromSec(get_parameter("transform_timeout").as_double());
  const double max_linear_speed = get_parameter("max_linear_speed").as_double();
  const double max_linear_deceleration =
      get_parameter("max_linear_deceleration").as_double();
  const double max_angular_speed =
      get_parameter("max_angular_speed").as_double();
  const double max_angular_deceleration =
      get_parameter("max_angular_deceleration").as_double();

  tf2::Transform robot_goal_pose;
  try {
    tf2::Transform goal_pose;
    tf2::fromMsg(goal_pose_->pose, goal_pose);
    tf2::Transform goal_pose_to_robot;
    tf2::fromMsg(tf_buf_
                     ->lookupTransform(robot_frame, goal_pose_->header.frame_id,
                                       tf2::TimePointZero, transform_timeout)
                     .transform,
                 goal_pose_to_robot);
    robot_goal_pose = goal_pose_to_robot * goal_pose;
  } catch (tf2::TransformException &e) {
    RCLCPP_ERROR(get_logger(), "Failed to lookup transform: %s", e.what());
    return;
  }

  // 2ax = v^2 - v_0^2
  // <=> v = sqrt(v_0^2 + 2ax)
  const auto position_diff = robot_goal_pose.getOrigin();
  const auto linear_direction = position_diff.normalized();
  const auto linear_velocity_mag =
      std::sqrt(2 * max_linear_deceleration * position_diff.length());
  auto linear_velocity = linear_direction * linear_velocity_mag;
  if (linear_velocity.length() > max_linear_speed) {
    linear_velocity.normalize();
    linear_velocity *= max_linear_speed;
  }

  const auto angle_diff = tf2::getYaw(robot_goal_pose.getRotation());
  const auto angular_velocity_mag =
      std::sqrt(2 * max_angular_deceleration * std::abs(angle_diff));
  auto angular_velocity = std::copysign(angular_velocity_mag, angle_diff);
  if (std::abs(angular_velocity) > max_angular_speed) {
    angular_velocity = std::copysign(max_angular_speed, angular_velocity);
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_velocity.x();
  cmd_vel.linear.y = linear_velocity.y();
  cmd_vel.angular.z = angular_velocity;
  cmd_vel_pub_->publish(cmd_vel);
}

void SimplePoseControllerNode::goal_pose_callback(
    geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_pose_ = *msg;
}

} // namespace simple_pose_controller

RCLCPP_COMPONENTS_REGISTER_NODE(
    simple_pose_controller::SimplePoseControllerNode)

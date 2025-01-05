#pragma once
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace ransac_localization {

class MultiScanSubscription {
public:
  struct MergedScan {
    std_msgs::msg::Header header;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  };

private:
  rclcpp::Node *node_;
  std::string odom_frame_, robot_frame_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Duration merge_window_;
  float min_range_;
  rclcpp::Duration transform_timeout_;
  std::function<void(MergedScan)> callback_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
      subscriptions_;
  tf2_ros::Buffer tf_buf_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr deadline_timer_;

  enum class State { WAIT, MERGING } state_;
  std::unordered_map<std::string, sensor_msgs::msg::LaserScan::ConstSharedPtr>
      scans_;

  void scan_callback(const std::string topic,
                     sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  void deadline_callback();

  void process_scans();

public:
  MultiScanSubscription(rclcpp::Node *node, const std::string robot_frame,
                        const std::string &odom_frame,
                        const std::vector<std::string> &scan_topics,
                        rclcpp::Duration merge_window, float min_range,
                        rclcpp::Duration transform_timeout,
                        std::function<void(MergedScan)> callback);
};

} // namespace ransac_localization

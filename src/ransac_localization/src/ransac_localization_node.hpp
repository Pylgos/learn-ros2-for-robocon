#pragma once
#include "corner_localization.hpp"
#include "multi_scan_subscription.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace ransac_localization {

class RansacLocalizationNode : public rclcpp::Node {
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::optional<MultiScanSubscription> multi_scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::optional<CornerLocalization> corner_localization_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      line_marker_pub_,
      corner_marker_pub_, tracked_corner_marker_pub_;

  void visualize_lines(const std::vector<Line> &lines,
                       const std_msgs::msg::Header &header);

  void visualize_corners(const std::vector<Corner> &corners,
                         const std_msgs::msg::Header &header);

  void visualize_tracked_corner(const std::optional<Corner> &corner,
                                const std_msgs::msg::Header &header);

public:
  RansacLocalizationNode(
      const std::string name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  RansacLocalizationNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void scan_callback(MultiScanSubscription::MergedScan msg);
};

} // namespace ransac_localization

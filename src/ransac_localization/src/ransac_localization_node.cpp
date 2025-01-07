#include "ransac_localization_node.hpp"
#include "multi_scan_subscription.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace std;

namespace ransac_localization {

RansacLocalizationNode::RansacLocalizationNode(
    const rclcpp::NodeOptions &options)
    : RansacLocalizationNode("ransac_localization", options) {}

RansacLocalizationNode::RansacLocalizationNode(
    const std::string name, const rclcpp::NodeOptions &options)
    : Node(name, options) {
  declare_parameter("map_frame", "map");
  declare_parameter("odom_frame", "odom");
  declare_parameter("robot_frame", "base_link");
  declare_parameter("merge_window", 0.1);
  declare_parameter("scan_topics", std::vector<std::string>{});
  declare_parameter("min_range", 0.0);
  declare_parameter("transform_timeout", 0.1);
  declare_parameter("transform_tolerance", 0.2);

  declare_parameter("initial_pose.x", 0.0);
  declare_parameter("initial_pose.y", 0.0);
  declare_parameter("initial_pose.yaw", 0.0);
  declare_parameter("corner_pose.x", 0.0);
  declare_parameter("corner_pose.y", 0.0);
  declare_parameter("corner_pose.yaw", 0.0);

  declare_parameter("line_detection.min_score_ratio", 0.1);
  declare_parameter("line_detection.inlier_threshold", 0.05);
  declare_parameter("line_detection.ransac_iterations", 1000);

  declare_parameter("position_lpf", 0.1);
  declare_parameter("rotation_lpf", 0.1);

  multi_scan_sub_.emplace(
      this, get_parameter("robot_frame").as_string(),
      get_parameter("odom_frame").as_string(),
      get_parameter("scan_topics").as_string_array(),
      rclcpp::Duration::from_seconds(get_parameter("merge_window").as_double()),
      static_cast<float>(get_parameter("min_range").as_double()),
      rclcpp::Duration::from_seconds(
          get_parameter("transform_timeout").as_double()),
      std::bind(&RansacLocalizationNode::scan_callback, this,
                std::placeholders::_1));

  tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "point_cloud", rclcpp::SystemDefaultsQoS{});

  line_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "detected_line_marker", rclcpp::SystemDefaultsQoS{});
  corner_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "detected_corner_marker", rclcpp::SystemDefaultsQoS{});
  tracked_corner_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
          "tracked_corner_marker", rclcpp::SystemDefaultsQoS{});

  CornerLocalizationOptions corner_options;
  corner_options.line_detector.min_score_ratio =
      get_parameter("line_detection.min_score_ratio").as_double();
  corner_options.line_detector.inlier_threshold =
      get_parameter("line_detection.inlier_threshold").as_double();
  corner_options.line_detector.ransac_iterations =
      get_parameter("line_detection.ransac_iterations").as_int();
  corner_options.position_lpf = get_parameter("position_lpf").as_double();
  corner_options.rotation_lpf = get_parameter("rotation_lpf").as_double();
  corner_localization_.emplace(get_logger(), corner_options);

  tf2::Transform initial_pose;
  Corner corner;
  initial_pose.setOrigin(
      tf2::Vector3(get_parameter("initial_pose.x").as_double(),
                   get_parameter("initial_pose.y").as_double(), 0));
  initial_pose.setRotation(tf2::Quaternion(
      tf2::Vector3(0, 0, 1), get_parameter("initial_pose.yaw").as_double()));
  corner.position = {get_parameter("corner_pose.x").as_double(),
                     get_parameter("corner_pose.y").as_double()};
  corner.direction = {std::cos(get_parameter("corner_pose.yaw").as_double()),
                      std::sin(get_parameter("corner_pose.yaw").as_double())};
  corner_localization_->set_initial_pose(initial_pose);
  corner_localization_->set_corner(corner);
}

void RansacLocalizationNode::scan_callback(
    MultiScanSubscription::MergedScan scan) {
  // RCLCPP_INFO(get_logger(), "Got scan with %lu points", scan.cloud->size());
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*scan.cloud, cloud_msg);
  cloud_msg.header = scan.header;
  cloud_pub_->publish(cloud_msg);

  tf2::Transform robot_pose;
  try {
    const auto tf_stamped = tf_buf_->lookupTransform(
        get_parameter("odom_frame").as_string(),
        get_parameter("robot_frame").as_string(), scan.header.stamp);
    tf2::fromMsg(tf_stamped.transform, robot_pose);
  } catch (tf2::TransformException &e) {
    RCLCPP_ERROR(get_logger(), "Failed to lookup transform: %s", e.what());
    return;
  }

  auto maybe_odom_to_map =
      corner_localization_->process(*scan.cloud, robot_pose);

  if (maybe_odom_to_map.has_value()) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    rclcpp::Time stamp = scan.header.stamp;
    auto tolerance = rclcpp::Duration::from_seconds(
        get_parameter("transform_tolerance").as_double());
    tf_stamped.header.stamp = stamp + tolerance;
    tf_stamped.header.frame_id = get_parameter("map_frame").as_string();
    tf_stamped.child_frame_id = get_parameter("odom_frame").as_string();
    tf_stamped.transform = tf2::toMsg(maybe_odom_to_map.value());
    tf_broadcaster_->sendTransform(tf_stamped);
  }

  auto lines = corner_localization_->get_detected_lines();
  auto corners = corner_localization_->get_detected_corners();
  visualize_lines(lines, scan.header);
  visualize_corners(corners, scan.header);
  visualize_tracked_corner(
      corner_localization_->get_tracked_corner().value_or(Corner{}),
      scan.header);
}

void RansacLocalizationNode::visualize_lines(
    const std::vector<Line> &lines, const std_msgs::msg::Header &header) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "detected_lines";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  for (const auto &line : lines) {
    Eigen::Vector2f a = line.pos + line.dir * 100.0;
    Eigen::Vector2f b = line.pos - line.dir * 100.0;
    geometry_msgs::msg::Point p1, p2;
    p1.x = a.x();
    p1.y = a.y();
    p2.x = b.x();
    p2.y = b.y();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  line_marker_pub_->publish(marker);
}

void RansacLocalizationNode::visualize_corners(
    const std::vector<Corner> &corners, const std_msgs::msg::Header &header) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "detected_corners";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.02;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  for (const auto &corner : corners) {
    Eigen::Rotation2Df rot_45{M_PI / 4};
    Eigen::Vector2f dir1 = rot_45 * corner.direction;
    Eigen::Vector2f dir2 = rot_45.inverse() * corner.direction;
    Eigen::Vector2f a = corner.position + dir1 * 0.5;
    Eigen::Vector2f b = corner.position + dir2 * 0.5;
    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = a.x();
    p1.y = a.y();
    p2.x = corner.position.x();
    p2.y = corner.position.y();
    p3.x = b.x();
    p3.y = b.y();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
  }
  corner_marker_pub_->publish(marker);
}

void RansacLocalizationNode::visualize_tracked_corner(
    const std::optional<Corner> &maybe_corner,
    const std_msgs::msg::Header &header) {
  if (!maybe_corner.has_value()) {
    return;
  }
  const auto corner = maybe_corner.value();
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "tracked_corner";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  geometry_msgs::msg::Point p;
  p.x = corner.position.x();
  p.y = corner.position.y();
  marker.points.push_back(p);
  tracked_corner_marker_pub_->publish(marker);
}

} // namespace ransac_localization

RCLCPP_COMPONENTS_REGISTER_NODE(ransac_localization::RansacLocalizationNode)

#include "multi_scan_subscription.hpp"
#include <chrono>
#include <memory>
#include <pcl/point_cloud.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace chrono = std::chrono;

namespace ransac_localization {

MultiScanSubscription::MultiScanSubscription(
    rclcpp::Node *node, const std::string robot_frame,
    const std::string &odom_frame, const std::vector<std::string> &scan_topics,
    rclcpp::Duration merge_window, float min_range,
    rclcpp::Duration transform_timeout,
    std::function<void(MergedScan)> callback)
    : node_{node}, odom_frame_{odom_frame}, robot_frame_{robot_frame},
      merge_window_{merge_window}, min_range_{min_range},
      transform_timeout_{transform_timeout}, callback_{callback},
      tf_buf_{node->get_clock()}, tf_listener_(tf_buf_, node_) {
  group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  for (const auto &topic : scan_topics) {
    auto callback = [this,
                     topic](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      scan_callback(topic, msg);
    };
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
        topic, rclcpp::SensorDataQoS{}, std::move(callback), options);
    subscriptions_.push_back(subscription);
  }
}

void MultiScanSubscription::scan_callback(
    const std::string topic, sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  scans_[topic] = msg;
  // RCLCPP_INFO(node_->get_logger(), "Got scan on topic %s", topic.c_str());

  if (scans_.size() == subscriptions_.size()) {
    // RCLCPP_INFO(node_->get_logger(), "All scans received");
    process_scans();
    return;
  }

  if (state_ == State::WAIT) {
    state_ = State::MERGING;
    deadline_timer_ = node_->create_timer(
        merge_window_.to_chrono<chrono::nanoseconds>(),
        std::bind(&MultiScanSubscription::deadline_callback, this), group_);
  }
}

void MultiScanSubscription::deadline_callback() {
  // RCLCPP_INFO(node_->get_logger(), "Deadline reached");
  process_scans();
}

void MultiScanSubscription::process_scans() {
  if (scans_.empty()) {
    return;
  }

  size_t total_points = 0;
  std::optional<rclcpp::Time> maybe_latest_stamp;
  for (const auto &pair : scans_) {
    total_points += pair.second->ranges.size();
    rclcpp::Time stamp{pair.second->header.stamp};
    if (!maybe_latest_stamp.has_value() || stamp > maybe_latest_stamp) {
      maybe_latest_stamp = stamp;
    }
  }
  auto latest_stamp = maybe_latest_stamp.value();

  auto robot_tf_stamped = tf_buf_.lookupTransform(
      odom_frame_, robot_frame_, latest_stamp, transform_timeout_);
  tf2::Vector3 robot_position;
  tf2::fromMsg(robot_tf_stamped.transform.translation, robot_position);

  auto min_range_sq = min_range_ * min_range_;

  std_msgs::msg::Header header;
  header.stamp = latest_stamp;
  header.frame_id = odom_frame_;

  auto points = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  points->reserve(total_points);

  for (const auto &[topic, scan] : scans_) {
    auto tf_stamped = tf_buf_.lookupTransform(
        odom_frame_, scan->header.stamp, scan->header.frame_id,
        scan->header.stamp, odom_frame_, transform_timeout_);
    tf2::Transform tf;
    tf2::fromMsg(tf_stamped.transform, tf);
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float range = scan->ranges[i];
      if (range < scan->range_min || scan->range_max < range) {
        continue;
      }
      float angle = scan->angle_min + i * scan->angle_increment;
      tf2::Vector3 point{range * std::cos(angle), range * std::sin(angle), 0};
      point = tf * point;
      auto distance_sq = (robot_position - point).length2();
      if (distance_sq < min_range_sq) {
        continue;
      }
      points->emplace_back(point.x(), point.y(), point.z());
    }
  }

  MergedScan merged_scan{header, points};
  callback_(std::move(merged_scan));

  state_ = State::WAIT;
  scans_.clear();
  deadline_timer_.reset();
}

} // namespace ransac_localization

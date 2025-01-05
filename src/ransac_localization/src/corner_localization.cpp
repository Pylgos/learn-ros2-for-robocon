#include "corner_localization.hpp"
#include <algorithm>
#include <optional>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <random>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

using Vec2f = Eigen::Vector2f;

namespace ransac_localization {

class LineDetector {
  LineDetectorOptions options_;
  std::mt19937 rng_;
  rclcpp::Logger logger_;

  std::pair<size_t, size_t> sample_indices(const std::vector<size_t> indices) {
    std::uniform_int_distribution<size_t> dist{0, indices.size() - 1};
    size_t i = dist(rng_);
    size_t j = dist(rng_);
    while (i == j) {
      j = dist(rng_);
    }
    return {indices[i], indices[j]};
  }

  static Vec2f to_vec2f(const pcl::PointXYZ &point) {
    return {point.x, point.y};
  }

  Line make_line_segment(const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
    return Line::from_points(to_vec2f(a), to_vec2f(b));
  }

  bool is_inlier(const Line &line, const pcl::PointXYZ &point) {
    return line.distance_to(to_vec2f(point)) < options_.inlier_threshold;
  }

  float score_point(float distance_to_line, float distance_to_center) {
    return distance_to_center / (0.5f + distance_to_line);
  }

  std::optional<Line> ransac(const pcl::PointCloud<pcl::PointXYZ> points,
                             const std::vector<size_t> indices,
                             const pcl::PointXYZ &center,
                             const float min_score) {
    if (indices.size() < 2) {
      return std::nullopt;
    }

    float best_score = 0;
    std::optional<Line> best_line;

    for (size_t i = 0; i < options_.ransac_iterations; ++i) {
      const auto [a_idx, b_idx] = sample_indices(indices);
      const auto line = make_line_segment(points[a_idx], points[b_idx]);
      float score = 0;
      for (size_t i = 0; i < points.size(); ++i) {
        if (is_inlier(line, points[i])) {
          float distance_to_line = line.distance_to(to_vec2f(points[i]));
          float distance_to_center = pcl::euclideanDistance(center, points[i]);
          score += score_point(distance_to_line, distance_to_center);
        }
      }
      if (score > best_score) {
        best_score = score;
        best_line = line;
      }
    }

    if (best_score < min_score) {
      return std::nullopt;
    }

    return best_line;
  }

public:
  LineDetector(rclcpp::Logger logger, LineDetectorOptions options)
      : options_{options}, rng_(std::random_device{}()), logger_{logger} {}

  std::vector<Line> detect_lines(const pcl::PointCloud<pcl::PointXYZ> &points,
                                 const pcl::PointXYZ &center) {
    const auto num_points = points.size();
    float highest_score = 0;
    for (size_t i = 0; i < points.size(); i++) {
      float distance_to_center = pcl::euclideanDistance(center, points[i]);
      highest_score += score_point(0, distance_to_center);
    }

    std::vector<size_t> remaining_indices;
    remaining_indices.resize(num_points);
    std::iota(remaining_indices.begin(), remaining_indices.end(), 0);
    std::vector<Line> result;

    while (true) {
      const auto maybe_best_line =
          ransac(points, remaining_indices, center,
                 highest_score * options_.min_score_ratio);
      if (!maybe_best_line.has_value()) {
        break;
      }
      const auto best_line = maybe_best_line.value();
      result.push_back(best_line);

      remaining_indices.erase(
          std::remove_if(
              remaining_indices.begin(), remaining_indices.end(),
              [&](size_t i) { return is_inlier(best_line, points[i]); }),
          remaining_indices.end());
    }
    return result;
  }
};

class CornerDetector {
  CornerDetectorOptions options_;

public:
  CornerDetector(CornerDetectorOptions options) : options_{options} {}

  std::vector<Corner> detect_corners(const std::vector<Line> lines,
                                     Vec2f robot_position) {
    std::vector<Corner> corners;
    for (size_t i = 0; i < lines.size(); ++i) {
      for (size_t j = i + 1; j < lines.size(); ++j) {
        const auto &line1 = lines[i];
        const auto &line2 = lines[j];

        const auto maybe_intersection = line1.intersection(line2);
        if (!maybe_intersection.has_value()) {
          continue;
        }
        const auto intersection = maybe_intersection.value();
        const auto intersection_angle = line1.angle_to(line2);
        if (intersection_angle < options_.min_corner_angle) {
          continue;
        }
        const std::array<Vec2f, 4> directions{
            (line1.dir + line2.dir) / 2, (line1.dir - line2.dir) / 2,
            (-line1.dir + line2.dir) / 2, (-line1.dir - line2.dir) / 2};

        const Vec2f robot_dir = (robot_position - intersection).normalized();
        Vec2f best_direction;
        float best_dot = -1;
        for (const auto &direction : directions) {
          const Vec2f dir = direction.normalized();
          const float dot = robot_dir.dot(dir);
          if (dot > best_dot) {
            best_dot = dot;
            best_direction = dir;
          }
        }
        corners.push_back({intersection, best_direction});
      }
    }
    return corners;
  }
};

CornerLocalization::CornerLocalization(rclcpp::Logger logger,
                                       CornerLocalizationOptions options)
    : logger_{logger}, options_{options}, tracked_{false} {}

std::optional<tf2::Transform>
CornerLocalization::process(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                            const tf2::Transform &robot_pose) {
  auto robot_position = robot_pose.getOrigin();
  LineDetector detector{logger_, options_.line_detector};
  auto lines = detector.detect_lines(
      cloud, pcl::PointXYZ{static_cast<float>(robot_position.x()),
                           static_cast<float>(robot_position.y()),
                           static_cast<float>(robot_position.z())});
  CornerDetector corner_detector{options_.corner_detector};
  auto corners = corner_detector.detect_corners(
      lines, {robot_position.x(), robot_position.y()});

  detected_lines_ = lines;
  detected_corners_ = corners;

  if (last_tracked_corner_.has_value()) {
    auto corner = search_closest_corner(corners, last_tracked_corner_.value());
    if (corner.has_value()) {
      tracked_ = true;
      last_tracked_corner_ = corner;
    } else {
      tracked_ = false;
    }
  } else if (initial_robot_to_map_.has_value() && map_corner_.has_value()) {
    auto robot_to_odom = robot_pose;
    auto map_to_robot = initial_robot_to_map_->inverse();
    auto map_to_odom = robot_to_odom * map_to_robot;
    tf2::Vector3 map_corner_position = {map_corner_->position.x(),
                                        map_corner_->position.y(), 0};
    tf2::Vector3 map_corner_direction = {map_corner_->direction.x(),
                                         map_corner_->direction.y(), 0};
    tf2::Vector3 tracked_corner_position = map_to_odom * map_corner_position;
    tf2::Vector3 tracked_corner_direction =
        map_to_odom.getBasis() * map_corner_direction;
    tracked_ = true;
    last_tracked_corner_ = Corner{
        Vec2f{tracked_corner_position.x(), tracked_corner_position.y()},
        Vec2f{tracked_corner_direction.x(), tracked_corner_direction.y()}};
    RCLCPP_INFO(logger_, "Initial localization: %f %f %f %f",
                tracked_corner_position.x(), tracked_corner_position.y(),
                tracked_corner_direction.x(), tracked_corner_direction.y());
    initial_robot_to_map_ = std::nullopt;
  }

  if (tracked_ && last_tracked_corner_.has_value() && map_corner_.has_value()) {
    auto tracked_corner = last_tracked_corner_.value();
    auto map_corner = map_corner_.value();
    float tracked_corner_angle =
        std::atan2(tracked_corner.direction.y(), tracked_corner.direction.x());
    float map_corner_angle =
        std::atan2(map_corner.direction.y(), map_corner.direction.x());
    tf2::Quaternion odom_to_map_rotation{
        tf2::Vector3{0, 0, 1}, map_corner_angle - tracked_corner_angle};
    tf2::Vector3 map_corner_position = {map_corner.position.x(),
                                        map_corner.position.y(), 0};
    tf2::Vector3 tracked_corner_position = {tracked_corner.position.x(),
                                            tracked_corner.position.y(), 0};
    tf2::Vector3 odom_to_map_translation =
        map_corner_position -
        (tf2::Matrix3x3{odom_to_map_rotation} * tracked_corner_position);

    if (smoothed_position_.has_value()) {
      smoothed_position_ =
          (1 - options_.position_lpf) * smoothed_position_.value() +
          options_.position_lpf * odom_to_map_translation;
    } else {
      smoothed_position_ = odom_to_map_translation;
    }
    if (smoothed_rotation_.has_value()) {
      smoothed_rotation_ = smoothed_rotation_.value().slerp(
          odom_to_map_rotation, options_.rotation_lpf);
    } else {
      smoothed_rotation_ = odom_to_map_rotation;
    }
    tf2::Transform odom_to_map{smoothed_rotation_.value(),
                               smoothed_position_.value()};
    return odom_to_map;
  }
  return std::nullopt;
}

std::optional<Corner>
CornerLocalization::search_closest_corner(const std::vector<Corner> &corners,
                                          const Corner &ref_corner) {
  float best_loss = std::numeric_limits<float>::max();
  std::optional<Corner> best_corner;
  for (const auto &corner : corners) {
    float distance = (corner.position - ref_corner.position).norm();
    float angle = std::acos(corner.direction.dot(ref_corner.direction));
    if (distance > options_.corner_search_radius ||
        angle > options_.corner_search_angle) {
      continue;
    }
    float loss = distance + angle;
    if (loss < best_loss) {
      best_loss = loss;
      best_corner = corner;
    }
  }
  return best_corner;
}

} // namespace ransac_localization

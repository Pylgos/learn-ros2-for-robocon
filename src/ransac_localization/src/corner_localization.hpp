#pragma once
#include <optional>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/transform_datatypes.hpp>

namespace ransac_localization {

struct Line {
  Eigen::Vector2f pos, dir;

  static Line from_points(Eigen::Vector2f a, Eigen::Vector2f b) {
    return Line{a, (b - a).normalized()};
  }

  float distance_to(Eigen::Vector2f point) const {
    Eigen::Vector2f v = point - pos;
    float cross = dir.x() * v.y() - dir.y() * v.x();
    return std::abs(cross);
  }

  std::optional<Eigen::Vector2f> intersection(const Line &other) const {
    float cross = dir.x() * other.dir.y() - dir.y() * other.dir.x();
    if (std::abs(cross) < 1e-6) {
      return std::nullopt;
    }
    Eigen::Vector2f diff = other.pos - pos;
    float t = (diff.x() * other.dir.y() - diff.y() * other.dir.x()) / cross;
    return pos + t * dir;
  }

  float angle_to(const Line &other) const {
    return std::acos(dir.dot(other.dir));
  }
};

struct Corner {
  Eigen::Vector2f position, direction;
  static Corner from_position_and_angle(Eigen::Vector2f position, float angle) {
    return Corner{position, {std::cos(angle), std::sin(angle)}};
  }
};

struct LineDetectorOptions {
  float min_score_ratio = 0.1, inlier_threshold = 0.1;
  size_t ransac_iterations = 1000;
};

struct CornerDetectorOptions {
  float min_corner_angle = M_PI / 2 - 0.1;
};

struct CornerLocalizationOptions {
  LineDetectorOptions line_detector;
  CornerDetectorOptions corner_detector;
  float corner_search_radius = 1.0;
  float corner_search_angle = 2 * M_PI;
  float position_lpf = 0.2;
  float rotation_lpf = 0.2;
};

class CornerLocalization {
  rclcpp::Logger logger_;
  CornerLocalizationOptions options_;

  std::optional<Corner> map_corner_;

  std::optional<tf2::Transform> initial_robot_to_map_;
  std::optional<Corner> last_tracked_corner_;
  bool tracked_;
  std::optional<tf2::Vector3> smoothed_position_;
  std::optional<tf2::Quaternion> smoothed_rotation_;
  std::optional<rclcpp::Time> tracked_corner_stamp_;
  std::optional<tf2::Transform> odom_to_map_;
  std::vector<Line> detected_lines_;
  std::vector<Corner> detected_corners_;

  std::optional<Corner>
  search_closest_corner(const std::vector<Corner> &corners,
                        const Corner &ref_corner);

public:
  CornerLocalization(rclcpp::Logger logger, CornerLocalizationOptions options);

  std::optional<tf2::Transform>
  process(const pcl::PointCloud<pcl::PointXYZ> &cloud,
          const tf2::Transform &robot_pose);

  void set_initial_pose(const tf2::Transform &initial_robot_to_map) {
    initial_robot_to_map_ = initial_robot_to_map;
  };

  void set_corner(const Corner &corner) { map_corner_ = corner; };

  std::optional<Corner> get_tracked_corner() const {
    return last_tracked_corner_;
  };

  std::optional<tf2::Transform> get_odom_to_map_transform() const {
    return odom_to_map_;
  };

  std::optional<rclcpp::Time> get_transform_stamp() const {
    return tracked_corner_stamp_;
  }

  const std::vector<Line> &get_detected_lines() const {
    return detected_lines_;
  };
  const std::vector<Corner> &get_detected_corners() const {
    return detected_corners_;
  };
};

} // namespace ransac_localization

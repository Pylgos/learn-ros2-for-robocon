#pragma once
#include "pid_controller.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <iostream>

class QuadOmniDriveController {
  float wheel_radius_, wheel_separation_, mass_, moment_of_inertia_;
  PidController linear_x_pid_, linear_y_pid_, angular_pid_;
  Eigen::Vector2f linear_vel_target_;
  float angular_vel_target_;

public:
  QuadOmniDriveController(PidGain linear_gain, PidGain angular_gain,
                          float wheel_radius, float wheel_separation,
                          float mass, float moment_of_inertia)
      : wheel_radius_(wheel_radius), wheel_separation_{wheel_separation},
        mass_{mass}, moment_of_inertia_{moment_of_inertia},
        linear_x_pid_(linear_gain), linear_y_pid_(linear_gain),
        angular_pid_(angular_gain) {
    set_target({0, 0}, 0);
  };

  void set_target(Eigen::Vector2f linear_vel, float angular_vel) {
    linear_x_pid_.set_target(linear_vel.x());
    linear_y_pid_.set_target(linear_vel.y());
    angular_pid_.set_target(angular_vel);
    linear_vel_target_ = linear_vel;
    angular_vel_target_ = angular_vel;
  }

  std::pair<Eigen::Vector2f, float>
  compute_current_velocity(std::array<float, 4> wheel_angular_velocities) {
    float sqrt2 = sqrt(2);
    float l = wheel_separation_ * sqrt(2);
    float angular = 0.25 * wheel_radius_ / l *
                    (wheel_angular_velocities[0] + wheel_angular_velocities[1] +
                     wheel_angular_velocities[2] + wheel_angular_velocities[3]);
    Eigen::Vector2f linear{
        0.25 * wheel_radius_ * sqrt2 *
            (-wheel_angular_velocities[0] - wheel_angular_velocities[1] +
             wheel_angular_velocities[2] + wheel_angular_velocities[3]),
        0.25 * wheel_radius_ * sqrt2 *
            (wheel_angular_velocities[0] - wheel_angular_velocities[1] -
             wheel_angular_velocities[2] + wheel_angular_velocities[3]),
    };
    return {linear, angular};
  }

  std::array<float, 4> compute_wheel_velocities() {
    float l = wheel_separation_ * sqrt(2);
    float inv_sqrt2 = 1 / sqrt(2);
    return {

        (linear_vel_target_.dot(Eigen::Vector2f{-inv_sqrt2, inv_sqrt2}) +
         l * angular_vel_target_) /
            wheel_radius_,

        (linear_vel_target_.dot(Eigen::Vector2f{-inv_sqrt2, -inv_sqrt2}) +
         l * angular_vel_target_) /
            wheel_radius_,

        (linear_vel_target_.dot(Eigen::Vector2f{inv_sqrt2, -inv_sqrt2}) +
         l * angular_vel_target_) /
            wheel_radius_,

        (linear_vel_target_.dot(Eigen::Vector2f{inv_sqrt2, inv_sqrt2}) +
         l * angular_vel_target_) /
            wheel_radius_,
    };
  }

  std::array<float, 4> compute_torque(const Eigen::Vector2f current_linear_vel,
                                      const float current_angular_vel,
                                      const float dt) {
    linear_x_pid_.update(current_linear_vel.x(), dt);
    linear_y_pid_.update(current_linear_vel.y(), dt);
    angular_pid_.update(current_angular_vel, dt);
    float l = wheel_separation_ * sqrt(2.0f);
    float sqrt2 = sqrt(2.0f);
    float fx = linear_x_pid_.get_output() * mass_;
    float fy = linear_y_pid_.get_output() * mass_;
    float ft = angular_pid_.get_output() * moment_of_inertia_;
    // F = τ/r <=> τ = F*r
    return {
        (-fx * sqrt2 + fy * sqrt2 + ft / l) * wheel_radius_ / 4.0f,
        (-fx * sqrt2 - fy * sqrt2 + ft / l) * wheel_radius_ / 4.0f,
        (fx * sqrt2 - fy * sqrt2 + ft / l) * wheel_radius_ / 4.0f,
        (fx * sqrt2 + fy * sqrt2 + ft / l) * wheel_radius_ / 4.0f,
    };
  }
};

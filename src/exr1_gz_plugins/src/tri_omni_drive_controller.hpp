#pragma once
#include "pid_controller.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <iostream>

class TriOmniDriveController {
  float wheel_radius_, mass_;
  Eigen::Matrix3f a_, j_;
  std::array<Eigen::Vector2f, 3> wheel_positions_, wheel_directions_;
  PidController linear_x_pid_, linear_y_pid_, angular_pid_;

public:
  TriOmniDriveController(PidGain linear_gain, PidGain angular_gain,
                         float wheel_radius, float mass,
                         float moment_of_inertia,
                         Eigen::Vector2f center_of_mass,
                         std::array<Eigen::Vector2f, 3> wheel_positions,
                         std::array<Eigen::Vector2f, 3> wheel_directions)
      : wheel_radius_(wheel_radius), mass_{mass},
        wheel_positions_(wheel_positions), wheel_directions_(wheel_directions),
        linear_x_pid_(linear_gain), linear_y_pid_(linear_gain),
        angular_pid_(angular_gain) {
    auto mr = mass_ * wheel_radius_;
    auto ir = moment_of_inertia / wheel_radius_;
    std::array<Eigen::Vector3f, 3> t{{
        {wheel_directions_[0].x(), wheel_directions_[0].y(), 0},
        {wheel_directions_[1].x(), wheel_directions_[1].y(), 0},
        {wheel_directions_[2].x(), wheel_directions_[2].y(), 0},
    }};
    std::array<Eigen::Vector3f, 3> p{{
        {wheel_positions_[0].x(), wheel_positions_[0].y(), 0},
        {wheel_positions_[1].x(), wheel_positions_[1].y(), 0},
        {wheel_positions_[2].x(), wheel_positions_[2].y(), 0},
    }};
    Eigen::Vector3f c{center_of_mass.x(), center_of_mass.y(), 0};
    a_ << t[0].x() / mr, t[1].x() / mr, t[2].x() / mr, //
        t[0].y() / mr, t[1].y() / mr, t[2].y() / mr,   //
        (p[0] - c).cross(t[0]).z() / ir, (p[1] - c).cross(t[1]).z() / ir,
        (p[2] - c).cross(t[2]).z() / ir;
    Eigen::AngleAxisf rot_90{-M_PI / 2, Eigen::Vector3f::UnitZ()};
    j_ << t[0].x(), t[0].y(), (rot_90 * (p[0] - c)).dot(t[0]), //
        t[1].x(), t[1].y(), (rot_90 * (p[1] - c)).dot(t[1]),   //
        t[2].x(), t[2].y(), (rot_90 * (p[2] - c)).dot(t[2]);
    set_target({0, 0}, 0);
  };

  void set_target(Eigen::Vector2f linear_vel, float angular_vel) {
    linear_x_pid_.set_target(linear_vel.x());
    linear_y_pid_.set_target(linear_vel.y());
    angular_pid_.set_target(angular_vel);
  }

  std::pair<Eigen::Vector2f, float>
  compute_current_velocity(std::array<float, 3> wheel_angular_velocities) {
    Eigen::Vector3f wheel_velocities{
        wheel_angular_velocities[0] * wheel_radius_,
        wheel_angular_velocities[1] * wheel_radius_,
        wheel_angular_velocities[2] * wheel_radius_};

    Eigen::Vector3f velocities =
        j_.colPivHouseholderQr().solve(wheel_velocities);
    return {velocities.head<2>(), velocities.z()};
  }

  std::array<float, 3> compute_torque(const Eigen::Vector2f current_linear_vel,
                                      const float current_angular_vel,
                                      const float dt) {
    linear_x_pid_.update(current_linear_vel.x(), dt);
    linear_y_pid_.update(current_linear_vel.y(), dt);
    angular_pid_.update(current_angular_vel, dt);
    Eigen::Vector3f accel{
        // linear_x_pid_.get_output(), linear_y_pid_.get_output(),
        // angular_pid_.get_output(),
        linear_x_pid_.get_target(),
        linear_y_pid_.get_target(),
        angular_pid_.get_target(),
    };
    Eigen::Vector3f torque_vec = a_.colPivHouseholderQr().solve(accel);
    return {torque_vec.x(), torque_vec.y(), torque_vec.z()};
  }
};

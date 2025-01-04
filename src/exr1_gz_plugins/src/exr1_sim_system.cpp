#include "exr1_controller.hpp"
#include <array>
#include <cstdlib>
#include <geometry_msgs/msg/twist.hpp>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <rclcpp/context.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/init_options.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Vector3.hpp>

using std::numbers::pi;

template <typename Src, typename Dst, size_t N>
std::array<Dst, N> convert_array(const std::array<Src, N> &src) {
  std::array<Dst, N> dst;
  for (size_t i = 0; i < N; i++) {
    dst[i] = src[i];
  }
  return dst;
}

namespace exr1_gz {

class Exr1System : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate,
                   public gz::sim::ISystemPostUpdate {
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link base_link_{gz::sim::kNullEntity};

  double max_wheel_torque_, wheel_radius_, wheel_separation_, linear_damping_,
      angular_damping_;

  /// Whether the system has been properly configured
  bool configured_{false};

  std::shared_ptr<Exr1Controller> controller_;

  template <typename T>
  bool get_element(const std::shared_ptr<const sdf::Element> &sdf,
                   const char *name, T &dest) {
    if (sdf->HasElement(name)) {
      dest = sdf->Get<T>(name);
      return true;
    } else {
      gzerr << "No " << name
            << " element present. Exr1System plugin could not be loaded."
            << std::endl;
      return false;
    }
  }

  std::array<float, 4>
  compute_wheel_angular_velocities(gz::math::Vector3d velocity,
                                   gz::math::Vector3d angular_velocity) {
    std::array<float, 4> result;
    for (size_t i = 0; i < 4; i++) {
      double position_angle = i * pi / 2 + pi / 4;
      double distance = std::sqrt(2) * wheel_separation_;
      double wheel_tangent_angle = position_angle + pi / 2;
      gz::math::Vector3d position{distance * std::cos(position_angle),
                                  distance * std::sin(position_angle), 0};
      gz::math::Vector3d tangent{std::cos(wheel_tangent_angle),
                                 std::sin(wheel_tangent_angle), 0};
      gz::math::Vector3d wheel_velocity =
          velocity + angular_velocity.Cross(position);
      result[i] = wheel_velocity.Dot(tangent) / wheel_radius_;
    }
    return result;
  }

  std::pair<gz::math::Vector3d, gz::math::Vector3d>
  step_simulation(std::array<float, 4> wheel_torques,
                  gz::math::Vector3d velocity,
                  gz::math::Vector3d angular_velocity) {
    auto total_force = gz::math::Vector3d::Zero;
    auto total_torque = gz::math::Vector3d::Zero;

    for (size_t i = 0; i < 4; i++) {
      double position_angle = i * pi / 2 + pi / 4;
      double distance = std::sqrt(2) * wheel_separation_;
      double wheel_tangent_angle = position_angle + pi / 2;
      gz::math::Vector3d position{distance * std::cos(position_angle),
                                  distance * std::sin(position_angle), 0};
      gz::math::Vector3d tangent{std::cos(wheel_tangent_angle),
                                 std::sin(wheel_tangent_angle), 0};
      // τ = F*r <=> F = τ/r
      auto force = wheel_torques[i] / wheel_radius_ * tangent;
      auto torque = position.Cross(force);
      total_force += force;
      total_torque += torque;
    }

    // damping
    auto damping_force =
        gz::math::Vector3d(velocity.X(), velocity.Y(), 0) * linear_damping_;
    auto damping_torque =
        gz::math::Vector3d(0, 0, angular_velocity.Z()) * angular_damping_;
    total_force -= damping_force;
    total_torque -= damping_torque;
    // gzmsg << "damping force: " << damping_force << std::endl;
    // gzmsg << "damping torque:" << damping_torque << std::endl;

    return {total_force, total_torque};
  }

  std::pair<gz::math::Vector3d, gz::math::Vector3d>
  get_local_wrench(const gz::sim::EntityComponentManager &ecm) {
    auto world_velocity = base_link_.WorldLinearVelocity(ecm);
    auto world_angular_velocity = base_link_.WorldAngularVelocity(ecm);
    auto world_pose = base_link_.WorldPose(ecm);
    if (!world_velocity || !world_angular_velocity || !world_pose) {
      return {{0, 0, 0}, {0, 0, 0}};
    }
    auto local_to_world_rot = world_pose->Rot();
    auto world_to_local_rot = local_to_world_rot.Inverse();
    auto local_velocity =
        world_to_local_rot.RotateVector(world_velocity.value());
    auto local_angular_velocity =
        world_to_local_rot.RotateVector(world_angular_velocity.value());
    return {local_velocity, local_angular_velocity};
  }

public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager & /*eventMgr*/) override {
    model_ = gz::sim::Model(entity);

    if (!model_.Valid(ecm)) {
      gzerr << "Exr1System plugin should be attached to a model "
            << "entity. Failed to initialize." << std::endl;
      return;
    }

    std::string base_link_name;
    if (!get_element(sdf, "base_link", base_link_name)) {
      return;
    }
    auto base_link_entity = model_.LinkByName(ecm, base_link_name);
    if (base_link_entity == gz::sim::kNullEntity) {
      gzerr << "Failed to find link named \'" << base_link_name
            << "\' in the model. " << "Exr1System plugin could not be loaded."
            << std::endl;
      return;
    }
    base_link_ = gz::sim::Link(base_link_entity);
    base_link_.EnableVelocityChecks(ecm);

    if (!get_element(sdf, "max_wheel_torque", max_wheel_torque_) ||
        !get_element(sdf, "wheel_radius", wheel_radius_) ||
        !get_element(sdf, "wheel_separation", wheel_separation_) ||
        !get_element(sdf, "linear_damping", linear_damping_) ||
        !get_element(sdf, "angular_damping", angular_damping_)) {
      return;
    }

    try {
      rclcpp::InitOptions init_options;
      init_options.shutdown_on_signal = false;
      rclcpp::init(0, nullptr, init_options);
    } catch (const rclcpp::ContextAlreadyInitialized &e) {
      // Noop
    }

    rclcpp::NodeOptions node_options;
    auto params_file = std::getenv("EXR1_PARAMS_FILE");
    if (params_file) {
      node_options.arguments({"--ros-args", "--params-file", params_file});
    }
    controller_ = std::make_shared<Exr1Controller>(node_options);

    configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!configured_ || info.paused) {
      return;
    }

    rclcpp::spin_some(controller_);

    auto target_wheel_torques = controller_->get_wheel_torques();

    // {
    //   std::cout << "wheel torques: ";
    //   for (size_t i = 0; i < 4; i++) {
    //     std::cout << target_wheel_torques[i] << " ";
    //   }
    //   std::cout << std::endl;
    // }

    auto [local_velocity, local_angular_velocity] = get_local_wrench(ecm);
    // gzmsg << "local velocity: " << local_velocity
    //       << "  local angular : " << local_angular_velocity << std::endl;

    const auto [local_force, local_torque] = step_simulation(
        target_wheel_torques, local_velocity, local_angular_velocity);

    auto local_to_world_rot = base_link_.WorldPose(ecm)->Rot();
    auto world_force = local_to_world_rot.RotateVector(local_force);
    auto world_torque = local_to_world_rot.RotateVector(local_torque);
    base_link_.AddWorldWrench(ecm, world_force, world_torque);
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (!configured_ || info.paused) {
      return;
    }
    auto [local_velocity, local_angular_velocity] = get_local_wrench(ecm);
    // gzmsg << "local velocity: " << local_velocity
    //       << "  local angular : " << local_angular_velocity << std::endl;
    auto wheel_angular_velocities = compute_wheel_angular_velocities(
        local_velocity, local_angular_velocity);
    controller_->update(wheel_angular_velocities);
  }
};

} // namespace exr1_gz

GZ_ADD_PLUGIN(exr1_gz::Exr1System, gz::sim::System,
              exr1_gz::Exr1System::ISystemConfigure,
              exr1_gz::Exr1System::ISystemPreUpdate,
              exr1_gz::Exr1System::ISystemPostUpdate)

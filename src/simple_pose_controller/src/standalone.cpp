#include "simple_pose_controller_node.hpp"
#include <memory>
#include <rclcpp/executors.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<simple_pose_controller::SimplePoseControllerNode>(
          "simple_pose_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "ransac_localization_node.hpp"
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<ransac_localization::RansacLocalizationNode>(
          "ransac_localization");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

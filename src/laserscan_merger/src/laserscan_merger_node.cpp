#include "laserscan_merger/laserscan_merger.hpp"
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<laserscan_merger::LaserScanMarger>(
          "laserscan_merger_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

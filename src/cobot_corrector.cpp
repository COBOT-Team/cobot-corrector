#include "cobot_corrector/cobot_corrector_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cobot_corrector");
  CobotCorrectorNode corrector(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

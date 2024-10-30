#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tug_mr2/KinematicNode.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tug_mr2::KinematicNode>());
  rclcpp::shutdown();
  return 0;
}

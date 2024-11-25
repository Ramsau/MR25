#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tug_turtlebot4/SimpleTurtleNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tug_turtlebot4::SimpleTurtleNode>());
  rclcpp::shutdown();
  return 0;
}
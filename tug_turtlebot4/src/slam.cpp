#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tug_turtlebot4/SlamNode.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tug_turtlebot4::SlamNode>());
  rclcpp::shutdown();
  return 0;
}

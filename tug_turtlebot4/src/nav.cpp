#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

#include "tug_turtlebot4/NavNode.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tug_turtlebot4::NavNode>());
  rclcpp::shutdown();
  return 0;
}

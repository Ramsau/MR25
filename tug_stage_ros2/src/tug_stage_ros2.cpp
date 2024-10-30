#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tug_stage_ros2/StageNode.hpp"

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout << argv[0] << " <worldfile>" << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  tug_stage_ros2::StageNode::SharedPtr node
    = std::make_shared<tug_stage_ros2::StageNode>(argc, argv);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

#include "ign_plugin/WheelEncoder.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include "ignition/common/Console.hh"
#include "ignition/gazebo/components.hh"
#include "ignition/plugin/Register.hh"

namespace tug_turtlebot4
{

namespace ign_plugin
{

// -----------------------------------------------------------------------------
WheelEncoder::WheelEncoder()
{

}

// -----------------------------------------------------------------------------
WheelEncoder::~WheelEncoder()
{
  executor_->cancel();
  ros_thread_->join();
  rclcpp::shutdown();
}

// -----------------------------------------------------------------------------
void WheelEncoder::Configure(
  const Entity&,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm,
  EventManager&
)
{
  // Gazebo config
  std::string left_joint_name;
  std::string right_joint_name;

  const bool& left_joint_found = sdf->Get(
    "left_joint",
    left_joint_name,
    std::string()
  );
  const bool& right_joint_found = sdf->Get(
    "right_joint",
    right_joint_name,
    std::string()
  );

  if (!left_joint_found | !right_joint_found)
  {
    ignerr
      << "Parameters 'left_joint' and 'right_joint' must be set"
      << std::endl;
    return;
  }

  left_joint_entity_ = ecm.EntityByComponents(
    ignition::gazebo::components::Joint(),
    ignition::gazebo::components::Name(left_joint_name)
  );

  right_joint_entity_ = ecm.EntityByComponents(
    ignition::gazebo::components::Joint(),
    ignition::gazebo::components::Name(right_joint_name)
  );

  // ROS config
  std::vector<const char*> node_args = {"wheel_encoder"};
  rclcpp::init(node_args.size(), node_args.data());

  node_ = rclcpp::Node::make_shared("wheel_encoder", "tug_turtlebot4");
  wheel_enc_pub_ = node_->create_publisher<tug_msgs::msg::WheelEncoder>(
    "wheel_encoder",
    10
  );
  executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();

  executor_->add_node(node_);

  ros_thread_ = std::make_shared<std::thread>([this](){executor_->spin();});
}

// -----------------------------------------------------------------------------
void WheelEncoder::PostUpdate(
  const UpdateInfo&,
  const EntityComponentManager& ecm
)
{
  using ignition::gazebo::components::JointPosition;
  const double& left_rad
    = ecm.Component<JointPosition>(left_joint_entity_)->Data()[0];
  const double& right_rad
    = ecm.Component<JointPosition>(right_joint_entity_)->Data()[0];

  constexpr double ticks_per_rot = 2573.0;
  const double left_tick = left_rad / (2.0 * M_PI) * ticks_per_rot;
  const double right_tick = right_rad / (2.0 * M_PI) * ticks_per_rot;

  tug_msgs::msg::WheelEncoder wheel_enc;
  wheel_enc.left_counter = static_cast<uint16_t>(left_tick);
  wheel_enc.right_counter = static_cast<uint16_t>(right_tick);

  wheel_enc_pub_->publish(wheel_enc);
}

} /* namespace ign_plugin */

} /* namespace tug_turtlebot4 */

// -----------------------------------------------------------------------------
IGNITION_ADD_PLUGIN(
  tug_turtlebot4::ign_plugin::WheelEncoder,
  tug_turtlebot4::ign_plugin::WheelEncoder::System,
  tug_turtlebot4::ign_plugin::WheelEncoder::ISystemConfigure,
  tug_turtlebot4::ign_plugin::WheelEncoder::ISystemPostUpdate
)
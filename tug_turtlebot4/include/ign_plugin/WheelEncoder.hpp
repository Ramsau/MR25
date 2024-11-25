#ifndef _TUG_TURTLEBOT4__IGN_PLUGIN__WHEEL_ENCODER_HPP_
#define _TUG_TURTLEBOT4__IGN_PLUGIN__WHEEL_ENCODER_HPP_

#include <memory>
#include <thread>

#include "ignition/gazebo/System.hh"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

#include "tug_msgs/msg/wheel_encoder.hpp"

namespace tug_turtlebot4
{

namespace ign_plugin
{

class WheelEncoder :
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPostUpdate
{
  // Directives ----------------------------------------------------------------
  private:
    using Entity = ignition::gazebo::Entity;
    using EntityComponentManager = ignition::gazebo::EntityComponentManager;
    using EventManager = ignition::gazebo::EventManager;
    using UpdateInfo = ignition::gazebo::UpdateInfo;
  
  // Variables -----------------------------------------------------------------
  private:
    // Entities
    Entity left_joint_entity_;
    Entity right_joint_entity_;

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::Publisher<tug_msgs::msg::WheelEncoder>::SharedPtr wheel_enc_pub_;

    std::shared_ptr<std::thread> ros_thread_;

  // Methods -------------------------------------------------------------------
  public:
    WheelEncoder();
    virtual ~WheelEncoder();

    virtual void Configure(
      const Entity& entity,
      const std::shared_ptr<const sdf::Element>& sdf,
      EntityComponentManager& ecm,
      EventManager& event_mgr
    ) override;

    virtual void PostUpdate(
      const UpdateInfo& info,
      const EntityComponentManager& ecm
    ) override;
};

} /* namespace ign_plugin */

} /* namespace tug_turtlebot4 */

#endif /* _TUG_TURTLEBOT4__IGN_PLUGIN__WHEEL_ENCODER_HPP_ */

#ifndef _TUG_STAGE_ROS2__STAGE_NODE_HPP_
#define _TUG_STAGE_ROS2__STAGE_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tug_msgs/msg/bicycle.hpp"
#include "tug_msgs/msg/robot_pose.hpp"

#include "stage.hh"

namespace tug_stage_ros2
{

class StageNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  public:
    using SharedPtr = std::shared_ptr<StageNode>;

  private:
    using Bicycle = tug_msgs::msg::Bicycle;
    using Clock = rosgraph_msgs::msg::Clock;
    using LaserScan = sensor_msgs::msg::LaserScan;
    using Odometry = nav_msgs::msg::Odometry;
    using RobotPose = tug_msgs::msg::RobotPose;
    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using Twist = geometry_msgs::msg::Twist;

  // Variables -----------------------------------------------------------------
  private:
    // Callback group
    rclcpp::CallbackGroup::SharedPtr stage_cb_group_;

    // Publisher
    rclcpp::Publisher<Clock>::SharedPtr clock_pub_;

    std::vector<rclcpp::Publisher<LaserScan>::SharedPtr> laser_pubs_;
    std::vector<rclcpp::Publisher<Odometry>::SharedPtr> odom_pubs_;

    // Subscriber
    std::vector<rclcpp::Subscription<Bicycle>::SharedPtr> bicycle_sub_;
    std::vector<rclcpp::Subscription<Twist>::SharedPtr> cmd_vel_sub_;
    std::vector<rclcpp::Subscription<RobotPose>::SharedPtr> robot_pose_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr stage_thread_timer_;

    // Transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

    // Message
    Clock clock_msg_;

    // World
    Stg::World* world_;

    // Model
    std::vector<Stg::ModelPosition*> position_models_;
    std::vector<Stg::ModelRanger*> laser_models_;

    // Misc
    std::mutex data_mutex_;
    rclcpp::Time last_cmd_;

  // Methods -------------------------------------------------------------------
  public:
    StageNode(int argc, char** argv);
    ~StageNode();

  private:
    void stageThread(int argc, char** argv);
    void stageUpdateCallback();
    std::string mapName(const std::string& topic, unsigned robot_id) const;

    void bicycleCallback(
      const Bicycle::ConstSharedPtr& msg,
      unsigned ribit_id
    );
    void cmdVelCallback(const Twist::ConstSharedPtr& msg, unsigned robot_id);
    void robotPoseCallback(
      const RobotPose::ConstSharedPtr& msg,
      unsigned robot_id
    );

    void broadcastTF();
    void broadcastStaticTF();

}; /* class StageNode */

} /* namespace tug_stage_ros2 */

#endif /* _TUG_STAGE_ROS2__STAGE_NODE_HPP_ */
#ifndef _TUG_TURTLEBOT4__NAV_NODE_HPP_
#define _TUG_TURTLEBOT4__NAV_NODE_HPP_

#include "rclcpp/node.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace tug_turtlebot4
{

class NavNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  private:
    using LaserScan = sensor_msgs::msg::LaserScan;
    using Pose = geometry_msgs::msg::Pose;
    using PoseArray = geometry_msgs::msg::PoseArray;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using Twist = geometry_msgs::msg::Twist;

  // Variables -----------------------------------------------------------------
  private:
    // Publishers
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;

    // Subscriptions
    rclcpp::Subscription<PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<PoseArray>::SharedPtr pose_sub_;

    // Variables
    Pose goal_pose;
    Pose last_pose;

    double yaw_goal_pose;
    double yaw_last_pose;

    double theta_pos_goal;
    double dist_goal;

  // Methods -------------------------------------------------------------------
  public:
    NavNode();
    ~NavNode();

  private:
    void goalPoseCallback(const PoseStamped::ConstSharedPtr& goal);
    void laserScanCallback(const LaserScan::ConstSharedPtr& scan);
    void poseCallback(const Pose& pose);

    double yawFrowmPose(Pose pose);
};

} /* namespace tug_turtlebot4 */

#endif /* _TUG_TURTLEBOT4__NAV_NODE_HPP_ */
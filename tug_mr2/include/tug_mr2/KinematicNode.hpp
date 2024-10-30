#ifndef _TUG_MR2__MR2_NODE_HPP_
#define _TUG_MR2__MR2_NODE_HPP_

#include "rclcpp/node.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tug_msgs/msg/bicycle.hpp"
#include "tug_msgs/msg/movement.hpp"
#include "tug_msgs/msg/robot_pose.hpp"

namespace tug_mr2
{

class KinematicNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  private:
    using Twist = geometry_msgs::msg::Twist;

    using Bicycle = tug_msgs::msg::Bicycle;
    using Movement = tug_msgs::msg::Movement;
    using RobotPose = tug_msgs::msg::RobotPose;

  // Variables -----------------------------------------------------------------
  private:
    // Publisher
    rclcpp::Publisher<Bicycle>::SharedPtr bicycle_pub_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<RobotPose>::SharedPtr robot_pose_pub_;

    // Subscriber
    rclcpp::Subscription<Movement>::SharedPtr movement_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Messages
    std::vector<Bicycle> bicycle_msgs_;
    std::vector<Twist> cmd_vel_msgs_;
    std::vector<RobotPose> robot_pose_msgs_;

    // Misc
    unsigned mode_;
    unsigned step_count_;

  // Methods -------------------------------------------------------------------
  public:
    KinematicNode();
    ~KinematicNode();

  private:
    void movementCallback(const Movement::ConstSharedPtr& msg);

    void step();

}; /* class KinematicNode */

} /* namespace tug_mr2 */

#endif /* _TUG_MR2__MR2_NODE_HPP_ */
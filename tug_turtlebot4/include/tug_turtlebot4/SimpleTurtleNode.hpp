#ifndef _TUG_TURTLEBOT4__SIMPLE_TURTLE_HPP_
#define _TUG_TURTLEBOT4__SIMPLE_TURTLE_HPP_

#include <fstream>
#include <vector>

#include "rclcpp/node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tug_msgs/msg/wheel_encoder.hpp"

namespace tug_turtlebot4
{

class SimpleTurtleNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  private:
    using Twist = geometry_msgs::msg::Twist;
    using Pose = geometry_msgs::msg::Pose;
    using PoseArray = geometry_msgs::msg::PoseArray;
    using WheelEncoder = tug_msgs::msg::WheelEncoder;

  // Variables -----------------------------------------------------------------
  private:
    rclcpp::Subscription<WheelEncoder>::SharedPtr wheel_enc_sub_;
    rclcpp::Subscription<PoseArray>::SharedPtr pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
    std::vector<Twist> cmd_vel_msgs_;

    std::fstream odom_file_;
    std::fstream pose_file_;

  // Methods -------------------------------------------------------------------
  public:
    SimpleTurtleNode();
    ~SimpleTurtleNode();

  private:
    void wheelEncoderCallback(const WheelEncoder::ConstSharedPtr& msg);
    void poseCallback(const Pose& msg);
    void publishTransform(
      const tf2::Quaternion& rotation,
      const tf2::Vector3& translation
    );
    void step();
    void initMotionPattern();

}; /* class SimpleTurtleNode */

} /* namespace tug_turtlebot4 */

#endif /* _TUG_TURTLEBOT4__SIMPLE_TURTLE_HPP_ */
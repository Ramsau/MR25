#include "tug_mr2/KinematicNode.hpp"

#include <chrono>
#include <functional>

namespace tug_mr2
{
  
KinematicNode::KinematicNode() :
  Node("kinematic_node", "tug_mr2"),
  mode_(0),
  step_count_(0)
{
  // Init Publisher
  robot_pose_pub_ = create_publisher<RobotPose>(
    "/tug_stage_ros2/robot_0/cmd_pose",
    10
  );
  cmd_vel_pub_ = create_publisher<Twist>("/tug_stage_ros2/robot_0/cmd_vel", 10);
  bicycle_pub_ = create_publisher<Bicycle>(
    "/tug_stage_ros2/robot_0/cmd_bicycle",
    10
  );

  // Init Subscription
  movement_sub_ = create_subscription<Movement>(
    "init_movement",
    10,
    std::bind(&KinematicNode::movementCallback, this, std::placeholders::_1)
  );

  // Init timer
  timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::milliseconds(50),
    std::bind(&KinematicNode::step,this)
  );
}

// -----------------------------------------------------------------------------
KinematicNode::~KinematicNode()
{

}

// -----------------------------------------------------------------------------
void KinematicNode::movementCallback(const Movement::ConstSharedPtr& msg)
{
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Starting new movement with mode: "
      << static_cast<unsigned>(msg->mode)
  );

  mode_ = msg->mode;
  step_count_ = 0;

  robot_pose_msgs_.clear();

  // TODO: Compute the robot's position and heading
  //       Push the calculated values to the robot_pose_msgs_ vector

  float a = 5;
  float b = 4;
  float path_time = 30.0;
  float command_freq = 20.0;
  int num_commands = path_time * command_freq;
  float pi = 3.14159265358979323846;
  for (int i = 0; i < num_commands; i++) {
    auto pose = RobotPose();
    float t = static_cast<float>(i) / static_cast<float>(num_commands);
    pose.set__x(a * sin(2 * pi * t));
    pose.set__y(b * sin(4 * pi * t));

    pose.set__yaw(atan2(b * 4 * pi * cos(4 * pi * t), a * 2 * pi * cos(2 * pi * t)));
    robot_pose_msgs_.push_back(pose);
  }

  cmd_vel_msgs_.clear();

  // TODO: Compute the robot's linear and angular velocities
  //       Push the calculated values to the cmd_vel_msgs_ vector

  bicycle_msgs_.clear();

  // TODO: Compute the robot's translational velocity and steering angle
  //       Push the calculated values to the bicycle_msgs_ vector

  step_count_++;
}

// -----------------------------------------------------------------------------
void KinematicNode::step()
{
  switch (mode_)
  {
    case Movement::MOVEMENT_POSE:
    {
      if (step_count_ < robot_pose_msgs_.size())
        robot_pose_pub_->publish(robot_pose_msgs_[step_count_++]);
      break;
    }

    case Movement::MOVEMENT_VELOCITY:
    {
      if (step_count_ < cmd_vel_msgs_.size())
        cmd_vel_pub_->publish(cmd_vel_msgs_[step_count_++]);
      break;
    }

    case Movement::MOVEMENT_BICYCLE:
    {
      if (step_count_ < bicycle_msgs_.size())
        bicycle_pub_->publish(bicycle_msgs_[step_count_++]);
      break;
    }
    
    default:
      RCLCPP_WARN_STREAM(get_logger(), "Invalid movement mode: " << mode_);
  }
}

} /* namespace tug_mr2 */
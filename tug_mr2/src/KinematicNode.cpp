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

  // Compute the robot's position and heading
  //       Push the calculated values to the robot_pose_msgs_ vector

  double a = 5;
  double b = 4;
  double path_time = 30.0;
  double command_freq = 20.0;
  int num_commands = path_time * command_freq;
  double pi = 3.14159265358979323846;

  double Omega = 2 * pi / path_time;

  for (int i = 0; i < num_commands; i++) {
    auto pose = RobotPose();
    double t = static_cast<double>(i) / command_freq;
    pose.set__x(a * sin(Omega * t));
    pose.set__y(b * sin(2 * Omega * t));

    pose.set__yaw(atan2(b * 2 * Omega * cos(2 * Omega * t), a * Omega * cos(Omega * t)));
    robot_pose_msgs_.push_back(pose);
  }

  cmd_vel_msgs_.clear();

  // TODO: Compute the robot's linear and angular velocities
  //       Push the calculated values to the cmd_vel_msgs_ vector

  for (int i = 0; i < num_commands; i++) {
    double t = static_cast<double>(i) / command_freq;

    auto vel = Twist();
    double vx = a * Omega * cos(Omega * t);
    double vy = b * 2 * Omega * cos(2 * Omega * t);
    double speed = hypot(vx, vy);
    geometry_msgs::msg::Vector3 linear;
    linear.set__x(speed);
    linear.set__y(0.0);
    linear.set__z(0.0);
    vel.set__linear(linear);

    double ax = -1 * pow(Omega, 2) * a * sin(Omega * t);
    double ay = -4 * pow(Omega, 2) * b * sin(2 * Omega * t);
    
    double k = (vx * ay - ax * vy) / sqrt(pow(pow(vx, 2) + pow(vy, 2), 3));
    double w = (vx * ay - ax * vy) / (pow(vx, 2) + pow(vy,2));

    double r = 1 / k;
    //both yaw_rates should be the same
    double yaw_rate = w;
    yaw_rate = speed / r;
    if (i % 10 == 0)
      RCLCPP_WARN_STREAM(get_logger(), "curveture: " << k << " w: " << w << " r: " << r << " yaw: " << yaw_rate);

    geometry_msgs::msg::Vector3 angular;
    angular.set__x(0.0);
    angular.set__y(0.0);
    angular.set__z(yaw_rate);
    vel.set__angular(angular);
    cmd_vel_msgs_.push_back(vel);
  }

  bicycle_msgs_.clear();

  // TODO: Compute the robot's translational velocity and steering angle
  //       Push the calculated values to the bicycle_msgs_ vector

  double L = 0.8;

  for (int i = 0; i < num_commands; i++) {
    double t = static_cast<double>(i) / command_freq;

    auto bike_command = Bicycle();
    double vx = a * Omega * cos(Omega * t);
    double vy = b * 2 * Omega * cos(2 * Omega * t);
    double speed = hypot(vx, vy);
    bike_command.set__velocity(speed);

    double ax = -1 * pow(Omega, 2) * a * sin(Omega * t);
    double ay = -4 * pow(Omega, 2) * b * sin(2 * Omega * t);
    
    double k = (vx * ay - ax * vy) / sqrt(pow(pow(vx, 2) + pow(vy, 2), 3));
    double r = 1 / k;
    double gamma = atan(L/r);

    if (i % 10 == 0)
      RCLCPP_WARN_STREAM(get_logger(), "curveture: " << k << " r: " << r << " angle: " << gamma);

    bike_command.set__steering_angle(gamma);
    bicycle_msgs_.push_back(bike_command);
  }

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
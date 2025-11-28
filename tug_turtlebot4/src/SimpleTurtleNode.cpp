#include "tug_turtlebot4/SimpleTurtleNode.hpp"

#include <cstdint>
#include <functional>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tug_turtlebot4
{

// -----------------------------------------------------------------------------
SimpleTurtleNode::SimpleTurtleNode() :
  Node("simple_turtle", "tug_turtlebot4")
{
  wheel_enc_sub_ = create_subscription<WheelEncoder>(
    "wheel_encoder",
    10,
    std::bind(
      &SimpleTurtleNode::wheelEncoderCallback,
      this,
      std::placeholders::_1
    )
  );

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "pose",
    10,
    [this](const PoseArray::ConstSharedPtr& msg)
    {
      poseCallback(msg->poses[0]);
    }
  );

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  step_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::milliseconds(50),
    std::bind(&SimpleTurtleNode::step, this)
  );

  cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);

  initMotionPattern();

  odom_file_.open("odom.dat", std::ios::out | std::ios::trunc);
  pose_file_.open("pose.dat", std::ios::out | std::ios::trunc);
}

// -----------------------------------------------------------------------------
SimpleTurtleNode::~SimpleTurtleNode()
{
  odom_file_.close();
  pose_file_.close();
}

// -----------------------------------------------------------------------------
void SimpleTurtleNode::wheelEncoderCallback(
  const WheelEncoder::ConstSharedPtr& msg
)
{
  // Add your odometry estimation here
  static size_t call_num = 0;
  static int last_left_val = msg->left_counter;
  static int last_right_val = msg->right_counter;
  static tf2::Vector3 translation{0, 0, 0};
  static double theta = 0;

  const double ticks_per_rev = 2573;
  const double wheel_diam = 0.0715;
  const double wheel_circum = wheel_diam * M_PI;
  const double wheel_base = 0.233;

  int moved_ticks_left = msg->left_counter - last_left_val;
  int moved_ticks_right = msg->right_counter - last_right_val;
  if (moved_ticks_left > 1 << 15) {
    moved_ticks_left -= 1 << 16;
  } else if (moved_ticks_left < -(1 << 15)) {
    moved_ticks_left += 1 << 16;
  }
  if (moved_ticks_right > 1 << 15) {
    moved_ticks_right -= 1 << 16;
  } else if (moved_ticks_right < -(1 << 15)) {
    moved_ticks_right += 1 << 16;
  }
  last_left_val = msg->left_counter;
  last_right_val = msg->right_counter;

  double dist_left = moved_ticks_left * wheel_circum / ticks_per_rev;
  double dist_right = moved_ticks_right * wheel_circum / ticks_per_rev;

  double delta_s = (dist_left + dist_right) / 2.0;
  double delta_theta = (dist_right - dist_left) / wheel_base;

  translation += tf2::Vector3(
    delta_s * cos(theta + delta_theta / 2.0),
    delta_s * sin(theta + delta_theta / 2.0),
    0
  );
  theta += delta_theta;

  tf2::Quaternion quat{0, 0, 0, 1};
  quat.setRPY(0, 0, theta);
  publishTransform(quat, translation);
}

// -----------------------------------------------------------------------------
void SimpleTurtleNode::poseCallback(const Pose& msg)
{
  // TODO: Use pose callback for calibration
}

// -----------------------------------------------------------------------------
void SimpleTurtleNode::publishTransform(
  const tf2::Quaternion& rotation, 
  const tf2::Vector3& translation
)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = get_clock()->now();
  transform.header.frame_id  = "odom";
  transform.child_frame_id = "my_base";
  transform.transform.rotation = tf2::toMsg(rotation);
  transform.transform.translation = tf2::toMsg(translation);

  tf_broadcaster_->sendTransform(transform);
}

// -----------------------------------------------------------------------------
void SimpleTurtleNode::step()
{
  static size_t step = 0;

  if (step < cmd_vel_msgs_.size())
    cmd_vel_pub_->publish(cmd_vel_msgs_[step++]);
  else if (step == cmd_vel_msgs_.size())
  {
    cmd_vel_pub_->publish(Twist());
    step++;
  }
}

// -----------------------------------------------------------------------------
void SimpleTurtleNode::initMotionPattern()
{
  //TODO: Add motion pattern
  cmd_vel_msgs_.push_back(Twist());
}

} /* namespace tug_turtlebot4 */

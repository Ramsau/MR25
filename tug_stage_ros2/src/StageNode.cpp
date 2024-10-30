#include "tug_stage_ros2/StageNode.hpp"

#include <chrono>
#include <functional>
#include <sstream>

#include "tf2/LinearMath/Quaternion.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tug_stage_ros2
{

// -----------------------------------------------------------------------------
StageNode::StageNode(int argc, char** argv) :
  Node("stage_node", "tug_stage_ros2")
{
  // Create callback group
  stage_cb_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive
  );

  // Start stage thread
  stage_thread_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this, argc, argv](){ stageThread(argc, argv); },
    stage_cb_group_
  );
}

// -----------------------------------------------------------------------------
StageNode::~StageNode()
{
  
}

// -----------------------------------------------------------------------------
void StageNode::stageThread(int argc, char** argv)
{
  // Disable timer
  stage_thread_timer_->cancel();

  // Init stage
  Stg::Init(&argc, &argv);
  world_ = new Stg::WorldGui(800, 600, "TUG Stage ROS2");

  world_->Load(argv[1]);
  world_->AddUpdateCallback(
    [](Stg::World*, void* raw_node)
    {
      reinterpret_cast<StageNode*>(raw_node)->stageUpdateCallback();
      return 0;
    },
    this
  );

  world_->ForEachDescendant(
    [](Stg::Model* model, void* raw_node)
    {
      StageNode* node = reinterpret_cast<StageNode*>(raw_node);

      if (Stg::ModelPosition* pm = dynamic_cast<Stg::ModelPosition*>(model))
        node->position_models_.push_back(pm);

      if (Stg::ModelRanger* lm = dynamic_cast<Stg::ModelRanger*>(model))
        node->laser_models_.push_back(lm);
      
      return 0;
    },
    this
  );

  if (position_models_.size() != laser_models_.size())
  {
    RCLCPP_FATAL(
      get_logger(),
      "Number of position models and ranger models must be equal"
    );
    return;
  }

  // Create publishers and subscribers
  for (unsigned i = 0; i < position_models_.size(); i++)
  {
    position_models_[i]->Subscribe();
    laser_models_[i]->Subscribe();

    laser_pubs_.push_back(create_publisher<LaserScan>(mapName("scan", i), 10));
    odom_pubs_.push_back(create_publisher<Odometry>(mapName("odom", i), 10));

    bicycle_sub_.push_back(
      create_subscription<Bicycle>(
        mapName("cmd_bicycle", i),
        10,
        [this, i](const Bicycle::ConstSharedPtr& msg)
        {
          bicycleCallback(msg, i);
        }
      )
    );

    cmd_vel_sub_.push_back(
      create_subscription<Twist>(
        mapName("cmd_vel", i),
        10,
        [this, i](const Twist::ConstSharedPtr& msg)
        {
          cmdVelCallback(msg, i);
        }
      )
    );

    robot_pose_sub_.push_back(
      create_subscription<RobotPose>(
        mapName("cmd_pose", i),
        10,
        [this, i](const RobotPose::ConstSharedPtr& msg){
          robotPoseCallback(msg, i);
        }
      )
    );
  }

  // Create clock publisher
  clock_pub_ = create_publisher<Clock>("/clock", 10);

  // Create transform broadcaster
  tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  // Publish static TF
  broadcastStaticTF();

  // Init last cmd
  last_cmd_ = get_clock()->now();

  // Start simulation
  world_->Start();

  // Spin the simulation until either ros is terminated or the stage simulator
  // is closed
  while (rclcpp::ok() && !world_->TestQuit())
  {
    Fl::wait();
  }

  if (world_->TestQuit())
    rclcpp::shutdown(nullptr, "Stage simulator closed");
}

// -----------------------------------------------------------------------------
void StageNode::stageUpdateCallback()
{
  // Lock data mutex
  std::scoped_lock lock(data_mutex_);

  // Update clock
  unsigned long sim_time_usec = world_->SimTimeNow();
  clock_msg_.clock.sec = sim_time_usec / 1000000UL;
  clock_msg_.clock.nanosec = (sim_time_usec % 1000000UL) * 1000UL;
  clock_pub_->publish(clock_msg_);

  // Update TF tree
  broadcastTF();

  for (unsigned i = 0; i < position_models_.size(); i++)
  {
    // Stop the robot if no speed command is received for more than 1 second
    if ((get_clock()->now() - last_cmd_) > std::chrono::milliseconds(50))
      position_models_[i]->SetSpeed(0.0, 0.0, 0.0);

    // Publish laser scan
    const Stg::ModelRanger::Sensor& sensor = laser_models_[i]->GetSensors()[0];

    LaserScan scan;
    scan.header.stamp = get_clock()->now();
    scan.header.frame_id = mapName("base_laser_link", i);
    scan.angle_min = -sensor.fov / 2.0;
    scan.angle_max = sensor.fov / 2.0;
    scan.angle_increment
      = sensor.fov / static_cast<double>(sensor.sample_count - 1);
    scan.range_min = sensor.range.min;
    scan.range_max = sensor.range.max;
    scan.ranges.resize(sensor.ranges.size());
    scan.intensities.resize(sensor.intensities.size());

    for (unsigned j = 0; j < sensor.ranges.size(); j++)
    {
      scan.ranges[j] = sensor.ranges[sensor.ranges.size() - 1 - j];
      scan.intensities[j]
        = sensor.intensities[sensor.intensities.size() - 1 - j];
    }

    laser_pubs_[i]->publish(scan);

    // Publish odom
    Odometry odom;
    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = mapName("base_footprint", i);
    odom.pose.pose.position.x = position_models_[i]->est_pose.x;
    odom.pose.pose.position.y = position_models_[i]->est_pose.y;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, position_models_[i]->est_pose.a);
    odom.pose.pose.orientation = tf2::toMsg(orientation);

    const Stg::Velocity& vel = position_models_[i]->GetVelocity();
    odom.twist.twist.linear.x = vel.x;
    odom.twist.twist.linear.y = vel.y;
    odom.twist.twist.angular.z = vel.a;

    odom_pubs_[i]->publish(odom);
  }
}

// -----------------------------------------------------------------------------
std::string StageNode::mapName(
  const std::string& topic,
  unsigned robot_id
) const
{
  std::stringstream ss;
  ss << "robot_" << robot_id << "/" << topic;

  return ss.str();
}

// -----------------------------------------------------------------------------
void StageNode::bicycleCallback(
  const Bicycle::ConstSharedPtr& msg,
  unsigned robot_id
)
{
  std::scoped_lock lock(data_mutex_);
  constexpr double BICYCLE_LENGTH = 0.8;
  const double& vel = msg->velocity;

  position_models_[robot_id]->SetSpeed(
    vel,
    0.0,
    vel * std::tan(msg->steering_angle) / BICYCLE_LENGTH
  );

  last_cmd_ = get_clock()->now();
}

// -----------------------------------------------------------------------------
void StageNode::cmdVelCallback(
  const Twist::ConstSharedPtr& msg,
  unsigned robot_id
)
{
  std::scoped_lock lock(data_mutex_);

  position_models_[robot_id]->SetSpeed(
    msg->linear.x,
    msg->linear.y,
    msg->angular.z
  );

  last_cmd_ = get_clock()->now();
}

// -----------------------------------------------------------------------------
void StageNode::robotPoseCallback(
  const RobotPose::ConstSharedPtr& msg,
  unsigned robot_id
)
{
  std::scoped_lock lock(data_mutex_);

  position_models_[robot_id]->SetPose({msg->x, msg->y, 0.0, msg->yaw});

  last_cmd_ = get_clock()->now();
}

// -----------------------------------------------------------------------------
void StageNode::broadcastTF()
{
  for (unsigned i = 0; i < position_models_.size(); i++)
  {
    TransformStamped tf;
    Stg::Pose robot_pose = position_models_[i]->est_pose;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, robot_pose.a);

    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = mapName("base_footprint", i);
    tf.transform.translation.x = robot_pose.x;
    tf.transform.translation.y = robot_pose.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = tf2::toMsg(rotation);

    tf_->sendTransform(tf);
  }
}

// -----------------------------------------------------------------------------
void StageNode::broadcastStaticTF()
{
  for (unsigned i = 0; i < position_models_.size(); i++)
  {
    TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = mapName("base_footprint", i);
    tf.child_frame_id = mapName("mounting_plate", i);
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.1;

    tf_static_->sendTransform(tf);

    Stg::Pose laser_pose = laser_models_[i]->GetPose();
    tf2::Quaternion rotation;
    rotation.setRPY(M_PI, 0.0, laser_pose.a);

    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = mapName("mounting_plate", i);
    tf.child_frame_id = mapName("base_laser_link", i);
    tf.transform.translation.x = laser_pose.x;
    tf.transform.translation.y = laser_pose.y;
    tf.transform.translation.z = 0.05;
    tf.transform.rotation = tf2::toMsg(rotation);

    tf_static_->sendTransform(tf);
  }
}

} /* namespace tug_stage_ros2 */
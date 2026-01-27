#include "tug_turtlebot4/NavNode.hpp"

#include <functional>

namespace tug_turtlebot4
{

// -----------------------------------------------------------------------------
NavNode::NavNode()
  : Node("nav", "tug_turtlebot4")
{
  // Publisher
  cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);

  // Subscriptions
  goal_pose_sub_ = create_subscription<PoseStamped>(
    "goal_pose",
    10,
    std::bind(&NavNode::goalPoseCallback, this, std::placeholders::_1)
  );

  laser_scan_sub_ = create_subscription<LaserScan>(
    "scan",
    10,
    std::bind(&NavNode::laserScanCallback, this, std::placeholders::_1)
  );

  pose_sub_ = create_subscription<PoseArray>(
    "pose",
    10,
    [this](const PoseArray::ConstSharedPtr& pose_array)
    {
      poseCallback(pose_array->poses[0]);
    }
  );

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    this
  );
}

// -----------------------------------------------------------------------------
NavNode::~NavNode()
{}

// -----------------------------------------------------------------------------
void NavNode::goalPoseCallback(const PoseStamped::ConstSharedPtr& goal)
{
  // TODO: Implement reactive navigation

  goal_pose = goal->pose;

  std::cout << "goal Pose x: " << goal->pose.position.x << std::endl;
  std::cout << "goal Pose y: " << goal->pose.position.y << std::endl;
  std::cout << "goal Pose z: " << goal->pose.position.z << std::endl;

  std::cout << "goal Rot x: " << goal->pose.orientation.x << std::endl;
  std::cout << "goal Rot y: " << goal->pose.orientation.y << std::endl;
  std::cout << "goal Rot z: " << goal->pose.orientation.z << std::endl;
  std::cout << "goal Rot w: " << goal->pose.orientation.w << std::endl;
}

// -----------------------------------------------------------------------------
void NavNode::laserScanCallback(const LaserScan::ConstSharedPtr& scan)
{
  float ci = 1;
  float a = 10;
  float b = 1;

  int bins = 20;
  int threshold = 500;

  float mi[scan->ranges.size()];
  float hk[bins] = {0};
  bool hk_bin[bins];

  assert(scan->ranges.size() % bins == 0);

  float angle = scan->angle_min;

  std::cout << "angle min: " << angle << " angle inc: " << scan->angle_increment << std::endl;

  geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform("rplidar_link", scan->header.frame_id, scan->header.stamp);

  bool first_range = true;
  for (size_t i = 0; i < scan->ranges.size(); i++) 
  {
    float range = scan->ranges[i];
    // if (std::isinf(range) || first_range) 
    // {
    //   angle += scan->angle_increment;
    //   first_range = false;
    //   continue;
    // }

    // mi[i] = ci * ci * (a - b * range);
    mi[i] = range; 
  }

  int ranges_per_bin = scan->ranges.size() / bins;

  for (size_t i = 0; i < bins; i++)
  {
    for (size_t j = 0; j < ranges_per_bin; j++)
    {
      hk[i] += mi[i * ranges_per_bin + j];
    } 
  }
  
  for (size_t i = 0; i < bins; i++)
  {
    hk_bin[i] = hk[i] > threshold;
  }

  for (size_t i = 0; i < bins; i++)
  {
    std::cout << "histogram [" << i << "]: " << hk_bin[i] << " " << hk[i] << std::endl;
  }

  // get saved pose and publish cmd_vel here
  // std::cout << "last Pose x: " << last_pose.position.x << std::endl;
  // std::cout << "last Pose y: " << last_pose.position.y << std::endl;
  // std::cout << "last Pose z: " << last_pose.position.z << std::endl;

  Twist test;

  test.linear.x = 1;

  //cmd_vel_pub_->publish(test);
  

  // std::cout << "num ranges: " << scan->ranges.size() << std::endl;
}

// -----------------------------------------------------------------------------
void NavNode::poseCallback(const Pose& pose)
{
  // TODO: Implement reactive navigation

  last_pose = pose;

  // save pose for lidar update
}

} /* namespace tug_turtlebot4 */
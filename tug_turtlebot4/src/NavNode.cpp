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
}

// -----------------------------------------------------------------------------
NavNode::~NavNode()
{}

// -----------------------------------------------------------------------------
void NavNode::goalPoseCallback(const PoseStamped::ConstSharedPtr& goal)
{
  // TODO: Implement reactive navigation

  goal_pose = goal->pose;

  yaw_goal_pose = yawFrowmPose(goal_pose);

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

  int bins = 40;
  int threshold = 130;

  float mi[scan->ranges.size()];
  float hk[bins] = {0};
  float hk_rot[bins] = {0};
  bool hk_bin[bins];

  assert(scan->ranges.size() % bins == 0);

  float angle = scan->angle_min;

  // std::cout << "angle min: " << angle << " angle inc: " << scan->angle_increment << std::endl;

  bool first_range = true;
  for (size_t i = 0; i < scan->ranges.size(); i++) 
  {
    float range = scan->ranges[i];
    if (std::isinf(range) || first_range) 
    {
      // angle += scan->angle_increment;
      first_range = false;
      continue;
    }

    mi[i] = ci * ci * (a - b * range);
    // mi[i] = range; 
  }

  int ranges_per_bin = scan->ranges.size() / bins;

  for (size_t i = 0; i < bins; i++)
  {
    for (size_t j = 0; j < ranges_per_bin; j++)
    {
      hk[i] += mi[i * ranges_per_bin + j];
    } 
  }

  // rotate bins by 90° to better align with robot between index 9 and 10 is straight forawrds and 0 to 19 straight backwards
  for (size_t i = 0; i < bins; i++)
  {
    hk_rot[(i + bins / 4) % bins ] = hk[i];
    // hk_rot[i] = hk[i];
  }
  
  for (size_t i = 0; i < bins; i++)
  {
    hk_bin[i] = hk_rot[i] > threshold;
  }

  for (size_t i = 0; i < bins; i++)
  {
    std::cout << "histogram [" << i << "]: " << hk_bin[i] << " " << hk_rot[i] << std::endl;
  }

  // get saved pose and publish cmd_vel here
  // std::cout << "last Pose x: " << last_pose.position.x << std::endl;
  // std::cout << "last Pose y: " << last_pose.position.y << std::endl;
  // std::cout << "last Pose z: " << last_pose.position.z << std::endl;

  Twist test;

  test.linear.x = 1;
  //cmd_vel_pub_->publish(test);

  double yaw = yawFrowmPose(last_pose);

  double total_yaw = std::fmod(yaw - theta_pos_goal + M_PI, 2 * M_PI) - M_PI;

  double bin_angle_inc = 2 * M_PI / bins;

  int bin = int((total_yaw + M_PI) / bin_angle_inc);

  std::cout << "yaw: " << yaw << " total yaw to goal: " << total_yaw << " bin: " << bin << std::endl;

  bool goal_in_valley = !hk_bin[bin];
  
  std::cout << "goal in valley: " << goal_in_valley << std::endl;
  std::cout << "distance to goal: " << dist_goal << std::endl;


  double epsilon = 0.1;
  double epsilon_dist = 0.1;
  double rotation_speed = 2;
  double linear_speed = 0.5;

  // if (goal_in_valley || dist_goal < epsilon_dist)
  // {
  //   Twist twist_cmd;

  //   if (abs(total_yaw) > epsilon && dist_goal > epsilon_dist)
  //   {
      
  //     rotation_speed *= abs (total_yaw / M_PI);

  //     twist_cmd.angular.z = total_yaw > 0 ? rotation_speed * -1 : rotation_speed;

  //     cmd_vel_pub_->publish(twist_cmd);
  //   }
  //   else
  //   {
  //     if (dist_goal > epsilon_dist)
  //     {
  //       twist_cmd.linear.x = dist_goal > 0.2 ? linear_speed : linear_speed * 0.5;
  //     }

  //     double goal_last_yaw_delta = yaw_goal_pose - yaw_last_pose;
  //     std::cout << "goal yaw delta: " << goal_last_yaw_delta << " dist to goal: " << dist_goal << std::endl;

  //     if (dist_goal <= epsilon_dist && abs(goal_last_yaw_delta) > epsilon)
  //     {
  //       rotation_speed *= abs (goal_last_yaw_delta / M_PI);

  //       twist_cmd.angular.z = goal_last_yaw_delta > 0 ? rotation_speed * -1 : rotation_speed;
  //     }      

  //     cmd_vel_pub_->publish(twist_cmd);
  //   }
  // }

  

  if (!goal_in_valley)
  {
    // find closest sector in a valley to goal sector
    int closest_valley_index = 99;
    for (size_t i = 1; i <= bins / 2; i++)
    {
      if(hk_bin[(bin + i) % bins] == 0)
      {
        closest_valley_index = (bin + i) % bins;
        break;
      }

      if (hk_bin[(bin - i + bins) % bins] == 0)
      {
        closest_valley_index = (bin - i + bins) % bins;
        break;
      }
    }

    std::cout << "closest valley index: " << closest_valley_index << std::endl;

    int valley_width_right = 0;
    int valley_width_left = 0;
    for (size_t i = 1; i <= bins / 2; i++)
    {
      if(hk_bin[(closest_valley_index + i) % bins] == 0)
      {
        valley_width_right++;
      }
      else
        break;
    }

    for (size_t i = 1; i <= bins / 2; i++)
    {
      if (hk_bin[(closest_valley_index - i + bins) % bins] == 0)
      {
        valley_width_left++;
      }
      else
        break;
    }

    std::cout << "valley right: " << valley_width_right << " valley left: " << valley_width_left << std::endl;
    assert((valley_width_left == 0 && valley_width_right > 0) || (valley_width_right == 0 && valley_width_left > 0));

    int valley_width = valley_width_left > 0 ? valley_width_left : valley_width_right;

    std::cout << "valley width: " << valley_width << std::endl;

    int k_sol = 0;
    int m = 2;

    if (valley_width >= m)
    {
      if (valley_width_left > 0)
      {
        k_sol = (closest_valley_index - (m / 2)) % bins;
      }

      if (valley_width_right > 0)
      {
        k_sol = (closest_valley_index + (m / 2)) % bins;
      }
    }
    else
    {
      if (valley_width_left > 0)
      {
        k_sol = (closest_valley_index - (valley_width / 2)) % bins;
      }

      if (valley_width_right > 0)
      {
        k_sol = (closest_valley_index + (valley_width / 2)) % bins;
      }
    }

    std::cout << "ksol: " << k_sol << std::endl;

    double yaw_sol = k_sol * bin_angle_inc - M_PI;

    total_yaw = std::fmod(yaw - yaw_sol + M_PI, 2 * M_PI) - M_PI;

    std::cout << "yaw: " << yaw << " total yaw to k sol: " << total_yaw << " bin (k sol): " << k_sol << std::endl;

  }

  if (true)
  {
    Twist twist_cmd;

    if (abs(total_yaw) > epsilon && dist_goal > epsilon_dist)
    {
      
      rotation_speed *= abs (total_yaw / M_PI);

      twist_cmd.angular.z = total_yaw > 0 ? rotation_speed * -1 : rotation_speed;

      
    }
    else
    {
      if (dist_goal > epsilon_dist)
      {
        twist_cmd.linear.x = dist_goal > 0.2 ? linear_speed : linear_speed * 0.5;
      }

      double goal_last_yaw_delta = yaw_goal_pose - yaw_last_pose;
      std::cout << "goal yaw delta: " << goal_last_yaw_delta << " dist to goal: " << dist_goal << std::endl;

      if (dist_goal <= epsilon_dist && abs(goal_last_yaw_delta) > epsilon)
      {
        rotation_speed *= abs (goal_last_yaw_delta / M_PI);

        twist_cmd.angular.z = goal_last_yaw_delta > 0 ? rotation_speed * -1 : rotation_speed;
      }      
    }

    cmd_vel_pub_->publish(twist_cmd);
  }

  if ((goal_pose.position.x != 0 || goal_pose.position.x != 0) && dist_goal < epsilon_dist && abs(yaw_goal_pose - yaw_last_pose) < epsilon)
  {
    std::cout << "destination reached!!!" << std::endl;
  }


  // std::cout << "num ranges: " << scan->ranges.size() << std::endl;
}

// -----------------------------------------------------------------------------
void NavNode::poseCallback(const Pose& pose)
{
  // TODO: Implement reactive navigation

  last_pose = pose;
  yaw_last_pose = yawFrowmPose(last_pose);

  if (goal_pose.position.x != 0 || goal_pose.position.x != 0)
  {
    // std::cout << "goal found!" << std::endl;

    float delta_x = goal_pose.position.x - pose.position.x;
    float delta_y = goal_pose.position.y - pose.position.y;


    theta_pos_goal = std::atan2(delta_y, delta_x);

    dist_goal = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    // std::cout << "theta pos goal: " << theta_pos_goal << std::endl;
  }
  else
  {
    // std::cout << "no goal!" << std::endl;
    dist_goal = 0;
    theta_pos_goal = 0;
  }

  

  // save pose for lidar update
}

double NavNode::yawFrowmPose(Pose pose)
{
  double x = pose.orientation.x;
  double y = pose.orientation.y;
  double z = pose.orientation.z;
  double w = pose.orientation.w;

  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

} /* namespace tug_turtlebot4 */
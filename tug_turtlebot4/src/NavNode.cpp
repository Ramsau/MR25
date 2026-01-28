#include "tug_turtlebot4/NavNode.hpp"

#include <complex>
#include <functional>

namespace tug_turtlebot4
{

// -----------------------------------------------------------------------------
NavNode::NavNode()
  : Node("nav", "tug_turtlebot4")
{
  // Publisher
  cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);
  marker_pub_ = create_publisher<MarkerArray>("marker", 10);

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
}

// -----------------------------------------------------------------------------
void NavNode::laserScanCallback(const LaserScan::ConstSharedPtr& scan)
{

  const float ci = 1;
  const float a = 2;
  const float b = 0.5;
  const int m = 6;

  const int bins = 40;
  int ranges_per_bin = scan->ranges.size() / bins;
  int threshold = (a - b * 1.0) * ranges_per_bin;

  float mi[scan->ranges.size()];
  float hk[bins] = {0};
  float hk_rot[bins] = {0};
  bool hk_bin[bins];

  assert(scan->ranges.size() % bins == 0);

  bool first_range = true;
  double closest_range = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < scan->ranges.size(); i++) 
  {
    float range = scan->ranges[i];
    if (std::isinf(range) || first_range) 
    {
      // angle += scan->angle_increment;
      first_range = false;
      // assign infinite value
      mi[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    mi[i] = ci * ci * (a - b * range);
    if (range < closest_range) closest_range = range;
  }


  for (size_t i = 0; i < bins; i++)
  {
    for (size_t j = 0; j < ranges_per_bin; j++)
    {
      float val = mi[i * ranges_per_bin + j];
      if (!std::isnan(val)) {
        hk[i] += val;
      }
    }
  }

  // rotate bins by 90° to align with robot
  for (size_t i = 0; i < bins; i++)
  {
    hk_rot[(i + bins - bins / 4) % bins ] = hk[i];
  }

  // threshold
  for (size_t i = 0; i < bins; i++)
  {
    hk_bin[i] = hk_rot[i] > threshold;
  }

  double yaw_self = yawFrowmPose(last_pose);
  double yaw_to_goal = normalizeAngle(yaw_self - theta_pos_goal);

  double bin_angle_inc = 2 * M_PI / bins;
  int bin_in_goal_direction = (static_cast<int>(-yaw_to_goal / bin_angle_inc) + bins) % bins;

  bool goal_in_valley = !hk_bin[bin_in_goal_direction];
  int closest_valley_index = -1;

  // find closest sector in a valley to goal sector
  for (size_t i = 0; i <= bins / 2; i++)
  {
    if(hk_bin[(bin_in_goal_direction + i) % bins] == 0)
    {
      closest_valley_index = (bin_in_goal_direction + i) % bins;
      break;
    }

    if (hk_bin[(bin_in_goal_direction - i + bins) % bins] == 0)
    {
      closest_valley_index = (bin_in_goal_direction - i + bins) % bins;
      break;
    }
  }

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

  int valley_width = valley_width_left > 0 ? valley_width_left : valley_width_right;

  // std::cout << "valley width: " << valley_width << std::endl;

  int k_sol = -1;
  double yaw_sol = 0;
  if (goal_in_valley) {
    k_sol = bin_in_goal_direction;
    yaw_sol = yaw_to_goal;
  }
  else if (valley_width >= m)
  {
    if (valley_width_left > 0)
    {
      k_sol = (closest_valley_index - (m / 2) + bins) % bins;
    }

    if (valley_width_right > 0)
    {
      k_sol = (closest_valley_index + (m / 2) + bins) % bins;
    }
    yaw_sol = normalizeAngle(-k_sol * bin_angle_inc);
  }
  else
  {
     if (valley_width_left > 0)
    {
      k_sol = (closest_valley_index - (valley_width / 2) + bins) % bins;
    }

    if (valley_width_right > 0)
    {
      k_sol = (closest_valley_index + (valley_width / 2) + bins) % bins;
    }
    yaw_sol = normalizeAngle(-k_sol * bin_angle_inc);
  }

  publishCmdVel(std::min(closest_range, dist_goal), yaw_sol, yaw_goal_pose);

  // std::cout << "num ranges: " << scan->ranges.size() << std::endl;
  publishMarkers(hk_rot, bins, a, b, ranges_per_bin, -yaw_to_goal, -yaw_sol, threshold, k_sol, closest_valley_index, valley_width_left, valley_width_right);
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

void NavNode::publishMarkers(float* bins, int size, float a, float b, int ranges_per_bin, float yaw_goal_relative, float yaw_selected_relative, double threshold, int selected_bin, int closest_valley_index, int valley_width_left, int valley_width_right) {
  visualization_msgs::msg::MarkerArray marker_array;

  Marker del_marker;
  del_marker.action = Marker::DELETEALL;
  marker_array.markers.push_back(del_marker);

  // histograms
  for (int i = 0; i < size; i++) {
    float angle = i * 2 * M_PI / size;
    float length = (bins[i] / ranges_per_bin - a) / -b;
    float x = length * std::cos(angle);
    float y = length * std::sin(angle);

    Marker marker;
    marker.header.frame_id = "base_link";
    marker.ns = "histogram";
    marker.id = i;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.05;
    marker.color.a = 1.0;
    marker.color.r = bins[i] > threshold ? 1.0 : 0.0;
    marker.color.g = bins[i] > threshold ? 0.0 : 0.7;
    if (i == selected_bin) {
      marker.color.b = 1.0;
    } else if (
      ((closest_valley_index - i + size) % size <= valley_width_left) ||
      ((i - closest_valley_index + size) % size <= valley_width_right)
    ){
      marker.color.b = 0.6;
    } else {
      marker.color.b = 0.0;
    }

    Point start;
    start.x = 0;
    start.y = 0;
    start.z = 0;
    Point end;
    end.x = x;
    end.y = y;
    end.z = 0;
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker_array.markers.push_back(marker);
  }

  // goal yaw
  Marker marker;
  marker.header.frame_id = "base_link";
  marker.ns = "goal_yaw";
  marker.id = 0;
  marker.type = Marker::ARROW;
  marker.action = Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  Point start;
  start.x = 0;
  start.y = 0;
  start.z = 0;
  Point end;
  end.x = 0.5 * std::cos(yaw_goal_relative);
  end.y = 0.5 * std::sin(yaw_goal_relative);
  end.z = 0;
  marker.points.push_back(start);
  marker.points.push_back(end);
  marker_array.markers.push_back(marker);

  // selected yaw
  Marker marker_sel;
  marker_sel.header.frame_id = "base_link";
  marker_sel.ns = "goal_yaw";
  marker_sel.id = 1;
  marker_sel.type = Marker::ARROW;
  marker_sel.action = Marker::ADD;
  marker_sel.scale.x = 0.05;
  marker_sel.scale.y = 0.2;
  marker_sel.color.a = 1.0;
  marker_sel.color.r = 0.0;
  marker_sel.color.g = 1.0;
  marker_sel.color.b = 1.0;
  Point start_sel;
  start_sel.x = 0;
  start_sel.y = 0;
  start_sel.z = 0;
  Point end_sel;
  end_sel.x= 0.5 * std::cos(yaw_selected_relative);
  end_sel.y= 0.5 * std::sin(yaw_selected_relative);
  end_sel.z= 0;
  marker_sel.points.push_back(start_sel);
  marker_sel.points.push_back(end_sel);
  marker_array.markers.push_back(marker_sel);

  marker_pub_->publish(marker_array);
}

double NavNode::normalizeAngle(double angle) {
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle - M_PI;
}

void NavNode::publishCmdVel(double dist_to_goal, double yaw_to_goal, double final_yaw) {
  double epsilon_theta = 0.1;
  double max_yaw_delta_while_moving = M_PI / 4;
  double epsilon_dist = 0.1;
  double rotation_speed = 0.8;
  double linear_speed = 0.8;

  double yaw_goal_delta = normalizeAngle(final_yaw - yaw_last_pose);
  static bool destination_reached = false;

  Twist twist_cmd;
  if (dist_to_goal > epsilon_dist) {
    destination_reached = false;
    twist_cmd.angular.z = -rotation_speed * yaw_to_goal / M_2_PI;
    if (std::abs(yaw_to_goal) < std::abs(max_yaw_delta_while_moving)) {
      twist_cmd.linear.x = std::min(linear_speed, linear_speed * dist_to_goal);
    }
  } else if (std::abs(yaw_goal_delta) > epsilon_theta) {
    destination_reached = false;
    twist_cmd.angular.z =  rotation_speed * yaw_goal_delta / M_2_PI;
  }
  else if (!destination_reached)
  {
    std::cout << "destination reached!!!" << std::endl;
    destination_reached = true;
  }
  cmd_vel_pub_->publish(twist_cmd);


}
} /* namespace tug_turtlebot4 */
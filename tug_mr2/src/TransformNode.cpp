#include "tug_mr2/TransformNode.hpp"

#include <functional>
#include <eigen3/Eigen/Core>

namespace tug_mr2
{

// -----------------------------------------------------------------------------
TransformNode::TransformNode() :
  Node("transform_node", "tug_mr2")
{
  // Publisher
  point_cloud_pub_ = create_publisher<PointCloud>("point_cloud", 10);

  // Subscriber
  laser_sub_ = create_subscription<LaserScan>(
    "/tug_stage_ros2/robot_0/scan",
    10,
    std::bind(&TransformNode::laserCallback, this, std::placeholders::_1)
  );
  odom_sub_ = create_subscription<Odometry>(
    "/tug_stage_ros2/robot_0/odom",
    10,
    std::bind(&TransformNode::odomCallback, this, std::placeholders::_1)
  );
}

// -----------------------------------------------------------------------------
TransformNode::~TransformNode()
{
  
}

// -----------------------------------------------------------------------------
void TransformNode::laserCallback(const LaserScan::ConstSharedPtr& msg)
{
  PointCloud pc;
  pc.header.stamp = get_clock()->now();
  pc.header.frame_id = "odom";

  // TODO: Add your manual transformation calculation here
  //       You can add variables to the class if neded
  float angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
      // base point: laser frame
      Eigen::Vector4d pt = {
        (float)msg->ranges[i] * cos(angle),
        (float)msg->ranges[i] * sin(angle),
        0.0,
        1.0
      };

      // laser -> mounting plate
      double phi = 84.0 / 180.0 * M_PI;
      double theta = M_PI;
      Eigen::MatrixX4d Translate_laser_mounting_plate{
        {1, 0, 0, 0.04},
        {0, 1, 0, 0},
        {0, 0, 1, 0.05},
        {0, 0, 0, 1},
      };
      Eigen::MatrixX4d Phi_laser_mounting_plate{
        {cos(phi), -sin(phi), 0, 0},
        {sin(phi), cos(phi), 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
      };
      Eigen::MatrixX4d Theta_laser_mounting_plate{
        {1, 0, 0, 0},
        {0, cos(theta), -sin(theta), 0},
        {0, sin(theta), cos(theta), 0},
        {0, 0, 0, 1}
      };
      pt = Translate_laser_mounting_plate * Phi_laser_mounting_plate * Theta_laser_mounting_plate * pt;

      // mounting plate -> base footprint
      Eigen::MatrixX4d Translate_mounting_plate_base_footprint{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0.1},
        {0, 0, 0, 1},
      };
      pt = Translate_mounting_plate_base_footprint * pt;

      // base footprint -> odom
      pt = footprint_odom_transform_ * pt;

      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(pt[0]);
      point.y = static_cast<float>(pt[1]);
      point.z = static_cast<float>(pt[2]);
      pc.points.push_back(point);
    }
    angle += msg->angle_increment;
  }

  // publish transformated point cloud
  point_cloud_pub_->publish(pc);
}

// -----------------------------------------------------------------------------
void TransformNode::odomCallback(const Odometry::ConstSharedPtr& msg)
{
  // TODO: Add your manual transformation calculation here
  //       You can add variables to the class if neded
  Eigen::MatrixX4d translate_odom_base{
    {1, 0, 0, msg->pose.pose.position.x},
    {0, 1, 0, msg->pose.pose.position.y},
    {0, 0, 1, msg->pose.pose.position.z},
    {0, 0, 0, 1}
  };

  // use quaternion rotation
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  // normalize
  double mag = sqrt(x * x + y * y + z * z + w * w);
  x /= mag;
  y /= mag;
  z /= mag;
  w /= mag;

  // create homogeneous rotation matrix
  Eigen::MatrixX4d rotate_odom_base {
    {1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 0},
    {2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x, 0},
    {2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y, 0},
    {0, 0, 0, 1}
  };
}

} /* namespace tug_mr2 */
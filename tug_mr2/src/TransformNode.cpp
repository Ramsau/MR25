#include "tug_mr2/TransformNode.hpp"

#include <functional>

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


  // publish transformated point cloud
  point_cloud_pub_->publish(pc);
}

// -----------------------------------------------------------------------------
void TransformNode::odomCallback(const Odometry::ConstSharedPtr& msg)
{
  // TODO: Add your manual transformation calculation here
  //       You can add variables to the class if neded
}

} /* namespace tug_mr2 */
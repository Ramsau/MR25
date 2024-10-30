#ifndef _TUG_MR2__TRANSFORM_NODE_HPP_
#define _TUG_MR2__TRANSFORM_NODE_HPP_

#include "rclcpp/node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace tug_mr2
{

class TransformNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  private:
    using Odometry = nav_msgs::msg::Odometry;

    using LaserScan = sensor_msgs::msg::LaserScan;
    using PointCloud = sensor_msgs::msg::PointCloud;
    
  // Variables -----------------------------------------------------------------
  private:
    // Publisher
    rclcpp::Publisher<PointCloud>::SharedPtr point_cloud_pub_;

    // Subscriber
    rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

  // Methods -------------------------------------------------------------------
  public:
    TransformNode();
    ~TransformNode();

  private:
    void laserCallback(const LaserScan::ConstSharedPtr& msg);
    void odomCallback(const Odometry::ConstSharedPtr& msg);

}; /* class TransformNode */

} /* namespace tug_mr2 */

#endif /* _TUG_MR2__TRANSFORM_NODE_HPP_ */
#ifndef _TUG_TURTLEBOT4__SLAM_NODE_HPP_
#define _TUG_TURTLEBOT4__SLAM_NODE_HPP_

#include <functional>

#include "rclcpp/node.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tug_turtlebot4/OccupancyGridMap.hpp"

namespace tug_turtlebot4
{

class SlamNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  private:
    using Point = geometry_msgs::msg::Point;

    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using LaserScan = sensor_msgs::msg::LaserScan;

  // Variables -----------------------------------------------------------------
  private:
    // Publishers
    rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // Subscriptions
    rclcpp::Subscription<LaserScan>::SharedPtr laser_scan_sub_;

    // Occupancy grid map
    OccupancyGridMap::SharedPtr occupancy_grid_map_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Methods -------------------------------------------------------------------
  public:
    SlamNode();
    ~SlamNode();

    void laserScanCallback(const LaserScan::ConstSharedPtr& msg);
}; /* class SlamNode */

} /* namespace tug_turtlebot4 */

#endif /* _TUG_TURTLEBOT4__SLAM_NODE_HPP_ */
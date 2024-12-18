#ifndef _TUG_TURTLEBOT4__OCCUPANCY_GRID_MAP_HPP_
#define _TUG_TURTLEBOT4__OCCUPANCY_GRID_MAP_HPP_

#include <memory>

#include "rclcpp/node.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace tug_turtlebot4
{

class OccupancyGridMap
{
  // Directives ----------------------------------------------------------------
  public:
    using SharedPtr = std::shared_ptr<OccupancyGridMap>;

  private:
    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  // Variables -----------------------------------------------------------------
  private:
    // ROS node
    rclcpp::Node* node_;

    // Map parameters
    uint32_t width_;
    uint32_t height_;
    float resolution_;

    // Map data
    float* data_;

    // Publishers
    rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // Message
    OccupancyGrid::SharedPtr occupancy_grid_;

  // Methods -------------------------------------------------------------------
  public:
    OccupancyGridMap(
      rclcpp::Node* const node,
      uint32_t width,
      uint32_t height,
      float resolution
    );
    ~OccupancyGridMap();

    bool worldToCell(
      double world_x,
      double world_y,
      uint32_t& cell_x,
      uint32_t& cell_y
    ) const;
    void updateCell(uint32_t cell_x, uint32_t cell_y, float value);
    float getCell(uint32_t cell_x, uint32_t cell_y) const;
}; /* class OccupancyGridMap */

} /* namespace tug_turtlebot4 */

#endif /* _TUG_TURTLEBOT4__OCCUPANCY_GRID_MAP_HPP_ */
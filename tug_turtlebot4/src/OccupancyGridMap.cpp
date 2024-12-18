#include "tug_turtlebot4/OccupancyGridMap.hpp"

#include "rclcpp/clock.hpp"

namespace tug_turtlebot4
{

// -----------------------------------------------------------------------------
OccupancyGridMap::OccupancyGridMap(
  rclcpp::Node* const node,
  uint32_t width,
  uint32_t height,
  float resolution
) :
  node_(node),
  width_(width),
  height_(height),
  resolution_(resolution)
{
  data_ = new float[width * height];

  for (size_t i = 0 ; i < width * height; i++)
    data_[i] = 0.0F;

  // Publisher
  occupancy_grid_pub_ = node->create_publisher<OccupancyGrid>("map", 10);

  // Message
  occupancy_grid_ = std::make_shared<OccupancyGrid>();
  occupancy_grid_->header.frame_id = "odom";
  occupancy_grid_->info.width = width;
  occupancy_grid_->info.height = height;
  occupancy_grid_->info.resolution = resolution;
  occupancy_grid_->info.map_load_time = node->get_clock()->now();
  occupancy_grid_->info.origin.position.x = -(width * resolution / 2.0);
  occupancy_grid_->info.origin.position.y = -(height * resolution / 2.0);
  occupancy_grid_->data.resize(width * height);

  for (int8_t& cell : occupancy_grid_->data)
    cell = -1;
}

// -----------------------------------------------------------------------------
OccupancyGridMap::~OccupancyGridMap()
{
  delete[] data_;
  data_ = nullptr;

  node_ = nullptr;
}

// -----------------------------------------------------------------------------
bool OccupancyGridMap::worldToCell(
  double world_x,
  double world_y,
  uint32_t& cell_x,
  uint32_t& cell_y
) const
{
  const double& map_origin_x = occupancy_grid_->info.origin.position.x;
  const double& map_origin_y = occupancy_grid_->info.origin.position.y;

  if (world_x < map_origin_x || world_y < map_origin_y)
    return false;

  cell_x = static_cast<uint32_t>((world_x - map_origin_x) / resolution_);
  cell_y = static_cast<uint32_t>((world_y - map_origin_y) / resolution_);

  if (cell_x >= width_ || cell_y >= height_)
    return false;

  return true;
}

// -----------------------------------------------------------------------------
void OccupancyGridMap::updateCell(uint32_t cell_x, uint32_t cell_y, float value)
{
  if (cell_x < width_ && cell_y < height_)
  {
    const size_t cell_index = cell_x + cell_y * width_;
    data_[cell_index] = value;

    bool changed = false;

    if (value > 0.95F)
    {
      occupancy_grid_->data[cell_index] = 100;
      changed = true;
    }

    else if (value < -0.95F)
    {
      occupancy_grid_->data[cell_index] = 0;
      changed = true;
    }

    if (changed)
    {
      occupancy_grid_->header.stamp = node_->get_clock()->now();
      occupancy_grid_pub_->publish(*occupancy_grid_);
    }
  }
}

// -----------------------------------------------------------------------------
float OccupancyGridMap::getCell(uint32_t cell_x, uint32_t cell_y) const
{
  if (cell_x < width_ && cell_y < height_)
    return data_[cell_x + cell_y * width_];

  return std::numeric_limits<float>::infinity();
}

} /* namespace tug_turtlebot4 */
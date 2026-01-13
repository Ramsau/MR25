#include "tug_turtlebot4/SlamNode.hpp"

#include <functional>
#include <memory>

namespace tug_turtlebot4
{

// -----------------------------------------------------------------------------
SlamNode::SlamNode() :
  Node("slam", "tug_turtlebot4")
{
  // Publishers
  occupancy_grid_pub_ = create_publisher<OccupancyGrid>("map", 10);

  // Subscriptions
  laser_scan_sub_ = create_subscription<LaserScan>(
    "scan",
    10,
    std::bind(&SlamNode::laserScanCallback, this, std::placeholders::_1)
  );

  // Occupancy grid map
  occupancy_grid_map_ = std::make_shared<OccupancyGridMap>(
    this,
    300U,
    300U,
    0.1F
  );
  

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    this
  );
}

// -----------------------------------------------------------------------------
SlamNode::~SlamNode()
{

}

// -----------------------------------------------------------------------------
void SlamNode::laserScanCallback(const LaserScan::ConstSharedPtr& msg)
{
  // map estimation
  float angle = msg->angle_min;


  float prob_free = 0.4;
  float prob_prior = 0.5;
  float prob_occupied = 0.75;

  float prob_free_l = probabilityToLogOdd(prob_free);
  float prob_prior_l = probabilityToLogOdd(prob_prior);
  float prob_occupied_l = probabilityToLogOdd(prob_occupied);
  // i know full well that this is a memory leak, but the assignment says we should incorporate all past measurements
  float *optimize_map = new float[300 * 300];
  for (auto range: msg->ranges) {
    if (std::isinf(range)) {
      angle += msg->angle_increment;
      continue;
    }

    // get point in odom frame
    tf2::Vector3 point_robot(range * cos(angle), range * sin(angle), 0.0);
    tf2::Vector3 zero_robot(0.0, 0.0, 0.0);
    tf2::Vector3 point_map, zero_map;
    try {
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform("odom", msg->header.frame_id, msg->header.stamp);

      tf2::Transform tf2_transform;
      tf2::fromMsg(transform.transform, tf2_transform);
      point_map = tf2_transform * point_robot;
      zero_map = tf2_transform * zero_robot;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Could not transform point: %s", ex.what());
      return;
    }

    //RCLCPP_INFO_STREAM(get_logger(),"prob free log odd: " << prob_free_l);
    //RCLCPP_INFO_STREAM(get_logger(),"prob free prob: " << logOddToProbability(prob_free_l));

    float prob_l = prob_prior_l;
    // float prob = prob_prior;
    float og_prob = 0;

    // paint on occupancy grid map using bresenham
    // see https://de.wikipedia.org/wiki/Bresenham-Algorithmus
    uint32_t x_end, y_end, x_start, y_start;
    occupancy_grid_map_->worldToCell(point_map.x(), point_map.y(), x_end, y_end);
    occupancy_grid_map_->worldToCell(zero_map.x(), zero_map.y(), x_start, y_start);
    int dx = x_end - x_start;
    int dy = y_end - y_start;
    int x = x_start;
    int y = y_start;
    if (x_end == 0) {
      RCLCPP_WARN_STREAM(get_logger(),"x_end is 0");
    }
    int incx = dx > 0 ? +1 : dx < 0 ? -1 : 0;
    int incy = dy > 0 ? +1 : dy < 0 ? -1 : 0;

    if(dx < 0) dx = -dx;
    if(dy < 0) dy = -dy;

    int t, pdx, pdy, ddx, ddy, deltaslowdirection, deltafastdirection, err;

    /* determine, which distance is greater */
    if (dx > dy)
    {
      /* x is fast direction */
      pdx = incx; pdy = 0;    /* pd. is parallel step */
      ddx = incx; ddy = incy; /* dd. is diagonal step */
      deltaslowdirection = dy;   deltafastdirection = dx;
    }
    else
    {
      /* y ist fast direction */
      pdx = 0;    pdy = incy; /* pd. is parallel step */
      ddx = incx; ddy = incy; /* dd. is diagonal step */
      deltaslowdirection = dx;   deltafastdirection = dy;
    }

    x = x_start;
    y = y_start;
    err = deltafastdirection / 2;

    prob_l = occupancy_grid_map_->getCell(x, y) + prob_free_l + prob_prior_l;
    occupancy_grid_map_->updateCell(x, y, prob_l);
    optimize_map[x * 300 + y] = prob_free_l;


    /* calculate pixels */
    for (t = 0; t < deltafastdirection; ++t)
    {
      /* Aktualisierung Fehlerterm */
      err -= deltaslowdirection;
      if (err < 0)
      {
        /* Fehlerterm wieder positiv (>= 0) machen */
        err += deltafastdirection;
        /* Schritt in langsame Richtung, Diagonalschritt */
        x += ddx;
        y += ddy;
      }
      else
      {
        /* Schritt in schnelle Richtung, Parallelschritt */
        x += pdx;
        y += pdy;
      }

      prob_l = occupancy_grid_map_->getCell(x, y) + prob_free_l + prob_prior_l;
      occupancy_grid_map_->updateCell(x, y, prob_l);
      optimize_map[x + y * 300] = prob_free_l;
    }

    prob_l = occupancy_grid_map_->getCell(x_end, y_end) + prob_occupied_l + prob_prior_l;
    occupancy_grid_map_->updateCell(x_end, y_end, prob_l);
    optimize_map[x_end + y_end * 300] = prob_occupied_l;


    angle += msg->angle_increment;
  }

  auto now = rclcpp::Time(this->now());
  if (now - last_optimize_time_ > std::chrono::seconds(1)) {
    // optimize
    past_scans_.push_back(optimize_map);
    last_optimize_time_ = now;
    optimizeMap();
  }
}

float SlamNode::probabilityToLogOdd(float probability)
{
  return std::log(probability / (1 - probability));
}

float SlamNode::logOddToProbability(float logOdd)
{
  return 1 - (1 / (1 + std::exp(logOdd)));
}

float SlamNode::probToOccValue(float prob)
{
  return -1 + 2 *prob;
}

float SlamNode::occValueToProb(float occ_value)
{
  return 0.5 + 0.5 *occ_value;
}

  void SlamNode::optimizeMap() {
    RCLCPP_INFO(get_logger(), "optimize with %d measurements", past_scans_.size());
    bool improved = true;
    while (improved) {
      improved = false;
      for (int x = 0; x < 300U; x++) {
        for (int y = 0; y < 300U; y++) {
          float value = occupancy_grid_map_->getCell(x, y);
          if (value == 0) {
            continue;
          }

          float past_value = 0;
          for (auto map: past_scans_) {
            past_value += map[x + y * 300];
          }
          if (abs(-value - past_value) < abs(value - past_value)) {
            occupancy_grid_map_->updateCell(x, y, -value);
            // improved = true;
            // RCLCPP_INFO(get_logger(), "updated cell %d %d", x, y);
          }
        }
      }
    }
  }

} /* namespace tug_turtlebot */
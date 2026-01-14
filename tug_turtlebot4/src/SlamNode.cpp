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
  bool inverse_sensor_model = false;
  if (inverse_sensor_model) {
    inverseSensorModel(msg);
  } else {
    forwardSensorModel(msg);
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

  void SlamNode::forwardSensorModel(const LaserScan::ConstSharedPtr& msg) {

    // map estimation
    float angle = msg->angle_min;


    float prob_free = 0.4;
    float prob_prior = 0.5;
    float prob_occupied = 0.75;

    float prob_free_l = probabilityToLogOdd(prob_free);
    float prob_prior_l = probabilityToLogOdd(prob_prior);
    float prob_occupied_l = probabilityToLogOdd(prob_occupied);
    bool first_range = true;
    for (auto range: msg->ranges) {
      if (std::isinf(range) || first_range) {
        angle += msg->angle_increment;
        first_range = false;
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
      }

      prob_l = occupancy_grid_map_->getCell(x_end, y_end) + prob_occupied_l + prob_prior_l;
      occupancy_grid_map_->updateCell(x_end, y_end, prob_l);



      angle += msg->angle_increment;
    }


  }
  void SlamNode::inverseSensorModel(const LaserScan::ConstSharedPtr& msg) {

    float prob_free = 0.05;
    float prob_prior = 0.5;
    float prob_occupied = 0.95;

    float prob_free_l = probabilityToLogOdd(prob_free);
    float prob_prior_l = probabilityToLogOdd(prob_prior);
    float prob_occupied_l = probabilityToLogOdd(prob_occupied);

    float cos_angle_uncertainty = cos(msg->angle_increment / 2);
    float range_uncertainty_relative = 0.01;

    static rclcpp::Time last_optimize_time = rclcpp::Time(this->now());
    static rclcpp::Time last_publish_time = rclcpp::Time(this->now());
    auto now = rclcpp::Time(this->now());
    if (now - last_optimize_time > std::chrono::seconds(5)) {
      // optimize
      bool first_range = true;
      float angle = msg->angle_min;
      float last_x = 0;
      float last_y = 0;
      for (auto range: msg->ranges) {
        if (std::isinf(range) || first_range) {
          angle += msg->angle_increment;
          first_range = false;
          continue;
        }

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
        past_measurements_.push_back(LaserMeasurement{
          zero_map.x(),
          zero_map.y(),
          point_map.x(),
          point_map.y(),
          range,
        });
        angle += msg->angle_increment;
      }
      last_optimize_time = now;
      RCLCPP_INFO(get_logger(), "past measurements: %d", past_measurements_.size());
    }

    if (now - last_publish_time > std::chrono::seconds(30)) {
      RCLCPP_INFO(get_logger(), "optimize with %d measurements", past_measurements_.size());
      for (int x = 0; x < 300U; x++) {
        for (int y = 0; y < 300U; y++) {
          double real_x = (x - 150) * 0.1;
          double real_y = (y - 150) * 0.1;
          double map_prob = 0;
          for (auto past_scan: past_measurements_) {
            double point_x = past_scan.x_end - past_scan.x_start;
            double point_y = past_scan.y_end - past_scan.y_start;
            double map_x = real_x - past_scan.x_start;
            double map_y = real_y - past_scan.y_start;
            double dist_point = sqrt(point_x * point_x + point_y * point_y);
            double dist_map = sqrt(map_x * map_x + map_y * map_y);
            double cos_theta = (point_x * map_x + point_y * map_y) / (dist_point * dist_map);

            // only look at points within angle uncertainty
            if (cos_theta > cos_angle_uncertainty) {
              if (dist_map < dist_point * (1 - range_uncertainty_relative)) {
                map_prob += prob_free_l;
              } else if (dist_map < dist_point * (1 + range_uncertainty_relative)) {
                map_prob += prob_occupied_l;
              }
            }
          }
          occupancy_grid_map_->updateCell(x, y, map_prob);
        }
      }
      last_publish_time = now;
    }
  }

} /* namespace tug_turtlebot */
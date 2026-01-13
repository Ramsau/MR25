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


  //-100 light gray
  // -1 light gray
  // -0.5 blueish gray
  // 0 blueish gray
  // 0.5 blueish gray
  // 1 black
  // 50 black
  // 100 black
  // 200 still black
  // for (size_t row = 0; row < 3000; row++)
  // {
  //   for (size_t col = 0; col < 3000; col++)
  //   {
  //     occupancy_grid_map_->updateCell(col, row, 0);
  //   }
  // }
  

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
  for (auto range: msg->ranges) {
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

    float prob_free = 0.35;
    float prob_prior = 0.5;
    float prob_occupied = 0.9;

    float prob_free_l = probabilityToLogOdd(prob_free);
    float prob_prior_l = probabilityToLogOdd(prob_prior);
    float prob_occupied_l = probabilityToLogOdd(prob_occupied);

    //RCLCPP_INFO_STREAM(get_logger(),"prob free log odd: " << prob_free_l);
    //RCLCPP_INFO_STREAM(get_logger(),"prob free prob: " << logOddToProbability(prob_free_l));

    float prob_l = prob_prior_l;
    float prob = prob_prior;
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

    //RCLCPP_INFO_STREAM(get_logger(),"prob map: " << occupancy_grid_map_->getCell(x, y));
    og_prob = occValueToProb(occupancy_grid_map_->getCell(x, y));
    prob_l = probabilityToLogOdd(og_prob) + prob_free_l - prob_prior_l;
    prob = logOddToProbability(prob_l);

    if( x == 148 && y == 139)
      RCLCPP_INFO_STREAM(get_logger(),"Current pos Probability occupied: " << prob << " orioginal prob: " << og_prob << " x: " << x_end << " y: " << y_end);

    // occupancy_grid_map_->updateCell(x, y, logOddToProbability(prob_l));
    occupancy_grid_map_->updateCell(x, y, probToOccValue(prob));


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

      og_prob = occValueToProb(occupancy_grid_map_->getCell(x, y));
      prob_l = probabilityToLogOdd(og_prob) + prob_free_l - prob_prior_l;
      prob = logOddToProbability(prob_l);

      if( x == 148 && y == 139)
        RCLCPP_INFO_STREAM(get_logger(),"Free Probability occupied: " << prob << " orioginal prob: " << og_prob << " x: " << x_end << " y: " << y_end);

      occupancy_grid_map_->updateCell(x, y, probToOccValue(prob));
      // occupancy_grid_map_->updateCell(x, y, 100);

    }

    // just for reference, ill paint the actual point in -100
    og_prob = occValueToProb(occupancy_grid_map_->getCell(x_end, y_end));
    prob_l = probabilityToLogOdd(og_prob) + prob_occupied_l - prob_prior_l;
    prob = logOddToProbability(prob_l);

    if( x_end == 148 && y_end == 139)
    {
      RCLCPP_INFO_STREAM(get_logger(),"Probability occupied: " << prob << " orioginal prob: " << og_prob << " x: " << x_end << " y: " << y_end);
    }

    occupancy_grid_map_->updateCell(x_end, y_end, probToOccValue(prob));
    // occupancy_grid_map_->updateCell(x_end, y_end, -100);


    angle += msg->angle_increment;
    //RCLCPP_INFO_STREAM(get_logger(),"Angle of Lidar ray: " << angle);
    //RCLCPP_DEBUG(get_logger(), *get_clock(), 2000, "Could not transform point: %f", angle);
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

} /* namespace tug_turtlebot */
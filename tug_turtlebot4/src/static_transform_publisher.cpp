#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace tug_turtlebot4
{

class StaticTransformPublisherNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  private:
    using TransformStamped = geometry_msgs::msg::TransformStamped;

    using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
    using ParameterType = rcl_interfaces::msg::ParameterType;

  // Variables -----------------------------------------------------------------
  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf;

  // Methods -------------------------------------------------------------------
  public:
    StaticTransformPublisherNode() :
      Node("static_transform_publisher", "tug_turtlebot4")
    {
      std::vector<std::string> from = declare_parameter(
        "from",
        std::vector<std::string>()
      );

      std::vector<std::string> to = declare_parameter(
        "to",
        std::vector<std::string>()
      );

      if (from.empty() || to.empty())
      {
        RCLCPP_ERROR(
          get_logger(),
          "Parameters 'from' and 'to' must not be empty"
        );
        rclcpp::shutdown();
        return;
      }

      if (from.size() != to.size())
      {
        RCLCPP_ERROR(
          get_logger(),
          "Parameters 'from' and 'to' must have the same size"
        );
        rclcpp::shutdown();
        return;
      }

      static_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

      for (unsigned i = 0; i < from.size(); i++)
      {
        TransformStamped t;
        t.header.stamp = get_clock()->now();
        t.header.frame_id = from[i];
        t.child_frame_id = to[i];

        static_tf->sendTransform(t);
      }
    };

    ~StaticTransformPublisherNode() = default;

}; /* class StaticTransformPublisherNode */

} /* namespace tug_turtlebot4 */

// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<tug_turtlebot4::StaticTransformPublisherNode>()
  );
  rclcpp::shutdown();

  return 0;
}
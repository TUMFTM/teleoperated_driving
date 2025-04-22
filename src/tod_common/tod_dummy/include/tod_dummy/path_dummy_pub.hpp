/**
 * @file path_dummy_pub.hpp
 * @brief  A simple ROS2 node that publishes a simulated path message.
 * @copyright TUMFTM 2025
 */
 #ifndef PATH_DUMMY_PUB_HPP
#define PATH_DUMMY_PUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
namespace tod_dummy{

/**
 * @class PathDummyPub
 * @brief A simple ROS2 node that publishes a simulated path message.
 * 
 * This node publishes a path message consisting of 100 points with incrementally increasing x and y coordinates.
 * The message is published to the "/Path" topic at regular intervals (every 500ms).
 */
class PathDummyPub : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for PathDummyPub
     * 
     * Initializes the ROS2 node, publisher, and timer for periodic publishing.
     */
    PathDummyPub();

  private:
    /**
     * @brief Callback function for timer
     * 
     * Publishes a path message with 100 points every 500ms.
     */
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_; /**< Timer to trigger the callback */
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Path>::SharedPtr publisher_; /**< Publisher for the Path message */
};

#endif // PATH_DUMMY_PUB_HPP
}

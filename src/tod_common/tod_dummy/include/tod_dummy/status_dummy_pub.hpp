/**
 * @file status_dummy_pub.hpp
 * @brief  A ROS2 node that publishes status messages for both the operator and the vehicle.
 * @copyright TUMFTM 2025
 */ 
 #ifndef DUMMY_PUB_HPP
#define DUMMY_PUB_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tod_status_msgs/msg/status.hpp"

using namespace std::chrono_literals;
namespace tod_dummy{

/**
 * @class DummyPub
 * @brief A ROS2 node that publishes status messages for both the operator and the vehicle.
 * 
 * This node simulates publishing status messages to two different topics at regular intervals (every 50ms).
 * The messages contain information about teleoperation status and vehicle control mode.
 */
class DummyPub : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for DummyPub
     * 
     * Initializes the ROS2 node, publishers for the operator and vehicle status, and a timer.
     */
    DummyPub();

  private:
    /**
     * @brief Callback function for the timer
     * 
     * Publishes status messages for both the operator and vehicle every 50ms.
     */
    void timer_callback();

    rclcpp::TimerBase::SharedPtr _timer; /**< Timer to trigger the callback */
    rclcpp::Publisher<tod_status_msgs::msg::Status>::SharedPtr _publisher_operator; /**< Publisher for operator status */
    rclcpp::Publisher<tod_status_msgs::msg::Status>::SharedPtr _publisher_vehicle; /**< Publisher for vehicle status */
    size_t _count; /**< Counter to track the number of published messages */
};

#endif // DUMMY_PUB_HPP
}
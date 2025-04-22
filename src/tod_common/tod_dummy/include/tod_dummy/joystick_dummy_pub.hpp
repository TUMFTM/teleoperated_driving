/**
 * @file joystick_dummy_pub.hpp
 * @brief  A simple ROS2 node that publishes joystick-like messages.
 * @copyright TUMFTM 2025
 */

#ifndef JOYSTICK_DUMMY_PUB_HPP
#define JOYSTICK_DUMMY_PUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

namespace tod_dummy{
/**
 * @class JoystickDummyPub
 * @brief A simple ROS2 node that publishes joystick-like messages.
 * 
 * This node simulates the publishing of joystick-like data to a topic named "output/joystick".
 * The message includes axes values representing steering, throttle, and brake,
 * as well as buttons for various vehicle functions.
 */
class JoystickDummyPub : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for JoystickDummyPub
     * 
     * Initializes the ROS2 node, publisher, and timer.
     */
    JoystickDummyPub();

  private:
    /**
     * @brief Callback function for timer
     * 
     * Publishes joystick-like messages at regular intervals.
     */
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_; /**< Timer to trigger the callback */
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_; /**< Publisher for joystick messages */
};

#endif // JOYSTICK_DUMMY_PUB_HPP
}

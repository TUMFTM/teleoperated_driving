
/**
 * @file ros_interface.hpp
 * @brief ROS Abstraction for usb input devices
 * @copyright 2025 TUMFTM
 **/

 
#pragma once
#define BOOST_BIND_NO_PLACEHOLDERS

#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/logger.hpp"
#include "tod_operator_msgs/srv/input_device.hpp"
#include <boost/bind.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::placeholders;

namespace tod_input_device {

class InputDeviceController; // Forward declaration

/**
 * @class RosInterface
 * @brief Manages the ROS2 node and provides a bridge for input device operations.
 *
 * This class initializes and manages a ROS2 node, handling publishers, subscribers,
 * parameters, and services to enable interaction with input devices.
 * It communicates with the `InputDeviceController` for device management and configurations.
 */
class RosInterface : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the RosInterface class.
     * @param argc Number of command-line arguments.
     * @param pArgv Pointer to an array of command-line arguments.
     * @param parent Pointer to the InputDeviceController managing this interface.
     *
     * Sets up the ROS2 node, declares necessary parameters, and initializes the input device configuration.
     */
    explicit RosInterface(int argc, char **pArgv, InputDeviceController* parent);

    /**
     * @brief Destructor for the RosInterface class.
     *
     * Safely shuts down the ROS2 node and terminates threads.
     */
    ~RosInterface();

    /**
     * @brief Initializes the ROS2 interface.
     * @return `true` if initialization is successful, `false` otherwise.
     *
     * Creates publishers, subscribers, and threads necessary for node operation.
     */
    bool init();

    /**
     * @brief Terminates the ROS2 interface.
     *
     * Signals the interface to stop and prepares it for shutdown.
     */
    void terminate() { _active = false; }

    bool has_param(const std::string& paramName){ return node->has_parameter(paramName); }

    /**
     * @brief Clears the joystick message buffer.
     * @return `true` if the message buffer is cleared successfully, `false` otherwise.
     *
     * Resets the stored joystick message to its default state.
     */
    bool clear_joystick_msg();

    std::string type_;

    std::shared_ptr<rclcpp::Node> node;     ///< Shared pointer to the ROS2 node.
    std::vector<rclcpp::Service<tod_operator_msgs::srv::InputDevice>::SharedPtr> services; ///< List of ROS2 services.

    // Getters
    std::string get_node_name();
    static std::string get_package_path();

    // Setters
    void set_axis(const int &axis, const float &value);
    void set_button(const int &button, const int &state);

    /**
     * @brief Enables debug mode for the node.
     *
     * Sets the logger level to DEBUG for detailed output.
     */
    void set_debug_mode();

    template <typename T, typename TF, typename... Ts>
    void add_subscriber(
        const std::string& topicName, TF&& func , Ts... args) {
        if (node) {
            _subscriber.push_back(node->create_subscription<T>(topicName, 1, boost::bind(func, _1, args...)));
        }
    }

    template<typename T>
    bool get_param(const std::string& paramName, T &retVal) {
        if (!node->get_parameter(paramName).as_bool()){
            RCLCPP_ERROR(this->get_logger(), this->get_node_name().c_str(), paramName.c_str()); //TODO: check
            return false;
        }
        return true;
    }

    template<typename T>
    bool get_optional_param(const std::string& paramName, T &retVal) {
        if (!node->get_parameter(paramName).as_bool()){
            RCLCPP_DEBUG(this->get_logger(), this->get_node_name().c_str(), paramName.c_str()); //TODO: check
            return false;
        }
        return true;
    }

private:
    int _init_argc;                ///< Argument count from the command line.
    char** _p_init_argv;            ///< Pointer to command-line arguments.
    bool _active{true};             ///< Indicates whether the interface is active.
    std::thread _ros_thread;         ///< Thread for running the ROS2 node loop.
    sensor_msgs::msg::Joy _operator_msg; ///< Cached joystick message.

    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> _subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr _joystick_msg_publisher;

    InputDeviceController* _parent; ///< Pointer to the parent InputDeviceController instance.

    /**
     * @brief Runs the main ROS2 node loop with a rate of 100 Hz.
     *
     * Publishes joystick messages and handles periodic updates.
     */
    void run();
};

} // namespace tod_input_device

/**
 * @file ros_interface.cpp
 * @brief RosInterface for the window application that performs publishing of the key and mouse events in the scene as well as performs the translation of the ROS transform system into the 3D world 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/ros_interface.hpp"

#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace tod_gl {

RosInterface::RosInterface(int argc, char **pArgv)
    : Node("RosInterface"),
      _init_argc(argc),
      _p_init_argc(pArgv),
      _tf_buffer{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
      _tf_listener{std::make_unique<tf2_ros::TransformListener>(*_tf_buffer)}, 
      debug_{false},
      config_path_()
    {}

RosInterface::~RosInterface() {
    _ros_thread.join();
}

bool RosInterface::init() {
    // Vehicle specific parameter
    this->declare_parameter("vehicleID", "");
    this->declare_parameter("config_path", "");
    // tod_visual parameter
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<bool>("enable_driving_lane", false);
    this->declare_parameter<bool>("enable_lanelet_map", false);
    this->declare_parameter<bool>("enable_object_list", false);
    this->declare_parameter<bool>("enable_point_cloud", false);
    this->declare_parameter<bool>("enable_trajectory", false);
    this->declare_parameter<std::string>("map", "");
    this->declare_parameter<std::vector<double>>("map_origin", {});
    
    get_param<std::string>("config_path", config_path_);
    get_param<bool>("debug", debug_);
    get_param<bool>("enable_lanelet_map", debug_);
    
    if (debug_) {
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(this->get_logger(), "Logger level set to DEBUG");
    }

     // Publisher TODO: Publisher also with components like subscriber?
    _mouse_moved_publisher = 
        create_publisher<geometry_msgs::msg::PointStamped>("output/mouse_position_moved", 1);
    _mouse_click_publisher = 
        create_publisher<geometry_msgs::msg::PointStamped>("output/mouse_position_click", 1);
    _mouse_released_publisher =
        create_publisher<geometry_msgs::msg::PointStamped>("output/mouse_position_released", 1);
    _key_press_publisher =
        create_publisher<tod_operator_msgs::msg::KeyPress>("output/key_press", 1);
    _key_release_publisher =
        create_publisher<tod_operator_msgs::msg::KeyPress>("output/key_released", 1);

    _ros_thread = std::thread([this] { run(); });
    // rclcpp::sleep_for(5s);  // wait for tf _publisher to publish
    return true;
}

void RosInterface::run() {
    rclcpp::Rate r(1000);
    while (rclcpp::ok()) {
        r.sleep();
        rclcpp::spin_some(this->get_node_base_interface());
        if (_new_mouse_click) {
            _mouse_click_publisher->publish(_mouse_position);
            _new_mouse_click = false;
        }
        if (_new_key_press) {
            _key_press_publisher->publish(_key_press);
            _new_key_press = false;
        }
        if (_new_mouse_moved) {
            _mouse_moved_publisher->publish(_mouse_position_moved);
            _new_mouse_moved = false;
        }
        if (_new_mouse_released) {
            _mouse_released_publisher->publish(_mouse_position);
            _new_mouse_released = false;
        }

        if (_new_key_release) {
            _key_release_publisher->publish(_key_press);
            _new_key_release = false;
        }
    }
}

std::string RosInterface::get_node_name() {
    return get_name();
}

std::string RosInterface::get_package_path(std::string &&path) {
    return ament_index_cpp::get_package_share_directory(path);
}

std::string RosInterface::get_package_path() {
    return get_package_path("tod_visual");
}

geometry_msgs::msg::TransformStamped RosInterface::tf_lookup(const std::string &target_frame,
                                                            const std::string &source_frame) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = _tf_buffer->lookupTransform(target_frame, source_frame, now());
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Look up transform for %s failed with %s", source_frame.c_str(), ex.what());
    }
    return tf;
}

void RosInterface::set_mouse_click_for_publish(const geometry_msgs::msg::PointStamped &mousePosition) {
    _mouse_position = mousePosition;
    _new_mouse_click = true;
}

void RosInterface::set_key_press_for_publish(const tod_operator_msgs::msg::KeyPress &keyPress) {
    _key_press = keyPress;
    _key_press.header.stamp = this->now();
    _new_key_press = true;
}

void RosInterface::set_mouse_release_for_publish(const geometry_msgs::msg::PointStamped &mousePosition) {
    _mouse_position = mousePosition;
    _new_mouse_released = true;
}

void RosInterface::set_mouse_moved_for_publish(const geometry_msgs::msg::PointStamped &mousePosition) {
    _mouse_position_moved = mousePosition;
    _new_mouse_moved = true;
}

void RosInterface::set_key_release_for_publish(const tod_operator_msgs::msg::KeyPress &keyPress) {
    _key_press = keyPress;
    _key_press.header.stamp = this->now();
    _new_key_release = true;
}

}  // namespace tod_gl
/**
 * @file ros_interface.hpp
 * @brief RosInterface for the window application that performs publishing of the key and mouse events in the scene as well as performs the translation of the ROS transform system into the 3D world 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include "tod_gl/scene/entity.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tod_operator_msgs/msg/key_press.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace tod_gl {

class RosInterface : public rclcpp::Node {
  public:
    RosInterface(int argc, char **pArgv);
    virtual ~RosInterface();

    bool init();
    bool has_param(const std::string &paramName) { return this->has_parameter(paramName); }
    void set_mouse_click_for_publish(const geometry_msgs::msg::PointStamped &mousePosition);
    void set_key_press_for_publish(const tod_operator_msgs::msg::KeyPress &keyiPress);
    std::string get_node_name();
    static std::string get_package_path(std::string &&path);
    static std::string get_package_path();
    geometry_msgs::msg::TransformStamped tf_lookup(const std::string &target_frame, const std::string &source_frame);
    
    void set_mouse_release_for_publish(const geometry_msgs::msg::PointStamped &mousePosition);
    void set_mouse_moved_for_publish(const geometry_msgs::msg::PointStamped &mousePosition);
    void set_key_release_for_publish(const tod_operator_msgs::msg::KeyPress &keyiPress);

    //TODO: Remove legacy code 
    template <typename T, typename TF>
    void add_subscriber(const std::string &topicName, TF &&func) {
        _subscriber_list.push_back(this->create_subscription<T>(topicName, 1, func));
    }
    
    template <typename T, typename TF>
    void add_subscriber(const std::string &topicName, TF &&func, const rclcpp::QoS &qos) {
        _subscriber_list.push_back(this->create_subscription<T>(topicName, qos, func));
    }

    template <typename T>
    bool get_param(const std::string &paramName, T &retVal) {
        if (!get_parameter(paramName, retVal)) {
            RCLCPP_ERROR(get_logger(), "Could not get param %s", paramName.c_str());
            return false;
        }
        return true;
    }

    const std::string& get_config_path() const { return config_path_; };

  private:
    int _init_argc;
    char **_p_init_argc;
    std::thread _ros_thread;
    // TODO(Andi): validate use of SubscriptionBase works
    std::vector<rclcpp::SubscriptionBase::SharedPtr> _subscriber_list;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _mouse_click_publisher;
    rclcpp::Publisher<tod_operator_msgs::msg::KeyPress>::SharedPtr _key_press_publisher;
    bool _new_mouse_click{false};
    bool _new_key_press{false};
    tod_operator_msgs::msg::KeyPress _key_press;
    geometry_msgs::msg::PointStamped _mouse_position;

    // TRAJECTORY GUIDANCE
    rclcpp::Publisher<tod_operator_msgs::msg::KeyPress>::SharedPtr _key_release_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _mouse_moved_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _mouse_released_publisher;

    bool _new_mouse_released{false};
    bool _new_mouse_moved{false};
    bool _new_key_release{false};

    geometry_msgs::msg::PointStamped _mouse_position_moved;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer{nullptr};
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};

    void run();

    // ROS Params
    bool debug_;
    std::string config_path_;
};

}  // namespace tod_gl
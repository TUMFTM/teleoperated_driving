/**
 * @file driving_lane_component.hpp
 * @brief Contains data of the predicted path of the vehicle while driving in direct control mode
 * @copyright TUMFTM 2024
 **/

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "nav_msgs/msg/path.hpp"

namespace tod_gl {
  
class DrivingLaneComponent : public SubscribingComponent<nav_msgs::msg::Path> {
  public:
    explicit DrivingLaneComponent(std::shared_ptr<rclcpp::Node> sub_node, std::string topic_name)
      : SubscribingComponent(sub_node, topic_name),
      has_received_path_(false),
      topic_name_(topic_name),
      path_(nav_msgs::msg::Path())
    {}

    bool has_received_path() const { return has_received_path_; };
    const nav_msgs::msg::Path& get_path() const { return path_; };
    std::string get_topic() const { return topic_name_; }

  private:
    void cb_message(const nav_msgs::msg::Path::SharedPtr msg) override;
    bool has_received_path_;
    std::string topic_name_;
    nav_msgs::msg::Path path_;
};

class DrivingLaneComponentFrontLeft : public DrivingLaneComponent {
  public:
    explicit DrivingLaneComponentFrontLeft(std::shared_ptr<rclcpp::Node> sub_node)
        : DrivingLaneComponent(sub_node, "input/driving_lane_front_left")
      {}
};

class DrivingLaneComponentFrontRight : public DrivingLaneComponent {
  public:
    explicit DrivingLaneComponentFrontRight(std::shared_ptr<rclcpp::Node> sub_node)
        : DrivingLaneComponent(sub_node, "input/driving_lane_front_right")
      {}
};

class DrivingLaneComponentRearLeft : public DrivingLaneComponent {
  public:
    explicit DrivingLaneComponentRearLeft(std::shared_ptr<rclcpp::Node> sub_node)
        : DrivingLaneComponent(sub_node, "input/driving_lane_rear_left")
      {}
};

class DrivingLaneComponentRearRight : public DrivingLaneComponent {
  public:
    explicit DrivingLaneComponentRearRight(std::shared_ptr<rclcpp::Node> sub_node)
        : DrivingLaneComponent(sub_node, "input/driving_lane_rear_right")
      {}
};

}  // namespace tod_gl
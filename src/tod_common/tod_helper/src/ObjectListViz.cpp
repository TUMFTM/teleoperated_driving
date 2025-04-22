// Copyright 2021 Schimpe
#include <tod_helper/object_list/Helpers.h>

#include <rclcpp/rclcpp.hpp>
#include <tod_automation_msgs/msg/object_list.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ObjectListViz");
  auto pubMarker =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("object_marker", 100);
  auto subTrajectory = node->create_subscription<tod_automation_msgs::msg::ObjectList>(
    "object_list", 1, [&](const tod_automation_msgs::msg::ObjectList::SharedPtr msg) {
      visualization_msgs::msg::MarkerArray markerArray =
        tod_helper::ObjectList::to_marker_array(*msg);
      pubMarker->publish(markerArray);
    });
  rclcpp::spin(node);
  return 0;
}

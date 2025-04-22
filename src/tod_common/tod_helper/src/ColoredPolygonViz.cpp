// Copyright 2021 Schimpe
#include <tod_helper/colored_polygon/Helpers.h>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tod_operator_msgs/msg/colored_polygon.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ColoredPolygonViz");
  auto pubPolygon = node->create_publisher<geometry_msgs::msg::PolygonStamped>("polygon", 1);
  auto subColoredPolygon = node->create_subscription<tod_operator_msgs::msg::ColoredPolygon>(
    "colored_polygon", 1, [&](const tod_operator_msgs::msg::ColoredPolygon::SharedPtr msg) {
      geometry_msgs::msg::PolygonStamped polygon = tod_helper::ColoredPolygon::to_polygon(*msg);
      pubPolygon->publish(polygon);
    });
  rclcpp::spin(node);
  return 0;
}

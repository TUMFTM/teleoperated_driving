// Copyright 2020 Feiler

#include <cmath>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"

#include "tod_helper/geometry/Helpers.h"
TEST(test_calc_horizontal_distance, fixed_for_further_use)
{
  geometry_msgs::msg::Point point1;
  point1.x = point1.y = 0.0;
  geometry_msgs::msg::Point point2;
  point1.x = point1.y = 1.0;
  double expectedHorizontalDistance = std::sqrt(2);

  double result = tod_helper::Geometry::calc_horizontal_distance(point1, point2);

  ASSERT_DOUBLE_EQ(expectedHorizontalDistance, result);
}
TEST(test_calc_horizontal_distance, z_is_not_used)
{
  geometry_msgs::msg::Point point1;
  point1.x = point1.y = point1.z = 0.0;
  geometry_msgs::msg::Point point2;
  point1.x = point1.y = point1.z = 1.0;
  double expectedHorizontalDistance = std::sqrt(2);

  double result = tod_helper::Geometry::calc_horizontal_distance(point1, point2);

  ASSERT_DOUBLE_EQ(expectedHorizontalDistance, result);
}
TEST(test_calc_relative_position, Test_calc_relative_position)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  geometry_msgs::msg::Point point;
  point.x = 2;
  point.y = 3;

  ASSERT_FLOAT_EQ(tod_helper::Geometry::calc_relative_position(point, pose).x, 2.0);
  ASSERT_FLOAT_EQ(tod_helper::Geometry::calc_relative_position(point, pose).y, 3.0);
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tod_geometry_helpers_test");
  return RUN_ALL_TESTS();
}

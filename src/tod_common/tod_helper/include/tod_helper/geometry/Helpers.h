// Copyright 2020 Hoffmann
#ifndef TOD_HELPER__GEOMETRY__HELPERS_H_
#define TOD_HELPER__GEOMETRY__HELPERS_H_

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <cmath>


namespace tod_helper::Geometry
{
inline double calc_horizontal_distance(
  const geometry_msgs::msg::Point & pt0, const geometry_msgs::msg::Point & pt1)
{
  return std::sqrt(std::pow((pt0.x - pt1.x), 2) + std::pow((pt0.y - pt1.y), 2));
}
// From:
// https://github.com/Autoware-AI/common/blob/master/libwaypoint_follower/src/libwaypoint_follower.cpp
inline geometry_msgs::msg::Point calc_relative_position(
  geometry_msgs::msg::Point point_msg, geometry_msgs::msg::Pose current_pose)
{
  // Man will die Position von point_msg im von KS(0) in KS(current pose)
  // K(c_p) = R0_cp.inv + point_msg = Rcp_0 * point_msg
    
    tf2::Transform t;
    tf2::convert(current_pose, t);
    tf2::Transform inverse = t.inverse();
    geometry_msgs::msg::Pose inverse_msg;
    tf2::Vector3 v_in;
    v_in.setX(point_msg.x);
    v_in.setY(point_msg.y);
    v_in.setZ(point_msg.z);
    tf2::Vector3 v_out = inverse * v_in;
    
    geometry_msgs::msg::Point result;
    result.x = v_out.getX();
    result.y = v_out.getY();
    result.z = v_out.getZ();

    return result;
}
inline static double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(orientation);
}
inline static double perpendicular_from_pt_on_line(
  const geometry_msgs::msg::Point & pt, const geometry_msgs::msg::Point & line0,
  const geometry_msgs::msg::Point & line1, geometry_msgs::msg::Point & perpendicular)
{
  double x0 = line0.x;
  double y0 = line0.y;
  double x1 = line1.x;
  double y1 = line1.y;
  double x2 = pt.x;
  double y2 = pt.y;
  // first convert line to normalized unit vector
  double dx = x1 - x0;
  double dy = y1 - y0;
  double mag = sqrt(dx * dx + dy * dy);
  dx /= mag;
  dy /= mag;
  // translate the point and get the dot product
  double lambda = (dx * (x2 - x0)) + (dy * (y2 - y0));
  perpendicular.x = (dx * lambda) + x0;
  perpendicular.y = (dy * lambda) + y0;
  return lambda;
}
};  // namespace tod_helper::Geometry

#endif  // TOD_HELPER__GEOMETRY__HELPERS_H_

// Copyright 2021 Schimpe
#ifndef TOD_HELPER__COLORED_POLYGON__HELPERS_H_
#define TOD_HELPER__COLORED_POLYGON__HELPERS_H_


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tod_operator_msgs/msg/colored_polygon.hpp>
namespace tod_helper::ColoredPolygon
{
inline static geometry_msgs::msg::PolygonStamped to_polygon(
  const tod_operator_msgs::msg::ColoredPolygon & coloredPolygon)
{
  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header = coloredPolygon.header;
  for (const auto & pt : coloredPolygon.points) {
    polygon.polygon.points.emplace_back(pt.point);
  }
  return polygon;
}

inline tod_operator_msgs::msg::ColoredPolygon to_colored_polygon(const geometry_msgs::msg::PolygonStamped &polygonStamped,
        const std_msgs::msg::ColorRGBA color) {
    tod_operator_msgs::msg::ColoredPolygon coloredPolygon;
    coloredPolygon.header = polygonStamped.header;
    for (const auto &pt : polygonStamped.polygon.points) {
        tod_operator_msgs::msg::ColoredPoint cPoint;
        cPoint.color = color;
        cPoint.point = pt;
        coloredPolygon.points.emplace_back(cPoint);
    }
    return coloredPolygon;
}


#if NOT_OPEN_SOURCE_MACRO
inline void transform(
  tod_operator_msgs::msg::ColoredPolygon & coloredPolygon, const geometry_msgs::msg::TransformStamped & tf)
{
  // TODO(Andi): check that frame ids match
  for (tod_operator_msgs::msg::ColoredPoint & cpt : coloredPolygon.points) {
    geometry_msgs::msg::PoseStamped poseInSourceFrame, poseInTargetFrame;
    poseInSourceFrame.header.frame_id = tf.child_frame_id;
    poseInSourceFrame.pose.position.x = cpt.point.x;
    poseInSourceFrame.pose.position.y = cpt.point.y;
    poseInSourceFrame.pose.position.z = cpt.point.z;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    tf2::convert(quat, poseInSourceFrame.pose.orientation);
    tf2::doTransform<geometry_msgs::msg::PoseStamped>(poseInSourceFrame, poseInTargetFrame, tf);
    cpt.point.x = poseInTargetFrame.pose.position.x;
    cpt.point.y = poseInTargetFrame.pose.position.y;
    cpt.point.z = poseInTargetFrame.pose.position.z;
  }
  coloredPolygon.header.frame_id = tf.header.frame_id;
}
#endif
};  // namespace tod_helper::ColoredPolygon

#endif  // TOD_HELPER__COLORED_POLYGON__HELPERS_H_

// Copyright 2020 Andreas Schimpe
#ifndef TOD_HELPER__OBJECT_LIST__HELPERS_H_
#define TOD_HELPER__OBJECT_LIST__HELPERS_H_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tod_automation_msgs/msg/object_list.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace tod_helper::ObjectList
{
inline std::vector<visualization_msgs::msg::Marker> to_marker_vector(
  const std::vector<tod_automation_msgs::msg::ObjectData> & objects)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  for (unsigned int i = 0; i < objects.size(); ++i) {
    auto & cube = markers.emplace_back(visualization_msgs::msg::Marker());
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, objects.at(i).yaw_angle);
    cube.id = i;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.scale.x = 0.1;
    cube.color.r = 1.0;
    cube.color.a = 1.0;
    cube.pose.orientation.x = quaternion.x();
    cube.pose.orientation.y = quaternion.y();
    cube.pose.orientation.z = quaternion.z();
    cube.pose.orientation.w = quaternion.w();
    cube.pose.position.x = objects.at(i).dist_center_x;
    cube.pose.position.y = objects.at(i).dist_center_y;
    cube.scale.x = objects.at(i).dim_x;
    cube.scale.y = objects.at(i).dim_y;
    cube.scale.z = objects.at(i).dim_z;
    cube.pose.position.z = -0.5 * cube.scale.z;
    cube.lifetime = rclcpp::Duration::from_seconds(0.105);
  }
  return markers;
}
inline visualization_msgs::msg::MarkerArray to_marker_array(
  const tod_automation_msgs::msg::ObjectList & objectList)
{
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers = to_marker_vector(objectList.object_list);
  for (auto & marker : markerArray.markers) {
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = objectList.header.frame_id;
  }
  return markerArray;
}
#if NOT_OPEN_SOURCE_MACRO
inline void transform(
  std::vector<tod_automation_msgs::msg::ObjectData> & objects, geometry_msgs::msg::TransformStamped & tf)
{
  // TODO(Andi): check that frame ids match
  for (tod_automation_msgs::msg::ObjectData & object : objects) {
    geometry_msgs::msg::PoseStamped poseInSourceFrame, poseInTargetFrame;
    poseInSourceFrame.header.frame_id = tf.child_frame_id;
    poseInSourceFrame.pose.position.x = object.dist_center_x;
    poseInSourceFrame.pose.position.y = object.dist_center_y;
    poseInSourceFrame.pose.position.z = 0.0;
    tf2::Quaternion quat;  // (object.yaw_angle, 0.0, 0.0);
    tf2::convert(quat, poseInSourceFrame.pose.orientation);
    tf2::doTransform<geometry_msgs::msg::PoseStamped>(poseInSourceFrame, poseInTargetFrame, tf);
    object.dist_center_x = static_cast<float>(poseInTargetFrame.pose.position.x);
    object.dist_center_y = static_cast<float>(poseInTargetFrame.pose.position.y);
    tf2::convert(poseInTargetFrame.pose.orientation, quat);
    tf2::Matrix3x3 eulerMat(quat);
    double r, p, y;
    eulerMat.getRPY(r, p, y);
    object.yaw_angle = static_cast<float>(y);
  }
}
#endif
};  // namespace tod_helper::ObjectList

#endif  // TOD_HELPER__OBJECT_LIST__HELPERS_H_

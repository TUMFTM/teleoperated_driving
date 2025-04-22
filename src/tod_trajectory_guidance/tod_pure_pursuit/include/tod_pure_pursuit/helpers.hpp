#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tod_pure_pursuit {

inline double calc_horizontal_distance(const geometry_msgs::msg::Point& pt0, const geometry_msgs::msg::Point& pt1) {
    return std::sqrt(std::pow((pt0.x - pt1.x), 2) + std::pow((pt0.y - pt1.y), 2));
}

inline double rwa2swa(const double roadWheelAngle, const double max_steering_wheel_angle, const double max_road_wheel_angle) {
    return roadWheelAngle * (max_steering_wheel_angle / max_road_wheel_angle);
}
inline double curvature_to_rwa(const double& wheel_base, const double& kappa) {
    return atan(wheel_base * kappa);
}

inline std::vector<float> get_distance_of_trajectory_points(
    const tod_trajectory_guidance_msgs::msg::Trajectory& trajectory, const geometry_msgs::msg::Point& point) {
    std::vector<float> distances;
    for (const auto& traj_point : trajectory.points) {
        distances.push_back(std::abs(calc_horizontal_distance(traj_point.pose.position, point)));
    }
    return distances;
}

inline geometry_msgs::msg::Point calc_relative_position(geometry_msgs::msg::Point point_msg,
                                                        geometry_msgs::msg::Pose current_pose) {
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
    result.x = v_out.x();
    result.y = v_out.y();
    result.z = v_out.z();

    return result;
}

inline bool is_point_ahead_in_x_dir(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::Pose& pose) {
    return calc_relative_position(point, pose).x > 0.00001;
}

// Check for negative velocity and then reverse if negative
inline int get_closest_trajectory_point_in_x_dir(const tod_trajectory_guidance_msgs::msg::Trajectory& trajectory,
                                                 const geometry_msgs::msg::Pose& pose) {
    if (trajectory.points.size() == 0) {
        return -1;
    }

    std::vector<float> distances = get_distance_of_trajectory_points(trajectory, pose.position);
    auto firstPointAhead = std::find_if(trajectory.points.begin(), trajectory.points.end(), [&pose](const auto& point) {
        return is_point_ahead_in_x_dir(point.pose.position, pose);
    });

    if (firstPointAhead == trajectory.points.end()) {
        return -1;
    }

    // get min distance of all points ahead
    return std::distance(distances.begin(),
                         std::min_element(distances.begin() + std::distance(trajectory.points.begin(), firstPointAhead),
                                          distances.end()));
}

inline int get_closest_trajectory_point(const tod_trajectory_guidance_msgs::msg::Trajectory& trajectory,
                                        const geometry_msgs::msg::Pose pose) {
    if (trajectory.points.size() == 0) {
        return -1;
    }
    std::vector<float> distances = get_distance_of_trajectory_points(trajectory, pose.position);
    return std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
}

inline void transform_child_frame(geometry_msgs::msg::Pose& pose,
                                  const geometry_msgs::msg::TransformStamped& transformToChild) {
    double yaw_c = tf2::getYaw(pose.orientation);
    pose.position.x += transformToChild.transform.translation.x * std::cos(float(yaw_c)) -
                       transformToChild.transform.translation.y * std::sin(float(yaw_c));
    pose.position.y += transformToChild.transform.translation.x * std::sin(float(yaw_c)) +
                       transformToChild.transform.translation.y * std::cos(float(yaw_c));
    // === NEW ===
    tf2::Quaternion quat;
    tf2::fromMsg(transformToChild.transform.rotation, quat);
    tf2::Quaternion result_quat = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_c) * quat;
    pose.orientation = tf2::toMsg(result_quat);
    // pose.orientation = tf::createQuaternionMsgFromYaw(yaw_c +
    //     tf2::getYaw(transformToChild.transform.rotation));
}

}  // namespace tod_pure_pursuit
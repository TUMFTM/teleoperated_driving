/**
 * @file pure_pursuit.hpp
 * @brief Implements a adaptive Pure Pursuit controller, where the look-ahead distance is proportional to the vehicle's
 * velocity @see https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#include <utility>

#include "tod_pure_pursuit/pure_pursuit.hpp"

namespace tod_pure_pursuit {

PurePursuit::PurePursuit(rclcpp::Node *Node, std::shared_ptr<tod_core::param_set::Vehicle> vehParams) {
    static bool first_instance = true;
    _vehParam = std::move(vehParams);
    _ppConfig = PurePursuitConfig(Node);  // TODO change param initlization
    first_instance = false;
}

bool PurePursuit::calc_control_command(const tod_trajectory_guidance_msgs::msg::Trajectory &trajectory,
                                       const geometry_msgs::msg::PoseStamped &poseRearAxle,
                                       tod_vehicle_msgs::msg::PrimaryControlCmd &cmd, const double currentVelocity) {
    _ppConfig.set_lookahead_distance(currentVelocity);
    int _nextWaypoint = find_next_point(trajectory, poseRearAxle);
    next_wp = _nextWaypoint;
    performance_check(trajectory, poseRearAxle);

    if (_nextWaypoint == -1) {
        cmd.velocity = cmd.steering_wheel_angle = 0.0;
        cmd.acceleration = -3.0;
        RCLCPP_INFO_ONCE(rclcpp::get_logger("tod_pure_pursuit"), "No next waypoint, cannot calculate control command");

        return false;
    }

    // TODO
    // Also make this work with the check_trajectory_outdated setting (see launch file, set to True)
    if ((std::abs(time_error_to_closest_pose) > 1.0 || pose_error > 1.0) && _ppConfig.check_trajectory_outdated) {
        cmd.velocity = cmd.steering_wheel_angle = 0.0;
        cmd.acceleration = -3.0;
        RCLCPP_INFO(rclcpp::get_logger("tod_pure_pursuit"), "Trajectory too old or remote");
        return false;
    }

    double kappa = calc_curvature(trajectory.points.at(_nextWaypoint).pose.position, poseRearAxle);
    cmd.velocity = _desiredSpeed;
    cmd.steering_wheel_angle = rwa2swa(curvature_to_rwa(_vehParam->get_wheel_base(), kappa),
                                       _vehParam->get_max_swa_rad(), _vehParam->get_max_rwa_rad());

    return true;
}

double PurePursuit::calc_curvature(const geometry_msgs::msg::Point &target,
                                   const geometry_msgs::msg::PoseStamped &poseRearAxle) {
    static double distToTarget, relPosToVehicleY;
    distToTarget = tod_pure_pursuit::calc_horizontal_distance(target, poseRearAxle.pose.position);
    relPosToVehicleY = tod_pure_pursuit::calc_relative_position(target, poseRearAxle.pose).y;
    return distToTarget != 0 ? (2 * relPosToVehicleY) / pow(distToTarget, 2) : 0.0;
}

/*
 * @brief Searches the next point based on the look-ahead distance that is proportional to the vehicle's velocity
 */
int PurePursuit::find_next_point(const tod_trajectory_guidance_msgs::msg::Trajectory &_trajectory,
                                 const geometry_msgs::msg::PoseStamped &_poseRearAxle) {
    int closest_waypoint = tod_pure_pursuit::get_closest_trajectory_point_in_x_dir(_trajectory,  // in_x_dir -> ahead
                                                                                   _poseRearAxle.pose);
    if (closest_waypoint == -1) {
        if (_debug) {
            std::cout << "Closest waypoint is -1 \n";
        }
        return -1;
    }
    _desiredSpeed = _trajectory.points.at(closest_waypoint).longitudinal_velocity_mps;

    if (_debug) {
        std::cout << "no of waypoints: " << _trajectory.points.size() << "\n";
        std::cout << "closest wp     : " << closest_waypoint << "\n";
        std::cout << "desired speed  : " << _desiredSpeed << "\n";
    }
    auto lookAheadPoint =
        std::find_if(_trajectory.points.begin() + closest_waypoint, _trajectory.points.end(),
                     [&_poseRearAxle, this](const auto &point) {
                         return tod_pure_pursuit::calc_horizontal_distance(
                                    point.pose.position, _poseRearAxle.pose.position) > _ppConfig.lookahead_distance;
                     });

    if (lookAheadPoint == _trajectory.points.end()) {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("tod_pure_pursuit"), "lookAheadPoint = end of trajectory -> returning -1");
        return -1;
    }
    return std::distance(_trajectory.points.begin(), lookAheadPoint);
}

std::string PurePursuit::get_rear_axle_frame_id() {
    return _ppConfig.rear_axle_frame_id;
}

void PurePursuit::performance_check(const tod_trajectory_guidance_msgs::msg::Trajectory &traj,
                                    const geometry_msgs::msg::PoseStamped &pose) {
    int closest_waypoint = tod_pure_pursuit::get_closest_trajectory_point(traj, pose.pose);
    closest_wp = closest_waypoint;

    auto traj_time_sec = rclcpp::Time(traj.points.at(closest_waypoint).time_from_start.sec).seconds();
    auto pose_time_sec = rclcpp::Time(pose.header.stamp.sec).seconds();
    time_error_to_closest_pose = traj_time_sec - pose_time_sec;

    auto first_pose_time_sec = rclcpp::Time(traj.points.at(0).time_from_start.sec).seconds();
    time_error_to_first_pose = first_pose_time_sec - pose_time_sec;

    pose_error =
        tod_pure_pursuit::calc_horizontal_distance(traj.points.at(closest_waypoint).pose.position, pose.pose.position);
}

/*
 * @brief Simulate a trajectory using a kinematic bicycle model @see
 * https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
 * @details Loads the vehicle params of the given vehicle config for the \ref "VehicleModel" and simulates the
 * trajectory performance in discretized time stamps
 */
tod_trajectory_guidance_msgs::msg::Trajectory PurePursuit::simulate_trajectory(
    const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr input_trajectory, double simulation_step) {
    tod_trajectory_guidance_msgs::msg::Trajectory simulated_trajectory;
    simulated_trajectory.header = input_trajectory->header;

    if (input_trajectory->points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("tod_pure_pursuit"), "Input trajectory is empty.");
        return simulated_trajectory;
    }

    if (input_trajectory->points.front().longitudinal_velocity_mps == 0) {
        RCLCPP_WARN(rclcpp::get_logger("tod_pure_pursuit"), "Input trajectory contains 0 velocity returning early.");
    }

    double wb = _vehParam->get_wheel_base();
    double lf = wb / 2.0;
    double lr = wb / 2.0;
    double max_rwa = _vehParam->get_max_rwa_rad();
    double max_swa = _vehParam->get_max_swa_rad();
    double front_bumper_distance = _vehParam->get_distance_front_bumper();

    _desiredSpeed = 0.0;
    next_wp = 0;
    closest_wp = 0;
    time_error_to_first_pose = 0.0;
    time_error_to_closest_pose = 0.0;
    pose_error = 0.0;

    VehicleModel model;
    model.set_params(static_cast<float>(lf), static_cast<float>(lr), static_cast<float>(max_rwa),
                    static_cast<float>(max_swa));

    const auto &start_pose = input_trajectory->points.front().pose;

    tf2::Quaternion q(start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z,
                      start_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    model.reset_initial_position(0, 0, 0);

    double simulation_time = 0.0;

    while (rclcpp::ok()) {
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header = simulated_trajectory.header;
        current_pose.header.stamp =
            rclcpp::Time(simulated_trajectory.header.stamp) + rclcpp::Duration::from_seconds(simulation_time);
        current_pose.pose.position.x = model.get_x();
        current_pose.pose.position.y = model.get_y();
        current_pose.pose.position.z = 0.0;

        tf2::Quaternion q_current;
        q_current.setRPY(0.0, 0.0, model.get_psi());
        current_pose.pose.orientation = tf2::toMsg(q_current);

        double current_velocity = model.get_velocity_x();

        tod_vehicle_msgs::msg::PrimaryControlCmd cmd;
        bool success = calc_control_command(*input_trajectory, current_pose, cmd, current_velocity);
        if (!success) {
            break;
        }

        model.update_position(cmd.velocity, cmd.steering_wheel_angle, 1, simulation_step);

        tod_trajectory_guidance_msgs::msg::TrajectoryPoint tp;
        tp.time_from_start.sec = static_cast<int32_t>(std::floor(simulation_time + simulation_step));
        tp.time_from_start.nanosec = static_cast<uint32_t>(
            (simulation_time + simulation_step - std::floor(simulation_time + simulation_step)) * 1e9);

        tp.pose.position.x = model.get_x();
        tp.pose.position.y = model.get_y();
        tp.pose.position.z = 0.0;
        tf2::Quaternion q_new;
        q_new.setRPY(0.0, 0.0, model.get_psi());
        tp.pose.orientation = tf2::toMsg(q_new);

        tp.longitudinal_velocity_mps = static_cast<float>(model.get_velocity_x());
        tp.lateral_velocity_mps = static_cast<float>(model.get_velocity_y());

        tp.acceleration_mps2 = static_cast<float>(model.get_angular_x());

        tp.heading_rate_rps = static_cast<float>(model.get_psi_p());

        tp.front_wheel_angle_rad = static_cast<float>(model.get_steering_angle());
        tp.rear_wheel_angle_rad = 0.0f;

        auto vx = static_cast<float>(model.get_velocity_x());
        tp.curvature = (std::abs(vx) > 1e-6f) ? (tp.heading_rate_rps / vx) : 0.0f;

        // other fields
        tp.v_max_curv = 0;
        tp.sent = true;
        tp.validated = false;

        simulated_trajectory.points.push_back(tp);

        simulation_time += simulation_step;
    }

    if (!simulated_trajectory.points.empty()) {
        const auto &last_point = simulated_trajectory.points.back();
        double last_yaw = tf2::getYaw(last_point.pose.orientation);

        tod_trajectory_guidance_msgs::msg::TrajectoryPoint front_point = last_point;

        front_point.pose.position.x = last_point.pose.position.x + front_bumper_distance * std::cos(last_yaw);
        front_point.pose.position.y = last_point.pose.position.y + front_bumper_distance * std::sin(last_yaw);

        front_point.time_from_start.sec = last_point.time_from_start.sec;
        front_point.time_from_start.nanosec = last_point.time_from_start.nanosec + 1;

        simulated_trajectory.points.push_back(front_point);
    }

    // reset
    model.reset_initial_position(start_pose.position.x, start_pose.position.y, yaw);

    _desiredSpeed = 0.0;
    next_wp = 0;
    closest_wp = 0;
    time_error_to_first_pose = 0.0;
    time_error_to_closest_pose = 0.0;
    pose_error = 0.0;

    return simulated_trajectory;
}
}  // namespace tod_pure_pursuit

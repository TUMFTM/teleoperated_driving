
/**
 * @file pure_pursuit.hpp
 * @brief Implements a adaptive Pure Pursuit controller, where the look-ahead distance is proportional to the vehicle's
 * velocity @see https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_point.hpp"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"

#include "helpers.hpp"
#include "tod_core/param_set/VehicleParameters.hpp"
#include "vehicle_model.hpp"

namespace tod_pure_pursuit {


/**
 * @struct PurePursuitConfig
 * @brief Configuration parameters for the PurePursuit algorithm.
 *
 * @param lookahead_ratio Defines the lookahead distance as a ratio of velocity.
 * @param min_lookahead_distance Minimum lookahead distance to ensure stability.
 * @param rear_axle_frame_id Frame ID of the rear axle for transformation purposes.
 * @param check_trajectory_outdated Flag to determine if outdated trajectory checking is enabled.
 */
struct PurePursuitConfig {
    PurePursuitConfig() = default;

    explicit PurePursuitConfig(rclcpp::Node* nh) { load_from_parameter_workspace(nh); }

    void load_from_parameter_workspace(rclcpp::Node* nh) {
        if (!nh->has_parameter("lookahead_ratio")) {
            nh->declare_parameter<float>("lookahead_ratio", 0.5);
        }
        if (!nh->has_parameter("min_lookahead_distance")) {
            nh->declare_parameter<float>("min_lookahead_distance", 4.0);
        }
        if (!nh->has_parameter("rear_axle_frame_id")) {
            nh->declare_parameter<std::string>("rear_axle_frame_id", "base_link");
        }
        if (!nh->has_parameter("check_trajectory_outdated")) {
            nh->declare_parameter<bool>("check_trajectory_outdated", false);
        }

        if (!nh->get_parameter("lookahead_ratio", lookahead_ratio)) {
            RCLCPP_ERROR(rclcpp::get_logger("PurePursuit.hpp"),
                         "%s: Could not get parameter look lookahead_ratio - using %f",
                         std::string(nh->get_name()).c_str(), lookahead_ratio);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("PurePursuit.hpp"), "Parameter lookahead_ratio is %f",
                        nh->get_parameter("lookahead_ratio").as_double());
        }

        if (!nh->get_parameter("min_lookahead_distance", min_lookahead)) {
            RCLCPP_ERROR(rclcpp::get_logger("PurePursuit.hpp"),
                         "%s: Could not get parameter min_lookahead_distance - using %f",
                         std::string(nh->get_name()).c_str(), min_lookahead);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("PurePursuit.hpp"), "Parameter min_lookahead_distance is %f",
                        nh->get_parameter("min_lookahead_distance").as_double());
        }

        if (!nh->get_parameter("rear_axle_frame_id", rear_axle_frame_id)) {
            RCLCPP_ERROR(rclcpp::get_logger("PurePursuit.hpp"),
                         "%s: Could not get parameter rear_axle_frame_id - using %s",
                         std::string(nh->get_name()).c_str(), rear_axle_frame_id.c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("PurePursuit.hpp"), "Parameter rear_axle_frame_id is %s",
                        nh->get_parameter("rear_axle_frame_id").as_string().c_str());
        }

        if (!nh->get_parameter("check_trajectory_outdated", check_trajectory_outdated)) {
            RCLCPP_ERROR(rclcpp::get_logger("PurePursuit.hpp"),
                         "%s: Could not get parameter check_trajectory_outdated - using %d",
                         std::string(nh->get_name()).c_str(), check_trajectory_outdated);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("PurePursuit.hpp"), "Parameter check_trajectory_outdated is %d",
                        nh->get_parameter("check_trajectory_outdated").as_bool());
        }
    }

    double lookahead_ratio{0.5};  // Todo: ceck if in reasonable range (<10)
    double min_lookahead{4.0};
    std::string rear_axle_frame_id{"rear_axle_footprint"};
    bool check_trajectory_outdated{false};
    double lookahead_distance{0.0};

    /**
     * @brief Sets the lookahead distance based on velocity.
     * @param velocity Current velocity of the vehicle.
     */
    void set_lookahead_distance(const double velocity) {
        lookahead_distance =
            (velocity * lookahead_ratio) > min_lookahead ? (velocity * lookahead_ratio) : min_lookahead;
    }

    /**
     * @brief Gets the computed lookahead distance.
     * @param velocity Current velocity of the vehicle.
     * @return The computed lookahead distance.
     */
    float get_lookahead_distance(const double velocity) {
        lookahead_distance =
            (velocity * lookahead_ratio) > min_lookahead ? (velocity * lookahead_ratio) : min_lookahead;
        return lookahead_distance;
    }
};


/**
 * @class PurePursuit
 * @brief Implements the pure pursuit algorithm for trajectory tracking.
 */
class PurePursuit {
  public:
    explicit PurePursuit(rclcpp::Node* nodeHandle, std::shared_ptr<tod_core::param_set::Vehicle> vehParams);
    
    /**
     * @brief Computes the control command based on the given trajectory and vehicle state.
     * @param trajectory Reference trajectory for tracking.
     * @param poseRearAxle Current pose of the vehicle's rear axle.
     * @param cmd Output control command.
     * @param _currentVelocity Current velocity of the vehicle.
     * @return True if the computation was successful, false otherwise.
     */
    bool calc_control_command(const tod_trajectory_guidance_msgs::msg::Trajectory& trajectory,
                              const geometry_msgs::msg::PoseStamped& poseRearAxle,
                              tod_vehicle_msgs::msg::PrimaryControlCmd& cmd, const double _currentVelocity);
    std::string get_rear_axle_frame_id();
    
    /**
     * @brief Finds the next waypoint in the trajectory based on the vehicle's position.
     * @param _trajectory The reference trajectory.
     * @param _poseRearAxle Current pose of the vehicle's rear axle.
     * @return Index of the next waypoint.
     */
    int find_next_point(const tod_trajectory_guidance_msgs::msg::Trajectory& _trajectory,
                        const geometry_msgs::msg::PoseStamped& _poseRearAxle);
    
    /**
     * @brief Finds the closest waypoint in the trajectory to the vehicle's position.
     * @param trajectory The reference trajectory.
     * @param poseRearAxle Current pose of the vehicle's rear axle.
     * @return Index of the closest waypoint.
     */
    int get_closest_point(const tod_trajectory_guidance_msgs::msg::Trajectory& trajectory,
                          const geometry_msgs::msg::PoseStamped& poseRearAxle);
    
    
    /**
     * @brief Computes the curvature required for trajectory tracking.
     * @param target The target point on the trajectory.
     * @param poseRearAxle Current pose of the vehicle's rear axle.
     * @return Computed curvature value.
     */
    double calc_curvature(const geometry_msgs::msg::Point& target, const geometry_msgs::msg::PoseStamped& poseRearAxle);

    /**
     * @brief Simulates a trajectory based on the given input using a kinematic bicycle model.
     * @param input_trajectory Pointer to the input trajectory.
     * @param simulation_step Time step interval used for simulation.
     * @return Simulated trajectory.
     */
    tod_trajectory_guidance_msgs::msg::Trajectory simulate_trajectory(
        const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr input_trajectory, double simulation_step = 0.1);

    double time_error_to_first_pose;
    double time_error_to_closest_pose;
    double pose_error;
    int closest_wp;
    int next_wp;

  private:
    double _desiredSpeed{0};
    bool _debug{false};
    PurePursuitConfig _ppConfig;
    std::shared_ptr<tod_core::param_set::Vehicle> _vehParam;
    void performance_check(const tod_trajectory_guidance_msgs::msg::Trajectory& traj,
                           const geometry_msgs::msg::PoseStamped& pose);
};
}  // namespace tod_pure_pursuit
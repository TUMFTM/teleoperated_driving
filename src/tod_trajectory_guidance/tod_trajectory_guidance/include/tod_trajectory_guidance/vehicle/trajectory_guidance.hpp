/**
 * @file trajectory_guidance.cpp
 * @brief Contains implementation for the state machine @ref tod_trajectory_guidance_statemachine
 * @copyright 2024  TUMFTM
 * @ingroup tod_trajectory_guidance
 */
#pragma once
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "tod_trajectory_guidance_msgs/msg/path_point.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_point.hpp"

namespace tod_trajectory_guidance {

class TrajectoryGuidance {
  public:
    TrajectoryGuidance();

    std::vector<tod_trajectory_guidance_msgs::msg::TrajectoryPoint> get_active_trajectory();

    bool process_trajectory_received(std::vector<tod_trajectory_guidance_msgs::msg::PathPoint> &path);
    bool process_update_trajectory(std::vector<tod_trajectory_guidance_msgs::msg::PathPoint> &path);
    bool process_start_trajectory(bool start_signal);
    bool process_execution_canceled(int next_waypoint);
    bool process_execution_finished();
    bool process_path_update_recieved(std::vector<tod_trajectory_guidance_msgs::msg::PathPoint> &path);
    bool process_trajectory_rejected();
    bool process_watchdog_triggered(int next_waypoint);
    bool process_reset_triggered(int next_waypoint);
    bool process_velocity_update_recieved();

    void calc_trajectory();
    void update_velocity_active_trajectory();
    void start_trajectory();
    bool validate_velocity();
    bool has_valid_trajectory();
    void calc_stop_trajectory();
    void reset_trajectory();
    void clear_active_trajectory();
    void clear_inactive_trajectory();
    bool get_drive_status();
    double get_target_velocity();
    void set_new_velocity(double new_velocity);
    auto &get_inactive_trajectory() { return _inactive_trajectory; }
    void set_inactive_trajectory(
        const std::vector<tod_trajectory_guidance_msgs::msg::TrajectoryPoint> &validated_inactive_trajectory) {
        _inactive_trajectory = validated_inactive_trajectory;
    }

  private:
    struct _statemachine;
    std::shared_ptr<_statemachine> _tg_statemachine;
    std::vector<tod_trajectory_guidance_msgs::msg::PathPoint> _input_path;

    std::vector<tod_trajectory_guidance_msgs::msg::TrajectoryPoint> _inactive_trajectory;
    std::vector<tod_trajectory_guidance_msgs::msg::TrajectoryPoint> _active_trajectory;

    double _target_velocity_kmh{10.0};  //[km/h]
    double _new_velocity_kmh{10.0};     //[km/h] should be equal to target velo init
    double _max_velocity_kmh{15.0};     // TODO Should me manipulated via variable in node
    double _max_deceleration{0.2};      //[m/s^2]
    int _next_waypoint{0};
    bool _drive_status{false};
};

}  // namespace tod_trajectory_guidance
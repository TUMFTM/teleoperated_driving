// Copyright 2024 TUMFTM
#include "tod_trajectory_guidance/vehicle/trajectory_guidance.hpp"
#include "tod_trajectory_guidance/vehicle/statemachine_trajectory_guidance_vehicle.hpp"  // needs to be here to prevent circular import

namespace tod_trajectory_guidance {

struct TrajectoryGuidance::_statemachine
    : public boost::sml::sm<tod_trajectory_guidance::statemachine::StateMachineTrajectoryGuidance> {
    explicit _statemachine(TrajectoryGuidance *tg)
        : boost::sml::sm<tod_trajectory_guidance::statemachine::StateMachineTrajectoryGuidance>(
              static_cast<TrajectoryGuidance *>(tg)){};
};

TrajectoryGuidance::TrajectoryGuidance() : _tg_statemachine(std::make_shared<_statemachine>(this)) {}

/**
 * @brief Retrieves the currently active trajectory being executed
 * @details Returns the trajectory points that are currently being followed by the vehicle
 */
std::vector<tod_trajectory_guidance_msgs::msg::TrajectoryPoint> TrajectoryGuidance::get_active_trajectory() {
    return _active_trajectory;
};

/**
 * @brief Processes a velocity update event in the state machine
 * @details Triggers a state transition when a new velocity value is received and validated
 */
bool TrajectoryGuidance::process_velocity_update_recieved() {
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::VELOCITY_UPDATE_RECEIVED{});
}

// /**
//  * @brief Processes a velocity update event in the state machine
//  * @details Triggers a state transition when a new velocity value is received and validated
//  */
// bool TrajectoryGuidance::update_target_velocity() {
//   return _tg_statemachine->process_event(
//       tod_trajectory_guidance::statemachine::events::VELOCITY_UPDATE_RECEIVED{});
// }

/**
 * @brief Processes a path update event in the state machine
 * @details Handles updates to the path while maintaining the current trajectory state
 */
bool TrajectoryGuidance::process_path_update_recieved(std::vector<tod_trajectory_guidance_msgs::msg::PathPoint> &path) {
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::TRAJECTORY_RECEIVED{});
}

/**
 * @brief Processes a trajectory rejection event in the state machine
 * @details Handles cases where a proposed trajectory fails validation criteria
 */
bool TrajectoryGuidance::process_trajectory_rejected() {
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::TRAJECTORY_REJECTED{});
}

/**
 * @brief Processes a new trajectory received event
 * @details Updates the input path and triggers trajectory calculation in the state machine
 */
bool TrajectoryGuidance::process_trajectory_received(std::vector<tod_trajectory_guidance_msgs::msg::PathPoint> &path) {
    _input_path = path;
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::TRAJECTORY_RECEIVED{});
}

/**
 * @brief Processes a start trajectory command
 * @details Updates drive status and initiates trajectory execution if conditions are met
 */
bool TrajectoryGuidance::process_start_trajectory(bool start_signal) {
    _drive_status = start_signal;
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::START_TRAJECTORY{});
}

/**
 * @brief Processes trajectory execution cancellation
 * @details Handles early termination of trajectory execution, updating the next waypoint
 */
bool TrajectoryGuidance::process_execution_canceled(int next_waypoint) {
    _next_waypoint = next_waypoint;
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::EXECUTION_CANCELED{});
}

/**
 * @brief Processes completion of trajectory execution
 * @details Handles normal completion of trajectory following
 */
bool TrajectoryGuidance::process_execution_finished() {
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::EXECUTION_FINISHED{});
}
/**
 * @brief Processes a reset trigger event
 * @details Resets the trajectory guidance system to initial state with updated waypoint
 */
bool TrajectoryGuidance::process_reset_triggered(int next_waypoint) {
    _next_waypoint = next_waypoint;
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::RESET_TRIGGERED{});
}

/**
 * @brief Processes a watchdog timeout event
 * @details Handles safety-critical timeouts by updating waypoint and triggering appropriate state transition
 */
bool TrajectoryGuidance::process_watchdog_triggered(int next_waypoint) {
    _next_waypoint = next_waypoint;
    return _tg_statemachine->process_event(tod_trajectory_guidance::statemachine::events::WATCHDOG_TRIGGERED{});
}

/**
 * @brief Calculates a new trajectory from the input path
 * @details Generates velocity profiles considering target velocity, deceleration limits, and curvature constraints
 */
void TrajectoryGuidance::calc_trajectory() {
    auto last_path_point = std::prev(_input_path.end());
    double deceleration_dist = ((_target_velocity_kmh / 3.6) * (_target_velocity_kmh / 3.6)) / (2 * _max_deceleration);

    // Clear old trajectories
    _inactive_trajectory.clear();

    if (_input_path.empty()) {
        return;
    }

    for (auto path_point = _input_path.begin(); path_point != _input_path.end(); ++path_point) {
        tod_trajectory_guidance_msgs::msg::TrajectoryPoint trajectory_point;
        trajectory_point.set__pose(path_point->pose);

        trajectory_point.sent = true;
        path_point->sent = true;

        trajectory_point.curvature = static_cast<float>(path_point->curvature);

        // Calc distance to path end
        double dx = last_path_point->pose.position.x - path_point->pose.position.x;
        double dy = last_path_point->pose.position.y - path_point->pose.position.y;
        double remaining_dist = std::sqrt(dx * dx + dy * dy);  // 2D euclidean distance for now

        // Ramp velocity down at the end of the trajectory
        if (remaining_dist <= deceleration_dist) {
            int num_remaining_points = std::distance(path_point, _input_path.end());
            if (num_remaining_points <= 1) {
                return;
            }
            double velocity_decrease = (_target_velocity_kmh / 3.6) / (num_remaining_points - 1);
            double current_velocity = _target_velocity_kmh / 3.6;
            for (auto end_point = path_point; end_point != _input_path.end(); ++end_point) {
                current_velocity -= velocity_decrease;

                // If maximum velocity would be > max lateral velocity limit it to max velocity
                if (current_velocity >= path_point->v_max_curv) {
                    current_velocity = path_point->v_max_curv;
                }

                trajectory_point.set__pose(end_point->pose);
                trajectory_point.set__longitudinal_velocity_mps(std::max(current_velocity, 0.0));
                trajectory_point.set__v_max_curv(path_point->v_max_curv);
                _inactive_trajectory.push_back(trajectory_point);
            }
            break;
        } else {
            auto target_vel_per_point = static_cast<float>(_target_velocity_kmh) / 3.6f;

            if (target_vel_per_point >= path_point->v_max_curv) {
                target_vel_per_point = path_point->v_max_curv;
            }

            trajectory_point.set__longitudinal_velocity_mps(target_vel_per_point);
            trajectory_point.set__v_max_curv(path_point->v_max_curv);

            _inactive_trajectory.push_back(trajectory_point);
        }
    }
}

/**
 * @brief Calculates an emergency stop trajectory
 * @details Modifies the active trajectory to bring the vehicle to a immediate safe stop on the current trajectory
 */
void TrajectoryGuidance::calc_stop_trajectory() {
    if (_active_trajectory.empty()) return;

    _active_trajectory.erase(_active_trajectory.begin() + _next_waypoint, _active_trajectory.end());
    for (auto &trajectory_point : _active_trajectory) {
        trajectory_point.set__longitudinal_velocity_mps(0.0f);
    };
}

/*
 * @brief Checks if the recieved trajectory will be valid i.e. the checks in the path creator went through
 * Does NOT check for feasibility.
 */
bool TrajectoryGuidance::has_valid_trajectory() {
    return !_inactive_trajectory.empty() && _inactive_trajectory.back().validated;
}

/**
 * @brief Updates the velocity profile of the active trajectory
 * @details Recalculates velocity profiles considering current constraints and target velocitys
 */
void TrajectoryGuidance::update_velocity_active_trajectory() {
    if (_active_trajectory.empty()) {
        return;
    }

    double new_velocity_target = _target_velocity_kmh;

    const auto &last_path_point = std::prev(_active_trajectory.end());
    double deceleration_dist = ((new_velocity_target / 3.6) * (new_velocity_target / 3.6)) / (2 * _max_deceleration);

    for (auto traj_point = _active_trajectory.begin(); traj_point != _active_trajectory.end();
         ++traj_point) {  // Calc distance to path end
        double dx = last_path_point->pose.position.x - traj_point->pose.position.x;
        double dy = last_path_point->pose.position.y - traj_point->pose.position.y;
        double remaining_dist = std::sqrt(dx * dx + dy * dy);  // 2D euclidean distance for now

        // Ramp velocity down at the end of the trajectory
        if (remaining_dist <= deceleration_dist) {
            int num_remaining_points = std::distance(traj_point, _active_trajectory.end());
            double velocity_decrease = (new_velocity_target / 3.6) / (num_remaining_points - 1);
            double current_velocity = new_velocity_target / 3.6;

            for (auto end_point = traj_point; end_point != _active_trajectory.end(); ++end_point) {
                current_velocity -= velocity_decrease;

                // If maximum velocity would be > max lateral velocity limit it to max velocity
                if (current_velocity >= traj_point->v_max_curv) {
                    current_velocity = traj_point->v_max_curv;
                }

                traj_point->set__longitudinal_velocity_mps(std::max(current_velocity, 0.0));
            }
            break;
        } else {
            auto target_vel_per_point = static_cast<float>(_target_velocity_kmh) / 3.6f;
            if (target_vel_per_point >= traj_point->v_max_curv) {
                target_vel_per_point = traj_point->v_max_curv;
            }
            traj_point->set__longitudinal_velocity_mps(target_vel_per_point);
        }
    }
}

/**
 * @brief Activates the current validated inactive trajectory
 * @details Transfers the validated inactive trajectory to active status for execution
 */
void TrajectoryGuidance::start_trajectory() {
    if (has_valid_trajectory()) {
        _active_trajectory = _inactive_trajectory;
        _inactive_trajectory.clear();
    } else {
        std::cout << "Trajectory is Not valid\n" << std::endl;
    }
}

bool TrajectoryGuidance::get_drive_status() {
    return _drive_status;
}

/**
 * @brief Validates a new velocity value
 * @details Checks if the proposed velocity is within acceptable limits
 */
bool TrajectoryGuidance::validate_velocity() {
    std::cout << "Validating velocity: new=" << _new_velocity_kmh << " max=" << _max_velocity_kmh << std::endl;
    if (_new_velocity_kmh <= _max_velocity_kmh) {
        _target_velocity_kmh = _new_velocity_kmh;
        return true;
    } else {
        std::cout << "Velocity validation failed: exceeds maximum" << std::endl;
        return false;
    }
}

/**
 * @brief Resets all trajectory data
 * @details Clears both active and inactive trajectories
 */
void TrajectoryGuidance::reset_trajectory() {
    if (!_active_trajectory.empty()) {
        _active_trajectory.clear();
    }
     if (!_inactive_trajectory.empty()) {
        _inactive_trajectory.clear();
    }  
}

double TrajectoryGuidance::get_target_velocity() {
    return _target_velocity_kmh;
}

void TrajectoryGuidance::set_new_velocity(double new_velocity) {
    _new_velocity_kmh = new_velocity;
}

}  // namespace tod_trajectory_guidance
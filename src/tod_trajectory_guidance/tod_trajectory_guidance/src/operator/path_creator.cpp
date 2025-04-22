/**
 * @file PathCreator.cpp
 * @brief defintion of the PathCreator Class, TimerManager Class and the Path State
 * @details Main file for the process of path generation and operator side validation for the trajectory guidance
 * process. Path Generation using a cubic hermit spline @see https://kluge.in-chemnitz.de/opensource/spline/ by Tino
 * Kluge
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */
#include "tod_trajectory_guidance/operator/path_creator.hpp"

namespace tod_trajectory_guidance {

enum eGearPosition {
    GEARPOSITION_PARK = 0,
    GEARPOSITION_REVERSE = 1,
    GEARPOSITION_NEUTRAL = 2,
    GEARPOSITION_DRIVE = 3,
    GEARPOSITION_SPORT = 4,
    GEARPOSITION_HAUL = 5
};

void PathCreator::callback_trajectory_guidance_state(
    const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::SharedPtr msg) {
    PathState local_state;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        local_state.lastTGState = _state.lastTGState;
        local_state.lastTGEvent = _state.lastTGEvent;
        local_state.vehicle_target_velocity = _state.vehicle_target_velocity;
    }

    if (local_state.lastTGState != msg->current_state || local_state.lastTGEvent != msg->last_event) {
        if (msg->last_event == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_FINISHED) {
            {
                std::unique_lock<std::shared_mutex> lock(_statemutex);
                _state.control_points = tod_trajectory_guidance_msgs::msg::ControlPoints();
                _state.validation_trajectory = tod_trajectory_guidance_msgs::msg::Trajectory();
            }

            // control_points.points.clear();
            // validation_trajectory.points.clear();
            RCLCPP_DEBUG(this->get_logger(), "Path is cleared after execution finished");
            publish_visualization_path();
            _validation_visualization_trajectory_pub->publish(_state.validation_trajectory);
        } else if (msg->last_event == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_CANCELED &&
                   msg->current_state ==
                       tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY) {
            RCLCPP_DEBUG(this->get_logger(), "Path is cleared after execution canceled");
            {
                std::unique_lock<std::shared_mutex> lock(_statemutex);
                _state.control_points.points.clear();
            }
            _control_points_pub->publish(_state.control_points);
            publish_visualization_path();
        }
    }
    if (local_state.vehicle_target_velocity != msg->target_velocity) {
        // std::unique_lock<std::shared_mutex> lock(_statemutex);
        _state.vehicle_target_velocity = msg->target_velocity;
    }

    {
        // std::unique_lock<std::shared_mutex> lock(_statemutex);
        _state.lastTGState = msg->current_state;
        _state.lastTGEvent = msg->last_event;
    }
}

void PathCreator::callback_status_msg(const tod_status_msgs::msg::Status::SharedPtr msg) {
    {
        // std::unique_lock<std::shared_mutex> lock(_statemutex);
        _state.status = msg->tod_status;
        _state.control_mode = msg->operator_control_mode;
    }
}


void PathCreator::callback_validation_trajectory_visualization(
    const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr msg) {
    PathState local_state;
    local_state.validation_trajectory = *msg;
    _validation_visualization_trajectory_pub->publish(local_state.validation_trajectory);
    {
        std::unique_lock<std::shared_mutex> lock(_statemutex);
        _state.validation_trajectory = local_state.validation_trajectory;
        _state.text_state = "Validation Trajectory Recieved";
    }
}

void PathCreator::callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odom_message) {
    tf2::Quaternion q;
    double roll, pitch, yaw;

    x = odom_message->pose.pose.position.x;
    y = odom_message->pose.pose.position.y;

    tf2::fromMsg(odom_message->pose.pose.orientation, q);

    q.normalize();
    tf2::Matrix3x3 r_m(q);
    r_m.getRPY(roll, pitch, yaw);

    // Check if yaw is NaN or always 0
    if (std::isnan(yaw)) {
        _phi = 0.0;  // Default to facing positive x-axis
    } else {
        _phi = yaw;
    }

    double vx = odom_message->twist.twist.linear.x;
    double vy = odom_message->twist.twist.linear.y;
    double velocity = sqrt(vx * vx + vy * vy);
    if (velocity > 1) {
        _odom_counter++;
        _odom_counter = _odom_counter++;

        if (_odom_counter % PATH_CHECK_FREQUENCY == 0) {
            _odom_counter = 0;
            clean_driven_path();
        }
    }
}
/**
 * @brief Handles keyboard input for path manipulation and vehicle control
 * @param keyPress Incoming key press message
 * @details Processes key commands:
 *          - Backspace: Remove last control point
 *          - Enter: Publish path in teleoperation mode
 *          - V: Validate and send start signal
 *          - Delete: Reset path
 *          - Space: Send stop signal
 *          - W/S: Increment/decrement target velocity (kmh)
 */
void PathCreator::callback_key_action(const tod_operator_msgs::msg::KeyPress::SharedPtr keyPress) {
    {
        std::unique_lock<std::shared_mutex> lock(_statemutex);
        if (_state.control_mode != tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE) {
            return;
        }
    }

    switch (keyPress->key) {
        case 259:  // Backspace
            if (!_holding) {
                pop_last_control_point();
                publish_visualization_path();
                _edit_at = 1;
            }
            break;
        case 257:  // Enter
            if (_state.status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
                publish_sendPath();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Activate Teleoperation Mode to send trajectory to vehicle");
            }
            break;
        case 86:  // V
            send_start_signal();
            break;
        case 261:  // Delete
            send_reset_path();
            break;
        case 32:  // Space
            send_stop_signal();
            break;
        case 87:  // W
            send_incremeent_target_velo();
            break;
        case 83:  // S
            send_decremeent_target_velo();
            break;
        default:
            break;
    }
}

void PathCreator::callback_mouse_click(const geometry_msgs::msg::PointStamped::SharedPtr point) {
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        if (_state.control_mode != tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE)
            return;
    }

    _holding = true;

    find_closest_control_point(point);
    if (_edit_at == -1) {
        add_control_point_to_list(point);
        publish_visualization_path();
    }
    publish_visualization_path();
}

void PathCreator::callback_mouse_position_moved(const geometry_msgs::msg::PointStamped::SharedPtr point) {
    PathState local_state;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        local_state.control_points = _state.control_points;
    }

    if (_holding == true) {
        if (_edit_at == -1) {
            edit_control_point_position(local_state.control_points.points.size() - 1, point);
            publish_visualization_path();
        } else {
            edit_control_point_position(_edit_at, point);
            publish_visualization_path();
        }
    }
}

void PathCreator::callback_mouse_released(const geometry_msgs::msg::PointStamped::SharedPtr point) {
    _holding = false;
    _edit_at = -1;
}

/**
 * @brief Validates the incoming trajectory against the sent path
 * @details Compares validation trajectory points with sent path points using a validation margin.
 *          Marks points as validated if they fall within the margin.
 * @return true if path is fully validated, false otherwise
 */
bool PathCreator::check_incoming_trajectory_validated() {
    PathState local_state;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        local_state.send_path = _state.send_path;
        local_state.validation_trajectory = _state.validation_trajectory;
    }

    if (local_state.send_path.points.empty() || local_state.validation_trajectory.points.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Path validation_trajectory or  send_path  empty cant validate ");
        return false;
    } else {
        auto _validationMargin = this->get_parameter("validation_margin").as_double();
        for (size_t i = 0; i < local_state.validation_trajectory.points.size(); ++i) {
            auto& valPoint = local_state.validation_trajectory.points[i];
            auto& sendPoint = local_state.send_path.points[i];
            auto marginX =
                std::abs(static_cast<float>(valPoint.pose.position.x) - static_cast<float>(sendPoint.pose.position.x));
            auto marginY =
                std::abs(static_cast<float>(valPoint.pose.position.y) - static_cast<float>(sendPoint.pose.position.y));

            if (marginX <= _validationMargin && marginY <= _validationMargin) {
                local_state.validation_trajectory.points[i].validated = true;
                local_state.send_path.points[i].validated = true;

            } else {
                local_state.validation_trajectory.points[i].validated = false;
                local_state.send_path.points[i].validated = false;
            }
        }

        // Update Path path for validation results
        {
            std::unique_lock<std::shared_mutex> lock(_statemutex);
            _state.send_path = local_state.send_path;
            _state.validation_trajectory = local_state.validation_trajectory;

            if (local_state.send_path.points.back().validated) {
                for (auto point : _state.control_points.points) {
                    point.validated = true;
                }
            }
        }

        const auto lastSendPoint = local_state.send_path.points.back();
        const auto lastValPoint = local_state.validation_trajectory.points.back();
        if (lastSendPoint.validated && lastValPoint.validated) {
            RCLCPP_DEBUG(this->get_logger(), "Path is validated, sending validated path to vehicle");
            return true;
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Path validation failed, path will NOT be sent to vehicles");
            return false;
        }
    }
}

/**
 * @brief Removes passed control points from the path based on vehicle position
 * @details Erases control points that the vehicle has already passed, updating the path
 *          accordingly. Handles both forward and reverse driving modes.
 */
void PathCreator::clean_driven_path() {
    PathState local_state;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        local_state.send_path = _state.send_path;
        local_state.control_points = _state.control_points;
        local_state.text_state = _state.text_state;
    }

    if (local_state.send_path.points.empty() || local_state.control_points.points.empty()) {
        return;
    }
    auto working_points = local_state.control_points.points;

    // should not be necessary since handled in the vehicle data
    // if (reverse_mode) {
    //     std::reverse(working_points.begin(), working_points.end());
    // }

    Vector2 vehicle_pos = {x, y};
    const double proximity_threshold = .50;

    for (size_t i = 0; i < working_points.size(); i++) {
        Vector2 point_pos = {working_points[i].x, working_points[i].y};
        double dist = PathHelper::calculate_euclidean_distance(vehicle_pos.x, vehicle_pos.y, point_pos.x, point_pos.y);

        if (dist < proximity_threshold) {
            bool already_visited = false;
            for (const auto& visited : _last_visited_points) {
                if (std::abs(visited.x - point_pos.x) < 0.001 && std::abs(visited.y - point_pos.y) < 0.001) {
                    already_visited = true;
                    break;
                }
            }

            if (!already_visited) {
                _last_visited_points.push_back(working_points[i]);

                if (_last_visited_points.size() > 2) {
                    auto oldest_point = _last_visited_points.front();

                    for (auto it = working_points.begin(); it != working_points.end(); ++it) {
                        if (std::abs(it->x - oldest_point.x) < 0.001 && std::abs(it->y - oldest_point.y) < 0.001) {
                            working_points.erase(it);
                            break;
                        }
                    }
                    _last_visited_points.pop_front();
                }

                {
                    std::unique_lock<std::shared_mutex> lock(_statemutex);
                    if (working_points.size() >= 3) {
                        _state.control_points.points = working_points;
                    } else {
                        _state.control_points.points.clear();
                        _state.send_path.points.clear();
                    }
                }
                publish_visualization_path();
                break;
            }
        }
    }
}

void PathCreator::edit_control_point_position(const size_t index,
                                              const geometry_msgs::msg::PointStamped::SharedPtr point) {
    std::unique_lock<std::shared_mutex> lock(_statemutex);
    _state.control_points.points[index].x = point->point.x;
    _state.control_points.points[index].y = point->point.y;
}

void PathCreator::add_control_point_to_list(const geometry_msgs::msg::PointStamped::SharedPtr point) {
    std::unique_lock<std::shared_mutex> lock(_statemutex);

    double dist = 0.0;
    if (!_state.control_points.points.empty()) {
        auto newest_point = _state.control_points.points.front();
        dist = PathHelper::calculate_euclidean_distance(newest_point.x, newest_point.y, point->point.x, point->point.y);
    }

    if (dist > 1000) {
        _state.control_points.points.clear();
        RCLCPP_DEBUG(this->get_logger(),
                     "Exceeding max distance between control points, clearing "
                     "previous control points");
    }

    if (_state.control_points.points.size() == 0)  // prevent infinitely long paths
    {
        _state.control_points.points.clear();

        // geometry_msgs::msg::Pose first_pose;
        tod_trajectory_guidance_msgs::msg::ControlPoint first_point;
        first_point.x = x;
        first_point.y = y;
        first_point.z = 0;

        _state.control_points.points.push_back(first_point);

        // geometry_msgs::msg::Pose second_pose;
        tod_trajectory_guidance_msgs::msg::ControlPoint second_point;
        Vector2 headVec = PathHelper::calculate_vector_from_point_and_heading(x, y, _phi);
        second_point.x = x + 0.1 * headVec.x;
        second_point.y = y + 0.1 * headVec.y;
        second_point.z = 1.0;

        _state.control_points.points.push_back(second_point);

        tod_trajectory_guidance_msgs::msg::ControlPoint last_point;

        last_point.x = point->point.x;
        last_point.y = point->point.y;
        last_point.z = 1.0;

        _state.control_points.points.push_back(last_point);
    } else {
        // geometry_msgs::msg::Pose last_pose;
        tod_trajectory_guidance_msgs::msg::ControlPoint last_point;
        last_point.x = point->point.x;
        last_point.y = point->point.y;
        last_point.z = 1.0;

        _state.control_points.points.push_back(last_point);
    }
}

void PathCreator::pop_last_control_point() {
    std::unique_lock<std::shared_mutex> lock(_statemutex);

    if (_state.control_points.points.size() > 3) {
        _state.control_points.points.pop_back();

    } else if (_state.control_points.points.size() == 3) {
        _state.control_points.points.clear();
    } else {
        RCLCPP_INFO(this->get_logger(), "points_are_empty");
    }
}

void PathCreator::publish_visualization_path() {
    PathState local_state;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        local_state.control_points = _state.control_points;
    }

    auto spline_vis = std::make_shared<tod_trajectory_guidance_msgs::msg::Path>();
    auto spline_send = std::make_shared<tod_trajectory_guidance_msgs::msg::Path>();

    try {
        const auto control_points =
            std::make_shared<tod_trajectory_guidance_msgs::msg::ControlPoints>(local_state.control_points);

        auto result = PathHelper::generate_spline_with_step_size(control_points, true, _step_size);
        spline_vis = result.first;
        spline_send = result.second;
        spline_vis->header.frame_id = "map";
        spline_send->header.frame_id = "map";

    } catch (...) {
        RCLCPP_DEBUG(this->get_logger(), "Error when creating a spline, change the path contol points location!");
        return;
    }

    if (spline_vis->points.empty()) {
        _path_visualization_pub->publish(*spline_vis);
        return;
    }

    // clear validation path, signal operator that
    // Niklas: only clear if _state.validation_trajectory is smaller then current visPath? or always?
    {
        std::unique_lock<std::shared_mutex> lock(_statemutex);
        // if (!_state.validation_trajectory.points.empty()) {
        //     if (spline_vis->points.size() < _state.validation_trajectory.points.size())
        //         _state.validation_trajectory = tod_trajectory_guidance_msgs::msg::Trajectory();
        // }
        _state.send_path = *spline_send;
    }

    // _validation_visualization_trajectory_pub->publish(_state.validation_trajectory);
    _path_visualization_pub->publish(*spline_vis);
    publish_control_points();
}

void PathCreator::send_reset_path() {
    {
        std::unique_lock<std::shared_mutex> lock(_statemutex);
        _state.driving = false;
        _state.control_points.points.clear();
        _state.control_points.header.frame_id = "map";
        _state.validation_trajectory =
            tod_trajectory_guidance_msgs::msg::Trajectory();  // clear _state.validation_trajectory
        _state.text_state = "Reset Everyting";
    }

    _control_points_pub->publish(_state.control_points);
    _validation_visualization_trajectory_pub->publish(_state.validation_trajectory);

    publish_visualization_path();

    auto msg = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl();
    msg.drive = false;
    msg.target_velocity_operator = 1;
    msg.reset = true;

    _timer_manager.start_repeating_task(
        "reset_path", this,
        [this](PathState& _state) -> bool {
            if (_state.status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
                RCLCPP_ERROR(this->get_logger(), "Not in teleoperation mode");
                return true;
            }

            if (_state.lastTGEvent == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::RESET_TRIGGERED ||
                _state.lastTGState ==
                    tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY ||
                _state.lastTGState ==
                    tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY) {
                RCLCPP_INFO(this->get_logger(), "Reset signal succesful");
                return true;
            }

            return false;
        },
        _state, _statemutex,

        [this, msg](PathState& _state) { _trajectory_guidance_control_pub->publish(msg); });
}

void PathCreator::publish_sendPath() {
    PathState local_state;
    {
        std::unique_lock<std::shared_mutex> lock(_statemutex);

        for (auto point : _state.control_points.points) {
            point.sent = true;
        }
        _state.validation_trajectory = tod_trajectory_guidance_msgs::msg::Trajectory();
        _state.text_state = "Path got send";

        local_state.send_path = _state.send_path;
    }

    publish_control_points();
    publish_visualization_path();

    auto send_path = local_state.send_path;

    _timer_manager.start_repeating_task(
        "send_path", this,
        [this](PathState& _state) -> bool {
            if (_state.status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
                RCLCPP_ERROR(this->get_logger(), "Not in teleoperation mode");
                return true;
            }

            if (!_state.validation_trajectory.points.empty() ||
                _state.lastTGEvent == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_REJECTED ||
                _state.lastTGEvent == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_RECEIVED) {
                RCLCPP_INFO(this->get_logger(), "Path send completed");
                return true;
            }

            return false;
        },
        _state, _statemutex,

        [this, send_path](PathState& _state) { _path_send_to_vehicle_pub->publish(send_path); });
}

void PathCreator::send_stop_signal() {
    {
        std::unique_lock<std::shared_mutex> lock(_statemutex);
        _state.driving = false;
        _state.text_state = "Stop signal";
    }

    auto msg = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl();
    msg.drive = false;

    _timer_manager.start_repeating_task(
        "stop", this,
        [this](PathState& _state) -> bool {
            if (_state.status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
                RCLCPP_ERROR(this->get_logger(), "Not in teleoperation mode");
                return true;
            }

            if ((_state.lastTGEvent ==
                     tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_CANCELED ||  // when
                                                                                                        // executing
                 _state.lastTGState == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::
                                           WAITING_FOR_TRAJECTORY ||  // basically everwhere else
                 _state.lastTGState ==
                     tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATING_TRAJECTORY)) {
                RCLCPP_INFO(this->get_logger(), "stop signal succesful");
                return true;
            }
            RCLCPP_INFO(this->get_logger(), "sending stop signal");

            return false;
        },
        _state, _statemutex,

        [this, msg](PathState& _state) { _trajectory_guidance_control_pub->publish(msg); });
};

void PathCreator::send_start_signal() {
    auto msg = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl();

    {
        std::unique_lock<std::shared_mutex> lock(_statemutex);
        _state.driving = true;
        msg.drive = _state.driving;
        msg.target_velocity_operator = _state.vehicle_target_velocity;  // initial velo of the vehicle
        _state.text_state = "Start Signal Send";

        if (_state.validation_trajectory.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "VAL PATH EMPTY FOR SOME REASON");
            return;
        }

        for (auto& point : _state.validation_trajectory.points) {
            point.validated = true;
        }

        for (auto& point : _state.control_points.points) {
            point.validated = true;
        }
    }
    _control_points_pub->publish(_state.control_points);
    _validation_visualization_trajectory_pub->publish(_state.validation_trajectory);

    _timer_manager.start_repeating_task(
        "start", this,
        [this](PathState& _state) -> bool {
            if (_state.status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
                RCLCPP_ERROR(this->get_logger(), "Not in teleoperation mode");
                return true;
            }

            if (_state.lastTGEvent ==
                    tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::START_TRAJECTORY ||  // when executing
                _state.lastTGEvent == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_REJECTED ||
                _state.lastTGState == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::
                                          EXECUTING_TRAJECTORY ||  // basically everwhere else
                _state.lastTGState ==
                    tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY) {
                RCLCPP_INFO(this->get_logger(), "start signal succesful");
                return true;
            }

            return false;
        },
        _state, _statemutex,

        [this, msg](PathState& _state) { _trajectory_guidance_control_pub->publish(msg); });
}

void PathCreator::send_incremeent_target_velo() {
    auto msg = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl();
    float vehicle_target_velocity;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        msg.drive = _state.driving;
        vehicle_target_velocity = _state.vehicle_target_velocity;
        _state.text_state = "Increment Velo";
    }
    auto velocity_update = vehicle_target_velocity + (INCREMENT_VELOCITY);

    if (velocity_update < this->get_parameter("max_velocity").as_double()) {
        msg.target_velocity_operator = velocity_update;
        //_trajectory_guidance_control_pub->publish(msg);
        _velocity_update_rejected = false;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Max velocity reached, no going further beyond");

        return;
    }

    _timer_manager.start_repeating_task(
        "increment", this,
        [this, vehicle_target_velocity, velocity_update](PathState& _state) -> bool {
            if (_state.status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
                RCLCPP_ERROR(this->get_logger(), "Not in teleoperation mode");
                return true;
            }

            if (std::abs(_state.vehicle_target_velocity - velocity_update) < 0.001) {
                RCLCPP_INFO(this->get_logger(), "Reached desired velocity: %f", velocity_update);
                return true;
            }

            if (_state.lastTGEvent ==
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_REJECTED) {
                if (!_velocity_update_rejected) {
                    _velocity_update_rejected = true;
                    return false;
                }
                return true;
            }

            return false;
        },
        _state, _statemutex,

        [this, msg](PathState& _state) { _trajectory_guidance_control_pub->publish(msg); });
}

void PathCreator::send_decremeent_target_velo() {
    auto msg = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl();
    float vehicle_target_velocity;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        msg.drive = _state.driving;
        vehicle_target_velocity = _state.vehicle_target_velocity;
        _state.text_state = "Decrement Velo";
    }
    auto velocity_update = vehicle_target_velocity - (INCREMENT_VELOCITY);

    if (velocity_update > this->get_parameter("min_velocity").as_double()) {
        msg.target_velocity_operator = velocity_update;
        _velocity_update_rejected = false;

    } else {
        RCLCPP_DEBUG(this->get_logger(), "Min velocity reached");
    }

    _timer_manager.start_repeating_task(
        "decrement", this,
        [this, velocity_update](PathState& _state) -> bool {
            if (_state.status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
                RCLCPP_ERROR(this->get_logger(), "Not in teleoperation mode");
                return true;
            }

            if (std::abs(_state.vehicle_target_velocity - velocity_update) < 0.001) {
                RCLCPP_INFO(this->get_logger(), "Reached desired velocity: %f", velocity_update);
                return true;
            }

            if (_state.lastTGEvent ==
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_REJECTED) {
                if (!_velocity_update_rejected) {
                    _velocity_update_rejected = true;
                    return false;
                }
                return true;
            }

            return false;
        },
        _state, _statemutex,

        [this, msg](PathState& _state) { _trajectory_guidance_control_pub->publish(msg); });
}

void PathCreator::publish_control_points() {
    _control_points_pub->publish(_state.control_points);
}

void PathCreator::find_closest_control_point(const geometry_msgs::msg::PointStamped::SharedPtr point) {
    PathState local_state;
    {
        std::shared_lock<std::shared_mutex> lock(_statemutex);
        local_state.control_points = _state.control_points;
    }

    if (local_state.control_points.points.empty())
        return;

    Vector2 click_point;
    click_point.x = point->point.x;
    click_point.y = point->point.y;

    for (int i = 0; i < static_cast<int>(local_state.control_points.points.size()); i++) {
        Vector2 m;
        m.x = local_state.control_points.points[i].x;
        m.y = local_state.control_points.points[i].y;

        double dist = PathHelper::calculate_euclidean_distance(point->point.x, point->point.y, m.x, m.y);

        if (dist < 1.) {
            _edit_at = i;
        }
    }

    _edit_at - 1;
}

}  // namespace tod_trajectory_guidance

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_trajectory_guidance::PathCreator>());
    rclcpp::shutdown();
    return 0;
}
// Copyright 2024 TUMFTM
#include "tod_trajectory_guidance/vehicle/trajectory_guidance_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace tod_trajectory_guidance {

TrajectoryGuidanceNode::TrajectoryGuidanceNode() : Node("trajectory_guidance_node") {
    // Subscription
    subPath_ = this->create_subscription<tod_trajectory_guidance_msgs::msg::Path>(
        "input/path", 1, [this](const tod_trajectory_guidance_msgs::msg::Path::SharedPtr msg) { this->callback_path(*msg); });

    subTodStatus_ = this->create_subscription<tod_status_msgs::msg::Status>(
        "input/vehicle_status", 1, [this](const tod_status_msgs::msg::Status::SharedPtr msg) { this->callback_tod_state(*msg); });

    subTrajectoryControl_ = this->create_subscription<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl>(
        "input/trajectory_guidance_control_cmd", 1,
        [this](const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl::SharedPtr msg) {
            this->callback_trajectory_control_cmd(*msg);
        });

    _sub_control_state = this->create_subscription<tod_trajectory_guidance_msgs::msg::PpLog>(
        "input/ptc_logging", 1,
        [this](const tod_trajectory_guidance_msgs::msg::PpLog::SharedPtr msg) { this->callback_control_state(*msg); });

    // /vehicle/trajectory_guidance/VehicleWatchdog/node_status
    _sub_watchdog = this->create_subscription<tod_safety_msgs::msg::SafetyState>(
        "input/safety_status", 10,
        [this](const tod_safety_msgs::msg::SafetyState::SharedPtr msg) { this->callback_watchdog_state(*msg); });

    // Publisher
    _pub_active_trajectory =
        this->create_publisher<tod_trajectory_guidance_msgs::msg::Trajectory>("output/trajectory", 1);

    _pub_trajectory_validation =
        this->create_publisher<tod_trajectory_guidance_msgs::msg::Trajectory>("output/validation_trajectory", 1);

    _pub_pose_array = this->create_publisher<geometry_msgs::msg::PoseArray>("output/path_array_for_rviz", 1);

    // trajectory_guidance_state
    _pub_trajectory_guidance_state = this->create_publisher<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState>(
        "output/trajectory_guidance_state", 1);
    _timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                     [this] { publish_trajectory_guidance_state(); });

    _state_msg = std::make_shared<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState>();
    _state_msg->current_state = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY;
    _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EVENT_NONE;
    _state_msg->target_velocity = _tg.get_target_velocity();
}

/**
 * @brief Publishes the current state of the trajectory guidance system
 */
void TrajectoryGuidanceNode::publish_trajectory_guidance_state() {
    if (_debug) {
        std::string state = state_to_string(_state_msg->current_state);
        std::string event = event_to_string(_state_msg->last_event);

        log_state_event(state, event);
    }

    _pub_trajectory_guidance_state->publish(*_state_msg);
}

/**
 * @brief Callback handling new path messages
 * @details Processes incoming path data and triggers trajectory calculation:
 *          - Updates the trajectory guidance system with new path
 *          - Publishes validation request if successful
 *          - Updates system state to reflect validation phase
 */
void TrajectoryGuidanceNode::callback_path(const tod_trajectory_guidance_msgs::msg::Path &msg) {
    // TRAJECTORY_RECEIVED
    auto path = msg.points;
    bool successful = _tg.process_trajectory_received(path);
    RCLCPP_INFO(this->get_logger(), "Transition triggered by TRAJECTORY_RECEIVED: %s", successful ? "true" : "false");

    if (successful) {
        auto validation_msg = tod_trajectory_guidance_msgs::msg::Trajectory();
        auto validation_msg_data = _tg.get_inactive_trajectory();
        validation_msg.points = validation_msg_data;
        validation_msg.header.frame_id = _trajectory_frame_id;
        _pub_trajectory_validation->publish(validation_msg);

        _state_msg->current_state = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATING_TRAJECTORY;
        _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_RECEIVED;
    }
}

/**
 * @brief Callback for vehicle status updates
 * @details Monitors vehicle state transitions and triggers reset events:
 *          - Handles transition to uplink-only mode
 *          - Initiates stop trajectory when necessary
 *          - Updates system state accordingly
 */
void TrajectoryGuidanceNode::callback_tod_state(const tod_status_msgs::msg::Status &msg) {
    // RESET_TRIGGERED
    bool stop = msg.tod_status == tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY;
    if (msg.vehicle_control_mode == tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE &&
        _last_tod_status.tod_status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION && stop) {
        bool successful = _tg.process_reset_triggered(_next_waypoint);
        RCLCPP_INFO(this->get_logger(), "Transition triggered by RESET_TRIGGERED: %s", successful ? "true" : "false");

        if (successful) {
            this->publish_trajectory();
            _state_msg->current_state =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY;
            _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::RESET_TRIGGERED;
        }
    }
    _last_tod_status = msg;
}

/**
 * @brief Callback for trajectory control commands
 * @details Processes control commands for the trajectory guidance system:
 *          - Handles reset commands
 *          - Processes velocity updates
 *          - Manages trajectory execution start/stop
 *          - Updates system state based on command outcomes
 */
void TrajectoryGuidanceNode::callback_trajectory_control_cmd(
    const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl &msg) {
    // EXECUTION_CANCELED

    auto drive_s = msg.drive ? "drive" : "not drive";
    auto reset_s = msg.reset ? "reset" : "not reset";

    RCLCPP_DEBUG(this->get_logger(), "Drive %s reset %s, target_velo %.2f", drive_s, reset_s,
                 msg.target_velocity_operator);

    if (msg.reset && !msg.drive) {
        bool successful = _tg.process_reset_triggered(0);
        RCLCPP_INFO(this->get_logger(), "Transition triggered by RESET RECIEVED: %s", successful ? "true" : "false");
        if (successful) {
            _state_msg->current_state =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY;
            _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::RESET_TRIGGERED;
        }
        return;
    }


    if (_drive_status == true && msg.drive == false) {
        _drive_status = msg.drive;
        bool successful = _tg.process_execution_canceled(_next_waypoint);
        RCLCPP_INFO(this->get_logger(), "Transition triggered by EXECUTION_CANCELED: %s",
                    successful ? "true" : "false");
        if (successful) {
            this->publish_trajectory();
            _tg.reset_trajectory();
            
            _state_msg->current_state =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY;
            _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_CANCELED;
            
        }
        return;
    } else if (msg.drive == true) {  //  START_TRAJECTORY
        _drive_status = msg.drive;
        if (_state_msg->current_state ==
            tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATING_TRAJECTORY) {
            /*
             * The operator has signaled the okay for the trajectory
             */
            for (auto &point : _tg.get_inactive_trajectory()) {
                point.validated = true;
            }
            
            bool successful = _tg.process_start_trajectory(msg.drive);
            RCLCPP_INFO(this->get_logger(), "Transition triggered by START_TRAJECTORY: %s",
                        successful ? "true" : "false");
            if (successful) {
                this->publish_trajectory();
                _state_msg->current_state =
                    tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_TRAJECTORY;
                _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::START_TRAJECTORY;
            } else {
                _state_msg->last_event =
                    tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_REJECTED;
            }
        }
        return;
    }

    if (msg.reset) {
        bool successful = _tg.process_reset_triggered(_next_waypoint);
        RCLCPP_INFO(this->get_logger(), "Transition triggered by RESET_TRIGGERED: %s", successful ? "true" : "false");

        if (successful) {
            this->publish_trajectory();
            _state_msg->current_state =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY;
            _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::RESET_TRIGGERED;
        }
        return;
    }

    if (msg.target_velocity_operator != _tg.get_target_velocity()) {
        auto prev_velo = _tg.get_target_velocity();
        _tg.set_new_velocity(msg.target_velocity_operator);
        auto successful = _tg.process_velocity_update_recieved();
        RCLCPP_INFO(this->get_logger(), "Transition triggered by velocity UPDATE RECIEVED: %s",
                    successful ? "true" : "false");

        _state_msg->target_velocity = _tg.get_target_velocity();
        if (successful) {
            this->publish_trajectory();
            _state_msg->last_event =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_RECEIVED;
        } else {
            _state_msg->target_velocity = prev_velo;
            _tg.set_new_velocity(prev_velo);
            _state_msg->last_event =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_REJECTED;
        }
        return;
    }

}

/**
 * @brief Callback for trajectory controller state updates
 * @details Monitors execution progress and completion:
 *          - Tracks current waypoint
 *          - Detects trajectory completion
 *          - Updates system state on completion
 */
void TrajectoryGuidanceNode::callback_control_state(const tod_trajectory_guidance_msgs::msg::PpLog &msg) {
    // EXECUTION_FINISHED
    _next_waypoint = msg.next_wp;
    if (msg.next_wp == -1 && 
        (_state_msg->current_state == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_TRAJECTORY ||
         _state_msg->current_state == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY)) { // && !_tg.get_active_trajectory().empty()
        bool successful = _tg.process_execution_finished();
        RCLCPP_INFO(this->get_logger(), "Transition triggered by EXECUTION_FINISHED: %s",
                    successful ? "true" : "false");
        if (successful) {
            
            this->publish_trajectory();
            
            _state_msg->current_state =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY;
            _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_FINISHED;
        }
    }
}

void TrajectoryGuidanceNode::callback_watchdog_state(const tod_safety_msgs::msg::SafetyState &msg) {
    // WATCHDOG_TRIGGERED
    if (!msg.issues.empty()) {
        bool successful = _tg.process_watchdog_triggered(_next_waypoint);

        if (successful) {
            this->publish_trajectory();
            RCLCPP_INFO(this->get_logger(), "Transition triggered by WATCHDOG_TRIGGERED: %s",
                        successful ? "true" : "false");
            // Niklas: kinda redundant since disconnect
            _state_msg->current_state =
                tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY;
            _state_msg->last_event = tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WATCHDOG_TRIGGERED;
        };
    }
}

/**
 * @brief Publishes active trajectory and _debug trajectory
 */
void TrajectoryGuidanceNode::publish_trajectory() {
    // Update Trajectory for Controller
    const auto& points =  _tg.get_active_trajectory(); 
    if (points.empty()) return;
    auto trajectory_msg = std::make_shared<tod_trajectory_guidance_msgs::msg::Trajectory>();
    trajectory_msg->header.stamp = this->now();
    trajectory_msg->header.frame_id = _trajectory_frame_id;
    trajectory_msg->points =points;
    _pub_active_trajectory->publish(*trajectory_msg);

    // Update PoseArray for Debugging in rviz
    auto poseArrayMsg = std::make_shared<geometry_msgs::msg::PoseArray>();
    poseArrayMsg->header.stamp = this->now();
    poseArrayMsg->header.frame_id = _trajectory_frame_id;

    for (auto trajectoryPoint = trajectory_msg->points.begin(); trajectoryPoint != trajectory_msg->points.end();
         ++trajectoryPoint) {
        geometry_msgs::msg::Pose pose;
        pose.position = trajectoryPoint->pose.position;
        pose.orientation = trajectoryPoint->pose.orientation;
        poseArrayMsg->poses.push_back(pose);
    }
    _pub_pose_array->publish(*poseArrayMsg);
}

/**
 * @brief Callback for trajectory validation results
 * @details Processes validation feedback for proposed trajectories:
 *          - Handles validation failures and rejections
 *          - Updates system state based on validation outcome
 *          - Triggers appropriate state transitions
 */

std::string TrajectoryGuidanceNode::state_to_string(uint8_t state) {
    switch (state) {
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY:
            return "WAITING_FOR_TRAJECTORY";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATING_TRAJECTORY:
            return "VALIDATING_TRAJECTORY";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_TRAJECTORY:
            return "EXECUTING_TRAJECTORY";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY:
            return "EXECUTING_STOP_TRAJECTORY";
        default:
            return "UNKNOWN";
    }
}

std::string TrajectoryGuidanceNode::event_to_string(uint8_t event) {
    switch (event) {
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_RECEIVED:
            return "TRAJECTORY_RECEIVED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATED_TRAJECTORY_RECIEVED:
            return "VALIDATED_TRAJECTORY_RECEIVED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_REJECTED:
            return "TRAJECTORY_REJECTED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_RECEIVED:
            return "VELOCITY_UPDATE_RECEIVED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_REJECTED:
            return "VELOCITY_UPDATE_REJECTED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::START_TRAJECTORY:
            return "START_TRAJECTORY";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_FINISHED:
            return "EXECUTION_FINISHED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_CANCELED:
            return "EXECUTION_CANCELED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::RESET_TRIGGERED:
            return "RESET_TRIGGERED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WATCHDOG_TRIGGERED:
            return "WATCHDOG_TRIGGERED";
        case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EVENT_NONE:
            return "EVENT_NONE";
        default:
            return "UNKNOWN_EVENT";
    }
}

void TrajectoryGuidanceNode::log_state_event(const std::string &state, const std::string &event) {
    bool _state_changed = (state != current_state);
    bool event_changed = (event != last_event);

    // Only log if there's a state change OR a new event (avoid duplicates)
    if ((event != "EVENT_NONE") && (_state_changed || event_changed)) {
        // Add new transition to history
        _state_history.push_back({state, event, _state_changed});

        // Print entire history
        RCLCPP_INFO(this->get_logger(), "State Machine History:");
        for (size_t i = 0; i < _state_history.size(); ++i) {
            const auto &transition = _state_history[i];
            if (transition._state_changed) {
                RCLCPP_INFO(this->get_logger(), "[%zu] State Change: %s (Event: %s)", i + 1, transition.state.c_str(),
                            transition.event.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "[%zu] Event in state %s: %s", i + 1, transition.state.c_str(),
                            transition.event.c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "------------------------");

        // Update current state and event
        if (_state_changed) {
            current_state = state;
        }
        last_event = event;
    }
}
}  // namespace tod_trajectory_guidance

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tod_trajectory_guidance::TrajectoryGuidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

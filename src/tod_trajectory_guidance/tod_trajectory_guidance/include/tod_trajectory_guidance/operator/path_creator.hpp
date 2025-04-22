/**
 * @file PathCreator.hpp
 * @brief defintion of the PathCreator Class, TimerManager Class and the Path State
 * @details Main file for the process of path generation and operator side validation for the trajectory guidance
 * process.
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <utility>

#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"

#include "VehicleEnums.h"
#include "tod_trajectory_guidance/operator/path_helper.hpp"
#include "tod_trajectory_guidance/operator/spline.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tod_operator_msgs/msg/key_press.hpp"
#include "tod_status_msgs/msg/status.hpp"
#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"

#include "tod_trajectory_guidance_msgs/msg/control_point.hpp"
#include "tod_trajectory_guidance_msgs/msg/control_points.hpp"
#include "tod_trajectory_guidance_msgs/msg/path.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_guidance_control.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_guidance_state.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "VehicleEnums.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace tod_trajectory_guidance {
/*
 * @ingroup tod_trajectory_guidance
 */

/*
 * @brief Shared state of the teleoperation process containing only the information for the exchange between operator
 * and vehicle
 */
struct PathState {
    uint8_t control_mode{tod_status_msgs::msg::Status::CONTROL_MODE_NONE};
    uint8_t status{tod_status_msgs::msg::Status::TOD_STATUS_IDLE};
    uint8_t lastTGState{tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY};
    uint8_t lastTGEvent{tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EVENT_NONE};
    bool driving{false};
    float vehicle_target_velocity;
    tod_trajectory_guidance_msgs::msg::Path send_path;
    tod_trajectory_guidance_msgs::msg::Trajectory validation_trajectory;
    tod_trajectory_guidance_msgs::msg::ControlPoints control_points;
    std::string text_state;
};

/*
 * @brief Helper class to manage the sending of important control commands and checking against the current state of the
 * vehicle sided state machine.
 * @details A asynchronous repeating task is launched that is repeated until a check function is fullfilled or a max
 * numer of tries is achieved. The function is periodically executed using ROS2 Wallclock timer @see
 * https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1WallTimer.html
 */
class TimerManager {
  public:
    ~TimerManager() { timers.clear(); }

    static constexpr int MAX_ATTEMPTS = 100;
    static constexpr std::chrono::milliseconds INTERVAL{150};

    struct TimerContext {
        rclcpp::TimerBase::SharedPtr timer;
        int attempts{0};
    };

    std::unordered_map<std::string, std::unique_ptr<TimerContext>> timers;

    void start_repeating_task(const std::string& key, rclcpp::Node* node, std::function<bool(PathState&)> check_state,
                              PathState& shared_state, std::shared_mutex& _statemutex,
                              std::function<void(PathState&)> task) {
        stop_timer(key);

        auto ctx = std::make_unique<TimerContext>();

        ctx->timer =
            node->create_wall_timer(INTERVAL, [this, key, check = std::move(check_state), task = std::move(task),
                                               &shared_state, &_statemutex, ctx = ctx.get()]() {
                ctx->attempts++;

                PathState local_state;
                {
                    std::shared_lock<std::shared_mutex> lock(_statemutex);
                    local_state = shared_state;
                }

                auto it = timers.find(key);
                if (it == timers.end()) {
                    stop_timer(key);
                    return;
                }
                it->second->attempts++;

                if (it->second->attempts >= MAX_ATTEMPTS || check(local_state)) {
                    stop_timer(key);
                    return;
                }
                {
                    std::unique_lock<std::shared_mutex> lock(_statemutex);
                    task(shared_state);
                }
            });

        timers[key] = std::move(ctx);
    }

    void stop_timer(const std::string& key) {
        auto it = timers.find(key);
        if (it != timers.end()) {
            if (it->second->timer) {
                it->second->timer->cancel();
            }
            timers.erase(it);
        }
    }

    void stop_all() {
        for (auto& [key, ctx] : timers) {
            if (ctx->timer) {
                ctx->timer->cancel();
            }
        }
        timers.clear();
    }
};

/*
 *  @brief Main class of the operator side interaction for path generation and validation as well as control of the
 * vehicle during trajectory guidance.
 *  @details Incoming mouse clicks are used to create control points for the cubic hermit spline.
 *  The spline is chunked into equal length chunks and send to the vehicle which calculates the velocity profile for it.
 * The recieved path is checked against the send path and distributed to @ref tod_visual for validation. KeyCommdans:
 * Enter = send the current path, Space = Stop the vehicle, Backspace = Delete point from path, Delete = Reset the path,
 * V = start the vehicle, W & S = in-/decrement of the target velocity
 *  @see https://kluge.in-chemnitz.de/opensource/spline/spline.h
 *  @see https://arxiv.org/abs/2404.13697 from Wolf et al.
 */
class PathCreator : public rclcpp::Node {
  public:
    PathCreator() : Node("PathCreator") {

        this->declare_parameter("min_velocity", 0.0);
        this->declare_parameter("max_velocity", 15.0);
        this->declare_parameter("velocity_increment", 1.0);
        this->declare_parameter("y_max_curv", 3.0);
        this->declare_parameter("maxCurv", 0.133);  // 1/(Wendekreis/2)
        this->declare_parameter("step_size", 0.1);
        this->declare_parameter("validation_margin", 0.25);

        _velocityIncrement = this->get_parameter("velocity_increment").as_double();
        _step_size = this->get_parameter("step_size").as_double();

        // Publisher
        _path_visualization_pub =
            this->create_publisher<tod_trajectory_guidance_msgs::msg::Path>("output/path_visualization", 5);
        
        _path_send_to_vehicle_pub = this->create_publisher<tod_trajectory_guidance_msgs::msg::Path>("output/path", 5);
        
        _trajectory_guidance_control_pub =
            this->create_publisher<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl>(
                "output/trajectory_guidance_control_cmd", 1);
        
        _control_points_pub =
            this->create_publisher<tod_trajectory_guidance_msgs::msg::ControlPoints>("output/path_control_points", 1);
        
        _validation_visualization_trajectory_pub =
            this->create_publisher<tod_trajectory_guidance_msgs::msg::Trajectory>("output/validation_trajectory",
                                                                                  1);
        // Subscriber
        _status_sub = this->create_subscription<tod_status_msgs::msg::Status>(
            "input/operator_status", 1,
            [this](const tod_status_msgs::msg::Status::SharedPtr msg) { this->callback_status_msg(msg); });

        _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "input/odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->callback_odometry(msg); });

        _mouse_click_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "input/mouse_position_click", 1,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { this->callback_mouse_click(msg); });

        _mouse_position_moved_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "input/mouse_position_moved", 1, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                this->callback_mouse_position_moved(msg);
            });

        _mouse_click_released_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "input/mouse_position_released", 1,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { this->callback_mouse_released(msg); });

        _callback_key_action = this->create_subscription<tod_operator_msgs::msg::KeyPress>(
            "input/key_press", 1,
            [this](const tod_operator_msgs::msg::KeyPress::SharedPtr msg) { this->callback_key_action(msg); });

        _validation_trajectoy_sub = this->create_subscription<tod_trajectory_guidance_msgs::msg::Trajectory>(
            "input/validation_trajectory", 1, [this](const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr msg) {
                this->callback_validation_trajectory_visualization(msg);
            });

        _tg_state_sub = this->create_subscription<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState>(
            "input/trajectory_guidance_state", 1,
            [this](const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::SharedPtr msg) {
                this->callback_trajectory_guidance_state(msg);
            });
    }

  private:
    TimerManager _timer_manager;
    std::shared_mutex _statemutex;
    PathState _state;

    double x = 0;
    double y = 0;
    double _phi = 0;

    static constexpr float INCREMENT_VELOCITY{1.0f};
    static constexpr int PATH_CHECK_FREQUENCY{10};
    
    int _repeatCount{false};
    int _odom_counter{0};
    double _step_size;
    float _targetVelocity;
    float _velocityIncrement;
    bool _velocity_update_rejected = false;
    bool _holding = false;
    int _edit_at = -1;

    std::deque<tod_trajectory_guidance_msgs::msg::ControlPoint> _last_visited_points;

    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl>::SharedPtr
        _trajectory_guidance_control_pub;
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::Path>::SharedPtr _path_visualization_pub;
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::Path>::SharedPtr _path_send_to_vehicle_pub;
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::ControlPoints>::SharedPtr _control_points_pub;
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr
        _validation_visualization_trajectory_pub;

    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState>::SharedPtr _tg_state_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _mouse_click_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _mouse_position_moved_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _mouse_click_released_sub;
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr _validation_trajectoy_sub;
    rclcpp::Subscription<tod_operator_msgs::msg::KeyPress>::SharedPtr _callback_key_action;
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _status_sub;

    void callback_status_msg(const tod_status_msgs::msg::Status::SharedPtr msg);
    void callback_trajectory_guidance_state(const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::SharedPtr msg);
    void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odom_message);
    void callback_mouse_click(const geometry_msgs::msg::PointStamped::SharedPtr point);
    void callback_mouse_position_moved(const geometry_msgs::msg::PointStamped::SharedPtr point);
    void callback_mouse_released(const geometry_msgs::msg::PointStamped::SharedPtr point);
    void callback_key_action(const tod_operator_msgs::msg::KeyPress::SharedPtr keyPress);
    void callback_secondary_vehicle_state(
        const tod_vehicle_msgs::msg::SecondaryVehicleState::SharedPtr secondary_vehicle_state);
    void callback_validation_trajectory_visualization(const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr msg);

    void publish_visualization_path();
    void publish_sendPath();
    void publish_control_points();

    void send_reset_path();
    void send_stop_signal();
    void send_start_signal();
    void send_incremeent_target_velo();
    void send_decremeent_target_velo();
    bool check_incoming_trajectory_validated();

    // control point manipulation
    void find_closest_control_point(const geometry_msgs::msg::PointStamped::SharedPtr point);
    void clean_driven_path();
    void edit_control_point_position(const size_t index, const geometry_msgs::msg::PointStamped::SharedPtr point);
    void add_control_point_to_list(const geometry_msgs::msg::PointStamped::SharedPtr point);
    void pop_last_control_point();
};

}  // namespace tod_trajectory_guidance
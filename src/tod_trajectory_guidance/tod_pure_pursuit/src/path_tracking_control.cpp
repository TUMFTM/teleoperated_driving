
/**
 * @file path_tracking_control.cpp
 * @brief Management Node for the \ref PurePursuit Controller for Trajectory Guidance as well as simulating the
 * trajectory for the validation process
 * @details PathTrackingControl implements manages the interaction with the Controller i.e. making sure that the
 * coodinate system is correct and publishes the control cmds etc.
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#include "tod_pure_pursuit/path_tracking_control.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace tod_pure_pursuit {
/*
 * @ingroup tod_trajectory_guidance
 */

PathTrackingControl::PathTrackingControl() : Node("PathTrackingControl") {
    this->declare_parameter<std::string>("config_path", "N/A");
    std::string config_path;

    if (!this->get_parameter("config_path", config_path)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to retrieve 'config_path' parameter. Ensure it is set in the launch file.");
    }

    _vehParams = std::make_shared<tod_core::param_set::Vehicle>(this, config_path + "/vehicle_config/");
    _purePursuit = std::make_unique<PurePursuit>(this, _vehParams);
    auto sub_qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    sub_qos.best_effort();

    _subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "input/odom", 1, [this](const nav_msgs::msg::Odometry &msg) { this->callback_odometry(msg); });

    _subTrajectory = this->create_subscription<tod_trajectory_guidance_msgs::msg::Trajectory>(
        "input/trajectory", 10,
        [this](const tod_trajectory_guidance_msgs::msg::Trajectory &msg) { this->callback_trajectory(msg); });

    _subStatus = this->create_subscription<tod_status_msgs::msg::Status>(
        "input/vehicle_status", 1,
        [this](const tod_status_msgs::msg::Status::SharedPtr msg) { this->callback_status_msg(*msg); });

    _pubControlCmd = this->create_publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>("output/primary_control_cmd", 1);
    _pubLogging = this->create_publisher<tod_trajectory_guidance_msgs::msg::PpLog>("output/ptc_logging", 1);
    _poseArrayPub =
        this->create_publisher<geometry_msgs::msg::PoseArray>("output/path_array_for_rviz_controller", 1);

    _timer = this->create_wall_timer(10ms, [this] { loop(); });

    _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _transformListener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);
    _staticTransformBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Parameters
    this->declare_parameter<bool>("debug", false);

    bool _debug = false;
    if (!this->get_parameter("debug", _debug))
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get parameter debug. Using _debug " << _debug);
    else
        RCLCPP_INFO_STREAM(this->get_logger(), "_debug is set to " << _debug);

    this->declare_parameter<int>("ControlMode", 3);
    int mode = 0;
    if (!this->get_parameter("ControlMode", mode))
        RCLCPP_ERROR_STREAM(this->get_logger(), "No control mode provided at " << std::string(this->get_name()).c_str()
                                                                               << ". Default value " << mode
                                                                               << " used. Shutting down node.");
    else
        RCLCPP_INFO_STREAM(this->get_logger(), "Control mode provided is" << mode);
    _desiredControlMode = mode;
}

/*
 * @brief Function for the control loop per incoming active trajectory that publishes the calculated control commands
 */
void PathTrackingControl::loop() {
    if (_trajectory.points.size() > 0) {
        if (_actualControlMode == _desiredControlMode) {
            if (_trajectory.header.frame_id == _poseRearAxle.header.frame_id) {
                _purePursuit->calc_control_command(_trajectory, _poseRearAxle, _primaryControlCommand, _actualSpeed);
                _primaryControlCommand.header.stamp = rclcpp::Clock().now();
                _pubControlCmd->publish(_primaryControlCommand);
                publish_logging_output();
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Odometry(" << _poseRearAxle.header.frame_id << ") and Trajectory("
                                                << _trajectory.header.frame_id << ") provided in different frames");
                return;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Actual Control Mode: %d, Desired Control Mode: %d", _actualControlMode,
                        _desiredControlMode);
        }
    } else {
        if (_actualControlMode == _desiredControlMode) {
            _primaryControlCommand.header.stamp = rclcpp::Clock().now();
            _primaryControlCommand.velocity = 0.0;
            _primaryControlCommand.acceleration = -1.0;
            _primaryControlCommand.steering_wheel_angle = 0.0;
            _pubControlCmd->publish(_primaryControlCommand);
        }
    }
    //}
}

/*
 * @brief Pperforms the coordinate system checking of the incoming trajectory (map) to the rearAxelFootprint param and
 * prepares the control loop.
 */
void PathTrackingControl::callback_trajectory(const tod_trajectory_guidance_msgs::msg::Trajectory &trajectory) {
    // if (trajectory.points.size() <= 1)
    //     return;
    _trajectory = trajectory;

    // Currently the trajectory is defined in the 'ftm' frame. We need it in the 'map' frame
    // to successfully run the controller
    if (_trajectory.header.frame_id != _poseRearAxle.header.frame_id) {
        geometry_msgs::msg::TransformStamped tf =
            get_transform(_trajectory.header.frame_id, _poseRearAxle.header.frame_id);
        RCLCPP_INFO_ONCE(this->get_logger(), "Transform applied to the trajectory points: From %s to %s",
                         tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "Transform applied to the trajectory points: TRANSLATION - x: %f, y: %f, z: %f, ROTATION - x: "
                         "%f, y: %f, z: %f, w: %f",
                         tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
                         tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                         tf.transform.rotation.w);
        for (tod_trajectory_guidance_msgs::msg::TrajectoryPoint &point : _trajectory.points) {
            geometry_msgs::msg::Pose pose;
            pose = point.pose;
            geometry_msgs::msg::Pose poseTransformed;
            tf2::doTransform(pose, poseTransformed, tf);
            point.pose = poseTransformed;
        }
        _trajectory.header.frame_id = _poseRearAxle.header.frame_id;
    }

    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.stamp = this->get_clock()->now();
    poseArray.header.frame_id = _trajectory.header.frame_id;
    for (const auto &point : _trajectory.points) {
        poseArray.poses.push_back(point.pose);
    }
    _poseArrayPub->publish(poseArray);

}

void PathTrackingControl::callback_odometry(const nav_msgs::msg::Odometry &odometry) {
    _actualSpeed = odometry.twist.twist.linear.x;
    _poseRearAxle.pose = odometry.pose.pose;
    _poseRearAxle.pose.position.z = 0.0;  // Necessary to calculate correct y-offset between vehicle pose and next trajectory point

    if (odometry.child_frame_id != _purePursuit->get_rear_axle_frame_id()) {
        geometry_msgs::msg::TransformStamped tf =
            get_transform(_purePursuit->get_rear_axle_frame_id(), odometry.child_frame_id);
        transform_child_frame(_poseRearAxle.pose, tf);
    }
    _poseRearAxle.header = odometry.header;
    RCLCPP_INFO_ONCE(this->get_logger(), "Odometry in defined frame %s", _poseRearAxle.header.frame_id.c_str());
    this->_odometry = odometry;
}

void PathTrackingControl::callback_status_msg(const tod_status_msgs::msg::Status &msg) {
    if (_actualControlMode != msg.vehicle_control_mode)
        _trajectory.points.clear();

    _actualControlMode = msg.vehicle_control_mode;
    this->_previousTodStatus = msg.tod_status;
}

geometry_msgs::msg::TransformStamped PathTrackingControl::get_transform(const std::string &srcFrame,
                                                                        const std::string &trgFrame) {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time now = rclcpp::Clock().now();
    bool can_transform = _tfBuffer->canTransform(trgFrame, srcFrame, now, rclcpp::Duration::from_seconds(5.0));
    if (can_transform) {
        transform = _tfBuffer->lookupTransform(trgFrame, srcFrame, now, rclcpp::Duration::from_seconds(5.0));
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot get tf from " << srcFrame.c_str() << " to " << trgFrame.c_str()
                                                                      << " within 5 seconds");
    }
    return transform;
}

void PathTrackingControl::publish_logging_output() {
    tod_trajectory_guidance_msgs::msg::PpLog log;
    log.pose_error = _purePursuit->pose_error;
    log.time_error_to_closest_pose = _purePursuit->time_error_to_closest_pose;
    log.time_error_to_first_pose = _purePursuit->time_error_to_first_pose;
    log.closest_wp = _purePursuit->closest_wp;
    log.next_wp = _purePursuit->next_wp;
    _pubLogging->publish(log);
}
}  // namespace tod_pure_pursuit
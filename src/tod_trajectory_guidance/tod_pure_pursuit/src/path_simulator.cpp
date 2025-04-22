
/**
 * @file path_simulator.cpp
 * @brief Management Node for the \ref PurePursuit Controller simulating the driviving of the trajectory for the
 * validation process
 * @details PathSimulator implements manages the interaction with the Controller i.e. making sure that the coodinate
 * system is correct and publishes the control cmds etc.
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#include "tod_pure_pursuit/path_simulator.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace tod_pure_pursuit {
/*
 * @ingroup tod_trajectory_guidance
 */

PathSimulator::PathSimulator() : Node("PathSimulator") {
    this->declare_parameter<std::string>("config_path", "N/A");
    std::string config_path;

    if (!this->get_parameter("config_path", config_path)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to retrieve 'config_path' parameter. Ensure it is set in the launch file.");
    }

    _vehParams = std::make_shared<tod_core::param_set::Vehicle>(this, config_path + "/vehicle_config/");
    _simPurePursuit = std::make_unique<PurePursuit>(this, _vehParams);

    auto sub_qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    sub_qos.best_effort();

    _subValidationTrajectory = this->create_subscription<tod_trajectory_guidance_msgs::msg::Trajectory>(
        "input/validation_trajectory", 1, [this](const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr msg) {
            this->callback_simulated_trajectory(msg);
        });

    _pubSimulatedTrajectory =
        this->create_publisher<tod_trajectory_guidance_msgs::msg::Trajectory>("output/validation_trajectory", 1);
     
    _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _transformListener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);
    // Parameters
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<double>("simulation_step", 0.1);

    bool _debug = false;
    if (!this->get_parameter("debug", _debug))
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get parameter _debug. Using _debug " << _debug);
    else
        RCLCPP_INFO_STREAM(this->get_logger(), "_debug is set to " << _debug);

    _resultTimer = this->create_wall_timer(std::chrono::milliseconds(50), [this] { check_results(); });
}

/*
 * @brief Simulates a Trajectory based of the incoming inactive trajectory based on a kinematic bicycle model \ref
 * VehicleModel
 */
void PathSimulator::callback_simulated_trajectory(
    const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr trajectory) {
    if (trajectory->points.empty()) {
        return;
    }

    if (trajectory->header.frame_id != "base_link") {
        geometry_msgs::msg::TransformStamped tf = get_transform(trajectory->header.frame_id, "base_link");

        for (tod_trajectory_guidance_msgs::msg::TrajectoryPoint& point : trajectory->points) {
            geometry_msgs::msg::Pose pose;
            pose = point.pose;
            geometry_msgs::msg::Pose poseTransformed;
            tf2::doTransform(pose, poseTransformed, tf);
             point.pose = poseTransformed;
        }
        trajectory->header.frame_id = "base_link";
    }
    auto sim = std::make_unique<PurePursuit>(this, _vehParams);
    auto sim_time = this->get_parameter("simulation_step").as_double();
    _futures.push_back(std::async(std::launch::async, [sim = std::move(sim), trajectory, sim_time]() {
        return sim->simulate_trajectory(trajectory, sim_time);
    }));
};

geometry_msgs::msg::TransformStamped PathSimulator::get_transform(const std::string& srcFrame,
                                                                  const std::string& trgFrame) {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time now = this->now();
    bool can_transform = _tfBuffer->canTransform(trgFrame, srcFrame, now, rclcpp::Duration::from_seconds(5.0));
    if (can_transform) {
        transform = _tfBuffer->lookupTransform(trgFrame, srcFrame, now, rclcpp::Duration::from_seconds(5.0));
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot get tf from " << srcFrame.c_str() << " to " << trgFrame.c_str()
                                                                      << " within 5 seconds");
    }
    return transform;
}

void PathSimulator::check_results() {
    for (auto it = _futures.begin(); it != _futures.end();) {
        if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                auto result = it->get();
                if (result.header.frame_id != "map") {
                    auto tf = get_transform(result.header.frame_id, "map");
                    for (tod_trajectory_guidance_msgs::msg::TrajectoryPoint& point : result.points) {
                        geometry_msgs::msg::Pose pose;
                        pose = point.pose;
                        geometry_msgs::msg::Pose poseTransformed;
                        tf2::doTransform(pose, poseTransformed, tf);
                        point.pose = poseTransformed;
                    }
                    result.header.frame_id = "map";
                }
                _pubSimulatedTrajectory->publish(result);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Simulation failed: %s", e.what());
            }
            it = _futures.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace tod_pure_pursuit

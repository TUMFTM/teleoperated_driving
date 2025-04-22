
/**
 * @file path_simulator.hpp
 * @brief Management Node for the \ref PurePursuit Controller simulating the driviving of the trajectory for the
 * validation process
 * @details PathSimulator implements manages the interaction with the Controller i.e. making sure that the coodinate
 * system is correct and publishes the control cmds etc.
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#pragma once

#include <algorithm>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tod_status_msgs/msg/status.hpp"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tod_core/param_set/VehicleParameters.hpp"
#include "tod_pure_pursuit/pure_pursuit.hpp"
#include "tod_pure_pursuit/vehicle_model.hpp"
#include "tod_trajectory_guidance_msgs/msg/pp_log.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_point.hpp"

namespace tod_pure_pursuit {
/*
 * @ingroup tod_trajectory_guidance
 */

/*
 * @brief Manager Node for the \ref PurePursuit Controller for translating the incoming trajectory to control commands
 * and for simulating the trajectory for the operator
 */
class PathSimulator : public rclcpp::Node {
  public:
    PathSimulator();

  private:
    // timer
    rclcpp::TimerBase::SharedPtr _timer;

    // subscriber
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr _subValidationTrajectory;

    std::shared_ptr<tf2_ros::TransformListener> _transformListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> _tfBuffer;

    // callbacks
    void callback_simulated_trajectory(const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr trajectory);
    void check_results();
    // publisher
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr _pubSimulatedTrajectory;

    // feedback
    geometry_msgs::msg::TransformStamped get_transform(const std::string &srcFrame, const std::string &trgFrame);


    std::unique_ptr<PurePursuit> _simPurePursuit;
    std::shared_ptr<tod_core::param_set::Vehicle> _vehParams;
    std::vector<std::future<tod_trajectory_guidance_msgs::msg::Trajectory>> _futures;
    rclcpp::TimerBase::SharedPtr _resultTimer;
};
}  // namespace tod_pure_pursuit
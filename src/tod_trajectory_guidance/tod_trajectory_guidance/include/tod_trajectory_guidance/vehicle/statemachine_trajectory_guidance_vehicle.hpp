/**
 * @file StatemachineVehicle.hpp
 * @brief Boost SML
 * @copyright 2024 TUMFTM
 */

#include "boost/sml.hpp"
#include "trajectory_guidance.hpp"

namespace sml = boost::sml;

namespace tod_trajectory_guidance::statemachine {
/*
 * State machine based @see https://github.com/boost-ext/sml to monitor and control the behavior of the trajectory
 * guidance workflow similar to @see https://arxiv.org/abs/2402.10014
 * @defgroup tod_trajectory_guidance_statemachine
 * @ingroup tod_trajectory_guidance
 */

/**
 * @brief Container struct for all state machine states
 * @details Defines the different states that the trajectory guidance system can be in
 */
struct states {
    /**
     * @brief Initial state where the system awaits a new trajectory
     */
    struct WaitingForTrajectory {};
    /**
     * @brief State for validating received trajectories
     * @details The trajectories recieved in this state are send back to the operator for validation and only upon
     * validation the trajectory the transition to start is possible
     */
    struct ValidatingTrajectory {};
    /**
     * @brief Executing state in which the active trajectory is published for the trajectory following controler @ref
     * tod_pure_pursuit
     */
    struct ExecutingTrajectory {};
    /**
     * @brief Stopping and emergency state in which the vehicle deaccelerates to still stand as fast as possibe based on
     * the current trajectory
     */
    struct ExecutingStopTrajectory {};
};

/**
 * @brief Container struct for all state machine events
 * @details Defines the events that can trigger state transitions
 */
struct events {
    /**
     * @brief Event triggered when a new trajectory is received
     */
    struct TRAJECTORY_RECEIVED {};
    /**
     * @brief Event triggered when a validated trajectory is received
     */
    struct VALIDATED_TRAJECTORY_RECEIVED {
        std::vector<tod_trajectory_guidance_msgs::msg::TrajectoryPoint> trajectory;
    };
    /**
     * @brief Event triggered when a trajectory is rejected based on a set of requirements
     */
    struct TRAJECTORY_REJECTED {};
    /**
     * @brief Event triggered when a velocity update is received and applied
     */
    struct VELOCITY_UPDATE_RECEIVED {};
    /**
     * @brief Event triggered when a velocity update is rejected e.g. maximum velocity is reached
     */
    struct VELOCITY_UPDATE_REJECTED {};
    /**
     * @brief Event triggered when a start signal is send and a validated trajectory is available - which sets the
     * active_trajectory based on the validated trajectory
     */
    struct START_TRAJECTORY {};
    /**
     * @brief Event triggered when trajectory execution is complete
     */
    struct EXECUTION_FINISHED {};
    /**
     * @brief Event triggered when trajectory execution is manually canceled
     */
    struct EXECUTION_CANCELED {};
    /**
     * @brief Event triggered to reset the state machine back to the state @ref waiting_for_trajectory
     */
    struct RESET_TRIGGERED {};
    /**
     * @brief Event triggered when watchdog timeout occurs
     */
    struct WATCHDOG_TRIGGERED {};
};

// Actions

/**
 * @brief Calculates a new trajectory based on the path - sets a velocity profile based on the current parameter and the
 * lateral velocity limits of the trajectory
 * @param[in,out] tg Pointer to TrajectoryGuidance instance
 */
auto ac_calc_trajectory = [](TrajectoryGuidance *tg) { tg->calc_trajectory(); };

/**
 * @brief Updates the velocity of the active trajectory
 * @param[in,out] tg Pointer to TrajectoryGuidance instance
 */
auto ac_update_velocity_active_trajectory = [](TrajectoryGuidance *tg) { tg->update_velocity_active_trajectory(); };

/**
 * @brief Starts the trajectory execution if a valid trajectory exists
 * @param[in,out] tg Pointer to TrajectoryGuidance instance
 */
auto ac_start_trajectory = [](TrajectoryGuidance *tg) {
    if (tg->has_valid_trajectory()) {
        tg->start_trajectory();
    }
};

/**
 * @brief Sets the received validated trajectory as inactive
 * @param[in,out] tg Pointer to TrajectoryGuidance instance
 * @param[in] event Event containing the validated trajectory
 */
auto ac_validated_trajectory_received = [](TrajectoryGuidance *tg, const events::VALIDATED_TRAJECTORY_RECEIVED &event) {
    tg->set_inactive_trajectory(event.trajectory);
};

/**
 * @brief Calculates an emergency stop trajectory
 * @param[in,out] tg Pointer to TrajectoryGuidance instance
 */
auto ac_calc_stop_trajectory = [](TrajectoryGuidance *tg) { tg->calc_stop_trajectory(); };

/**
 * @brief Resets the trajectory guidance validation process
 * @param[in,out] tg Pointer to TrajectoryGuidance instance
 */
auto ac_reset_trajectory = [](TrajectoryGuidance *tg) { tg->reset_trajectory(); };

/**
 * @brief Updates the target velocity for the velocity profile of the trajecotry
 * @param[in,out] tg Pointer to TrajectoryGuidance instance
 */
auto ac_update_target_velocity = [](TrajectoryGuidance *tg) { tg->validate_velocity(); };

/**
 * @brief Guard condition checking if trajectory execution can start
 * @param[in] tg Pointer to TrajectoryGuidance instance
 * @return true if drive status is active and valid trajectory exists
 */
auto guard_can_start_trajectory = [](TrajectoryGuidance *tg) {
    return tg->get_drive_status() && tg->has_valid_trajectory();
};

/**
 * @brief Guard condition validating velocity updates
 * @param[in] tg Pointer to TrajectoryGuidance instance
 * @return true if velocity is valid within the bounds of max and min velocity
 */
auto guard_velocity_valid = [](TrajectoryGuidance *tg) { return tg->validate_velocity(); };

/**
 * @brief Main state machine class for trajectory guidance
 * @details Implements a finite state machine using boost-ext/sml to control trajectory guidance workflow
 * @see https://github.com/boost-ext/sml
 * @see https://arxiv.org/abs/2402.10014
 */
struct StateMachineTrajectoryGuidance {
    auto operator()() const {
        using namespace sml;
        return sml::make_transition_table(
            *state<states::WaitingForTrajectory> + event<events::TRAJECTORY_RECEIVED> / ac_calc_trajectory =
                state<states::ValidatingTrajectory>,
            state<states::WaitingForTrajectory> + event<events::VELOCITY_UPDATE_RECEIVED>[guard_velocity_valid] /
                                                      ac_update_target_velocity = state<states::WaitingForTrajectory>,
            state<states::ValidatingTrajectory> + event<events::TRAJECTORY_REJECTED> / ac_reset_trajectory =
                state<states::WaitingForTrajectory>,
            state<states::ValidatingTrajectory> + event<events::TRAJECTORY_RECEIVED> / ac_calc_trajectory =
                state<states::ValidatingTrajectory>,  // Update
            state<states::ValidatingTrajectory> +
                event<events::VALIDATED_TRAJECTORY_RECEIVED> / ac_validated_trajectory_received =
                state<states::ValidatingTrajectory>,
            state<states::ValidatingTrajectory> + event<events::START_TRAJECTORY>[guard_can_start_trajectory] /
                                                      ac_start_trajectory = state<states::ExecutingTrajectory>,
            state<states::ValidatingTrajectory> + event<events::VELOCITY_UPDATE_RECEIVED>[guard_velocity_valid] /
                                                      ac_calc_trajectory = state<states::ValidatingTrajectory>,
            state<states::ValidatingTrajectory> + event<events::RESET_TRIGGERED> / ac_reset_trajectory =
                state<states::WaitingForTrajectory>,
            state<states::ExecutingTrajectory> +
                event<events::VELOCITY_UPDATE_RECEIVED>[guard_velocity_valid] / ac_update_velocity_active_trajectory =
                state<states::ExecutingTrajectory>,
            state<states::ExecutingTrajectory> + event<events::EXECUTION_CANCELED> / ac_calc_stop_trajectory =
                state<states::ExecutingStopTrajectory>,
            state<states::ExecutingTrajectory> + event<events::RESET_TRIGGERED> / ac_calc_stop_trajectory =
                state<states::ExecutingStopTrajectory>,
            state<states::ExecutingTrajectory> + event<events::WATCHDOG_TRIGGERED> / ac_calc_stop_trajectory =
                state<states::ExecutingStopTrajectory>,
            state<states::ExecutingStopTrajectory> + event<events::EXECUTION_FINISHED> / ac_reset_trajectory =
                state<states::WaitingForTrajectory>,
            state<states::ExecutingTrajectory> + event<events::EXECUTION_FINISHED> / ac_reset_trajectory =
                state<states::WaitingForTrajectory>);
    };
};
}  // namespace tod_trajectory_guidance::statemachine
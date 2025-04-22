/**
 * @file edgarautoware_actuation_interface.hpp
 * @brief Actuation interface for the research vehicle EDGAR using Autoware as AV stack.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgarautoware_interface
 */

#pragma once

#include "tod_generic_interface/actuation_interface.hpp"

#include "tod_vehicle_msgs/VehicleEnums.h"

#include "tum_edgar_can_msgs/msg/tum_edgar_gateway_states.hpp"
#include "tum_edgar_can_msgs/msg/tum_edgar_motion.hpp"
#include "tum_edgar_utils/iav_can_enums.hpp"

#include "tum_autoware_vehicle_msgs/msg/head_light_report.hpp"
#include "tum_autoware_vehicle_msgs/msg/high_beam_report.hpp"
#include "tum_autoware_vehicle_msgs/msg/honk_report.hpp"
#include "tum_autoware_vehicle_msgs/msg/wiper_report.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"

#include "tier4_external_api_msgs/msg/heartbeat.hpp"

namespace tod_edgarautoware_interface {
/**
 * @defgroup tod_edgarautoware_interface
 * @ingroup tod_edgarautoware_interface
 * @brief Interfaces for the research vehicle EDGAR using Autoware as AV stack.
 */

/**
 * @brief Actuation interface for the research vehicle EDGAR using Autoware as AV stack.
 */
class ActuationInterface : public rclcpp::Node
{
    public:
        ActuationInterface(); 
        ~ActuationInterface();
        void run();
    private:
        // Subscriptions
        // functions for the tum message interface of edgar
        void edgar_gateway_states_handler(const tum_edgar_can_msgs::msg::TUMEdgarGatewayStates &msg);
        void edgar_motion_handler(const tum_edgar_can_msgs::msg::TUMEdgarMotion &msg);
        // functions for the aw report message interface of edgar 
        // IMPORTANT: EDGAR2AW provides its vehicle interface using Autoware's definitions and TUM extensions.
        void aw_velocity_report_handler(const autoware_auto_vehicle_msgs::msg::VelocityReport &msg);
        void aw_steering_report_handler(const autoware_auto_vehicle_msgs::msg::SteeringReport &msg);
        void aw_gear_report_handler(const autoware_auto_vehicle_msgs::msg::GearReport &msg);
        void aw_hazard_lights_report_handler(const autoware_auto_vehicle_msgs::msg::HazardLightsReport &msg);
        void aw_turn_indicators_report_handler(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport &msg);
        void tum_head_light_report_handler(const tum_autoware_vehicle_msgs::msg::HeadLightReport &msg);
        void tum_high_beam_report_handler(const tum_autoware_vehicle_msgs::msg::HighBeamReport &msg);
        void tum_honk_report_handler(const tum_autoware_vehicle_msgs::msg::HonkReport &msg);
        void tum_wiper_report_handler(const tum_autoware_vehicle_msgs::msg::WiperReport &msg);
        
        // Publisher
        // message builder for publisher
        autoware_auto_control_msgs::msg::AckermannControlCommand aw_ackermann_control_command_msg_builder();
        autoware_auto_vehicle_msgs::msg::GearCommand aw_gear_command_msg_builder();
        autoware_auto_vehicle_msgs::msg::HazardLightsCommand aw_hazard_lights_command_msg_builder();
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand aw_turn_indicators_command_msg_builder();
        tier4_external_api_msgs::msg::Heartbeat aw_heartbeat_msg_builder();
            
        // Params
        double kp_{2.0};  // proportional gain for PI controller
        double ki_{0.0};  // integral gain for PI controller´
        // Linear steering wheel to tire wheel transmission
        // tire angle range: [-0.610865, 0.610865] from edgar digital twin
        // steering wheel range: [-9.42, 9.42] common value -> 1.5x left/right rotation
        double max_steer_wheel_{9.42};       
        float wheel2tire_factor_ = 0.0648;

        // Signal maps
        const std::map<uint8_t, uint8_t> tod_to_aw_gear_map_ = {
            {eGearPosition::GEARPOSITION_PARK, autoware_auto_vehicle_msgs::msg::GearCommand::PARK},
            {eGearPosition::GEARPOSITION_REVERSE, autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE},
            {eGearPosition::GEARPOSITION_NEUTRAL, autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL},
            {eGearPosition::GEARPOSITION_DRIVE, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE},
            {eGearPosition::GEARPOSITION_SPORT, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE},
            {eGearPosition::GEARPOSITION_HAUL, autoware_auto_vehicle_msgs::msg::GearCommand::LOW},
        };
        const std::map<uint8_t, uint8_t> aw_to_tod_gear_map_ = {
            {autoware_auto_vehicle_msgs::msg::GearCommand::PARK, eGearPosition::GEARPOSITION_PARK},
            {autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE, eGearPosition::GEARPOSITION_REVERSE},
            {autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL, eGearPosition::GEARPOSITION_NEUTRAL},
            {autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE, eGearPosition::GEARPOSITION_DRIVE},
            {autoware_auto_vehicle_msgs::msg::GearCommand::LOW, eGearPosition::GEARPOSITION_HAUL},
        };
        const std::map<uint8_t, uint8_t> tod_to_aw_indicator_ = 
        {
            {eIndicator::INDICATOR_OFF, autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE},
            {eIndicator::INDICATOR_LEFT, autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT},
            {eIndicator::INDICATOR_RIGHT, autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT}
        };
        
        // Member variables
        double integral_;
        rclcpp::Time last_ackermann_command_time_;     // Time of the last ackermann command. Needed for dt 
        std::shared_ptr<tod_generic_interface::ActuationInterface> generic_actuation_interface_;
};

} // namespace tod_edgarautoware_interface
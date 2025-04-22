/**
 * @file edgar_actuation_interface.cpp
 * @brief Actuation interface for the research vehicle EDGAR without AV capabilities.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgar_interface
 */

#include "tod_edgar_interface/edgar_actuation_interface.hpp"

namespace tod_edgar_interface {

ActuationInterface::ActuationInterface() : rclcpp::Node("edgar_actuation_interface") 
{
    // Initialize parameters
    this->declare_parameter<double>("kp", 2.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("max_steer_wheel", 9.42);
    this->declare_parameter<double>("wheel2tire_factor", 0.0648);
    // Vehicle topics
    //// EDGAR Interface
    this->declare_parameter<std::string>("edgar_gateway_states_topic", "none");
    this->declare_parameter<std::string>("edgar_powertrain_topic", "none");
    this->declare_parameter<std::string>("edgar_misc_topic", "none");
    this->declare_parameter<std::string>("edgar_motion_topic", "none");
    this->declare_parameter<std::string>("edgar_steering_topic", "none");
    //// AW Bridge
    this->declare_parameter<std::string>("aw_gear_command_topic", "none");
    this->declare_parameter<std::string>("aw_ackermann_command_topic", "none");
    this->declare_parameter<std::string>("aw_hazard_light_command_topic", "none");
    this->declare_parameter<std::string>("aw_indicator_command_topic", "none");
    this->declare_parameter<std::string>("aw_velocity_report_topic", "none");
    this->declare_parameter<std::string>("aw_steering_report_topic", "none");
    this->declare_parameter<std::string>("aw_gear_report_topic", "none");
    this->declare_parameter<std::string>("aw_hazard_light_report_topic", "none");
    this->declare_parameter<std::string>("aw_turn_indicator_report_topic", "none");
    //// AW Bridge TUM Extension
    this->declare_parameter<std::string>("tum_head_light_report_topic", "none");
    this->declare_parameter<std::string>("tum_high_beam_report_topic", "none");
    this->declare_parameter<std::string>("tum_honk_report_topic", "none");
    this->declare_parameter<std::string>("tum_wiper_report_topic", "none");
    // this->declare_parameter<std::string>("tum_honk_command_topic", "none");
    // this->declare_parameter<std::string>("tum_wiper_command_topic", "none");

    // Get parameters
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    max_steer_wheel_ = this->get_parameter("max_steer_wheel").as_double();
    wheel2tire_factor_ = this->get_parameter("wheel2tire_factor").as_double();
    RCLCPP_INFO(this->get_logger(), "ActuationInterface created");
}

ActuationInterface::~ActuationInterface() 
{
    RCLCPP_INFO(this->get_logger(), "ActuationInterface shutting down");
}

void ActuationInterface::run()
{
    this->generic_actuation_interface_ = std::make_shared<tod_generic_interface::ActuationInterface>(
        std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this())
    );

    // EDGAR Interface
    std::string edgar_gateway_states_topic = this->get_parameter("edgar_gateway_states_topic").as_string();
    std::string edgar_powertrain_topic = this->get_parameter("edgar_powertrain_topic").as_string();
    std::string edgar_misc_topic = this->get_parameter("edgar_misc_topic").as_string();
    std::string edgar_motion_topic = this->get_parameter("edgar_motion_topic").as_string();
    std::string edgar_steering_topic = this->get_parameter("edgar_steering_topic").as_string();
    // AW Bridge
    std::string aw_gear_command_topic = this->get_parameter("aw_gear_command_topic").as_string();
    std::string aw_ackermann_command_topic = this->get_parameter("aw_ackermann_command_topic").as_string();
    std::string aw_hazard_light_command_topic = this->get_parameter("aw_hazard_light_command_topic").as_string();
    std::string aw_indicator_command_topic = this->get_parameter("aw_indicator_command_topic").as_string();
    std::string aw_velocity_report_topic = this->get_parameter("aw_velocity_report_topic").as_string();
    std::string aw_steering_report_topic = this->get_parameter("aw_steering_report_topic").as_string();
    std::string aw_gear_report_topic = this->get_parameter("aw_gear_report_topic").as_string();
    std::string aw_hazard_light_report_topic = this->get_parameter("aw_hazard_light_report_topic").as_string();
    std::string aw_turn_indicator_report_topic = this->get_parameter("aw_turn_indicator_report_topic").as_string();
    // AW Bridge TUM Extension
    std::string tum_head_light_report_topic = this->get_parameter("tum_head_light_report_topic").as_string();
    std::string tum_high_beam_report_topic = this->get_parameter("tum_high_beam_report_topic").as_string();
    std::string tum_honk_report_topic = this->get_parameter("tum_honk_report_topic").as_string();
    std::string tum_wiper_report_topic = this->get_parameter("tum_wiper_report_topic").as_string();
    // std::string horn_command_topic = this->get_parameter("horn_command_topic").as_string();
    // std::string wiper_command_topic = this->get_parameter("wiper_command_topic").as_string();

    // Initialize integral value
    integral_ = 0.0;

    // Set up subscriptions
    // Conditionally set up subscribers
    if (edgar_gateway_states_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_edgar_can_msgs::msg::TUMEdgarGatewayStates>(
            edgar_gateway_states_topic,
            [this](const tum_edgar_can_msgs::msg::TUMEdgarGatewayStates &msg) {
                this->safety_driver_status_from_can(msg);
            });
    }

    if (edgar_powertrain_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_edgar_can_msgs::msg::TUMEdgarPowertrain1>(
            edgar_powertrain_topic,
            [this](const tum_edgar_can_msgs::msg::TUMEdgarPowertrain1 &msg) {
                this->powertrain_handler(msg);
            });
    }

    if (edgar_misc_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_edgar_can_msgs::msg::TUMEdgarMisc1>(
            edgar_misc_topic,
            [this](const tum_edgar_can_msgs::msg::TUMEdgarMisc1 &msg) {
                this->misc_handler(msg);
            });
    }

    if (edgar_motion_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_edgar_can_msgs::msg::TUMEdgarMotion>(
            edgar_motion_topic,
            [this](const tum_edgar_can_msgs::msg::TUMEdgarMotion &msg) {
                this->motion_handler(msg);
            });
    }

    if (edgar_steering_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_edgar_can_msgs::msg::TUMEdgarSteering>(
            edgar_steering_topic,
            [this](const tum_edgar_can_msgs::msg::TUMEdgarSteering &msg) {
                this->steering_handler(msg);
            });
    }

    if (aw_velocity_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            aw_velocity_report_topic,
            [this](const autoware_auto_vehicle_msgs::msg::VelocityReport &msg) {
                this->aw_velocity_report_handler(msg);
            });
    }

    if (aw_steering_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<autoware_auto_vehicle_msgs::msg::SteeringReport>(
            aw_steering_report_topic,
            [this](const autoware_auto_vehicle_msgs::msg::SteeringReport &msg) {
                this->aw_steering_report_handler(msg);
            });
    }

    if (aw_gear_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<autoware_auto_vehicle_msgs::msg::GearReport>(
            aw_gear_report_topic,
            [this](const autoware_auto_vehicle_msgs::msg::GearReport &msg) {
                this->aw_gear_report_handler(msg);
            });
    }

    if (aw_hazard_light_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
            aw_hazard_light_report_topic,
            [this](const autoware_auto_vehicle_msgs::msg::HazardLightsReport &msg) {
                this->aw_hazard_light_report_handler(msg);
            });
    }

    if (aw_turn_indicator_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
            aw_turn_indicator_report_topic,
            [this](const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport &msg) {
                this->aw_turn_indicator_report_handler(msg);
            });
    }

    if (tum_head_light_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_autoware_vehicle_msgs::msg::HeadLightReport>(
            tum_head_light_report_topic,
            [this](const tum_autoware_vehicle_msgs::msg::HeadLightReport &msg) {
                this->tum_head_light_report_handler(msg);
            });
    }

    if (tum_high_beam_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_autoware_vehicle_msgs::msg::HighBeamReport>(
            tum_high_beam_report_topic,
            [this](const tum_autoware_vehicle_msgs::msg::HighBeamReport &msg) {
                this->tum_high_beam_report_handler(msg);
            });
    }

    if (tum_honk_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_autoware_vehicle_msgs::msg::HonkReport>(
            tum_honk_report_topic,
            [this](const tum_autoware_vehicle_msgs::msg::HonkReport &msg) {
                this->tum_honk_report_handler(msg);
            });
    }

    if (tum_wiper_report_topic != "none") {
        this->generic_actuation_interface_->add_subscriber<tum_autoware_vehicle_msgs::msg::WiperReport>(
            tum_wiper_report_topic,
            [this](const tum_autoware_vehicle_msgs::msg::WiperReport &msg) {
                this->tum_wiper_report_handler(msg);
            });
    }

    // Set up publishers
    this->generic_actuation_interface_->add_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>(
        aw_gear_command_topic,
        [this]() {
            return this->gear_msg_builder();
        },
        {"SecondaryCtrl_Gear"},
        20);

    this->generic_actuation_interface_->add_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        aw_ackermann_command_topic,
        [this]() {
            return this->ackermann_msg_builder();
        },
        {"PrimaryCtrl_Velocity", "PrimaryCtrl_Acceleration", "PrimaryVehicleData_Velocity",
          "PrimaryCtrl_SteeringWheelAngle", "SecondaryCtrl_Gear"},
        20);

    this->generic_actuation_interface_->add_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
        aw_hazard_light_command_topic,
        [this]() {
            return this->hazard_light_msg_builder();
        },
        {"SecondaryCtrl_Indicator"},
        50);
    
    this->generic_actuation_interface_->add_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
        aw_indicator_command_topic,
        [this]() {
            return this->indicator_msg_builder();
        },
        {"SecondaryCtrl_Indicator"},
        50);

    RCLCPP_INFO(this->get_logger(), "ActuationInterface subscribers and publishers initialized");
}

void ActuationInterface::safety_driver_status_from_can(const tum_edgar_can_msgs::msg::TUMEdgarGatewayStates &msg)
{
    this->generic_actuation_interface_->update_attribute("SafetyDriverStatus_Long", static_cast<int8_t>(msg.ai_state == GatewayStates::Active));
    this->generic_actuation_interface_->update_attribute("SafetyDriverStatus_Lat", static_cast<int8_t>(msg.si_state == GatewayStates::Active));
    this->generic_actuation_interface_->update_attribute("SafetyDriverStatus_EmergencyStopReleased", 
        static_cast<int8_t>((msg.ai_state == GatewayStates::Active) && (msg.si_state == GatewayStates::Active))
        );
}

void ActuationInterface::powertrain_handler(const tum_edgar_can_msgs::msg::TUMEdgarPowertrain1 &msg)
{
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Gear", static_cast<int8_t>(
        this->edgar_to_tod_gear_map_.find(static_cast<uint8_t>(msg.gear))->second
    ));
}

void ActuationInterface::misc_handler(const tum_edgar_can_msgs::msg::TUMEdgarMisc1 &msg)
{
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Indicator", static_cast<int8_t>(
        (!msg.turn_indicators_left ? (!msg.turn_indicators_right ? eIndicator::INDICATOR_OFF : eIndicator::INDICATOR_RIGHT) : eIndicator::INDICATOR_LEFT)
    ));
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Honk", static_cast<int8_t>(msg.horn));
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Wiper", static_cast<int8_t>(
        (msg.wipers ? eWiper::WIPER_ON : eWiper::WIPER_OFF)
    ));
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_HeadLight", static_cast<int8_t>(
        (msg.dimmed_headlights ? eHeadLight::HEADLIGHT_ON : eHeadLight::HEADLIGHT_OFF)
    ));
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_FlashLight", static_cast<int8_t>(
        (msg.high_beam ? eFlashLight::FLASHLIGHT_ON : eFlashLight::FLASHLIGHT_OFF)
    ));
}

void ActuationInterface::motion_handler(const tum_edgar_can_msgs::msg::TUMEdgarMotion &msg)
{
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_Velocity", static_cast<float>(msg.vehicle_velocity));
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_Acceleration", static_cast<float>(msg.longitudinal_acceleration));
}

void ActuationInterface::steering_handler(const tum_edgar_can_msgs::msg::TUMEdgarSteering &msg)
{    
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_SteeringWheelAngle", static_cast<float>(msg.steering_wheel_angle));
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_SteeringTireAngle", static_cast<float>(msg.steering_wheel_angle * this->wheel2tire_factor_));
}

autoware_auto_vehicle_msgs::msg::GearCommand ActuationInterface::gear_msg_builder()
{
    // no check if gear changed necessary, because the builder will only be executed if the gear input from secondary ctrl command has changed (timestamp logic inside BaseInterface)
    autoware_auto_vehicle_msgs::msg::GearCommand gear_output = autoware_auto_vehicle_msgs::msg::GearCommand();
    gear_output.stamp = this->now();
    gear_output.command = this->tod_to_aw_gear_map_.find(this->generic_actuation_interface_->get_attribute<int8_t>("SecondaryCtrl_Gear"))->second;
    return gear_output;
}

autoware_auto_control_msgs::msg::AckermannControlCommand ActuationInterface::ackermann_msg_builder()
{
    autoware_auto_control_msgs::msg::AckermannControlCommand ackermann_output = autoware_auto_control_msgs::msg::AckermannControlCommand();
    //grep required attributes for later computation (to avoid multiple attribute requests)
    float target_velocity = this->generic_actuation_interface_->get_attribute<float>("PrimaryCtrl_Velocity");  // m/s
    float target_acceleration = this->generic_actuation_interface_->get_attribute<float>("PrimaryCtrl_Acceleration"); // m/s^2
    float current_velocity = this->generic_actuation_interface_->get_attribute<float>("PrimaryVehicleData_Velocity"); // m/s
    float target_steering_wheel_angle = this->generic_actuation_interface_->get_attribute<float>("PrimaryCtrl_SteeringWheelAngle"); // rad
    bool reverse = this->generic_actuation_interface_->get_attribute<int8_t>("SecondaryCtrl_Gear") == eGearPosition::GEARPOSITION_REVERSE;

    // set output values to message input
    ackermann_output.stamp = this->now();
    ackermann_output.longitudinal.acceleration = target_acceleration; // m/s^2
    ackermann_output.longitudinal.speed = target_velocity;            // m/s
    // =============== Longitudinal commands ===============
    // The EDGAR actuator interface only accepts acceleration commands in m/s^2
    // see https://gitlab.lrz.de/av2.0/edgar/-/tree/develop/edgar_actuator_interface?ref_type=heads
    // Since we set desired velocities with most input modalities (virtual input device, velocity knob on steering wheel, etc.)
    // we need to convert the desired velocity to an acceleration command through usage of a PI controller.
    rclcpp::Time now = this->now(); // current time

    // Only compute a acceleration if this is not the first time a primary control command is recieved (we would not be able to
    // compute a dt needed for the I part) AND if the difference to recieving the last topic is smaller than 0.5 sec (if we do
    // not recieve primary control commands for a longer time period, e.g. because the operator has disconnected and connected
    // again, this would result in a large dt, therefore in a large I part and finally in a large acceleration)
    if (last_ackermann_command_time_.nanoseconds() != 0 && (now - last_ackermann_command_time_).seconds() < 0.5)
    {
        double dt = (now - last_ackermann_command_time_).seconds(); // time difference between now and last command
        // RCLCPP_INFO(this->get_logger(), "dt = %f", dt);
        // RCLCPP_INFO(this->get_logger(), "desired velocity = %f", target_velocity);
        // RCLCPP_INFO(this->get_logger(), "current velocity = %f", current_velocity);
        double error = target_velocity - current_velocity; // m/s
        // RCLCPP_INFO(this->get_logger(), "error = %f", error);
        integral_ += error * dt;
        double acceleration = kp_ * error + ki_ * integral_; // m/s^2
        // RCLCPP_INFO(this->get_logger(), "acceleration 1 = %f", acceleration);
        acceleration = std::clamp(acceleration, -5.0, 3.0); // limit acceleration to [- 5; 3] m/s^2
        // RCLCPP_INFO(this->get_logger(), "acceleration 2 = %f", acceleration);
        ackermann_output.longitudinal.acceleration = acceleration; // m/s^2
    }
    last_ackermann_command_time_ = now; // update last command time

    // =============== Lateral commands ===============
    // Linear steering wheel to tire wheel transmission
    // tire angle range: [-0.610865, 0.610865] from edgar digital twin
    // steering wheel range: [-9.42, 9.42] common value -> 1.5x left/right rotation
    if (std::abs(target_steering_wheel_angle) < this->max_steer_wheel_)
    {
        ackermann_output.lateral.steering_tire_angle = this->wheel2tire_factor_ * target_steering_wheel_angle; // rad
    }
    else
    {
        ackermann_output.lateral.steering_tire_angle = this->wheel2tire_factor_ * (target_steering_wheel_angle / std::abs(target_steering_wheel_angle)) * this->max_steer_wheel_;
    };

    // Check if reversing
    if (reverse)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "%s: In gear reverse, inverting longitudinal ctrl cmds", this->get_name());
        ackermann_output.longitudinal.acceleration *= -1.0;
        ackermann_output.longitudinal.speed *= -1.0;
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "%s: Converted first PrimaryControlCmd to AutowareControlCommand", this->get_name());

    return ackermann_output;
}

void ActuationInterface::aw_velocity_report_handler(const autoware_auto_vehicle_msgs::msg::VelocityReport &msg)
{
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_Velocity", static_cast<float>(msg.longitudinal_velocity));
}

void ActuationInterface::aw_steering_report_handler(const autoware_auto_vehicle_msgs::msg::SteeringReport &msg)
{
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_SteeringTireAngle", static_cast<float>(msg.steering_tire_angle));
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_SteeringWheelAngle", 
        static_cast<float>(msg.steering_tire_angle / this->wheel2tire_factor_));
}

void ActuationInterface::aw_gear_report_handler(const autoware_auto_vehicle_msgs::msg::GearReport &msg)
{
    auto gear_iter = aw_to_tod_gear_map_.find(msg.report);
    int8_t gear_position = (gear_iter != aw_to_tod_gear_map_.end()) ? gear_iter->second : eGearPosition::GEARPOSITION_NEUTRAL;
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Gear", static_cast<int8_t>(gear_position));
}

void ActuationInterface::aw_hazard_light_report_handler(const autoware_auto_vehicle_msgs::msg::HazardLightsReport &msg)
{
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Indicator", 
        static_cast<int8_t>(
            (msg.report == autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE) 
                ? eIndicator::INDICATOR_BOTH 
                : eIndicator::INDICATOR_OFF
        ));
    }

void ActuationInterface::aw_turn_indicator_report_handler(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport &msg)
{
    int8_t indicator;
    int8_t current_indicator = this->generic_actuation_interface_->get_attribute<int8_t>("SecondaryVehicleData_Indicator");
    if (current_indicator == eIndicator::INDICATOR_BOTH)
    {
        indicator = eIndicator::INDICATOR_BOTH;
    }
    else
    {
        switch (msg.report)
        {
        case autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT:
            indicator = eIndicator::INDICATOR_LEFT;
            break;
        case autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT:
            indicator = eIndicator::INDICATOR_RIGHT;
            break;
        default:
            indicator = eIndicator::INDICATOR_OFF;
            break;
        }
    }
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Indicator", static_cast<int8_t>(indicator));
}

void ActuationInterface::tum_head_light_report_handler(const tum_autoware_vehicle_msgs::msg::HeadLightReport &msg)
{
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_HeadLight", static_cast<int8_t>(
        (msg.report == tum_autoware_vehicle_msgs::msg::HeadLightReport::HEAD_LIGHT_AUTO || 
         msg.report == tum_autoware_vehicle_msgs::msg::HeadLightReport::HEAD_LIGHT_LOW_BEAM)
            ? eHeadLight::HEADLIGHT_ON 
            : eHeadLight::HEADLIGHT_OFF));
}

void ActuationInterface::tum_high_beam_report_handler(const tum_autoware_vehicle_msgs::msg::HighBeamReport &msg)
{
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_FlashLight", 
        static_cast<int8_t>(
            (msg.report == tum_autoware_vehicle_msgs::msg::HighBeamReport::HIGH_BEAM_ENABLE || 
            msg.report == tum_autoware_vehicle_msgs::msg::HighBeamReport::HIGH_BEAM_FLASH)
            ? eFlashLight::FLASHLIGHT_ON 
            : eFlashLight::FLASHLIGHT_OFF
        ));
}

void ActuationInterface::tum_honk_report_handler(const tum_autoware_vehicle_msgs::msg::HonkReport &msg)
{
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Honk", 
        static_cast<int8_t>(
            (msg.report == tum_autoware_vehicle_msgs::msg::HonkReport::HONK_ENABLE) 
            ? eHonk::HONK_ON 
            : eHonk::HONK_OFF
        ));
}

void ActuationInterface::tum_wiper_report_handler(const tum_autoware_vehicle_msgs::msg::WiperReport &msg)
{
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Wiper", 
        static_cast<int8_t>(
            (msg.report_front == tum_autoware_vehicle_msgs::msg::WiperReport::WIPER_DISABLE) 
                ? eWiper::WIPER_OFF 
                : eWiper::WIPER_ON));
}


autoware_auto_vehicle_msgs::msg::HazardLightsCommand ActuationInterface::hazard_light_msg_builder()
{
    autoware_auto_vehicle_msgs::msg::HazardLightsCommand hazard_light_msg;
    hazard_light_msg.stamp = this->now();
    if(this->generic_actuation_interface_->get_attribute<int8_t>("SecondaryCtrl_Indicator")
             == eIndicator::INDICATOR_BOTH)
    {
        hazard_light_msg.command = autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
    }
    else
    {
        hazard_light_msg.command = autoware_auto_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND;
    }
    return hazard_light_msg;
}

autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand ActuationInterface::indicator_msg_builder()
{
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand indicator_msg;
    indicator_msg.stamp = this->now();

    int8_t tod_indicator = this->generic_actuation_interface_->get_attribute<int8_t>("SecondaryCtrl_Indicator");
    auto turn_iter = tod_to_aw_indicator_.find(tod_indicator);    
    if (turn_iter != tod_to_aw_indicator_.end())
    {
        indicator_msg.command = turn_iter->second;
    }
    else {
        indicator_msg.command = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;
    }
    return indicator_msg;
}

} // namespace tod_edgar_interface
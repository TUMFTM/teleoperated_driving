// Copyright TUM-FTM
#include "tod_command_creation/command_creator.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

tod_command_creation::CommandCreator::CommandCreator() : Node("CommandCreator"), _count(0)
{
    _joystickSubs = this->create_subscription<sensor_msgs::msg::Joy>(
        "input/joystick", 
        1,
        std::bind(&CommandCreator::callback_joystick_msg, this, _1));
    
    _statusSubs = this->create_subscription<tod_status_msgs::msg::Status>(
        "input/operator_status", 
        1,
        std::bind(&CommandCreator::callback_status_msg, this, _1));

    _primaryControlPub = this->create_publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>("output/primary_control_cmd", 1);
    _secondaryControlPub = this->create_publisher<tod_vehicle_msgs::msg::SecondaryControlCmd>("output/secondary_control_cmd", 1);

    _timer = this->create_wall_timer(100ms, std::bind(&CommandCreator::timer_callback, this));

    this->declare_parameter<std::string>("config_path", "");
    std::string config_path;
    
    if(! this->get_parameter("config_path", config_path)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to retrieve 'config_path' parameter. Ensure it is set in the launch file.");
    }
 
    vehicleParamHandler_ =
        std::make_unique<tod_core::param_set::Vehicle>(this, config_path + "/vehicle_config/");

    _param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto cb_separatBrakingAxis = [this](const rclcpp::Parameter & p){ 
        this->_inputDeviceHasSeparateBrakingAxis = p.as_bool(); 
        RCLCPP_INFO(this->get_logger(), "New Input Device: Updated parameter inputDeviceHasSeparateBrakingAxis");
    }; 
    
    _cb_handle = _param_subscriber->add_parameter_callback( "input_device_has_separate_braking_axis", 
        std::bind(&CommandCreator::cb_change_param, this, std::placeholders::_1), "/operator/input_devices/InputDevice");

    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INCREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DECREASE_SPEED, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_LEFT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INDICATOR_RIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::FLASHLIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::FRONTLIGHT, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::HONK, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::INCREASE_GEAR, 0));
    _prevButtonState.insert(std::pair<joystick::ButtonPos, int>(joystick::ButtonPos::DECREASE_GEAR, 0));
    
    this->declare_parameter<bool>("ConstraintSteeringRate", false); 
    this->declare_parameter<bool>("InvertSteeringInGearReverse", true); 
    this->declare_parameter<float>("maxVelocity", 10.0f);
    this->declare_parameter<float>("maxAcceleration", 4.0f);
    this->declare_parameter<float>("maxDeceleration", 9.0f);
    this->declare_parameter<double>("maxSteeringWheelAngleRate", 7.5);
    const int minGear = this->declare_parameter<int>("minGearPosition", eGearPosition::GEARPOSITION_PARK);
    const int maxGear = this->declare_parameter<int>("maxGearPosition", eGearPosition::GEARPOSITION_SPORT);
    const int defaultGear = this->declare_parameter<int>("defaultGearPosition", eGearPosition::GEARPOSITION_PARK);

    try {
        _gearSelector = std::make_unique<GearSelector>(minGear, maxGear, defaultGear);
    } catch (const std::invalid_argument &error) {
        RCLCPP_FATAL(this->get_logger(), "Invalid gear configuration: %s", error.what());
        throw;
    }


     if (!this->get_parameter("ConstraintSteeringRate", _constraintSteeringRate))
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not get param /ConstraintSteeringRate - using "
                                                   << (_constraintSteeringRate ? "true" : "false"));

    if (!this->get_parameter("InvertSteeringInGearReverse", _invertSteeringInGearReverse))
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not get param /InvertSteeringInGearReverse - using "
                                                   << (_invertSteeringInGearReverse ? "true" : "false"));

    if (!this->get_parameter("maxVelocity", _maxSpeedms))
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not get param /maxVelocity - using "
                                                   << _maxSpeedms << " m/s");

    if (!this->get_parameter("maxAcceleration", _maxAcceleration))
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not get param /maxAcceleration - using "
                                                   << _maxAcceleration << " m/s^2");

    if (!this->get_parameter("maxDeceleration", _maxDeceleration))
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not get param /maxDeceleration - using "
                                                   << _maxDeceleration << " m/s^2");

    if (!this->get_parameter("maxSteeringWheelAngleRate", _maxSteeringWheelAngleRate))
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not get param /maxSteeringWheelAngleRate - using "
                                                   << _maxSteeringWheelAngleRate << " rad/s");                                        

    if (!this->get_parameter("vehicleID", _vehicleID))
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << ": Could not set param /vehicleID - using "
                                                   << _vehicleID);

    init_control_messages();
}

void tod_command_creation::CommandCreator::timer_callback() 
{
    if (_joystickInputSet && _status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
        _primaryControlMsg.header.stamp = this->get_clock()->now();
        _secondaryControlMsg.header.stamp = this->get_clock()->now();
        _primaryControlPub->publish(_primaryControlMsg);
        _secondaryControlPub->publish(_secondaryControlMsg);
    }
    _joystickInputSet = false;
}

void tod_command_creation::CommandCreator::callback_joystick_msg(const sensor_msgs::msg::Joy &msg) 
{
    
    if (_status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
        calculate_steering_wheel_angle(_primaryControlMsg, msg.axes);
        calculate_desired_velocity(_primaryControlMsg, msg, _secondaryControlMsg.gear_position);
        set_gear(_secondaryControlMsg, msg.buttons, _primaryControlMsg.velocity);
        set_indicator(_secondaryControlMsg, msg.buttons);
        set_light(_secondaryControlMsg, msg.buttons);
        set_honk(_secondaryControlMsg, msg.buttons);
        _joystickInputSet = true;
    }
}

void tod_command_creation::CommandCreator::callback_status_msg(const tod_status_msgs::msg::Status &msg) 
{
    if (_status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION
        && msg.tod_status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
        init_control_messages();
    }

    _status = msg.tod_status;
}

void tod_command_creation::CommandCreator::calculate_steering_wheel_angle(tod_vehicle_msgs::msg::PrimaryControlCmd &out,
        const std::vector<float>& axes) 
{
    
    static rclcpp::Time tPrev;
    static double oldSetSWA{0.0};
    // calc desired SWA
    double newDesiredSWA = axes.at(joystick::AxesPos::STEERING) * vehicleParamHandler_->get_max_swa_rad();

    if (_constraintSteeringRate) { // constraint steering rate
        static rclcpp::Duration dur = this->get_clock()->now() - tPrev;
        double newSetSWA = std::min(newDesiredSWA, oldSetSWA + dur.seconds() * _maxSteeringWheelAngleRate);
        newSetSWA = std::max(newSetSWA, oldSetSWA - dur.seconds() * _maxSteeringWheelAngleRate);
        oldSetSWA = newSetSWA;
        out.steering_wheel_angle = newSetSWA;
        tPrev = this->get_clock()->now();
    } else { // output unconstraint SWA
        out.steering_wheel_angle = newDesiredSWA;
    }

    if (_invertSteeringInGearReverse &&
        _secondaryControlMsg.gear_position ==  eGearPosition::GEARPOSITION_REVERSE)
        out.steering_wheel_angle = -out.steering_wheel_angle;
}

void tod_command_creation::CommandCreator::calculate_desired_velocity(tod_vehicle_msgs::msg::PrimaryControlCmd &out,
        const sensor_msgs::msg::Joy &msg, const int gear) 
{
    if (gear == eGearPosition::GEARPOSITION_PARK || gear == eGearPosition::GEARPOSITION_NEUTRAL) {
        out.velocity = 0.0;
        return;
    }

    static rclcpp::Time prevTime = this->get_clock()->now();
    float a_soll = 0;
    float changeOperator;

    // evaluate Speed Change by operator
    if (_inputDeviceHasSeparateBrakingAxis) {
        changeOperator = (msg.axes.at(joystick::AxesPos::THROTTLE) - msg.axes.at(joystick::AxesPos::BRAKE)) / 2.0;
    } else {
        changeOperator = msg.axes.at(joystick::AxesPos::THROTTLE);
    }

    //Calculate Acceleration demand by operator
    static float deadzoneThrottle{0.05}, deadzoneBrake{0.05};
    if (changeOperator >= 0) {
        a_soll = _maxAcceleration * (std::max(changeOperator, (float) deadzoneThrottle) - deadzoneThrottle); //acc
    } else {
        a_soll = _maxDeceleration * (std::min(changeOperator, (float) -deadzoneBrake) + deadzoneBrake); // decelerate
    }

    //Integrate Speed
    rclcpp::Duration dt = this->get_clock()->now() - prevTime;
    prevTime = this->get_clock()->now();
    out.velocity = out.velocity + dt.seconds() * a_soll;

    // Handle Speed Button Increase/Decrease
    if (msg.buttons.at(joystick::ButtonPos::INCREASE_SPEED) == 1
        && _prevButtonState.at(joystick::ButtonPos::INCREASE_SPEED) == 0) {
        out.velocity += 1.0 / 3.6; // kmh increments
    }
    if (msg.buttons.at(joystick::ButtonPos::DECREASE_SPEED) == 1
        && _prevButtonState.at(joystick::ButtonPos::DECREASE_SPEED) == 0) {
        out.velocity -= 1.0 / 3.6; // kmh increments
    }
    _prevButtonState.at(joystick::ButtonPos::INCREASE_SPEED) = msg.buttons.at(joystick::ButtonPos::INCREASE_SPEED);
    _prevButtonState.at(joystick::ButtonPos::DECREASE_SPEED) = msg.buttons.at(joystick::ButtonPos::DECREASE_SPEED);

    // Saturate Speed integration and limit Acceleration
    if (out.velocity > _maxSpeedms) {
        out.velocity = _maxSpeedms; //Limit demanded speed
    } else if (out.velocity < 0) {
        out.velocity = 0; //Limit demanded speed to zero
    }
    out.acceleration = a_soll;
}

void tod_command_creation::CommandCreator::set_gear(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState,
        const float &currentVelocity) 
{
    const bool increaseEdge =
        buttonState.at(joystick::ButtonPos::INCREASE_GEAR) == 1 &&
        _prevButtonState.at(joystick::ButtonPos::INCREASE_GEAR) == 0;
    const bool decreaseEdge =
        buttonState.at(joystick::ButtonPos::DECREASE_GEAR) == 1 &&
        _prevButtonState.at(joystick::ButtonPos::DECREASE_GEAR) == 0;

    out.gear_position = _gearSelector->select(
        out.gear_position, increaseEdge, decreaseEdge, currentVelocity);

    _prevButtonState.at(joystick::ButtonPos::INCREASE_GEAR) = buttonState.at(joystick::ButtonPos::INCREASE_GEAR);
    _prevButtonState.at(joystick::ButtonPos::DECREASE_GEAR) = buttonState.at(joystick::ButtonPos::DECREASE_GEAR);
}

void tod_command_creation::CommandCreator::set_indicator(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState) 
{
    if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 1
        && buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 0) {
        out.indicator = 1; // Indicator Left

    } else if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 0
               && buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 1) {
        out.indicator = 2; // Indicator Right

    } else if (buttonState.at(joystick::ButtonPos::INDICATOR_LEFT) == 1
               && buttonState.at(joystick::ButtonPos::INDICATOR_RIGHT) == 1) {
        out.indicator = 3; // Indicator Both

    } else {
        out.indicator = 0; // Indicator Off
    }
}

void tod_command_creation::CommandCreator::set_light(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState) 
{
    // flashLight
    out.flash_light = buttonState.at(joystick::ButtonPos::FLASHLIGHT);

    // headLight
    if (buttonState.at(joystick::ButtonPos::FRONTLIGHT) == 1
        && _prevButtonState.at(joystick::ButtonPos::FRONTLIGHT) == 0) {
        if (out.head_light == 0) {
            out.head_light = 1;
        } else {
            out.head_light = 0;
        }
    }
    _prevButtonState.at(joystick::ButtonPos::FRONTLIGHT) = buttonState.at(joystick::ButtonPos::FRONTLIGHT);
}

void tod_command_creation::CommandCreator::set_honk(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState) 
{
    out.honk = (bool) buttonState.at(joystick::ButtonPos::HONK);
}

void tod_command_creation::CommandCreator::init_control_messages() 
{
    _primaryControlMsg.acceleration = 0;
    _primaryControlMsg.steering_wheel_angle = 0;
    _primaryControlMsg.velocity = 0;
    _secondaryControlMsg.gear_position = _gearSelector->default_gear();
    _secondaryControlMsg.head_light = 0;
    _secondaryControlMsg.honk = 0;
    _secondaryControlMsg.indicator = 0;
    _secondaryControlMsg.wiper = 0;
}

void tod_command_creation::CommandCreator::cb_change_param(const rclcpp::Parameter & p){
    try{
        this->_inputDeviceHasSeparateBrakingAxis = p.as_bool();
    }
    catch(...){
        this->_inputDeviceHasSeparateBrakingAxis = true;
    }
}

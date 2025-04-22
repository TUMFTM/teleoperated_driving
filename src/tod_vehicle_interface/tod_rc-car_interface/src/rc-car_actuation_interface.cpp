/**
 * @file rc-car_actuation_interface.cpp
 * @brief RC-Car actuation interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_rc-car_interface
 */

#include "tod_rc-car_interface/rc-car_actuation_interface.hpp"

namespace tod_rccar_interface {

ActuationInterface::ActuationInterface()
    : rclcpp::Node("actuation_interface_node")
{
    // Initialize parameters
    this->declare_parameter<double>("servo_min", 0.15);
    this->declare_parameter<double>("servo_zero", 0.4325);
    this->declare_parameter<double>("servo_max", 0.7385);
    this->declare_parameter<std::string>("vehicleID", "rc-car");

    // topic parameters
    this->declare_parameter<std::string>("steering_wheel_topic", "/vesc/commands/servo/position");
    this->declare_parameter<std::string>("engine_speed_topic", "/vesc/commands/motor/speed");
    this->declare_parameter<std::string>("acceleration_topic", "/hedge_imu");
    std::string default_path = ament_index_cpp::get_package_share_directory("tod_rc-car_interface") + "/config/";
    this->declare_parameter<std::string>("parameter_folder", default_path);
    this->declare_parameter<std::string>("ackermann_command_topic", "/vesc/low_level/ackermann_cmd_mux/output");

    // Get parameters
    servo_min_ = this->get_parameter("servo_min").as_double();
    servo_max_ = this->get_parameter("servo_max").as_double();
    servo_zero_ = this->get_parameter("servo_zero").as_double();
    this->get_parameter("vehicleID").as_string();
}

void ActuationInterface::run()
{
    this->generic_actuation_interface_ = std::make_shared<tod_generic_interface::ActuationInterface>(
        std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this())
    );


    std::string parameter_folder = this->get_parameter("parameter_folder").as_string();
    vehicle_params_ = std::make_unique<tod_core::param_set::Vehicle>(this, parameter_folder);
    vehicle_params_->load_parameters();

    std::string steering_wheel_topic = this->get_parameter("steering_wheel_topic").as_string();
    std::string engine_speed_topic = this->get_parameter("engine_speed_topic").as_string();
    std::string acceleration_topic = this->get_parameter("acceleration_topic").as_string();
    std::string ackermann_command_topic = this->get_parameter("ackermann_command_topic").as_string();

    this->generic_actuation_interface_->add_subscriber<std_msgs::msg::Float64>(
            steering_wheel_topic,
            [this](const std_msgs::msg::Float64 &msg) {
                this->steering_wheel_handler(msg);
            });

    this->generic_actuation_interface_->add_subscriber<std_msgs::msg::Float64>(
            engine_speed_topic,
            [this](const std_msgs::msg::Float64 &msg) {
                this->engine_speed_handler(msg);
            });

    this->generic_actuation_interface_->add_subscriber<sensor_msgs::msg::Imu>(
            acceleration_topic,
            [this](const sensor_msgs::msg::Imu &msg) {
                this->acceleration_handler(msg);
            });
    
    this->generic_actuation_interface_->add_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        ackermann_command_topic,
        [this]() {
            return this->ackermann_msg_builder();
        },
        {"PrimaryCtrl_Acceleration", "PrimaryCtrl_Velocity", "PrimaryCtrl_Velocity", 
             "PrimaryCtrl_SteeringWheelAngle", "SecondaryCtrl_Gear"},
        50);
}

void ActuationInterface::steering_wheel_handler(const std_msgs::msg::Float64 &msg)
{
    float servo_zeroed = float(msg.data) - servo_zero_;
    float servo_scaled = (servo_zeroed < 0.0f) ?
            servo_zeroed / (servo_zero_ - servo_min_) : 
            servo_zeroed / (servo_max_ - servo_min_);
    float steering_wheel_angle = -servo_scaled * vehicle_params_->get_max_rwa_rad();
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_SteeringWheelAngle", static_cast<float>(steering_wheel_angle));
    RCLCPP_INFO_ONCE(this->get_logger(),"%s: Received first steering wheel angle.", this->get_name());
}

void ActuationInterface::engine_speed_handler(const std_msgs::msg::Float64& msg)
{
    this->generic_actuation_interface_->update_attribute("PrimaryVehicleData_Velocity", static_cast<float>(msg.data));
    int8_t gear = (msg.data <= -0.1) ? eGearPosition::GEARPOSITION_REVERSE : eGearPosition::GEARPOSITION_DRIVE;
    this->generic_actuation_interface_->update_attribute("SecondaryVehicleData_Gear", static_cast<int8_t>(gear));
}

void ActuationInterface::acceleration_handler(const sensor_msgs::msg::Imu &msg)
{
    this->generic_actuation_interface_->update_attribute("PrimaryCtrl_Acceleration", static_cast<float>(msg.linear_acceleration.x));
}

ackermann_msgs::msg::AckermannDriveStamped ActuationInterface::ackermann_msg_builder()
{
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
    ackermann_msg.header.stamp = this->now();
    ackermann_msg.drive.acceleration = this->generic_actuation_interface_->get_attribute<float>("PrimaryCtrl_Acceleration");
    ackermann_msg.drive.speed = this->generic_actuation_interface_->get_attribute<float>("PrimaryCtrl_Velocity");
    ackermann_msg.drive.steering_angle = this->generic_actuation_interface_->get_attribute<float>("PrimaryCtrl_SteeringWheelAngle");
    bool reverse = this->generic_actuation_interface_->get_attribute<int8_t>("SecondaryCtrl_Gear") == eGearPosition::GEARPOSITION_REVERSE;
    if (reverse) {
        ackermann_msg.drive.acceleration *= -1.0;
        ackermann_msg.drive.speed *= -1.0;
    }
    RCLCPP_INFO_ONCE(this->get_logger(),"%s: Converted first PrimaryControlCmd to AckermannDrive", this->get_name());
    RCLCPP_INFO(this->get_logger(),"%s: Converted first PrimaryControlCmd to AckermannDrive %f, %f", this->get_name(), ackermann_msg.drive.speed, ackermann_msg.drive.steering_angle);
    
    return ackermann_msg;
}

} // namespace tod_rccar_interface
#include "tod_dummy/status_dummy_pub.hpp"
using namespace tod_dummy;

DummyPub::DummyPub() : Node("StatusDummyPub"), _count(0)
{
  _publisher_operator = this->create_publisher<tod_status_msgs::msg::Status>("output/operator_status", 1);
  _publisher_vehicle = this->create_publisher<tod_status_msgs::msg::Status>("output/vehicle_status", 1);

  _timer = this->create_wall_timer(
    50ms, std::bind(&DummyPub::timer_callback, this));
}

void DummyPub::timer_callback()
{
  // Create a status message and populate it with sample data
  tod_status_msgs::msg::Status _msg;
  _msg.set__tod_status(tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION);
  _msg.set__vehicle_control_mode(tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT);
  _msg.set__vehicle_ip_address("10.183.93.85");
  _msg.set__operator_ip_address("10.183.93.85");

  _publisher_operator->publish(_msg);
  _publisher_vehicle->publish(_msg);

  // Optionally log the message
  // RCLCPP_INFO(this->get_logger(), "Publishing TOD_Status: %i, Control_Mode: %i", _msg.tod_status, _msg.vehicle_control_mode);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPub>());
  rclcpp::shutdown();
  return 0;
}

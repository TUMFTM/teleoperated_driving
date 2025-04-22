#include "tod_dummy/joystick_dummy_pub.hpp"
using namespace tod_dummy;


JoystickDummyPub::JoystickDummyPub()
: Node("joystick_dummy_pub")
{
  publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("output/joystick", 1);
  
  // Timer to call the timer_callback function every 500ms
  timer_ = this->create_wall_timer(
    500ms, std::bind(&JoystickDummyPub::timer_callback, this));
}

void JoystickDummyPub::timer_callback()
{
  sensor_msgs::msg::Joy _msg;
  std_msgs::msg::Header _header;
  
  // Joystick axes: Steering, Throttle, Brake
  std::vector<float> _axis = {0.2, 0.5, 0.0, 0.0}; 
  
  // Joystick buttons: INDICATOR_LEFT, INDICATOR_RIGHT, FLASHLIGHT, FRONTLIGHT, HONK,
  // INCREASE_SPEED, DECREASE_SPEED, INCREASE_GEAR, DECREASE_GEAR
  std::vector<int> _buttons = {0, 0, 0, 0, 0, 1, 0, 1, 0};

  // Set timestamp in header
  _header.set__stamp(this->get_clock()->now());
  _msg.set__header(_header);

 
  _msg.set__axes(_axis);
  _msg.set__buttons(_buttons);
  
  // Optionally log the message
  // RCLCPP_INFO(this->get_logger(), "Publishing: [Steering: %f, Throttle: %f, Brake: %f]", 
  //          _msg.axes.at(0), _msg.axes.at(1), _msg.axes.at(2));
  
  publisher_->publish(std::move(_msg));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickDummyPub>());
  rclcpp::shutdown();
  return 0;
}
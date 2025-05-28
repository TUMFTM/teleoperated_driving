/**
 * @file input_device_node.cpp
 * @brief ROS Abstraction for usb input devices
 * @copyright 2025 TUMFTM
 **/

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QApplication>
#include "tod_input_devices/input_device_controller.hpp"

int main(int argc, char **argv) {
  QApplication a(argc, argv);
  rclcpp::init(argc, argv);
  tod_input_device::InputDeviceController device(argc, argv);
  a.setQuitOnLastWindowClosed(false);
  return a.exec();
}

/**
 * @file usb_event_handler.cpp
 * @brief Converts USB event into Joy Messages
 * @copyright 2024 TUMFTM
 **/
 #include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "tod_input_devices/usb_event_device/usb_event_device.hpp"

using namespace std::placeholders;

class UsbEventHandler : public rclcpp::Node {
public:
    explicit UsbEventHandler()
        : Node("UsbEventDevice"), _joystick{"/dev/input/event"}{
        _sub =  this->create_subscription<std_msgs::msg::Float64>(
      "/Operator/CommandCreation/force_feedback", 5, std::bind(&UsbEventHandler::callback_force_feedback, this, _1));
    }
    ~UsbEventHandler() {}
    void run() {
        rclcpp::Rate r(20);
        while (rclcpp::ok() && _joystick.ok()) {
            r.sleep();
            rclcpp::spin_some(get_node_base_interface());
            static rclcpp::Time resetTime{rclcpp::Node::now()};
            if (rclcpp::Node::now() >= resetTime + rclcpp::Duration::from_seconds(25.0)) {
                if (_joystick.ok()) _joystick.reset();
                resetTime = rclcpp::Node::now();
            }
        }
    }

private:
    UsbEventDevice _joystick;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub;

    void callback_force_feedback(const std_msgs::msg::Float64::ConstSharedPtr &msg) {
        if (_joystick.ok()) {
            _joystick.set_force_feedback(msg->data);
        } else {
            // ROS_WARN_ONCE("%s: joystick not ok - not setting force feedback",
            //               rclcpp::Node::get_name().c_str());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    UsbEventHandler node;
    node.run();
    return 0;
}

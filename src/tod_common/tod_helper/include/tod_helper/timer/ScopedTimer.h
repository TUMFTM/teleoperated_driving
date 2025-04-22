#pragma once
#include "rclcpp/rclcpp.hpp"

namespace tod_helper::Timer {

class ScopedTimer {
public:
    explicit ScopedTimer(const std::string &name) : _name{name}, _start{rclcpp::Clock().now()} {}
    ~ScopedTimer() { RCLCPP_ERROR(rclcpp::get_logger("timer"), "Timer %s lived for %f ms", _name.c_str(), elapsedMS()); }
    double elapsedMS() const { return (rclcpp::Clock().now() - _start).seconds() * 1000.0; }

private:
    std::string _name;
    rclcpp::Time _start;
};

}; // namespace tod_helper::Timer

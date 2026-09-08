#include "tod_peanut01_interface/dry_run_interface_node.hpp"

#include <chrono>
#include <functional>

namespace tod_peanut01_interface {

namespace {

int64_t steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

DryRunInterfaceNode::DryRunInterfaceNode()
    : Node("peanut01_dry_run_interface")
{
    const auto steering_ratio = declare_parameter<double>("steering_ratio", 16.0);
    const auto timeout_ms = declare_parameter<int64_t>("command_timeout_ms", 300);
    state_ = std::make_unique<DryRunState>(steering_ratio, timeout_ms * 1000000LL);

    debug_publisher_ = create_publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>(
        "/debug/tod_peanut01/control_cmd", 10);
    command_subscriber_ = create_subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>(
        "input/primary_control_cmd", 10,
        std::bind(&DryRunInterfaceNode::command_callback, this, std::placeholders::_1));
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DryRunInterfaceNode::watchdog_callback, this));

    RCLCPP_WARN(
        get_logger(),
        "Peanut01 interface is in dry-run mode; output is debug-only");
}

void DryRunInterfaceNode::command_callback(
    const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr input)
{
    auto output = state_->on_command(*input, steady_now_ns());
    debug_publisher_->publish(output);
}

void DryRunInterfaceNode::watchdog_callback()
{
    tod_vehicle_msgs::msg::PrimaryControlCmd output;
    if (!state_->make_timeout_command(steady_now_ns(), output)) {
        return;
    }

    output.header.stamp = now();
    debug_publisher_->publish(output);
    RCLCPP_WARN(get_logger(), "Control command timed out; debug output reset to zero");
}

}  // namespace tod_peanut01_interface

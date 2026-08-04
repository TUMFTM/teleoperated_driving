#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tod_peanut01_interface/dry_run_state.hpp"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"

namespace tod_peanut01_interface {

class DryRunInterfaceNode : public rclcpp::Node
{
  public:
    DryRunInterfaceNode();

  private:
    void command_callback(const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr input);
    void watchdog_callback();

    std::unique_ptr<DryRunState> state_;
    rclcpp::Subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr command_subscriber_;
    rclcpp::Publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr debug_publisher_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace tod_peanut01_interface

#pragma once

#include <cstdint>
#include <stdexcept>

#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"

namespace tod_peanut01_interface {

class DryRunState
{
  public:
    DryRunState(double steering_ratio, int64_t timeout_ns);

    tod_vehicle_msgs::msg::PrimaryControlCmd on_command(
        const tod_vehicle_msgs::msg::PrimaryControlCmd & input,
        int64_t now_ns);
    bool make_timeout_command(
        int64_t now_ns,
        tod_vehicle_msgs::msg::PrimaryControlCmd & output);

  private:
    double steering_ratio_;
    int64_t timeout_ns_;
    int64_t last_command_ns_{0};
    bool command_received_{false};
    bool timeout_emitted_{false};
};

}  // namespace tod_peanut01_interface

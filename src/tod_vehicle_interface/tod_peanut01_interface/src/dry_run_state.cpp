#include "tod_peanut01_interface/dry_run_state.hpp"

namespace tod_peanut01_interface {

DryRunState::DryRunState(double steering_ratio, int64_t timeout_ns)
    : steering_ratio_(steering_ratio), timeout_ns_(timeout_ns)
{
    if (steering_ratio_ <= 0.0) {
        throw std::invalid_argument("steering_ratio must be positive");
    }
    if (timeout_ns_ <= 0) {
        throw std::invalid_argument("timeout must be positive");
    }
}

tod_vehicle_msgs::msg::PrimaryControlCmd DryRunState::on_command(
    const tod_vehicle_msgs::msg::PrimaryControlCmd & input,
    int64_t now_ns)
{
    auto output = input;
    output.steering_tire_angle =
        static_cast<float>(input.steering_wheel_angle / steering_ratio_);
    last_command_ns_ = now_ns;
    command_received_ = true;
    timeout_emitted_ = false;
    return output;
}

bool DryRunState::make_timeout_command(
    int64_t now_ns,
    tod_vehicle_msgs::msg::PrimaryControlCmd & output)
{
    if (!command_received_ || timeout_emitted_ || now_ns - last_command_ns_ < timeout_ns_) {
        return false;
    }

    output = tod_vehicle_msgs::msg::PrimaryControlCmd{};
    timeout_emitted_ = true;
    return true;
}

}  // namespace tod_peanut01_interface

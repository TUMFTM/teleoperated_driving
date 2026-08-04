#include <gtest/gtest.h>

#include "tod_peanut01_interface/dry_run_state.hpp"

using tod_peanut01_interface::DryRunState;
using tod_vehicle_msgs::msg::PrimaryControlCmd;

TEST(DryRunState, ConvertsSteeringAndPreservesLongitudinalFields)
{
    DryRunState state(16.0, 300000000);
    PrimaryControlCmd input;
    input.steering_wheel_angle = 1.6F;
    input.steering_tire_angle = 99.0F;
    input.velocity = 2.5F;
    input.acceleration = -0.4F;

    const auto output = state.on_command(input, 1000000000);

    EXPECT_FLOAT_EQ(output.steering_wheel_angle, 1.6F);
    EXPECT_FLOAT_EQ(output.steering_tire_angle, 0.1F);
    EXPECT_FLOAT_EQ(output.velocity, 2.5F);
    EXPECT_FLOAT_EQ(output.acceleration, -0.4F);
}

TEST(DryRunState, EmitsOneZeroCommandAtTimeout)
{
    DryRunState state(16.0, 300000000);
    PrimaryControlCmd input;
    input.velocity = 1.0F;
    state.on_command(input, 1000000000);

    PrimaryControlCmd output;
    EXPECT_FALSE(state.make_timeout_command(1299999999, output));
    EXPECT_TRUE(state.make_timeout_command(1300000000, output));
    EXPECT_FLOAT_EQ(output.steering_wheel_angle, 0.0F);
    EXPECT_FLOAT_EQ(output.steering_tire_angle, 0.0F);
    EXPECT_FLOAT_EQ(output.velocity, 0.0F);
    EXPECT_FLOAT_EQ(output.acceleration, 0.0F);
    EXPECT_FALSE(state.make_timeout_command(1400000000, output));
}

TEST(DryRunState, RearmsTimeoutAfterNextCommand)
{
    DryRunState state(16.0, 300000000);
    PrimaryControlCmd input;
    PrimaryControlCmd output;

    state.on_command(input, 1000000000);
    EXPECT_TRUE(state.make_timeout_command(1300000000, output));
    state.on_command(input, 2000000000);
    EXPECT_TRUE(state.make_timeout_command(2300000000, output));
}

TEST(DryRunState, RejectsInvalidSafetyParameters)
{
    EXPECT_THROW(DryRunState(0.0, 300000000), std::invalid_argument);
    EXPECT_THROW(DryRunState(16.0, 0), std::invalid_argument);
}

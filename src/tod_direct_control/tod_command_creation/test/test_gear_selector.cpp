#include <gtest/gtest.h>

#include "tod_command_creation/gear_selector.hpp"
#include "tod_vehicle_msgs/VehicleEnums.h"

using tod_command_creation::GearSelector;

TEST(GearSelector, StartsFromConfiguredNeutral)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_NEUTRAL, selector.default_gear());
}

TEST(GearSelector, StopsAtReverseAndDriveBoundaries)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_DRIVE, selector.select(GEARPOSITION_DRIVE, true, false, 0.0F));
  EXPECT_EQ(GEARPOSITION_REVERSE, selector.select(GEARPOSITION_REVERSE, false, true, 0.0F));
}

TEST(GearSelector, StepsOnlyThroughReverseNeutralDrive)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_DRIVE, selector.select(GEARPOSITION_NEUTRAL, true, false, 0.0F));
  EXPECT_EQ(GEARPOSITION_REVERSE, selector.select(GEARPOSITION_NEUTRAL, false, true, 0.0F));
}

TEST(GearSelector, BlocksChangesWhileMoving)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(
    GEARPOSITION_NEUTRAL,
    selector.select(GEARPOSITION_NEUTRAL, true, false, 0.01F));
}

TEST(GearSelector, NormalizesAnOutOfRangeGearToDefault)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_NEUTRAL, selector.select(GEARPOSITION_PARK, false, false, 0.0F));
  EXPECT_EQ(GEARPOSITION_NEUTRAL, selector.select(GEARPOSITION_SPORT, false, false, 0.0F));
}

TEST(GearSelector, RejectsInvalidConfiguration)
{
  EXPECT_THROW(GearSelector(3, 1, 2), std::invalid_argument);
  EXPECT_THROW(GearSelector(1, 3, 4), std::invalid_argument);
  EXPECT_THROW(GearSelector(-1, 3, 2), std::invalid_argument);
}

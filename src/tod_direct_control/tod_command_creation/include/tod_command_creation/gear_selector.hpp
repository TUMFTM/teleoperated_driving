#pragma once

#include <stdexcept>

#include "tod_vehicle_msgs/VehicleEnums.h"

namespace tod_command_creation {

class GearSelector
{
public:
  GearSelector(const int min_gear, const int max_gear, const int default_gear)
  : min_gear_(min_gear), max_gear_(max_gear), default_gear_(default_gear)
  {
    const bool known_bounds =
      min_gear_ >= GEARPOSITION_PARK && max_gear_ <= GEARPOSITION_HAUL;
    if (!known_bounds || min_gear_ > max_gear_ ||
      default_gear_ < min_gear_ || default_gear_ > max_gear_)
    {
      throw std::invalid_argument("invalid gear selection range");
    }
  }

  int default_gear() const { return default_gear_; }

  int select(
    const int current_gear, const bool increase_edge, const bool decrease_edge,
    const float current_velocity) const
  {
    int selected =
      current_gear >= min_gear_ && current_gear <= max_gear_ ? current_gear : default_gear_;
    if (current_velocity >= 0.01F) {
      return selected;
    }
    if (increase_edge && selected < max_gear_) {
      ++selected;
    }
    if (decrease_edge && selected > min_gear_) {
      --selected;
    }
    return selected;
  }

private:
  int min_gear_;
  int max_gear_;
  int default_gear_;
};

}  // namespace tod_command_creation

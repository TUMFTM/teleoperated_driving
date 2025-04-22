#pragma once

#include "tod_automation_msgs/msg/trajectory.hpp"
#include "tod_helper/trajectory/Helpers.h"

namespace tod_helper::Trajectory {
class Debug {
public:
    static void plot_trajectory(const tod_automation_msgs::msg::Trajectory& traj);
};
};
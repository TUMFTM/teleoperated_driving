// Copyright 2021 Feiler

#if PERCEPTION_MODIFICATION_TRUE_MACRO
#pragma once

#include "tod_automation_msgs/msg/trajectory.hpp"
#include "tod_helper/geometry/Helpers.h"
#include <iostream>

namespace tod_helper::Trajectory {

void brakeIntoStillStand(tod_automation_msgs::msg::Trajectory& trajectory,
                         const int indexOfStandstill, const double goalVelocityInMS,
                         const double brakeAccelerationMS2) {
    for ( int iterator = indexOfStandstill; iterator < trajectory.points.size();
         ++iterator ) {
        trajectory.points.at(iterator).twist.twist.linear.x = 0.0;
    }

    if ( indexOfStandstill >= trajectory.points.size() || indexOfStandstill < 0 ) {
        std::cout << "indexOfStandstill must be in appropriate range" << "\n";
        return;
    }

    double brakeDistance = std::abs(std::pow(goalVelocityInMS, 2) / (2.00 * brakeAccelerationMS2));
    double backwardsDistance = 0;
    for ( int trajPointIt = indexOfStandstill-1; trajPointIt >= 0; --trajPointIt ) {
        backwardsDistance += tod_helper::Geometry::calc_horizontal_distance(
            trajectory.points.at(trajPointIt).pose.pose.position,
            trajectory.points.at(trajPointIt+1).pose.pose.position);
        if ( backwardsDistance >= brakeDistance ) {
            break;
        }
        double xValue = brakeDistance - backwardsDistance;
        double velocity = std::sqrt(
            (2*brakeAccelerationMS2*xValue) + (std::pow(goalVelocityInMS, 2)));
        trajectory.points.at(trajPointIt).twist.twist.linear.x = velocity;
    }
}

}; // namespace tod_helper::Trajectory

#endif

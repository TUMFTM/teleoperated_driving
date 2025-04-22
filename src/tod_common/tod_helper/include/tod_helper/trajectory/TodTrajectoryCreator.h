// Copyright 2021 Feiler

#if PERCEPTION_MODIFICATION_TRUE_MACRO
#pragma once
#include "tod_msgs/Trajectory.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

namespace tod_helper::Trajectory {

class TodTrajectoryCreator {
public:
    TodTrajectoryCreator() = default;
    virtual tod_msgs::Trajectory createFrom(const std::vector<geometry_msgs::Point>& points) = 0;
    virtual ~TodTrajectoryCreator() = default;
    virtual void brakeIntoStillStand(tod_msgs::Trajectory& outTrajectory,
        const double& maxVelocity, const double& brakeAccelerationMS2,
        const int& amountOfBufferPointsTilBrake) = 0;
};

}; // namespace tod_helper::Trajectory

#endif

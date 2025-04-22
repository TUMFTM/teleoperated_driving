// Copyright 2021 Feiler

#if PERCEPTION_MODIFICATION_TRUE_MACRO
#pragma once
#include "TodTrajectoryCreator.h"
#include "rclcpp/rclcpp.hpp"
#include "../geometry/Helpers.h"

namespace tod_helper::Trajectory {

class StraightTrajectory : public TodTrajectoryCreator {
public:
    explicit StraightTrajectory(ros::NodeHandle& nodeHandleToParams)
        : nodeHandle(nodeHandleToParams) {
        loadParamsFromParamServer();
    }
    ~StraightTrajectory() {}

    tod_msgs::Trajectory createFrom(const std::vector<geometry_msgs::Point>& points) {
        tod_msgs::Trajectory outTrajectory;
        outTrajectory.child_frame_id = childFrameId;
        outTrajectory.header.frame_id = frameId;
        if ( points.size() < 2 ) {
            //        ROS_ERROR("No route provided");
            return outTrajectory;
        }
        for ( auto point = points.begin()+1; point != points.end(); ++point ) {
            addPointsBetweenTooWidelySpacedPoints(point, outTrajectory);
        }
        brakeIntoStillStand(outTrajectory);
        return outTrajectory;
    }

    void brakeIntoStillStand(tod_msgs::Trajectory& outTrajectory,
                             const double& maxVelocity, const double& brakeAccelerationMS2,
                             const int& amountOfBufferPointsTilBrake) {
        double brakeBeginInMeter = -std::pow(maxVelocity, 2) / (2.00 * brakeAccelerationMS2);
        double backwardsDistance = 0;
        for ( auto iterator = outTrajectory.points.rbegin(); iterator != outTrajectory.points.rend();
             ++iterator) {
            int backwardsIterationNumber = (int) (iterator - outTrajectory.points.rbegin());
            ROS_DEBUG_STREAM("\nbackwardsIterationNumber " <<
                             backwardsIterationNumber << " brakeBeginInMeter " << brakeBeginInMeter <<
                             " amountOfBufferPointsTilBrake " << amountOfBufferPointsTilBrake);
            if (backwardsIterationNumber <= amountOfBufferPointsTilBrake) {
                iterator->twist.twist.linear.x = 0.0;
                continue;
            }
            backwardsDistance += tod_helper::Geometry::calc_horizontal_distance(
                iterator->pose.pose.position, (iterator-1)->pose.pose.position);
            ROS_DEBUG_STREAM(" backwardsDistance " << backwardsDistance);
            if (backwardsDistance >= brakeBeginInMeter) {
                break;
            }
            double xValue = brakeBeginInMeter - backwardsDistance;
            double velocity = std::sqrt(
                (2*brakeAccelerationMS2*xValue) +
                (std::pow(maxVelocity, 2)));
            ROS_DEBUG_STREAM(" xValue " << xValue << " velocity " << velocity);
            iterator->twist.twist.linear.x = velocity;
        }
    }


private:
    ros::NodeHandle nodeHandle;
    std::string childFrameId;
    std::string frameId;
    double maxDistanceBetweenTwoPoints;
    double maxVelocity;
    double brakeAccelerationMS2;
    int amountOfBufferPointsTilBrake;

    void loadParamsFromParamServer() {
        std::string paramName = "child_frame_id";
        if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                                 childFrameId)) {
            printStandardRosWarn(paramName);
            childFrameId = "base_footprint";
        }
        paramName = "frame_id";
        if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                                 frameId)) {
            printStandardRosWarn(paramName);
            frameId = "ftm";
        }
        paramName = "maxDistanceBetweenTwoPoints";
        if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                                 maxDistanceBetweenTwoPoints)) {
            printStandardRosWarn(paramName);
            maxDistanceBetweenTwoPoints = 0.41;
        }
        paramName = "amountOfBufferPointsTilBrake";
        if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                                 amountOfBufferPointsTilBrake)) {
            printStandardRosWarn(paramName);
            amountOfBufferPointsTilBrake = 2;
        }
        paramName = "maxVelocity";
        if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                                 maxVelocity)) {
            printStandardRosWarn(paramName);
            maxVelocity = 1.5;
        }
        paramName = "brakeAccelerationMS2";
        if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                                 brakeAccelerationMS2)) {
            printStandardRosWarn(paramName);
            brakeAccelerationMS2 = 2.0;
        }
    }

    void printStandardRosWarn(const std::string& paramName) {
        ROS_WARN("could not find %s on param server at %s. Using default value.",
                 paramName.c_str(), ros::this_node::getName().c_str());
    }

    void addPointsBetweenTooWidelySpacedPoints(
        std::vector<geometry_msgs::Point>::const_iterator point, tod_msgs::Trajectory& outTrajectory) {
        // calcNewDistanceBetweenPoints()
        double distanceBetweenOriginalPoints = tod_helper::Geometry::calc_horizontal_distance(*(point-1), *point);
        double minAmountOfIntervals = distanceBetweenOriginalPoints/maxDistanceBetweenTwoPoints;
        int amountOfPoints = std::ceil(minAmountOfIntervals) + 1;
        double distanceBetweenNewPoints = distanceBetweenOriginalPoints / (amountOfPoints-1);
        // calcNormalizedDirection()
        geometry_msgs::Point normalizedDirectionToNextOriginalPoint;
        normalizedDirectionToNextOriginalPoint.x =
            (point->x - (point-1)->x) / distanceBetweenOriginalPoints;
        normalizedDirectionToNextOriginalPoint.y =
            (point->y - (point-1)->y) / distanceBetweenOriginalPoints;
        for ( int nthPoint = 1; nthPoint != amountOfPoints; ++nthPoint ) {
            tod_msgs::TrajectoryPoint newPoint;
            // createNextPoint()
            newPoint.twist.twist.linear.x = maxVelocity;
            newPoint.pose.pose.position.x = (point-1)->x +
                                            nthPoint*normalizedDirectionToNextOriginalPoint.x*distanceBetweenNewPoints;
            newPoint.pose.pose.position.y = (point-1)->y +
                                            nthPoint*normalizedDirectionToNextOriginalPoint.y*distanceBetweenNewPoints;
            newPoint.pose.pose.position.z = 0.0;
            outTrajectory.points.push_back(newPoint);
        }
    }

    double calcNewDistanceBetweenPoints(
        std::vector<geometry_msgs::Point>::const_iterator point);

    void brakeIntoStillStand(tod_msgs::Trajectory& outTrajectory) {
        brakeIntoStillStand(outTrajectory, maxVelocity, brakeAccelerationMS2,
                            amountOfBufferPointsTilBrake);
    }
};

}; // namespace tod_helper::Trajectory

#endif

/**
 * @file vehicle_model.hpp
 * @brief Simple kinematic bicycle model to represent the vehicle
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

// Copyright 2020
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>

namespace tod_pure_pursuit {

class VehicleModel {
  public:
    VehicleModel() { reset_initial_position(0.0, 0.0, 0.0); }
    ~VehicleModel() = default;
    void set_params(const float lf, const float lr, const float maxRWA, const float maxSWA);

    void reset_initial_position(const double xIn, const double yIn, const double yawIn);
    void update_position(const double desiredVelocity, const double swa, const int gearPosition, const double dt);

    double get_psi_p();
    double get_psi();
    double get_x();
    double get_y();
    double get_x_p();
    double get_y_p();
    double get_velocity_x();
    double get_velocity_y();
    double get_angular_x();
    double get_angular_y();
    double get_steering_angle();

  private:
    void limit_angle(double angle);
    double clip(double n, double lower, double upper);
    inline double swa2rwa(const double steering_wheel_angle, const double max_steering_wheel_angle,
                          const double max_road_wheel_angle);

    double _x{0.0f}, _y{0.0f}, _yaw{0.0f};
    double _xp{0.0f}, _yp{0.0f}, _vx{0.0f}, _vy{0.0f}, _ax{0.0f}, _ay{0.0f};
    double _rwa{0.0f}, _yawRate{0.0f};
    const double _min_acceleration{-8.0}, _max_acceleration{4.0};
    const double _max_rwa_rate{3.141 / 2};

    double _lf{1.0f}, _lr{1.0f};  // distance between CoM and front/rear axle
    float _max_rwa{1.0f}, _max_swa{1.0f};
};

}  // namespace tod_pure_pursuit
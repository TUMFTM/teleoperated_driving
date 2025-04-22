/**
 * @file vehicle_model.hpp
 * @brief Simple kinematic bicycle model.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_vehicle_sim
 */

#pragma once

#include <math.h>

#include "tod_helper/vehicle/Model.h"
#include "tod_vehicle_msgs/VehicleEnums.h"

namespace tod_vehicle_sim {
/**
 * @defgroup tod_vehicle_sim
 * @ingroup tod_vehicle_sim
 * @brief Simple vehicle simulation to emulate a vehicle interface.
 */

/**
 * @brief Single track model.
 */
class VehicleModel 
{
    public:
        VehicleModel() { reset_initial_position(0.0, 0.0, 0.0); }
        ~VehicleModel() {}
        void set_params(const float lf, const float lr, const float maxRWA, const float maxSWA);

        void reset_initial_position(const double xIn, const double yIn, const double yawIn);
        void update_position(const double desiredVelocity, const double swa, const uint8_t gearPosition, const double dt);

        double get_psi_p();
        double get_psi();
        double get_x();
        double get_y();
        double get_x_p();
        double get_y_p();
        double get_velocity_x();
        double get_velocity_y();
        double get_acceleration_x();
        double getAy();
        double get_steering_angle();

    private:
        void limit_angle(double angle);

        double clip(double n, double lower, double upper);

        double _x{0.0f}, _y{0.0f}, _yaw{0.0f};
        double _xp{0.0f}, _yp{0.0f}, _vx{0.0f}, _vy{0.0f}, _ax{0.0f}, _ay{0.0f};
        double _rwa{0.0f}, _yawRate{0.0f};
        const double _min_acceleration{-8.0}, _max_acceleration{4.0};
        const double _maxRWARate{tod_helper::Vehicle::Model::deg2rad(90.0)};

        double _lf{1.0f}, _lr{1.0f}; // distance between CoM and front/rear axle
        float _maxRWA{1.0f}, _maxSWA{1.0f};
};

} // namespace tod_vehicle_sim
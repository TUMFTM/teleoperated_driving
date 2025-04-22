/**
 * @file vehicle_model.cpp
 * @brief Simple kinematic bicycle model.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_vehicle_sim
 */

#include "tod_vehicle_sim/vehicle_model.hpp"

namespace tod_vehicle_sim {

void VehicleModel::set_params(const float lf, const float lr, const float maxRWA, const float maxSWA) 
{
    if (lf != 0.0 && lf != 0.0 && maxRWA != 0.0 && maxSWA != 0.0)
    {
        _lf = lf;
        _lr = lr;
        _maxRWA = maxRWA;
        _maxSWA = maxSWA;
    };
}

double VehicleModel::clip(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
}

void VehicleModel::reset_initial_position(const double xIn, const double yIn, const double yawIn) {
    _x = xIn;
    _y = yIn;
    _yaw = yawIn;
    _xp = _yp = _vx = _vy = _ax = _ay = _rwa = _yawRate = 0.0f;
}

void VehicleModel::update_position(const double desiredVelocity, const double swa,
                                  const uint8_t gearPosition, const double dt) {
    const auto abs_desired_velocity = std::abs(desiredVelocity);

    double previousVelocity = std::sqrt(std::pow(_vx, 2) + std::pow(_vy, 2));
    double maxVelocity = previousVelocity + dt * _max_acceleration;
    double minVelocity = previousVelocity + dt * _min_acceleration;
    double currentVelocity = clip(abs_desired_velocity, minVelocity, maxVelocity);

    double minRWA = std::max(_rwa - dt * _maxRWARate, double(-_maxRWA));
    double maxRWA = std::min(_rwa + dt * _maxRWARate, double(+_maxRWA));
    _rwa = std::clamp(tod_helper::Vehicle::Model::swa2rwa(swa, _maxSWA, _maxRWA),
                      minRWA, maxRWA);
    double beta = std::atan(std::tan(_rwa) * _lr / (_lr + _lf));

    // velocities in vehicle frame
    _vx = currentVelocity * std::cos(beta);
    _vy = currentVelocity * std::sin(beta);
    _yawRate = std::sin(beta) * currentVelocity / _lr;

    // velocity / position in odom frame
    _xp = currentVelocity * std::cos(beta + _yaw);
    _yp = currentVelocity * std::sin(beta + _yaw);

    // change sign in gear position reverse
    if (gearPosition == eGearPosition::GEARPOSITION_REVERSE) {
        _vx *= (-1);
        _vy *= (-1);
        _yawRate *= (-1);
        _xp *= (-1);
        _yp *= (-1);
    }

    // integrate positions
    _x += _xp * dt;
    _y += _yp * dt;
    _yaw += _yawRate * dt;
    limit_angle(_yaw); // limit from -pi to pi

    double radius = (_lf + _lr) / std::tan(_rwa);
    _ay = currentVelocity * currentVelocity / radius;
    _ax = (currentVelocity - previousVelocity) / dt;
}

void VehicleModel::limit_angle(double angle) 
{
    const double pi = 3.1415;
    int n = std::floor(std::abs(angle) / (2 * pi));
    n = (angle > 0) ? n : -1 * n;
    angle = angle - n * (2 * pi); // remove full turns

    if (angle > pi) angle = angle - 2 * pi;
    if (angle < -pi) angle = angle + 2 * pi;
}

double VehicleModel::get_psi() { return _yaw; }
double VehicleModel::get_psi_p() { return _yawRate; }
double VehicleModel::get_x() { return _x; }
double VehicleModel::get_y() { return _y; }
double VehicleModel::get_x_p() { return _xp; }
double VehicleModel::get_y_p() { return _yp; }
double VehicleModel::get_velocity_x() { return _vx; }
double VehicleModel::get_velocity_y() { return _vy; }
double VehicleModel::get_acceleration_x() { return _ax; }
double VehicleModel::getAy() { return _ay; }
double VehicleModel::get_steering_angle() { return _rwa; }

} // namespace tod_vehicle_sim
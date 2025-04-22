/**
 * @file path_helper.cpp
 * @brief Helpers for PathCreator related to the PathCalculations
 * @details Main file for the process of path generation and operator side validation for the trajectory guidance
 * process. Path Generation using a cubic hermit spline @see https://kluge.in-chemnitz.de/opensource/spline/ by Tino
 * Kluge
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#include "tod_trajectory_guidance/operator/path_helper.hpp"
#include <limits>
#include <vector>

namespace tod_trajectory_guidance {

/**
 * @brief Generates a spline path with specified step size from control points
 * @param control_points Input control points defining the path
 * @param showCurvature Flag to indicate if curvature visualization is needed
 * @param step_size Distance between generated path points
 * @return Pair of paths - one for visualization and one for vehicle control
 */
std::pair<tod_trajectory_guidance_msgs::msg::Path::SharedPtr, tod_trajectory_guidance_msgs::msg::Path::SharedPtr>
PathHelper::generate_spline_with_step_size(
    const tod_trajectory_guidance_msgs::msg::ControlPoints::SharedPtr& control_points, bool showCurvature,
    double step_size, double y_max_curv, double maxCurv) {
    std::vector<double> X, Y, T;
    double xpos = 0.0, ypos = 0.0;

    auto path_vis = std::make_shared<tod_trajectory_guidance_msgs::msg::Path>();
    auto path_send = std::make_shared<tod_trajectory_guidance_msgs::msg::Path>();

    for (const auto& point : control_points->points) {
        double dx = point.x - xpos;
        double dy = point.y - ypos;
        if (!X.empty()) {
            T.push_back(T.back() + std::sqrt(dx * dx + dy * dy));
        } else {
            T.push_back(0.0);
        }

        xpos = point.x;
        ypos = point.y;
        X.push_back(xpos);
        Y.push_back(ypos);
    }

    if (control_points->points.size() > 2) {
        const tk::spline sx(T, X), sy(T, Y);
        double current_t = T.front();

        while (current_t <= T.back()) {
            double next_t = find_next_t(sx, sy, current_t, step_size);
            const auto next_x = sx(next_t);
            const auto next_y = sy(next_t);

            tod_trajectory_guidance_msgs::msg::PathPoint pt_vis;
            tod_trajectory_guidance_msgs::msg::PathPoint pt_send;

            pt_vis.pose.position.x = next_x;
            pt_vis.pose.position.y = next_y;

            pt_send.pose.position.x = next_x;
            pt_send.pose.position.y = next_y;

            const double curv = curvature(sx, sy, next_t);
            pt_vis.curvature = curv;
            pt_send.curvature = curv;

            double v_max_curv;
            if (curv == 0.f) {
                v_max_curv = std::numeric_limits<double>::infinity();
            } else {
                v_max_curv = std::sqrt(y_max_curv / curv);
            }

            pt_vis.v_max_curv = v_max_curv;
            pt_send.v_max_curv = v_max_curv;

            if (curv > maxCurv) {
                pt_vis.pose.position.z = 1;
            } else {
                pt_vis.pose.position.z = 0;
            }

            path_vis->points.push_back(pt_vis);
            path_send->points.push_back(pt_send);

            current_t = next_t;
            if (current_t >= T.back())
                break;
        }
    }
    return std::make_pair(path_vis, path_send);
}

double PathHelper::curvature(const tk::spline& x, const tk::spline& y, double index) {
    double curvature = abs(x.deriv(1, index) * y.deriv(2, index) - x.deriv(2, index) * y.deriv(1, index)) /
                       (pow((pow(x.deriv(1, index), 2) + pow(y.deriv(1, index), 2)), 1.5));
    return curvature;
}

Vector2 PathHelper::calculate_orthogonal_vector(const Vector2& vec, const double scale, const Vector2 A) {
    Vector2 orthogonalVec;
    orthogonalVec.x = -vec.y;
    orthogonalVec.y = vec.x;

    double norm = calculate_normal(orthogonalVec);
    orthogonalVec.x = (orthogonalVec.x / norm) * scale;
    orthogonalVec.y = (orthogonalVec.y / norm) * scale;

    orthogonalVec.x = orthogonalVec.x + A.x;
    orthogonalVec.y = orthogonalVec.y + A.y;

    return orthogonalVec;
}

Vector2 PathHelper::calculate_vector(const Vector2& A, const Vector2& B) {
    Vector2 vec;
    vec.x = B.x - A.x;
    vec.y = B.y - A.y;
    return vec;
}

Vector2 PathHelper::calculate_vector_from_point_and_heading(double x, double y, double theta) {
    Vector2 result;
    result.x = cos(theta);
    result.y = sin(theta);

    if (std::abs(result.x) < 1e-5 && std::abs(result.y) < 1e-5) {
        result.x = 1.0;
        result.y = 0.0;
    }

    return result;
}

double PathHelper::calculate_normal(const Vector2& vec) {
    double norm = std::sqrt(vec.x * vec.x + vec.y * vec.y);
    return norm == 0 ? 1 : norm;
}

double PathHelper::arc_length_integral(const tk::spline& spx, const tk::spline& spy, double t0, double t1) {
    int n = 100;
    double dt = (t1 - t0) / n;
    double sum = 0.0;

    for (int i = 0; i <= n; i++) {
        double t = t0 + i * dt;
        double dx = spx.deriv(1, t);
        double dy = spy.deriv(1, t);

        sum += sqrt(dx * dx + dy * dy) * dt;
    }

    return sum;
}

double PathHelper::calculate_euclidean_distance(double x1, double y1, double x2, double y2) {
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

double PathHelper::find_next_t(const tk::spline& spx, const tk::spline& spy, double t0, double target_length,
                               double tolerance) {
    double t = t0 + 0.05;
    while (true) {
        double F = arc_length_integral(spx, spy, t0, t) - target_length;
        if (std::fabs(F) < tolerance)
            break;

        double dx = spx.deriv(1, t);
        double dy = spy.deriv(1, t);

        double F_prime = sqrt(dx * dx + dy * dy);
        const double delta = F / F_prime;
        t -= delta;
        if (t <= t0 || std::fabs(delta) < tolerance) {
            t = t0 + 0.01;
            break;
        }
    }

    return t;
}

}  // namespace tod_trajectory_guidance
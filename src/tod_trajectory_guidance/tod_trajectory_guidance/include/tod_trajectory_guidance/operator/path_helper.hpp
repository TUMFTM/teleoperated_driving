/**
 * @file PathHelper.hpp
 * @brief Helpers for PathCreator related to the PathCalculations
 * @details Main file for the process of path calulations process. Path Calculations using a cubic hermit spline @see
 * https://kluge.in-chemnitz.de/opensource/spline/ by Tino Kluge
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#pragma once
#include <cmath>
#include <memory>
#include <utility>

#include "tod_trajectory_guidance/operator/spline.h"
#include "tod_trajectory_guidance_msgs/msg/control_points.hpp"
#include "tod_trajectory_guidance_msgs/msg/path.hpp"

namespace tod_trajectory_guidance {

struct Vector2 {
    double x;
    double y;
};

class PathHelper {
  public:
    static std::pair<tod_trajectory_guidance_msgs::msg::Path::SharedPtr,
                     tod_trajectory_guidance_msgs::msg::Path::SharedPtr>
    generate_spline_with_step_size(const tod_trajectory_guidance_msgs::msg::ControlPoints::SharedPtr& points,
                                   bool showCurvature = false, double step_size = 0.1, double y_max_curv = 3.0,
                                   double maxCurv = 0.133);

    static double curvature(const tk::spline& x, const tk::spline& y, double index);
    static Vector2 calculate_orthogonal_vector(const Vector2& vec, const double scale, const Vector2 A);
    static Vector2 calculate_vector(const Vector2& A, const Vector2& B);
    static double calculate_normal(const Vector2& vec);
    static Vector2 calculate_vector_from_point_and_heading(const double x, const double y, const double theta);
    static double arc_length_integral(const tk::spline& spx, const tk::spline& spy, double t0, double t1);
    static double calculate_euclidean_distance(double x1, double y1, double x2, double y2);
    static double find_next_t(const tk::spline& spx, const tk::spline& spy, double t0, double target_length = 0.1,
                              double tolerance = 0.0005);
};

}  // namespace tod_trajectory_guidance
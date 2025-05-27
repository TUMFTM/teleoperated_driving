/**
 * @file trajectory_renderer.hpp
 * @brief Renders a fully triangulated trajectory.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/scriptable_entity.hpp"

namespace TodDynamicEntities {

/**
 * @class TrajectoryRenderer
 * @brief Renders a trajectory with full triangulation.
 */
class TrajectoryRenderer : public tod_gl::ScriptableEntity {
  public:
    /**
     * @brief Default constructor.
     */
    TrajectoryRenderer() = default;

    /**
     * @brief Initializes the trajectory renderer.
     */
    virtual void on_create() override;

    /**
     * @brief Cleans up resources used by the trajectory renderer.
     */
    virtual void on_destroy() override;

    /**
     * @brief Updates the trajectory rendering.
     *
     * @param delta_time Time elapsed since the last update.
     */
    virtual void on_update(float delta_time) override;

  private:
    // Line width for trajectory rendering.
    const float _line_width = 2.f;
    // Trajectory render width
    const float _trajectory_width = 2.f;
    // Trajectories are quite dense, stepsize is for performance.
    const int _step_size = 5;
};

}  // namespace TodDynamicEntities

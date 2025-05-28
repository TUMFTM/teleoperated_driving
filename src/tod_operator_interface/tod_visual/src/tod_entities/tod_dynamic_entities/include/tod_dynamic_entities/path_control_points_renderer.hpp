/**
 * @file path_control_points_renderer.hpp
 * @brief PathControlPointsRenderer renders the Control Points of the @ref TrajectoryGuidances control points for the Path.
 *
 * This file declares the PathControlPointsRenderer class, which is responsible for rendering the control points
 * used in trajectory guidance for path planning. These control points are visualized using lines and quadrilaterals,
 * providing a clear representation of the path in the 3D scene.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/scriptable_entity.hpp"

namespace TodDynamicEntities {

/**
 * @class PathControlPointsRenderer
 * @brief Renders the control points used in trajectory guidance for path planning.
 *
 * The PathControlPointsRenderer class visualizes the control points associated with the @ref TrajectoryGuidances.
 * It leverages the rendering system provided by tod_gl::ScriptableEntity to display these points with a specified
 * line width and quadrilateral size, aiding in the visualization and debugging of path trajectories.
 */
class PathControlPointsRenderer : public tod_gl::ScriptableEntity {
  public:
    /**
     * @brief Default constructor.
     */
    PathControlPointsRenderer() = default;

    /**
     * @brief Called when the entity is created.
     *
     * Initializes resources necessary for rendering the control points.
     */
    virtual void on_create() override;

    /**
     * @brief Called when the entity is destroyed.
     *
     * Cleans up and releases resources allocated for control point rendering.
     */
    virtual void on_destroy() override;

    /**
     * @brief Called on each update cycle.
     *
     * Updates the visualization of the control points, ensuring the rendered path reflects any changes.
     *
     * @param delta_time Time elapsed since the last update.
     */
    virtual void on_update(float delta_time) override;

  private:
    /**
     * @brief The width of the line connecting the control points.
     *
     * Determines the thickness of the lines drawn between the control points.
     */
    float _line_width = 2.7f;

   
    const float _quad_size = 0.5f;
};

}  // namespace TodDynamicEntities

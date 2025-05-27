/**
 * @file driving_lane_renderer.hpp
 * @brief DrivingLaneRenderer renders a projected driving lane based on the wheel position.
 *
 * This class renders the predicted driving lane based on the current steering wheel angle,
 * similar to the visual guidance provided by reversing cameras in most cars.
 * It calculates the lane projection from the wheel position to assist in vehicle maneuvering.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/scriptable_entity.hpp"

namespace TodDynamicEntities {

/**
 * @class DrivingLaneRenderer
 * @brief Renders a projected driving lane based on the current steering wheel angle.
 *
 * The DrivingLaneRenderer class computes and renders a driving lane prediction
 * based on the current wheel position. This prediction functions similarly to the visual aid
 * provided by reversing cameras in most cars, helping drivers gauge the vehicle's trajectory
 * during maneuvers.
 */
class DrivingLaneRenderer : public tod_gl::ScriptableEntity {
  public:
    /**
     * @brief Default constructor.
     */
    DrivingLaneRenderer() = default;

    /**
     * @brief Called when the entity is created.
     *
     * Initializes all resources necessary for rendering the predicted driving lane.
     */
    virtual void on_create() override;

    /**
     * @brief Called when the entity is destroyed.
     *
     * Releases resources allocated for lane rendering.
     */
    virtual void on_destroy() override;

    /**
     * @brief Updates the lane rendering.
     * 
     * @param delta_time Time elapsed since the last update, used for frame-based adjustments.
     *
     * Recalculates the lane projection based on the current steering wheel angle,
     * providing dynamic visual feedback similar to a reversing camera.
     */
    virtual void on_update(float delta_time) override;

  private:
    /**
     * @brief The width of the rendered lane.
     *
     * Defines the thickness of the lane visualization.
     */
    float line_width = .05f;
};

}  // namespace TodDynamicEntities

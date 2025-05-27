/**
 * @file position_controller.hpp
 * @brief Odometry controller for the TransformationSystem.
 * 
 * Handles the movement transformation based on odometry.
 * 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/scriptable_entity.hpp"

namespace TodDynamicEntities {

/**
 * @class PositionController
 * @brief Updates the entity's position based on odometry.
 */
class PositionController : public tod_gl::ScriptableEntity {
    public:
        /**
         * @brief Default constructor.
         */
        PositionController() = default;

        /**
         * @brief Initializes the position controller.
         */
        void on_create() override {};

        /**
         * @brief Updates the position based on the time step.
         * 
         * @param ts Time step for the update.
         */
        void on_update(float ts) override;

    private:
        /// Indicates if the offset has been set.
        bool _is_offset_set{false};
};

}  // namespace TodDynamicEntities

/**
 * @file wheel_controller.hpp
 * @brief Controls wheel turning for driving immersion.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/scriptable_entity.hpp"

namespace TodDynamicEntities {

/**
 * @class WheelController
 * @brief Base controller for turning wheels.
 */
class WheelController : public tod_gl::ScriptableEntity {
    public:
        /**
         * @brief Default constructor.
         */
        WheelController() = default;

        /**
         * @brief Initializes the wheel controller.
         */
        void on_create() override {};

        /**
         * @brief Updates wheel turning.
         *
         * @param ts Time step.
         */
        void on_update(float ts) override;
    private:
        /// Speed at which the wheels turn.
        float _turning_speed = 3.f;
};

/**
 * @class FrontWheelController
 * @brief Controller for turning the front wheels.
 */
class FrontWheelController : public WheelController {
    public:
        /**
         * @brief Updates front wheel turning.
         *
         * @param ts Time step.
         */
        void on_update(float ts) override;
};

}  // namespace TodDynamicEntities

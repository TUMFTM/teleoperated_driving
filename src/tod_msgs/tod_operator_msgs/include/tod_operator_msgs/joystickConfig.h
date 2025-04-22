#pragma once

/**
 * @file joystickConfig.h 
 * @brief Defines enumerations for joystick button and axis positions.
 * @copyright 2020 TUMFTM
 */

namespace joystick {

/**
 * @enum ButtonPos
 * @brief Enumerates the button positions on the joystick.
 * 
 * These represent the index of the field in the joystick 
 * message sent by tod_input_devices and their roles.
 */
enum ButtonPos {
    INDICATOR_LEFT  = 0,
    INDICATOR_RIGHT = 1,
    FLASHLIGHT      = 2,
    FRONTLIGHT      = 3,
    HONK            = 4,
    INCREASE_SPEED  = 5,
    DECREASE_SPEED  = 6,
    INCREASE_GEAR   = 7,
    DECREASE_GEAR   = 8
};

/**
 * @enum AxesPos
 * @brief Enumerates the axis positions on the joystick.
 * 
 * These represent specific joystick axes in the joystick
 * message sent by tod_input_devices and their roles.
 */
enum AxesPos {
    STEERING        = 0,
    THROTTLE        = 1,
    BRAKE           = 2
};
}; // namespace joystick

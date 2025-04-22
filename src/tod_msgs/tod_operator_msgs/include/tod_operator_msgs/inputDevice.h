#pragma once

/**
 * @file inputDevice.h
 * @brief Holds an enumeration for input devices used in the software stack
 * @copyright 2020 TUMFTM
 */

/**
 * @enum InputDevice
 * @brief Enumerates the different input devies supported by the software stack
 */
enum InputDevice {
    FANATEC             = 0,
    SENSOWHEEL          = 1,
    VIRTUALINPUTDEVICE  = 2,
    XBOXCONTROLLER      = 3,
    STORMJOYSTICK       = 4
};

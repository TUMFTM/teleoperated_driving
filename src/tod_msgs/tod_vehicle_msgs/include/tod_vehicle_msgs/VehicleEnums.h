#pragma once

/**
 * @file VehicleEnums.h
 * @brief Defines enumerations used in SecondaryControlCmd and SecondaryVehicleData messages.
 * @copyright 2020 TUMFTM
 */

/**
 * @enum eIndicator
 * @brief Enumerates the possible states of vehicle indicators (off/left/right/both).
 */
enum eIndicator {
    INDICATOR_OFF = 0,
    INDICATOR_LEFT = 1,
    INDICATOR_RIGHT = 2,
    INDICATOR_BOTH = 3
};

/**
 * @enum eGearPosition
 * @brief Enumerates the possible gear positions (park/reverse/neutral/drive/sport/haul) of a vehicle.
 */
enum eGearPosition {
    GEARPOSITION_PARK = 0,
    GEARPOSITION_REVERSE = 1,
    GEARPOSITION_NEUTRAL = 2,
    GEARPOSITION_DRIVE = 3,
    GEARPOSITION_SPORT = 4,
    GEARPOSITION_HAUL = 5
};

/**
 * @enum eHonk
 * @brief Enumerates the states of the vehicle horn (on/off).
 */
enum eHonk {
    HONK_OFF = 0,
    HONK_ON = 1
};

/**
 * @enum eWiper
 * @brief Enumerates the states of the vehicle wipers (on/off/interval).
 */
enum eWiper {
    WIPER_OFF = 0,
    WIPER_ON = 1,
    WIPER_INTERVAL = 2
};

/**
 * @enum eHeadLight
 * @brief Enumerates the states of the vehicle's headlights (on/off).
 */
enum eHeadLight {
    HEADLIGHT_OFF = 0,
    HEADLIGHT_ON = 1
};

/**
 * @enum eFlashLight
 * @brief Enumerates the states of the vehicle's flashlights (on/off).
 */
enum eFlashLight {
    FLASHLIGHT_OFF = 0,
    FLASHLIGHT_ON = 1
};

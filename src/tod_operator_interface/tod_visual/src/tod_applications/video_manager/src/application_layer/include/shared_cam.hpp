/**
 * @file shared_cam.hpp
 * @brief Defines the SharedCam structure for managing camera data in the application.
 * @copyright 2024 TUMFTM
 */

#ifndef SHARED_CAM_HPP
#define SHARED_CAM_HPP

#include <string>

/**
 * @struct SharedCam
 * @brief Represents a shared camera object with properties for its name, active state, and mapping.
 */
struct SharedCam {
    std::string name; ///< The name of the camera.
    bool is_active;    ///< Indicates whether the camera is active.
    int mapping;      ///< Mapping ID for the camera.

    /**
     * @brief Constructs a SharedCam object.
     * 
     * @param name Name of the camera.
     * @param is_active Initial active state of the camera.
     * @param mapping Mapping ID for the camera.
     */
    SharedCam(const std::string& name, bool is_active, int mapping)
        : name(name), is_active(is_active), mapping(mapping) {}
};

#endif // SHARED_CAM_HPP

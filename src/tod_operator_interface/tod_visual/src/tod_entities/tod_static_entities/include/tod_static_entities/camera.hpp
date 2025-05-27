/**
 * @file camera.hpp
 * @brief Camera entity that holds the camera component.
 *
 * This static entity is responsible for managing the camera component.
 * It provides methods to create a camera entity and update its state based on gear changes.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <memory>
#include <string>

#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/scene.hpp"

#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"

namespace TodStaticEntities {

/**
 * @class Camera
 * @brief Static camera entity for managing the camera component.
 *
 * The Camera class provides functionality to create a camera entity within a scene
 * and update the camera state based on gear changes.
 */
class Camera {
  private:
    /**
     * @brief Private default constructor.
     */
    Camera() = default;

  public:
    /**
     * @brief Creates a new camera entity.
     *
     * @param scene Shared pointer to the scene where the entity will be created.
     * @param name Name of the camera entity.
     * @param parent Parent entity to which the camera will be attached.
     * @return tod_gl::Entity The created camera entity.
     */
    static tod_gl::Entity create(std::shared_ptr<tod_gl::Scene> scene, std::string name, tod_gl::Entity parent);

    /**
     * @brief Updates the camera entity based on gear changes.
     *
     * @param msg Shared pointer to the secondary vehicle state message.
     * @param entity The camera entity to update.
     */
    static void onGearUpdate(const tod_vehicle_msgs::msg::SecondaryVehicleState::ConstSharedPtr& msg, tod_gl::Entity& entity);
};

}  // namespace TodStaticEntities

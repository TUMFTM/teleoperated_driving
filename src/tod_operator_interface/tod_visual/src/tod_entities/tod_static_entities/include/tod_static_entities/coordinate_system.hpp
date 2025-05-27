/**
 * @file coordinate_system.hpp
 * @brief Visualizes a component's location (e.g. World Frame, BaseFootPrint).
 *
 * Declares the CoordinateSystem entity used to visualize coordinate frames.
 *
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <memory>
#include <string>

#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/scene.hpp"

namespace TodStaticEntities {

/**
 * @class CoordinateSystem
 * @brief Entity for visualizing coordinate systems.
 *
 * The CoordinateSystem class provides functionality to create an entity that represents
 * a coordinate frame (e.g., World Frame, BaseFootPrint) in the scene.
 */
class CoordinateSystem {
  public:
    /**
     * @brief Creates a new CoordinateSystem entity.
     *
     * @param scene Shared pointer to the scene where the entity will be added.
     * @param name Name of the coordinate system entity.
     * @return tod_gl::Entity The created coordinate system entity.
     */
    static tod_gl::Entity create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name);

  private:
    /**
     * @brief Private default constructor.
     */
    CoordinateSystem() = default;
};

}  // namespace TodStaticEntities

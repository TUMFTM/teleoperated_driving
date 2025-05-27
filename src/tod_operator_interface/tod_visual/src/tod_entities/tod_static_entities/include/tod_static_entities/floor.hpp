/**
 * @file floor.hpp
 * @brief Represents the floor entity in the scene.
 *
 * Provides a method to create a floor entity.
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
 * @class Floor
 * @brief Static entity for creating a floor in the scene.
 */
class Floor {
  public:
    /// Creates a new floor entity.
    static tod_gl::Entity create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name,
                                 const std::string &packagePath, const tod_gl::Entity &parent);

  private:
    Floor() = default;
};

}  // namespace TodStaticEntities

/**
 * @file grid.hpp
 * @brief Renders an equally spaced grid on the floor.
 * 
 * Provides functionality to create a grid entity.
 * 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <memory>
#include <string>

#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/scene.hpp"

namespace TodStaticEntities {

/**
 * @class Grid
 * @brief Static entity for rendering a floor grid.
 */
class Grid {
  public:
    /// Creates a grid entity.
    static tod_gl::Entity create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name,
                                 const std::string &packagePath);

  private:
    Grid() = default;

    /// Initializes the grid mesh with specified spacing and size.
    static tod_gl::Mesh init_grid_mesh(const float gridSpacing, const int gridSize);
};

}  // namespace TodStaticEntities

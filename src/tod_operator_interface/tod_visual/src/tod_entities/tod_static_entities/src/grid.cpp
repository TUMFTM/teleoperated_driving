/**
 * @file grid.cpp
 * @brief renders a equally spaced grid on the floor
 * @copyright 2024 TUMFTM
 **/

#include "tod_static_entities/grid.hpp"

#include <vector>

#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"

namespace TodStaticEntities {

tod_gl::Entity Grid::create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name,
                            const std::string &packagePath) {
    tod_gl::Entity grid = scene->create_entity(name);
    unsigned int shader =
        tod_gl::ShaderSystem::create_shader_program((packagePath + "/resources/shaders/grid.vert").c_str(),
                                                  (packagePath + "/resources/shaders/grid.frag").c_str());
    auto Mesh = init_grid_mesh(1, 1000);
    grid.add_component<tod_gl::RenderableElementComponent>(shader, Mesh, GL_LINES);
    return grid;
}

tod_gl::Mesh Grid::init_grid_mesh(const float gridSpacing, const int gridSize) {
    std::vector<tod_gl::Vertex> vertices;
    float xOffset = gridSize / 2;
    float zPosition{-0.01f};
    for (int i = 0; i < gridSize; i++) {
        // Y Line
        vertices.emplace_back(glm::vec3(gridSpacing * (i - xOffset), gridSpacing * (gridSize / 2), zPosition),
                              glm::vec2(), glm::vec3(1.0f, 1.0f, 1.0f));
        vertices.emplace_back(glm::vec3(gridSpacing * (i - xOffset), gridSpacing * (-gridSize / 2), zPosition),
                              glm::vec2(), glm::vec3(1.0f, 1.0f, 1.0f));
        // X Line
        vertices.emplace_back(glm::vec3(gridSpacing * (-xOffset), gridSpacing * (i - gridSize / 2), zPosition),
                              glm::vec2(), glm::vec3(1.0f, 1.0f, 1.0f));
        vertices.emplace_back(
            glm::vec3(gridSpacing * (gridSize - xOffset), gridSpacing * (i - gridSize / 2), zPosition), glm::vec2(),
            glm::vec3(1.0f, 1.0f, 1.0f));
    }
    return tod_gl::Mesh(vertices);
}

};  // namespace TodStaticEntities

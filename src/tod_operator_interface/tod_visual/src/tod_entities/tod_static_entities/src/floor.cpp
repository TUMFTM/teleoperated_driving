/**
 * @file floor.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include "tod_static_entities/floor.hpp"

#include <vector>

#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"

namespace TodStaticEntities {

tod_gl::Entity Floor::create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name,
                             const std::string &packagePath, const tod_gl::Entity &parent) {
    tod_gl::Entity floorFor3DMouseClick = scene->create_entity(name);
    unsigned int grid_shader =
        tod_gl::ShaderSystem::create_shader_program((packagePath + "/resources/shaders/shader.vert").c_str(),
                                                  (packagePath + "/resources/shaders/shader.frag").c_str());
    std::vector<tod_gl::Vertex> vertices;
    float zPosition{-0.02f};
    vertices.emplace_back(glm::vec3(100.0f, -100.0f, zPosition));
    vertices.emplace_back(glm::vec3(-100.0f, 100.0f, zPosition));
    vertices.emplace_back(glm::vec3(100.0f, 100.0f, zPosition));
    vertices.emplace_back(glm::vec3(100.0f, -100.0f, zPosition));
    vertices.emplace_back(glm::vec3(-100.0f, -100.0f, zPosition));
    vertices.emplace_back(glm::vec3(-100.0f, 100.0f, zPosition));
    auto mesh = tod_gl::Mesh(vertices);
    floorFor3DMouseClick.add_component<tod_gl::RenderableElementComponent>(grid_shader, mesh);
    floorFor3DMouseClick.get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(0.0f, 0.0f, -0.02f));
    floorFor3DMouseClick.get_component<tod_gl::TransformComponent>().set_parent(parent);
    return floorFor3DMouseClick;
}

};  // namespace TodStaticEntities

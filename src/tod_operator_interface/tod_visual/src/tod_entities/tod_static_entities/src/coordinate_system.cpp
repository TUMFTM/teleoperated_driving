/**
 * @file coordinate_system.cpp
 * @brief CoordinateSystem Entity that visualizes a components location e.g. World Frame and BaseFootPrint
 * @copyright 2024 TUMFTM
 **/

#include "tod_static_entities/coordinate_system.hpp"

#include <vector>

#include "tod_gl/ros_interface/ros_interface.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"

namespace TodStaticEntities {

tod_gl::Entity CoordinateSystem::create(std::shared_ptr<tod_gl::Scene> scene, const std::string &name) {
    tod_gl::Entity coordinateSystem = scene->create_entity(name);
    unsigned int grid_shader = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());
    std::vector<tod_gl::Vertex> vertices;
    vertices.emplace_back(glm::vec3(0.00f, 0.0f, 0.0f), glm::vec2(), glm::vec3(1.0f, 0.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.25f, 0.0f, 0.0f), glm::vec2(), glm::vec3(1.0f, 0.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.00f, 0.0f), glm::vec2(), glm::vec3(0.0f, 1.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.25f, 0.0f), glm::vec2(), glm::vec3(0.0f, 1.0f, 0.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.0f, 0.00f), glm::vec2(), glm::vec3(0.0f, 0.0f, 1.0f));
    vertices.emplace_back(glm::vec3(0.0f, 0.0f, 0.25f), glm::vec2(), glm::vec3(0.0f, 0.0f, 1.0f));
    coordinateSystem.add_component<tod_gl::RenderableElementComponent>(grid_shader, tod_gl::Mesh(vertices), GL_LINES);
    coordinateSystem.get_component<tod_gl::RenderableElementComponent>().line_width = 5.0f;
    return coordinateSystem;
}

}  // namespace TodStaticEntities

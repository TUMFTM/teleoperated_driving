/**
 * @file path_control_points_renderer.cpp
 * @brief PathControlPointsRenderer renders the Control Points of the @ref TrajectoryGuidances control points for the Path
 * @copyright 2024 TUMFTM
 **/

#include "tod_dynamic_entities/path_control_points_renderer.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/ros_interface/subscribing_components/path_control_points_component.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/utils/utils.hpp"

namespace TodDynamicEntities {
void PathControlPointsRenderer::on_create() {
    unsigned int shaderProgram = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());

    auto Mesh = tod_gl::Mesh::non_empty_mesh();
    this->add_component<tod_gl::RenderableElementComponent>(shaderProgram, Mesh, GL_TRIANGLES);
    this->add_component<tod_gl::ExpirableComponent>(1000);
    this->add_component<tod_gl::DynamicDataComponent>();

    // Indices shouldn't be empty at the initialization.
    this->get_component<tod_gl::RenderableElementComponent>().meshes.front().indices = {0, 1, 2, 0, 2, 3};
    this->get_component<tod_gl::RenderableElementComponent>().line_width = 3.0f;
    this->get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(0.f, 0.f, 0.05f));
    this->get_component<tod_gl::TransformComponent>().is_map_frame = true;

}

void PathControlPointsRenderer::on_destroy() {}

void PathControlPointsRenderer::on_update(float) {
    auto &_stateManager = tod_gl::StateManager::get_instance();
    auto &dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto &renderable = this->get_component<tod_gl::RenderableElementComponent>();
    auto &mesh = renderable.meshes.front();

    if (_stateManager.contains_entity("PathControlPointsRenderer")) {
        if (!_stateManager.should_render_entity("PathControlPointsRenderer")) {
            auto &dynamic = this->get_component<tod_gl::DynamicDataComponent>();
            std::lock_guard<std::mutex> lock(*dynamic.mutex);
            dynamic.has_new_data = true;
            mesh.vertices.clear();
            mesh.indices.clear();
            return;
        }
    }

    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    this->get_component<tod_gl::ExpirableComponent>().restamp();

    mesh.vertices.clear();
    mesh.indices.clear();

    auto entity = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!entity.has_component<tod_gl::PathControlPointsComponent>()) {
        return;
    }

    const auto& pathControlPointsEnt = entity.get_component<tod_gl::PathControlPointsComponent>();
    const auto &PathControlPoints = pathControlPointsEnt.getPoints();

    size_t i = 0;
    for (const auto &point : PathControlPoints.points) {
        glm::vec4 position(point.x, point.y, 0.15f, 1.0f);

        auto relativePositionGame = tod_gl::TransformSystem::get_instance()->to_game_coordinates(position);

        glm::vec3 color;
        if (point.validated) {
            color = glm::vec3(0.3f, 0.733f, 0.1f); // green 
        }else if (point.sent) {
            color = glm::vec3(1.0f, .87f, .13f);
        } else {
            color= glm::vec3(1.0f, 0.0f, 0.0f);
        }

        tod_gl::Utils::add_quad_for_click(mesh, relativePositionGame, _quad_size, color);
    }
}
}  // namespace TodDynamicEntities

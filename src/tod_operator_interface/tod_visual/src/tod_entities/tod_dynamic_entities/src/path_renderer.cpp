/**
 * @file path_renderer.cpp
 * @brief PathRenderer path the vehicle will try to drive during @ref TrajectoryGuidances as well as equally spaced tics to grasp the depth progress of the AV
 * @copyright 2024 TUMFTM
 **/

#include "tod_dynamic_entities/path_renderer.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/ros_interface/subscribing_components/path_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/trajectory_guidance_state_component.hpp"

#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/utils/utils.hpp"

#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/path.hpp"

namespace TodDynamicEntities {

template<typename PathLike>
void PathRenderer<PathLike>::on_create()
{
    unsigned int shaderProgram = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());

    tod_gl::Mesh Mesh = tod_gl::Mesh::non_empty_mesh();
    this->template add_component<tod_gl::RenderableElementComponent>(shaderProgram, Mesh, GL_TRIANGLES);
    this->template add_component<tod_gl::ExpirableComponent>(1000);
    this->template add_component<tod_gl::DynamicDataComponent>();

    // Init tod_gl::RenderableElementComponent
    // Indices shouldn't be empty at the initialization.
    this->template get_component<tod_gl::RenderableElementComponent>().meshes.front().indices = {0, 1, 2, 0, 2, 3};
    this->template get_component<tod_gl::RenderableElementComponent>().line_width = 3.0f;

    // Init tod_gl::TransformComponent
    this->template get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(0.f, 0.f, 0.05f));
    this->template get_component<tod_gl::TransformComponent>().is_map_frame = true;

}

template<typename PathLike>
void PathRenderer<PathLike>::on_destroy() {}

template<typename PathLike>
void PathRenderer<PathLike>::on_update(float) {
    auto &_stateManager = tod_gl::StateManager::get_instance();
    auto &dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto &renderable = this->get_component<tod_gl::RenderableElementComponent>();
    auto &mesh = renderable.meshes.front();

    if (_stateManager.contains_entity("PathRenderer")) {
        if (!_stateManager.should_render_entity("PathRenderer")) {
            auto &dynamic = this->template get_component<tod_gl::DynamicDataComponent>();
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

    if (!entity.template has_component<tod_gl::PathComponent<PathLike>>()){ return; }

    if (!entity.template has_component<tod_gl::TrajectoryGuidanceStateComponent>()){ return; }


    auto &tgControl = entity.template get_component<tod_gl::TrajectoryGuidanceStateComponent>();

    const float maxVel = tgControl.target_velocity_; 


    auto &path = entity.template get_component<tod_gl::PathComponent<PathLike>>();
    auto PathPoints = path.get_path().points;
    if (PathPoints.size() <= 1)
        return;

    // TODO  use a parameter here 
    const float vehicleWidth = 2.3f;


    std::vector<glm::vec3> renderPoints;
    std::vector<glm::vec3> pathColors;
    std::vector<glm::vec3> pathValidationColors;


    float min_v = std::numeric_limits<float>::max();
    if (!PathPoints.empty()) {
        min_v = std::min_element(PathPoints.begin(), PathPoints.end(),
            [](const auto& a, const auto& b) { 
                return a.v_max_curv < b.v_max_curv; 
            })->v_max_curv;
    }

    const float tickLength = 0.3;
    const float tickSpacing = 5.0f;
    for (size_t i = 0; i < PathPoints.size(); ++i)
    {

        float z_pos;
        glm::vec3 statusColor;

        if (PathPoints[i].validated) {
            z_pos =  0.12f;
            statusColor = glm::vec3(0.3f, 0.733f, 0.1f); // green 
        } else if (PathPoints[i].sent) {
            z_pos =  0.12f;
            statusColor = _color;
        } else {
            z_pos = 0.1f;
            statusColor = glm::vec3(1.0f); // white
        }
        
        glm::vec3 pos = glm::vec3(PathPoints[i].pose.position.x, PathPoints[i].pose.position.y, z_pos);        
        glm::vec3 gamePos = tod_gl::TransformSystem::get_instance()->to_game_coordinates(pos);
        renderPoints.push_back(gamePos);
        
        // validated paths (rendered above)
        glm::vec3 curvature_color;
        if (PathPoints[i].pose.position.z >= 1) {
            curvature_color = glm::vec3(1.0f, 0.0f, 0.0f);
        } else if (PathPoints[i].validated) {
            float t = (PathPoints[i].v_max_curv - min_v) / (maxVel - min_v);
            curvature_color = glm::vec3{1.0f-t, t, 0.0f};
        } else {
            curvature_color = glm::vec3(1.0f);
        }
        pathColors.push_back(curvature_color);
        pathValidationColors.push_back(statusColor);
    }

    if (renderPoints.size() >= 2)
    {
        if (PathPoints.back().sent) {
            tod_gl::Utils::render_path_lines_simple(renderPoints, vehicleWidth,  mesh, pathValidationColors);
        }
        else {
            tod_gl::Utils::render_path_lines(renderPoints, vehicleWidth, pathColors, tickLength, tickSpacing, mesh, pathValidationColors);
        }
    }

}



template class PathRenderer<tod_trajectory_guidance_msgs::msg::Path>;
template class PathRenderer<tod_trajectory_guidance_msgs::msg::Trajectory>;

}  // namespace TodDynamicEntities
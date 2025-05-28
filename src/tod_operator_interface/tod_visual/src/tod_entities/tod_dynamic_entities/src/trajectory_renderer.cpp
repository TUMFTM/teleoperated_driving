/**
 * @file trajectory_renderer.cpp
 * @brief TrajectoryRenderer renders a trajectory with full triangulation 
 * @copyright 2024 TUMFTM 
 **/

#include "tod_dynamic_entities/trajectory_renderer.hpp"

#include "tod_gl/renderer/data_container.hpp"
#include "glm/gtx/string_cast.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/ros_interface/subscribing_components/trajectory_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/automation_status_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/trajectory_component.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/entity.hpp"

#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/utils/utils.hpp"

namespace TodDynamicEntities {

void TrajectoryRenderer::on_create() {
    unsigned int shader_program = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());

    tod_gl::Mesh mesh = tod_gl::Mesh::non_empty_mesh();
    this->add_component<tod_gl::RenderableElementComponent>(shader_program, mesh, GL_TRIANGLES);

    this->add_component<tod_gl::ExpirableComponent>(1000);
    this->add_component<tod_gl::DynamicDataComponent>();

    // Indices shouldn't be empty at the initialization.
    this->get_component<tod_gl::RenderableElementComponent>().meshes.front().indices = {0, 1, 2, 0, 1, 2};
    this->get_component<tod_gl::RenderableElementComponent>().opaque = false;

    this->get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(0.f, 0.f, 0.05f));
    this->get_component<tod_gl::TransformComponent>().is_map_frame = true;
}

void TrajectoryRenderer::on_destroy() {}

void TrajectoryRenderer::on_update(float) {
    auto& state_manager = tod_gl::StateManager::get_instance();
    // Check if TrajectoryRenderer is toggled on
    if (!state_manager.get_toggle_setting("enable_trajectory")){
        return;
    }

    auto& dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto& expirable = this->get_component<tod_gl::ExpirableComponent>();
    auto& renderable = this->get_component<tod_gl::RenderableElementComponent>();

    // Check if TrajectoryRenderer is in scene and should be displayed in current control mode
    if (state_manager.contains_entity("TrajectoryRenderer")) {
        if (!state_manager.should_render_entity("TrajectoryRenderer")) {
            std::lock_guard<std::mutex> lock(*dynamic.mutex);
            dynamic.has_new_data = true;
            std::for_each(renderable.meshes.begin(), renderable.meshes.end(), [](tod_gl::Mesh& mesh) {
                mesh.vertices.clear();
                mesh.indices.clear();
            });
            return;
        }
    }

    auto& mesh = renderable.meshes.front();
    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    expirable.restamp();

    mesh.vertices.clear();
    mesh.indices.clear();

    // Check if data sources are available
    auto subscription_manager = get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!subscription_manager.has_component<tod_gl::TrajectoryComponent>()) {
        return;
    }
    if (!subscription_manager.has_component<tod_gl::AutomationStatusComponent>()) {
        return;
    }

    // Get automation state    
    tod_gl::AutomationStatusComponent& automation_state_comp = subscription_manager.get_component<tod_gl::AutomationStatusComponent>();
    std::string automation_state = automation_state_comp.get_automation_status_string();

    // Get automation trajectory - do not render if only consisting of one point
    tod_gl::TrajectoryComponent& trajectory_comp = subscription_manager.get_component<tod_gl::TrajectoryComponent>();
    const std::vector<tod_gl::TrajectoryPoint>& trajectory = trajectory_comp.get_trajectory();
    if (trajectory.size() <= 1)
        return;

    // Get start and end point of the trajectory and their distance in the xy-plane
    // z-values are set to 0.0 since we only render the trajectory in the xy-plane
    glm::vec3 start_pos = {trajectory.at(0).pose.position.x, 
                           trajectory.at(0).pose.position.y, 
                           0.f};
    glm::vec3 end_pos = {trajectory.at(trajectory.size() - 1).pose.position.x, 
                         trajectory.at(trajectory.size() - 1).pose.position.y, 
                         0.f};

    float max_dist = glm::distance(start_pos, end_pos);
  
    // Create trajectory mesh
    for (int i = 0; i < trajectory.size() - _step_size; i += _step_size) {
        // Current point in xy-plane
        glm::vec3 current_pos = {trajectory.at(i).pose.position.x, 
                                 trajectory.at(i).pose.position.y, 
                                 0.f};
        glm::vec3 current_pos_game = tod_gl::TransformSystem::get_instance()->to_game_coordinates(current_pos);

        const auto& orientation = trajectory.at(i).pose.orientation;
        glm::vec3 right = orientation * glm::vec3(0.f, 1.f, 0.f);
        
        // Select color
        glm::vec3 color;
        if (automation_state != "REMOTE") 
        {
            float current_dist = glm::distance(start_pos, current_pos);

            // better to do the coloring according to time but current data doesn't have time info
            float gradient = current_dist / max_dist;

            color = glm::vec3(gradient, 1.f - gradient, 0.f);

        } else 
        {
            color = glm::vec3(0.35f, 0.35f, 0.35f);
        }

        tod_gl::Utils::triangulate_for_line(current_pos_game, right, color, mesh, _line_width);
    }
}

}  // namespace TodDynamicEntities

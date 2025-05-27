/**
 * @file driving_lane_renderer.cpp
 * @brief DrivingLaneRenderer renders a projected driving lane based on the wheel position
 * @copyright 2024 TUMFTM
 **/

#include "tod_dynamic_entities/driving_lane_renderer.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/ros_interface/subscribing_components/driving_lane_component.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/utils/utils.hpp"

namespace TodDynamicEntities {

void DrivingLaneRenderer::on_create() {
    unsigned int shaderProgram = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());

    tod_gl::Mesh mesh = tod_gl::Mesh::non_empty_mesh();
    this->add_component<tod_gl::RenderableElementComponent>(shaderProgram, mesh, GL_TRIANGLES);
    this->add_component<tod_gl::ExpirableComponent>(1000);
    this->add_component<tod_gl::DynamicDataComponent>();

    // Init RenderableElementComponent
    // Indices shouldn't be empty at the initialization.
    this->get_component<tod_gl::RenderableElementComponent>().meshes.front().indices = {0, 1, 2, 0, 2, 3};
    this->get_component<tod_gl::RenderableElementComponent>().line_width = line_width;

    // Init TransformComponent
    this->get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(0.f, 0.f, 0.05f));
    this->get_component<tod_gl::TransformComponent>().set_parent(this->get_bounded_scene().find_entity_with_tag("base_footprint"));

}

void DrivingLaneRenderer::on_destroy() {}

void DrivingLaneRenderer::on_update(float) {
    
    auto &_stateManager = tod_gl::StateManager::get_instance();

    if (!_stateManager.get_toggle_setting("enable_driving_lane")){
        return;
    }

    auto &dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto &renderable = this->get_component<tod_gl::RenderableElementComponent>();
    auto &mesh = renderable.meshes.front();

    if (_stateManager.contains_entity("DrivingLaneRenderer")) {
        if (!_stateManager.should_render_entity("DrivingLaneRenderer")) {
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

    auto subManager = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!subManager.has_component<tod_gl::DrivingLaneComponentFrontLeft>() || !subManager.has_component<tod_gl::DrivingLaneComponentFrontRight>() ||
         !subManager.has_component<tod_gl::DrivingLaneComponentRearLeft>() || !subManager.has_component<tod_gl::DrivingLaneComponentRearRight>()) {
        return;
    }

    auto& drivinPathFrontLeft = subManager.get_component<tod_gl::DrivingLaneComponentFrontLeft>();
    auto& drivinPathFrontRight = subManager.get_component<tod_gl::DrivingLaneComponentFrontRight>();

    auto& drivinPathRearLeft = subManager.get_component<tod_gl::DrivingLaneComponentRearLeft>();
    auto& drivinPathRearRight = subManager.get_component<tod_gl::DrivingLaneComponentRearRight>();

    std::vector<std::reference_wrapper<tod_gl::DrivingLaneComponent>> drivingLanes = {
        drivinPathFrontLeft,
        drivinPathFrontRight,
        drivinPathRearLeft,
        drivinPathRearRight
    };

    std::vector<std::vector<glm::vec3>> allPaths;
    // allPaths.reserve(drivingLanes.size());
    

    for (const auto& drivingLane : drivingLanes) {
        if (!drivingLane.get().has_received_path()) {
            continue;
        } 
        std::vector<glm::vec3> currentPath;
        const auto& path = drivingLane.get().get_path();
        currentPath.reserve(path.poses.size());  
        
        for (const auto& pose : path.poses) {
            const auto& pos = pose.pose.position;
            currentPath.push_back({pos.x, pos.y, pos.z});
        }
        
        if (!currentPath.empty()) {
            allPaths.push_back(std::move(currentPath));
        }
    }

    if (!allPaths.empty()) {
        
        tod_gl::Utils::render_multiple_paths(allPaths, line_width, mesh);
    }
}
}  // namespace TodDynamicEntities
/**
 * @file point_cloud_renderer.cpp
 * @brief PointCloudRenderer renders a pointcloud in a single color around the vehicles base_footprint
 * @copyright 2024 TUMFTM
 **/

#include "tod_dynamic_entities/point_cloud_renderer.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/ros_interface/subscribing_components/point_cloud_component.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/utils/utils.hpp"

#include <pcl/point_types.h>

namespace TodDynamicEntities {

void PointCloudRenderer::on_create() {
    unsigned int shaderProgram = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());

    auto mesh = tod_gl::Mesh::non_empty_mesh();
    this->add_component<tod_gl::RenderableElementComponent>(shaderProgram, mesh, GL_POINTS);
    this->get_component<tod_gl::RenderableElementComponent>().point_size = 5.0f;

    this->add_component<tod_gl::ExpirableComponent>(1000);
    this->add_component<tod_gl::DynamicDataComponent>();

    this->get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(0.f, 0.f, 0.05f));
}

void PointCloudRenderer::on_destroy() {}

void PointCloudRenderer::on_update(float deltaTime) {
    auto &_stateManager = tod_gl::StateManager::get_instance();
    
    if (!_stateManager.get_toggle_setting("enable_point_cloud")){
        return;
    }

    auto &dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto &renderable = this->get_component<tod_gl::RenderableElementComponent>();

    if (_stateManager.contains_entity("PointCloudRenderer")) {
        if (!_stateManager.should_render_entity("PointCloudRenderer")) {
            std::lock_guard<std::mutex> lock(*dynamic.mutex);
            dynamic.has_new_data = true;

            std::for_each(renderable.meshes.begin(), renderable.meshes.end(), [](tod_gl::Mesh &mesh) {
                mesh.vertices.clear();
                mesh.indices.clear();
            });
            return;
        }
    }
    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    dynamic.has_new_data = true;
    this->get_component<tod_gl::ExpirableComponent>().restamp();

    auto &mesh = renderable.meshes.front();
    mesh.vertices.clear();
    mesh.indices.clear();

    render_lidar_point_cloud(mesh);
}

void PointCloudRenderer::render_lidar_point_cloud(tod_gl::Mesh &mesh) {
    auto entity = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!entity.has_component<tod_gl::PointCloudComponent>()) {
        return;
    }
    auto &pointcloud = entity.get_component<tod_gl::PointCloudComponent>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pointcloud.get_point_cloud();

    if (!cloud || cloud->empty()) {
        // No valid point cloud data, clear the mesh
        mesh.vertices.clear();
        return;
    }

    mesh.vertices.clear();
    mesh.vertices.reserve(cloud->points.size());

    const glm::vec3 color(1.0f, 0.0f, 1.0f);  // Single color for all points (white)

    for (const auto &point : cloud->points) {
        mesh.vertices.emplace_back(glm::vec3(point.x, point.y, point.z), glm::vec2(0, 0), color);
    }
}

}  // namespace TodDynamicEntities

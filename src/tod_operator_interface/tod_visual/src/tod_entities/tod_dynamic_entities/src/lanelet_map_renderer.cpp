/**
 * @file lanelet_map_renderer.cpp
 * @brief LaneletMapRenderer renders a lanelet2 map
 * @copyright 2024 TUMFTM
 **/

#include "tod_dynamic_entities/lanelet_map_renderer.hpp"

#include <iostream>
#include <ostream>

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/renderer/renderer.hpp"
#include "tod_gl/ros_interface/subscribing_components/trajectory_component.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/utils/utils.hpp"

#include "tod_dynamic_entities/utils/lanelet_mgrs_projector.hpp"

#include <glm/gtx/string_cast.hpp>

namespace TodDynamicEntities {

void LaneletMapRenderer::on_create() {
    
    size_t valid_file  = map_path_.rfind(".osm");
    if (valid_file != std::string::npos) {
        load_map();
    } else {
        std::cout << "Lanelet file does not exist. If map should be used insert a valid map in params.yaml" << std::endl;
        return;
    }

    if (!map_ptr_) {
        return;
    }

    std::cout << "Lanelet successfully loaded" << std::endl;

    // Calc world to game offset
    calc_offset();

    unsigned int shaderProgram = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());

    std::vector<tod_gl::Mesh> meshes{};
    this->add_component<tod_gl::RenderableElementComponent>(shaderProgram, meshes);
    this->add_component<tod_gl::DynamicDataComponent>();
    this->get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(0.0f, 0.0f, 0.05f));
    this->get_component<tod_gl::TransformComponent>().is_map_frame = true;
}

void LaneletMapRenderer::on_update(float) {

    if (!map_ptr_) {
        return;
    }

    auto& state_manager = tod_gl::StateManager::get_instance();
    auto& dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto& renderable = this->get_component<tod_gl::RenderableElementComponent>();
    
    // Check if lanelet should be rendered in current state and if it is toggled on
    if (state_manager.contains_entity("Lanelet")) {
        if (!state_manager.should_render_entity("Lanelet") || !state_manager.get_toggle_setting("enable_lanelet_map")) {
            std::lock_guard<std::mutex> lock(*dynamic.mutex);
            dynamic.has_new_data = true;
            std::for_each(renderable.meshes.begin(), renderable.meshes.end(), [](tod_gl::Mesh& Mesh) {
                Mesh.vertices.clear();
                Mesh.indices.clear();
            });
            lanelet_mesh_empty_ = true;
            return;
        }
    }

    // Generate lanelet meshes
    if (lanelet_mesh_empty_) {
        if (!map_ptr_)
            return;
        try {
            std::lock_guard<std::mutex> lock(*dynamic.mutex);
            dynamic.has_new_data = true;
            update_meshes(renderable.meshes);
            tod_gl::Renderer::generate_meshes(renderable);
            lanelet_mesh_empty_ = false;
        } catch (const std::exception& e) {
            std::cerr << "Exception caught while locking mutex: " << e.what() << std::endl;
        }
    }
}

void LaneletMapRenderer::load_map() {
    lanelet::Origin origin({map_origin_[0], map_origin_[1], map_origin_[2]});
    std::shared_ptr<lanelet::Projector> mgrsProjectorPtr_ = std::make_shared<lanelet::projection::MGRSProjector>(origin);
    try {
        lanelet::ErrorMessages* err{};
        lanelet::LaneletMapPtr nonConstMapPtr_ = lanelet::load(map_path_, *mgrsProjectorPtr_, err);
        lanelet::Point3d origin = *(nonConstMapPtr_->pointLayer.begin());
        map_ptr_ = nonConstMapPtr_;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in load_map: " << e.what() << std::endl;
        map_ptr_ =  nullptr;
    } catch (...) {
        std::cerr << "Unknown exception caught in load_map" << std::endl;
        map_ptr_ =  nullptr;
    }
}

void LaneletMapRenderer::calc_offset(){
    double sum_x = 0.0;
    double sum_y = 0.0;
    size_t num_points = 0;
    for (const auto& point : map_ptr_->pointLayer) {
        sum_x += point.x();
        sum_y += point.y();
        ++num_points;
    }

    double offset_x = sum_x / num_points;
    double offset_y = sum_y / num_points;
    std::cout << "World to Game offset set to " << std::to_string(offset_x) << "  " << std::to_string(offset_y) << std::endl;
    tod_gl::TransformSystem::get_instance()->set_world_offset(glm::vec3(offset_x, offset_y, 0.0f));
}

void LaneletMapRenderer::update_meshes(std::vector<tod_gl::Mesh>& meshes) {
    std::cout << "update meshes" << std::endl;
    const lanelet::LaneletLayer& laneletLayer = map_ptr_->laneletLayer;
    std::map<lanelet::Id, int> lineCounter = {};

    for (const lanelet::ConstLanelet& lanelet : map_ptr_->laneletLayer) {
        if (lineCounter.find(lanelet.leftBound().id()) == lineCounter.end())
            lineCounter[lanelet.leftBound().id()] = 1;
        else
            lineCounter[lanelet.leftBound().id()]++;

        if (lineCounter.find(lanelet.rightBound().id()) == lineCounter.end())
            lineCounter[lanelet.rightBound().id()] = 1;
        else
            lineCounter[lanelet.rightBound().id()]++;
    }
    std::cout << "update meshes lines" << std::endl;

    for (const lanelet::ConstLanelet& lanelet : map_ptr_->laneletLayer) {
        glm::vec3 white(1, 1, 1);
        glm::vec3 yellow(0.8f, 0.8f, 0.2f);

        glm::vec3 color = lineCounter[lanelet.leftBound().id()] == 1 ? white : yellow;
        add_line_to_mesh(lanelet.leftBound(), meshes, color);
        color = lineCounter[lanelet.rightBound().id()] == 1 ? white : yellow;
        add_line_to_mesh(lanelet.rightBound(), meshes, color);
    }
}

void LaneletMapRenderer::add_line_to_mesh(const lanelet::ConstLineString3d& linestring, 
                                          std::vector<tod_gl::Mesh>& meshes,
                                          glm::vec3& color) {
    tod_gl::Mesh Mesh = tod_gl::Mesh::non_empty_mesh();
    Mesh.vertices.clear();
    for (int pointIdx = 0; pointIdx < linestring.size() - 1; pointIdx++) {
        const lanelet::ConstPoint3d& point = linestring[pointIdx];
        const lanelet::ConstPoint3d& next_point = linestring[pointIdx + 1];
        glm::vec3 posVec = tod_gl::TransformSystem::get_instance()->to_game_coordinates(glm::vec3((float)point.x(), (float)point.y(), 0.0f));
        glm::vec3 nextPosVec = tod_gl::TransformSystem::get_instance()->to_game_coordinates(glm::vec3((float)next_point.x(), (float)next_point.y(), 0.0f));
        glm::vec3 direction = nextPosVec - posVec;
        // Normalized Direction X Global Up vector to get the right vector.
        // Note: Assumes the area is flat. Should use the local normal vector instead if we
        // want to add elevation.
        glm::vec3 right = glm::normalize(glm::cross(direction, glm::vec3(0, 0, 1)));

        tod_gl::Utils::triangulate_for_line(posVec, right, color, Mesh, line_width_);

        if (pointIdx == linestring.size() - 2) {
            tod_gl::Utils::triangulate_for_line(nextPosVec, right, color, Mesh, line_width_);
        }
    }
    meshes.emplace_back(Mesh);
}

}  // namespace TodDynamicEntities
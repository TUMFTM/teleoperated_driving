/**
 * @file scene_serialization.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/scene/scene_serialization.hpp"

#include "tod_gl/scene/components.hpp"

using namespace YAML;

namespace tod_gl::serialization {

bool deserialize_entities(const std::string& vehicle_config_path, const std::string& vehicle_id, std::vector<Entity>& entities) {
    std::string filepath = get_scene_file_path_from(vehicle_config_path, vehicle_id);
    if (!file_exists(filepath)) {
        serialize_entities(vehicle_config_path, vehicle_id, entities);
        return false;

    }

    YAML::Node data = YAML::LoadFile(filepath);
    for (auto& entity : entities) {
        std::string tag = entity.get_component<TagComponent>().tag;
        auto entityConfig = data[tag];
        if (entityConfig) {
            if (entity.has_component<TransformComponent>()) {
                auto tfConfig = entityConfig["TransformComponent"];
                if (tfConfig) {
                    auto& tc = entity.get_component<TransformComponent>();
                    tc.translation = tfConfig["Translation"].as<glm::vec3>();
                    tc.rotation = tfConfig["Rotation"].as<glm::vec3>();
                    tc.scale = tfConfig["Scale"].as<glm::vec3>();
                }
            }

            // if (entity.has_component<RenderableElementComponent>()) {
            //     auto rcConfig = entityConfig["RenderableElementComponent"];
            //     if (rcConfig) {
            //         auto &rc = entity.get_component<RenderableElementComponent>();
            //         rc.static_show = rcConfig["StaticShow"].as<bool>();
            //         rc.render_mode = rcConfig["RenderMode"].as<GLenum>();
            //         rc.line_width = rcConfig["line_width"].as<float>();
            //         rc.point_size = rcConfig["point_size"].as<float>();
            //     }
            // }

            if (entity.has_component<VideoComponent>()) {
                auto pcConfig = entityConfig["VideoComponent"];
                if (pcConfig) {
                    auto& pc = entity.get_component<VideoComponent>();
                    pc.projection_mode = VideoComponent::ProjectionModeType(pcConfig["ProjectionMode"].as<int>());
                    pc.ground_plane_radius_min = pcConfig["GroundPlaneRadiusMin"].as<float>();
                    pc.sphere_radius = pcConfig["SphereRadius"].as<float>();
                    pc.sphere_longitude_min = pcConfig["SphereLongitudeMin"].as<float>();
                    pc.sphere_longitude_max = pcConfig["SphereLongitudeMax"].as<float>();
                    pc.sphere_latitude_min = pcConfig["SphereLatitudeMin"].as<float>();
                    pc.sphere_latitude_max = pcConfig["SphereLatitudeMax"].as<float>();
                }
            }
        }
    }
    std::cout << "serialzed" << std::endl;

    return true;
}

bool deserialize_video_component(const std::string& vehicle_config_path, const std::string& vehicle_id,const std::string& tag ,VideoComponent& VideoComp, TransformComponent& TransComp ) {
    std::string filepath = get_scene_file_path_from(vehicle_config_path, vehicle_id);
    //TODO: Add Serialization Variant for single videocomponent
    if (!file_exists(filepath)) {
        std::cout<< "VIDEO CONFIG FILE NOT FOUND: " << filepath << std::endl;
        // serialize_entities(vehicle_config_path, vehicle_id, entities);  /home/tum/wsp/install/tod_visual/share/tod_visual/edgar/visual-video.yaml
        //                                                                /home/tum/wsp/install/tod_visual/share/tod_visual/config/vehicle_config/edgar/visual-video.yaml
        return false;

    }

    YAML::Node data = YAML::LoadFile(filepath);
    
    auto entityConfig = data[tag];
    if (entityConfig) {
            auto tfConfig = entityConfig["TransformComponent"];
            if (tfConfig) {
                TransComp.translation = tfConfig["Translation"].as<glm::vec3>();
                TransComp.rotation = tfConfig["Rotation"].as<glm::vec3>();
                TransComp.scale = tfConfig["Scale"].as<glm::vec3>();
            }
            auto pcConfig = entityConfig["VideoComponent"];
            if (pcConfig) {
                VideoComp.projection_mode = VideoComponent::ProjectionModeType(pcConfig["ProjectionMode"].as<int>());
                VideoComp.ground_plane_radius_min = pcConfig["GroundPlaneRadiusMin"].as<float>();
                VideoComp.sphere_radius = pcConfig["SphereRadius"].as<float>();
                VideoComp.sphere_longitude_min = pcConfig["SphereLongitudeMin"].as<float>();
                VideoComp.sphere_longitude_max = pcConfig["SphereLongitudeMax"].as<float>();
                VideoComp.sphere_latitude_min = pcConfig["SphereLatitudeMin"].as<float>();
                VideoComp.sphere_latitude_max = pcConfig["SphereLatitudeMax"].as<float>();
            }
        }
    return true;
}


void serialize_entities(const std::string& vehicle_config_path, const std::string& vehicle_id, std::vector<Entity>& entities) {
    YAML::Emitter out;
    out << YAML::BeginMap;  // File

    for (auto& entity : entities) {
        out << YAML::Key << entity.get_component<TagComponent>().tag;
        out << YAML::BeginMap;  // Entity
        if (entity.has_component<TransformComponent>()) {
            out << YAML::Key << "TransformComponent";
            out << YAML::BeginMap;  // TransformComponent
            const auto& tc = entity.get_component<TransformComponent>();
            out << YAML::Key << "Translation" << YAML::Value << tc.translation;
            out << YAML::Key << "Rotation" << YAML::Value << tc.rotation;
            out << YAML::Key << "Scale" << YAML::Value << tc.scale;
            out << YAML::EndMap;  // TransformComponent
        }

        // if (entity.has_component<RenderableElementComponent>()) {
        //     out << YAML::Key << "RenderableElementComponent";
        //     out << YAML::BeginMap; // RenderableElementComponent
        //     const auto& rc = entity.get_component<RenderableElementComponent>();
        //     out << YAML::Key << "StaticShow" << YAML::Value << rc.static_show;
        //     out << YAML::Key << "RenderMode" << YAML::Value << rc.render_mode;
        //     out << YAML::Key << "line_width" << YAML::Value << rc.line_width;
        //     out << YAML::Key << "point_size" << YAML::Value << rc.point_size;
        //     out << YAML::EndMap; // RenderableElementComponent
        // }

        if (entity.has_component<VideoComponent>()) {
            out << YAML::Key << "VideoComponent";
            out << YAML::BeginMap;  // VideoComponent
            const auto& pc = entity.get_component<VideoComponent>();
            out << YAML::Key << "ProjectionMode" << YAML::Value << pc.projection_mode;
            out << YAML::Key << "ground_plane_radius_min" << YAML::Value << pc.ground_plane_radius_min;
            out << YAML::Key << "sphere_radius" << YAML::Value << pc.sphere_radius;
            out << YAML::Key << "sphere_longitude_min" << YAML::Value << pc.sphere_longitude_min;
            out << YAML::Key << "sphere_longitude_max" << YAML::Value << pc.sphere_longitude_max;
            out << YAML::Key << "sphere_latitude_min" << YAML::Value << pc.sphere_latitude_min;
            out << YAML::Key << "sphere_latitude_max" << YAML::Value << pc.sphere_latitude_max;
            out << YAML::EndMap;  // VideoComponent
        }

        out << YAML::EndMap;  // Entity
    }
    out << YAML::EndMap;      // File

    std::string filepath = get_scene_file_path_from(vehicle_config_path, vehicle_id);
    std::ofstream fout(filepath);
    fout << out.c_str();
}

} // namespace tod_gl::serialization
/**
 * @file scene_serialization.hpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/components.hpp"


#include "yaml-cpp/yaml.h"

namespace tod_gl::serialization {

bool deserialize_entities(const std::string &vehicle_config_path, const std::string &vehicle_id, std::vector<Entity> &entities);

bool deserialize_video_component(const std::string& vehicle_config_path, const std::string& vehicle_id,const std::string& tag ,VideoComponent& VideoComp, TransformComponent& TransComp);
// void SerializeVideoCompoenent(const std::string &pkgPath, const std::string &vehicleID, std::vector<VideoComponent> &entities);

void serialize_entities(const std::string &pkgPath, const std::string &vehicleID, std::vector<Entity> &entities);

static std::string get_scene_file_path_from(const std::string &vehicle_config_path, const std::string &vehicle_id) {
    return std::string(vehicle_config_path + "/vehicle_config/" + vehicle_id + "/visual-video.yaml");
}

static bool file_exists(const std::string &filepath) {
    return std::ifstream(filepath).good();
}

} // namespace tod_gl::serialization

namespace YAML {

static Emitter &operator<<(YAML::Emitter &out, const glm::vec3 &v) {
    out << YAML::Flow;
    out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
    return out;
}

static Emitter &operator<<(YAML::Emitter &out, const glm::vec4 &v) {
    out << YAML::Flow;
    out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
    return out;
}

template <>
struct convert<glm::vec3> {
    static Node encode(const glm::vec3 &rhs) {
        Node node;
        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        node.SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    static bool decode(const Node &node, glm::vec3 &rhs) {
        if (!node.IsSequence() || node.size() != 3)
            return false;

        rhs.x = node[0].as<float>();
        rhs.y = node[1].as<float>();
        rhs.z = node[2].as<float>();
        return true;
    }
};

template <>
struct convert<glm::vec4> {
    static Node encode(const glm::vec4 &rhs) {
        Node node;
        node.push_back(rhs.x);
        node.push_back(rhs.y);
        node.push_back(rhs.z);
        node.push_back(rhs.w);
        node.SetStyle(EmitterStyle::Flow);
        return node;
    }

    static bool decode(const Node &node, glm::vec4 &rhs) {
        if (!node.IsSequence() || node.size() != 4)
            return false;

        rhs.x = node[0].as<float>();
        rhs.y = node[1].as<float>();
        rhs.z = node[2].as<float>();
        rhs.w = node[3].as<float>();
        return true;
    }
};

} // namespace YAML

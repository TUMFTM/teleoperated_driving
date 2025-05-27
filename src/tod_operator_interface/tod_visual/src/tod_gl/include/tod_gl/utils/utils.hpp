/**
 * @file utils.hpp
 * @brief Rendering Utils
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <vector>
#include <string>

#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/ros_interface/ros_interface.hpp"

#include "yaml-cpp/yaml.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"

namespace tod_gl {

class Utils {
  public:
    /*
     * Given location, direction, color vectors and a line width value,
     * creates two vertices on the mesh, connects them to the last
     * created vertices with 2 triangles
     *
     * 1--3
     * |//|  triangulation of 1 line segment.
     * 0--2
     *
     * TODO: For a better performance this could be done in shaders.
     */
    static void triangulate_for_line(glm::vec3& posVec, glm::vec3& right, glm::vec3& color, Mesh& mesh, float lineWidth);

    static void render_path_lines(
    const std::vector<glm::vec3>& pathPoints,
    float pathWidth,
    const std::vector<glm::vec3>& colors,
    const float tickLength,
    const float tickSpacing,
    Mesh& mesh,
    const std::vector<glm::vec3> sideColor);

    static void add_quad_for_click(Mesh& mesh, const glm::vec3& position, float size, const glm::vec3& color);

    static void render_multiple_paths(const std::vector<std::vector<glm::vec3>>& pathPoints,
                               float lineWidth,
                               Mesh& mesh);

    static void render_path_lines_simple(
        const std::vector<glm::vec3>& pathPoints,
        float pathWidth,
        Mesh& mesh,
        const std::vector<glm::vec3> sideColor);


    template <typename T>
    static T get_setting(const std::string& key, const T& defaultValue = T()) {
        YAML::Node config = YAML::LoadFile(RosInterface::get_package_path() + "/config/tod_visual-settings.yaml");
        if (config[key]) {
            return config[key].as<T>();
        }

        return defaultValue;
    }

    static std::string get_Topic(const std::string& key) {
        std::string vehicle = Utils::get_setting<std::string>("vehicle", "carla");
        YAML::Node config = YAML::LoadFile(RosInterface::get_package_path() + "/config/" + vehicle + "-config.yaml");
        if (config["topics"][key]) {
            return config["topics"][key].as<std::string>();
        }

        return "";
    }

    static std::string get_lanelet() {
        std::string vehicle = Utils::get_setting<std::string>("vehicle", "carla");
        YAML::Node config = YAML::LoadFile(RosInterface::get_package_path() + "/config/" + vehicle + "-config.yaml");
        if (config["lanelet"]["map"]) {
            return RosInterface::get_package_path() + "/resources/maps/" + config["lanelet"]["map"].as<std::string>();
        }

        return "";
    }

    template <typename T>
    static T get_sensor_info(const std::string& key, const T& defaultValue = T()) {
        std::string vehicle = Utils::get_setting<std::string>("vehicle", "carla");
        YAML::Node config = YAML::LoadFile(RosInterface::get_package_path() + "/config/" + vehicle + "-config.yaml");
        if (config["sensors"][key]) {
            return config["sensors"][key].as<T>();
        }

        return defaultValue;
    }
};

}  // namespace tod_gl

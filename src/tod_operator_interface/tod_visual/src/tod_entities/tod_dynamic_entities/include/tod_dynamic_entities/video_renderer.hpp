/**
 * @file video_renderer.hpp
 * @brief Renders an incoming video stream in multiple modes.
 *
 * Declares the VideoRenderer class template which renders a video stream using various projection modes.
 * For more details, please refer to the README.
 * 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/renderer/renderer.hpp"
#include "tod_gl/ros_interface/subscribing_components/image_component.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/scriptable_entity.hpp"
#include "tod_gl/scene/scene_serialization.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/utils/utils.hpp"

#include "tod_core/camera_models/OcamModel.h"
#include "tod_core/camera_models/PinholeModel.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace TodDynamicEntities {

/**
 * @class VideoRenderer
 * @brief Templated video renderer.
 *
 * Renders an incoming video stream using multiple projection modes.
 *
 * @tparam ImageComp The image component type.
 */
template<typename ImageComp>
class VideoRenderer : public tod_gl::ScriptableEntity {
  public:
    /**
     * @brief Constructs a new VideoRenderer object.
     *
     * @param camName Name of the camera.
     * @param stateKey Identifier for the scene manager state.
     * @param cam_config_path Path to the camera configuration.
     * @param vehicleID Vehicle identifier.
     * @param config_path General configuration path.
     * @param isFisheye Flag indicating if the camera is fisheye.
     */
    VideoRenderer(const std::string& camName,
                  const std::string& stateKey,
                  const std::string& cam_config_path,
                  const std::string& vehicleID,
                  const std::string& config_path,
                  const bool isFisheye)
        : _cam_name(camName),
          _scene_manager_key(stateKey),
          _cam_config_path(cam_config_path),
          _vehicle_ID(vehicleID),
          _is_fisheye(isFisheye),
          _config_path(config_path) {};

    /**
     * @brief Initializes the video renderer.
     */
    virtual void on_create() override;

    /**
     * @brief Cleans up resources used by the video renderer.
     */
    virtual void on_destroy() override;

    /**
     * @brief Updates the video stream rendering.
     *
     * @param delta_time Time elapsed since the last update.
     */
    virtual void on_update(float delta_time) override;

  private:
    float _line_width = 2.7f;
    static constexpr float SPHERE_MESH_INCREMENT{0.02f};

    const bool _is_fisheye{false};
    std::unique_ptr<PinholeModel> _pinhole_cam_model;
    std::unique_ptr<OcamModel> _o_cam_model;
    const std::string _cam_name;
    const std::string _scene_manager_key;
    const std::string _cam_config_path; // TODO Niklas Change after config
    const std::string _config_path; 
    const std::string _vehicle_ID;
    bool initMesh{false};
    geometry_msgs::msg::TransformStamped _camera_transformation;

    void update_resolution() {
        // Resolution update logic moved from old VideoRenderer
        auto& video = this->get_component<tod_gl::VideoComponent>();
        auto& renderable = this->get_component<tod_gl::RenderableElementComponent>();
        auto& mesh = renderable.meshes.front();

        if (!video.last_image_msg) return;
        float newScalingX = float(video.last_image_msg->width) / float(video.width_raw);
        float newScalingY = float(video.last_image_msg->height) / float(video.height_raw);

        for (auto& vertex : mesh.vertices) {
            vertex.tex_coord.x = newScalingX * (vertex.tex_coord.x - 1.0f) / video.scaling_x + 1.0f;
            vertex.tex_coord.y = newScalingY * (vertex.tex_coord.y - 1.0f) / video.scaling_y + 1.0f;
        }

        video.scaling_x = newScalingX;
        video.scaling_y = newScalingY;

        video.pixel_buffers.at(0).width = video.last_image_msg->width;
        video.pixel_buffers.at(0).height = video.last_image_msg->height;

        get_component<tod_gl::DynamicDataComponent>().has_new_data = true;
    }

    template<typename CamModel>
    void init_mesh(const CamModel& camMdl) {
        auto &video = this->get_component<tod_gl::VideoComponent>();
        auto &renderable = this->get_component<tod_gl::RenderableElementComponent>();
        auto &mesh = renderable.meshes.front();
        mesh.vertices.clear();
        mesh.indices.clear();
        // init mesh depending on projection mode
        if (video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::RECTANGULAR) {
            mesh.vertices.push_back(tod_gl::Vertex(glm::vec3(0.0f, (-0.5f * video.width_raw) / video.height_raw, 1.0f),
                                                   glm::vec2(video.width_raw, 1.0f)));             // top right
            mesh.vertices.push_back(tod_gl::Vertex(glm::vec3(0.0f, (-0.5f * video.width_raw) / video.height_raw, 0.0f),
                                                   glm::vec2(video.width_raw, video.height_raw)));  // bottom right
            mesh.vertices.push_back(tod_gl::Vertex(glm::vec3(0.0f, (0.5f * video.width_raw) / video.height_raw, 0.0f),
                                                   glm::vec2(1.0f, video.height_raw)));            // bottom left
            mesh.vertices.push_back(tod_gl::Vertex(glm::vec3(0.0f, (0.5f * video.width_raw) / video.height_raw, 1.0f),
                                                   glm::vec2(1.0f, 1.0f)));                       // top left

            mesh.indices = {0, 1, 3, 1, 2, 3};
        } else {
            std::vector<std::vector<tod_gl::Vertex>> rowsOnSphere;
            size_t maxRowLength{0};
            if ((video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::GROUND_PLANE) ||
                (video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::HALF_SPHERE_WITH_GROUND_PLANE)) {
                get_points_on_ground_plane<CamModel>(rowsOnSphere, maxRowLength, camMdl);
            }
            if ((video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::SPHERE) ||
                (video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::HALF_SPHERE_WITH_GROUND_PLANE) ||
                (video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::ROBINSON)) {
                get_points_on_sphere<CamModel>(rowsOnSphere, maxRowLength, camMdl);
            }
            push_triangles_to_renderable(rowsOnSphere, maxRowLength);
        }
    }

    template <typename CamModel>
    void get_points_on_ground_plane(std::vector<std::vector<tod_gl::Vertex>> &rowsOnSphere, size_t &maxRowLength, const CamModel camMdl) {
        auto &video = this->get_component<tod_gl::VideoComponent>();
        for (float myRad = video.ground_plane_radius_min; myRad <= video.sphere_radius; myRad += SPHERE_MESH_INCREMENT) {
            std::vector<tod_gl::Vertex> verticesInRow;
            for (float lon = video.sphere_longitude_max; lon >= video.sphere_longitude_min; lon -= SPHERE_MESH_INCREMENT) {
                geometry_msgs::msg::PoseStamped ptOnSphere, poseOut;
                ptOnSphere.pose.orientation.w = 1.0;
                ptOnSphere.pose.position.x = myRad * std::cos(lon);
                ptOnSphere.pose.position.y = myRad * std::sin(lon);
                ptOnSphere.pose.position.z = 0.0;
                tf2::doTransform(ptOnSphere, poseOut, _camera_transformation);
                if (!(poseOut.pose.position.z > 0.0))
                    continue;  // only z > can be projected

                // _camera_transformation in pixel coords
                int x_px{0}, y_px{0};
                if (camMdl.point_on_image(poseOut.pose.position, x_px, y_px)) {
                    verticesInRow.push_back(tod_gl::Vertex(
                        glm::vec3(ptOnSphere.pose.position.x, ptOnSphere.pose.position.y, ptOnSphere.pose.position.z),
                        glm::vec2(float(x_px), float(y_px))));
                }
            }
            if (!verticesInRow.empty()) {
                rowsOnSphere.push_back(verticesInRow);
                maxRowLength = std::max(maxRowLength, verticesInRow.size());
            }
        }
    }

    template <typename CamModel>
    void get_points_on_sphere(std::vector<std::vector<tod_gl::Vertex>> &rowsOnSphere, size_t &maxRowLength, const CamModel &vidParams) {
        auto &video = this->get_component<tod_gl::VideoComponent>();
        float latMin = video.sphere_latitude_min;
        if (video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::HALF_SPHERE_WITH_GROUND_PLANE)
            latMin = std::max(latMin, 0.0f);
        float latMax = video.sphere_latitude_max;
        for (float lat = latMin; lat <= latMax; lat += SPHERE_MESH_INCREMENT) {
            std::vector<tod_gl::Vertex> verticesInRow;
            for (float lon = video.sphere_longitude_max; lon >= video.sphere_longitude_min; lon -= SPHERE_MESH_INCREMENT) {
                // _camera_transformation point on sphere in camera coords
                geometry_msgs::msg::PoseStamped ptOnSphere, poseOut;
                ptOnSphere.pose.orientation.w = 1.0;
                ptOnSphere.pose.position.x = video.sphere_radius * std::cos(lat) * std::cos(lon);
                ptOnSphere.pose.position.y = video.sphere_radius * std::cos(lat) * std::sin(lon);
                ptOnSphere.pose.position.z = video.sphere_radius * std::sin(lat);
                tf2::doTransform<geometry_msgs::msg::PoseStamped>(ptOnSphere, poseOut, _camera_transformation);
                if (!(poseOut.pose.position.z > 0.0))
                    continue;  // only z > can be projected

                // _camera_transformation in pixel coords
                int x_px{0}, y_px{0};
                if (vidParams.point_on_image(poseOut.pose.position, x_px, y_px)) {
                    if (video.projection_mode == tod_gl::VideoComponent::ProjectionModeType::ROBINSON) {
                        // equations following Savric et al.: A Polynomial Equation for the Natural Earth Projection
                        double A0{0.8507}, A1{0.9642}, A2{-0.1450};
                        double A3{-0.0013}, A4{-0.0104}, A5{-0.0129};
                        ptOnSphere.pose.position.x = video.sphere_radius;
                        ptOnSphere.pose.position.y =
                            video.sphere_radius * lon * (A0 + A2 * std::pow(lat, 2) + A4 * std::pow(lat, 4));
                        ptOnSphere.pose.position.z =
                            video.sphere_radius * (A1 * lat + A3 * std::pow(lat, 3) + A5 * std::pow(lat, 5));
                    }
                    verticesInRow.push_back(tod_gl::Vertex(
                        glm::vec3(ptOnSphere.pose.position.x, ptOnSphere.pose.position.y, ptOnSphere.pose.position.z),
                        glm::vec2(float(x_px), float(y_px))));
                }
            }
            if (!verticesInRow.empty()) {
                rowsOnSphere.push_back(verticesInRow);
                maxRowLength = std::max(maxRowLength, verticesInRow.size());
            }
        }
    }
    void push_triangles_to_renderable(std::vector<std::vector<tod_gl::Vertex>> &rowsOnSphere, const size_t maxRowLength) {
        auto &renderable = this->get_component<tod_gl::RenderableElementComponent>();
        for (auto &row : rowsOnSphere) {
            for (size_t i = row.size(); i < maxRowLength; ++i) {
                row.push_back(row.back());
            }
        }

        // connect to triangles
        int ptCount{0};
        auto &mesh = renderable.meshes.front();
        for (int i = 1; i < rowsOnSphere.size(); ++i) {
            int a = ptCount++;
            mesh.vertices.push_back(rowsOnSphere.at(i).at(0));
            int b = ptCount++;
            mesh.vertices.push_back(rowsOnSphere.at(i - 1).at(0));
            for (int j = 1; j < rowsOnSphere.at(i).size(); ++j) {
                mesh.vertices.push_back(rowsOnSphere.at(i).at(j));
                int c1 = ptCount++;
                mesh.indices.push_back(a);
                mesh.indices.push_back(b);
                mesh.indices.push_back(c1);

                mesh.vertices.push_back(rowsOnSphere.at(i - 1).at(j));
                int c2 = ptCount++;
                mesh.indices.push_back(b);
                mesh.indices.push_back(c1);
                mesh.indices.push_back(c2);
                a = c1;
                b = c2;
            }
        }
    }
};

template class VideoRenderer<tod_gl::ImageComponentRearRight>;
template class VideoRenderer<tod_gl::ImageComponentRearLeft>;
template class VideoRenderer<tod_gl::ImageComponentRearCenter>;
template class VideoRenderer<tod_gl::ImageComponentFrontRight>;
template class VideoRenderer<tod_gl::ImageComponentFrontLeft>;
template class VideoRenderer<tod_gl::ImageComponentFrontCenter>;

}  // namespace TodDynamicEntities

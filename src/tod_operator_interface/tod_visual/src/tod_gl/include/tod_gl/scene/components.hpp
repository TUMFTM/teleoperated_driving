/**
 * @file components.hpp
 * @brief Defintions of base components for entities  
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/scriptable_entity.hpp"

#include "openvr/openvr.h"
#include "sensor_msgs/msg/image.hpp"

namespace tod_gl {

/**
 * @brief Component Architecture Overview
 * 
 * The rendering system is built on an Entity-Component-System (ECS) architecture using EnTT:
 * 
 * Key Components:
 * 
 * 1. TransformComponent - Stores position, rotation, scale and parent reference
 *    - Handles transformation hierarchy via parent-child relationships
 *    - Provides methods for local-to-world matrix calculation
 * 
 * 2. RenderableElementComponent - Contains rendering data for an entity
 *    - Stores shader program reference
 *    - Contains vector of meshes to render
 *    - Specifies rendering mode (triangles, lines, points)
 *    - Controls transparency settings (opaque flag)
 * 
 * 3. CameraComponent - Defines the viewpoint for rendering
 *    - Stores view and projection matrices
 *    - Contains camera position, orientation, and target
 *    - Manages orbit points for camera controllers
 * 
 * 4. FrameBufferComponent - Represents a render target
 *    - References a camera entity for view/projection
 *    - Manages OpenGL framebuffer, renderbuffer, and texture objects
 *    - Controls render resolution and multisampling
 * 
 * 5. DynamicDataComponent - Marks entities with dynamic mesh data
 *    - Provides mutex for thread-safe updates
 *    - Tracks whether data has been modified and needs GPU upload
 * 
 * 6. ScriptComponent - Attaches custom behavior to entities
 *    - Links to a ScriptableEntity instance with custom logic
 *    - Provides lifecycle methods (on_create, on_update)
 * 
 * 7. VRComponent - Adds VR-specific functionality
 *    - Specifies VR eye (left/right) for stereo rendering
 *    - Works with VRSystem for specialized transformations
 * 
 * Render Pipeline Flow:
 * 1. Scripts update entity data (positions, meshes, etc.)
 * 2. Cameras and transforms are updated
 * 3. Dynamic mesh data is uploaded to GPU
 * 4. Framebuffers are bound and the scene is rendered
 * 5. For VR, the rendered textures are submitted to the VR system
 */


/*
* @brief Tag component holds a string to name and find entities in Entt
*/
struct TagComponent {
    std::string tag{""};

    TagComponent() = default;
    TagComponent(const TagComponent &) = default;
    explicit TagComponent(const std::string &tag) : tag(tag) {}
};


/*
* @brief TransformComponent component holds the transformation information inside the 3D scene for the rendering of the entity. The Transform component is used to construct a transform hierachy
*/
struct TransformComponent {
    glm::vec3 translation{0.0f, 0.0f, 0.0f};
    glm::vec3 rotation{0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f, 1.0f, 1.0f};
    Entity parent_entity;
    bool is_map_frame{false};

    TransformComponent() = default;
    TransformComponent(const TransformComponent &) = default;

    glm::mat4 get_transform() const {
        glm::mat4 cur_rotation = glm::mat4_cast(glm::quat(this->rotation));

        return glm::translate(glm::mat4(1.0f), this->translation) * cur_rotation * glm::scale(glm::mat4(1.0f), this->scale);
    }

    // tod_visual uses Right-handed coordinate system to match autoware
    // x-axis : forward, y-axis: left, z-axis: up
    glm::vec3 get_forward() const { return glm::quat(this->rotation) * glm::vec3(1.0, 0.0, 0.0); }

    glm::vec3 get_right() const { return glm::quat(this->rotation) * glm::vec3(0.0, -1.0, 0.0); }

    glm::vec3 get_up() const { return glm::quat(this->rotation) * glm::vec3(0.0, 0.0, 1.0); }

    void set_rotation(const glm::vec3 &rotation) { this->rotation = rotation; }
    void set_translation(const glm::vec3 &translation) { this->translation = translation; }
    void setScale(const glm::vec3 &scale) { this->scale = scale; }
    void set_parent(const Entity &entity) { parent_entity = entity; }
};


/*
* @brief RenderableElementComponent hold the references to the renderable mesh, every entity that is supposed to be rendered has to have a renderable component
*/
struct RenderableElementComponent {
    unsigned int shader_program;
    std::vector<Mesh> meshes;
    GLenum render_mode;
    float line_width{1.0f}, point_size{1.0f};
    bool static_show{true}, dynamic_show{true};
    bool opaque{true};

    RenderableElementComponent(const unsigned int shaderProgram, const std::vector<Mesh> &meshes,
                               const GLenum renderMode = GL_TRIANGLES, const bool opaque = true)
        : shader_program(shaderProgram), meshes(meshes), render_mode(renderMode), opaque(opaque) {
        for (Mesh &mesh : this->meshes) {
            if (mesh.vertices.empty()) {
                mesh.vertices = Mesh::vector_of_three_zero_vertices();
            }
        }
    }

    RenderableElementComponent(const unsigned int shaderProgram, const Mesh &mesh, GLenum renderMode = GL_TRIANGLES)
        : shader_program(shaderProgram), render_mode(renderMode) {
        meshes.emplace_back(mesh);
        if (mesh.vertices.empty()) {
            meshes.back().vertices = Mesh::vector_of_three_zero_vertices();
        }
    }
    std::vector<Mesh> deep_copy_meshes() {
        std::vector<Mesh> deepCopiedMeshes;

        for (const auto &mesh : this->meshes) {
            Mesh newMesh = Mesh(mesh.vertices, mesh.indices, mesh.textures);
            newMesh.vertex_array_object = mesh.vertex_array_object;
            newMesh.vertex_buffer = mesh.vertex_buffer;
            newMesh.index_buffer = mesh.index_buffer;
            deepCopiedMeshes.push_back(std::move(newMesh));
        }

        return deepCopiedMeshes;
    }
};

/*
* @brief DynamicDataComponent relic of the past
*/
struct DynamicDataComponent {
    bool has_new_data{false};
    std::shared_ptr<std::mutex> mutex{std::make_shared<std::mutex>()};

    DynamicDataComponent() = default;
    DynamicDataComponent(const DynamicDataComponent &) = default;
};

/*
* @brief ExpirableComponent relic of the past
*/
struct ExpirableComponent {
    uint64_t stamp_ms{0};
    uint64_t time_to_expire_ms{0};

    ExpirableComponent() = default;
    ExpirableComponent(const ExpirableComponent &) = default;
    explicit ExpirableComponent(const uint64_t &myTimeToExpireMs) : time_to_expire_ms(myTimeToExpireMs) {}

    uint64_t now() { return std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000; }
    bool expired() { return (now() >= (stamp_ms + time_to_expire_ms)); }
    void restamp() { stamp_ms = now(); }
};

/*
* @brief VideoComponent that holds special information about the specific rendering information and buffers for the rendering of videos 
*/
struct VideoComponent {
    enum ProjectionModeType {
        RECTANGULAR = 0,
        SPHERE = 1,
        HALF_SPHERE_WITH_GROUND_PLANE = 2,
        GROUND_PLANE = 3,
        ROBINSON = 4
    };


    struct PixelBuffer {
        Buffer buf;
        unsigned int width, height;
        std::string name;
        PixelBuffer(const unsigned int width, const unsigned int height, const std::string &name)
            : buf(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW), width{width}, height{height}, name{name} {}
    };

    std::string camera_name{""};
    bool is_fisheye{false};

    std::vector<PixelBuffer> pixel_buffers;
    std::shared_ptr<std::mutex> mutex{std::make_shared<std::mutex>()};
    sensor_msgs::msg::Image::SharedPtr last_image_msg{nullptr};

    sensor_msgs::msg::Image::ConstSharedPtr last_projected_image_msg{nullptr};

    ProjectionModeType projection_mode{ProjectionModeType::RECTANGULAR};
    float ground_plane_radius_min{0.01f};
    float sphere_radius{15.0f};  // also used as GroundPlaneRadiusMax
    float sphere_longitude_min{-3.1415f}, sphere_longitude_max{+3.1415f};
    float sphere_latitude_min{-1.5708f}, sphere_latitude_max{+1.0472f};

    int width_raw{0}, height_raw{0};
    float scaling_x{1.0}, scaling_y{1.0};

    VideoComponent() = default;
    VideoComponent(const VideoComponent &) = default;
    VideoComponent(const std::string &cameraName, const bool isFisheye)
        : camera_name(cameraName), is_fisheye{isFisheye} {}
};


/*
* @brief CameraState holds the current state of the camera such as position and lookat
*/
struct CameraState {
    float yaw;
    float radius;
    glm::vec3 lookat;
    glm::vec3 position;
    glm::vec3 up;
};

/*
* @brief OrbitPointState holds the current state of the orbit point used for orbital rotation
*/
struct OrbitPointState {
    float orbit_distance;
    glm::vec3 position;
};


/*
* @brief Camera Component describes the camera of the 3D scene from which the current framebuffer is generated
*/
struct CameraComponent {

    glm::mat4 projection;
    glm::mat4 view;
    CameraState initial = {0.0f, 1.5f, {0.5f, 0.0f, 2.5f}, {-8.5f, 0.0f, 5.5f}, {0.0f, 0.0f, 1.0f}};
    //CameraState initial = {0.0f, 1.5f, {3.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 1.0f}};
    float yaw = initial.yaw;
    float radius = initial.radius;
    glm::vec3 lookat = initial.lookat;
    glm::vec3 position = initial.position;
    glm::vec3 up = initial.up;

    glm::vec3 target_position = position;
    glm::vec3 target_lookat = lookat;
    OrbitPointState orbit_point{10.f, glm::vec3{0.0f,0.0f,0.0f }}; // position of the vehicle

    bool controllable{false};
    bool rotate = false;
    float near_plane{0.1f};
    float far_plane{100.0f};
    float field_of_view{45.0f};

    CameraComponent() = default;
    CameraComponent(const CameraComponent&) = default;
    CameraComponent(const glm::mat4 &projection, bool controllable)
        : projection(projection), controllable(controllable) {}
};

struct VRComponent {
    vr::Hmd_Eye eye;

    VRComponent() = default;
    VRComponent(const VRComponent &) = default;
    explicit VRComponent(const vr::Hmd_Eye &eye) : eye(eye) {}
};

/*
* @brief FrameBufferComponent that is used to generate the texture of the scene to be rendered  
*/
struct FrameBufferComponent {
    GLuint render_texture_id;
    unsigned int frame_buffer_object{0}, render_buffer_object{0};

    entt::entity camera_entity;
    int samples{0};

    bool is_default_framebuffer{false};
    bool should_render{true};
    unsigned int render_width{1280};
    unsigned int render_height{1280};

    FrameBufferComponent() = default;
    FrameBufferComponent(const FrameBufferComponent &) = default;
    explicit FrameBufferComponent(bool isDefaultFramebuffer) : is_default_framebuffer(isDefaultFramebuffer) {}
};

struct CharacterMapComponent {
    std::map<char, Character> characters;
    float pixel_per_meter_ratio{1000.0f};

    CharacterMapComponent() = default;
    CharacterMapComponent(const CharacterMapComponent &) = default;
};

/*
* @brief ScriptComponent holds the reference to a scriptable entity which manage their own rendering information that is updated in the on_update loop of the scene application
*/
struct ScriptComponent{
    ScriptableEntity* instance = nullptr;

    std::function<ScriptableEntity*()> instantiate_scriptable;
    std::function<void(ScriptComponent*)> destroy_script;

    template<typename T>
    void bind() {
        instantiate_scriptable = []() { return static_cast<ScriptableEntity*>(new T()); };
        destroy_script = [](ScriptComponent* sc) { delete sc->instance; sc->instance = nullptr; };
    }

    template<typename T, typename... Args>
    void bind_with_params(Args&&... args) {
        instantiate_scriptable = [boundArgs = std::make_tuple(std::forward<Args>(args)...)]() -> ScriptableEntity* {
            return std::apply([](auto&&... unpackedArgs) {
                return static_cast<ScriptableEntity*>(new T(std::forward<decltype(unpackedArgs)>(unpackedArgs)...));}, 
                boundArgs);
        };

        destroy_script = [](ScriptComponent* sc) { 
            delete sc->instance; 
            sc->instance = nullptr; 
        };
    }

};

}  // namespace tod_gl
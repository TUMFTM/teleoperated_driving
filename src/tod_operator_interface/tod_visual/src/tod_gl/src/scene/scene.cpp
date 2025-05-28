/**
 * @file scene.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/scene/scene.hpp"

#include <iterator>

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/renderer/renderer.hpp"
#include "tod_gl/systems/camera_system.hpp"
#include "tod_gl/systems/transform_system.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/camera_controller.hpp"

#include "glm/gtx/matrix_decompose.hpp"
#include "glm/gtx/string_cast.hpp"

namespace tod_gl {

Scene::Scene() {
    _vr_mode = _vr_system.vrInit();
}

Entity Scene::create_entity(const std::string &name) {
    Entity entity = Entity(registry.create(), this);
    entity.add_component<TransformComponent>();
    auto &tag = entity.add_component<TagComponent>();
    tag.tag = name.empty() ? "Entity" : name;
    _tag_entity_map[tag.tag] = entity;
    return entity;
}
/**
 * @brief Main update loop that handles entity scripts, camera updates, and rendering
 * 
 * This method represents the complete frame rendering pipeline with the following stages:
 * 1. Script execution - Updates all entities with ScriptComponent
 * 2. Camera system updates - Updates camera positions, view, and projection matrices
 * 3. Orbital point updates - Updates the orbit point for camera controllers
 * 4. Model matrix updates - Recalculates transformation matrices for all rendered entities
 * 5. Data upload - Uploads any dynamic mesh data to the GPU
 * 6. Framebuffer rendering - Renders the scene to all active framebuffers
 * 7. VR pose update - Updates VR system pose data if VR is enabled
 * 
 * The method includes performance timing points (t0-t4) for profiling different stages of the render loop.
 * 
 * @param timeStep Elapsed time since the last update in seconds
 */
void Scene::on_update(float timeStep) {
    auto &_stateManager = StateManager::get_instance();
    {
        registry.view<ScriptComponent>().each([=, &_stateManager](const entt::entity &entity, ScriptComponent &sc) {

            // Initialize
            if (!sc.instance) {
                sc.instance = sc.instantiate_scriptable();
                sc.instance->m_Entity = Entity{entity, this};
                sc.instance->on_create();
            }

            sc.instance->on_update(timeStep);
        });
    }
    
    auto t0 = std::chrono::high_resolution_clock::now();
    update_cameras(timeStep);

    update_oribtal_point();
    
    auto t1 = std::chrono::high_resolution_clock::now();
    // compute mdl matrix in every frame to account for dynamic parents,
    // set in msg callbacks when no odometry signal is received,
    // i.e., model pose does not change
    // if (model_pose_changed())
    update_model();

    auto t2 = std::chrono::high_resolution_clock::now();
    upload_data();

    auto t3 = std::chrono::high_resolution_clock::now();
    int nofFramebuffersRendered = render_on_framebuffer();

    auto t4 = std::chrono::high_resolution_clock::now();
    _vr_system.update_vr_pose();
}

/**
 * @brief Updates transformation matrices for all renderable entities
 * 
 * This method recalculates and uploads the model matrices for all renderable entities in the scene.
 * It takes into account:
 * - Parent-child relationships between entities (transformation hierarchy)
 * - Dynamic changes to entity positions and orientations
 * - Visibility settings from the StateManager
 * 
 * For each visible renderable entity, the method:
 * 1. Retrieves the entity's TransformComponent and RenderableElementComponent
 * 2. Checks if the entity should be rendered based on StateManager settings
 * 3. Calculates the complete model matrix using TransformSystem
 * 4. Sets the "Model" uniform in the entity's shader program
 * 
 * This method is called each frame to ensure all transformations are up-to-date before rendering.
 */
void Scene::update_model() {
    // auto &dynamic = registry.get<DynamicDataComponent>(_base_footprint);
    // std::lock_guard<std::mutex> lock(*dynamic.mutex);
    
    auto &_stateManager = StateManager::get_instance();

    auto view = registry.view<RenderableElementComponent>();
    for (auto &entity : view) {
        auto &renderable = registry.get<RenderableElementComponent>(entity);
        auto &transform = registry.get<TransformComponent>(entity);
        auto &tag = registry.get<TagComponent>(entity);
        if (_stateManager.contains_entity(tag.tag)) {
            if (!_stateManager.should_render_entity(tag.tag)) {
                continue;
            }
        }

        glm::mat4 model = tod_gl::TransformSystem::get_instance()->local_to_world(transform);
        ShaderSystem::set_shader_program_mat4(renderable.shader_program, "Model", model);
    }
    // dynamic.has_new_data = false;
}

/**
 * @brief Updates all camera entities in the scene
 * 
 * This method updates all cameras associated with framebuffers in the scene:
 * 1. For each framebuffer, retrieves its associated camera entity
 * 2. Transitions the camera using CameraController (handles animations/movements)
 * 3. Updates the view and projection matrices based on camera settings
 * 4. For VR cameras, calculates specialized matrices using VRSystem
 * 
 * Camera updates include:
 * - Position and orientation transitions
 * - View matrix calculation based on current position and look-at point
 * - Projection matrix updates (on initialization and window resize)
 * 
 * @param timeStep Elapsed time since the last update in seconds
 */
void Scene::update_cameras(float timeStep) {
    // Update Camera of each Framebuffer
    auto view = registry.view<FrameBufferComponent>();
    for (auto &entity : view) {
        auto &camera = registry.get<CameraComponent>(registry.get<FrameBufferComponent>(entity).camera_entity);
        auto &controller = CameraController::get_instance();
        controller.transition(camera, timeStep);
        auto &transform = registry.get<TransformComponent>(registry.get<FrameBufferComponent>(entity).camera_entity);
        
        
        if (registry.has<VRComponent>(entity)) {
            auto &vr = registry.get<VRComponent>(entity);
            _vr_system.calc_projection_matrix(camera, vr);  // Todo: Does not change.. only on init?
            _vr_system.calc_view_matrix(camera, vr, transform);
        } else {
            // TODO(Simon): Projection currently changed on init and on WIndow Resize... what about other framebuffers
            CameraSystem::calc_view_matrix(camera, transform);
        }
    }
}
/**
 * @brief Uploads dynamic mesh data to the GPU
 * 
 * This method processes all entities with both DynamicDataComponent and RenderableElementComponent:
 * 1. Acquires a mutex lock for thread safety
 * 2. Checks if the entity has new data that needs to be uploaded
 * 3. If new data exists, calls Renderer::update_meshes to upload vertex/index data to GPU buffers
 * 
 * This method ensures that dynamic meshes (those that change during runtime) have their
 * latest data available for rendering.
 */
void Scene::upload_data() {
    auto view = registry.view<DynamicDataComponent, RenderableElementComponent>();
    for (auto &entity : view) {
        auto &renderable = registry.get<RenderableElementComponent>(entity);
        auto &dynamic = registry.get<DynamicDataComponent>(entity);
        Renderer::update_meshes(renderable, dynamic);
    }
}
/**
 * @brief Renders the scene to all active framebuffers
 * 
 * This method manages the rendering process for each framebuffer in the scene:
 * 1. Iterates through all entities with FrameBufferComponent
 * 2. Sets up rendering parameters for each framebuffer (viewport, clear color, etc.)
 * 3. For active framebuffers, retrieves the associated camera's view and projection matrices
 * 4. Updates shader uniforms with the current projection and view matrices
 * 5. Renders all visible meshes to the framebuffer
 * 6. For VR framebuffers, submits the rendered texture to the VR system
 * 
 * The method returns the number of framebuffers that were actually rendered.
 * 
 * @return int Number of framebuffers rendered
 */
int Scene::render_on_framebuffer() {
    int nofFramebuffersRendered{0};
    for (auto &entity : registry.view<FrameBufferComponent>()) {
        auto &framebuffer = registry.get<FrameBufferComponent>(entity);

        Renderer::setup_rendering_for_framebuffer(framebuffer);
        if (!framebuffer.should_render) {
            continue;
        }

        // use projection and view of camera assigned to framebuffer
        if (registry.has<CameraComponent>(framebuffer.camera_entity)) {
            view = registry.get<CameraComponent>(framebuffer.camera_entity).view;
            projection = registry.get<CameraComponent>(framebuffer.camera_entity).projection;
            update_projection_and_view();
        } else {
            // TODO(Andi): reintroduce print
            // ROS_ERROR_STREAM("No Camera assigned to Framebuffer of Entity %s: " <<
            //                  registry.get<TagComponent>(entity).tag);
        }

        render_mesh();
        if (registry.has<VRComponent>(entity)) {
            auto &renderable = registry.get<RenderableElementComponent>(entity);
            auto &vr = registry.get<VRComponent>(entity);
            VRSystem::submit_texture(renderable, vr);
        }
        ++nofFramebuffersRendered;
    }
    return nofFramebuffersRendered;
}

/**
 * @brief Updates projection and view matrices for all renderable entities
 * 
 * This method sets the combined ProjectionView matrix uniform for all shader programs:
 * 1. Iterates through all entities with RenderableElementComponent
 * 2. Sets the "ProjectionView" uniform in each entity's shader program
 * 
 * The ProjectionView matrix is a pre-multiplied combination of the current projection
 * and view matrices, which optimizes the vertex transformation pipeline.
 */
void Scene::update_projection_and_view() {
    auto entity_view = registry.view<RenderableElementComponent>();
    for (auto &entity : entity_view) {
        auto &renderable = registry.get<RenderableElementComponent>(entity);
        ShaderSystem::set_shader_program_mat4(renderable.shader_program, "ProjectionView", projection * view);
    }
}

/**
 * @brief Renders all mesh entities to the current framebuffer
 * 
 * This method implements a two-pass rendering approach to handle transparent objects:
 * 1. First pass: Renders all opaque objects and collects transparent objects
 * 2. Second pass: Renders transparent objects in back-to-front order
 * 
 * For each renderable entity, the method:
 * 1. Checks if the entity should be rendered based on StateManager settings
 * 2. For opaque objects, renders immediately using Renderer::render_meshes
 * 3. For transparent objects, stores in a distance-sorted map for later rendering
 * 4. Checks and respects any ExpirableComponent settings (objects with limited lifetime)
 * 
 * The depth sorting ensures proper blending of transparent objects regardless of the
 * order they're defined in the scene.
 */
void Scene::render_mesh() {
    auto fview = registry.view<FrameBufferComponent>();
    CameraComponent *camera;
    for (auto &entity : fview) {
        auto &framebuffer = registry.get<FrameBufferComponent>(entity);
        camera = &registry.get<CameraComponent>(framebuffer.camera_entity);
    }

    std::map<float, const entt::entity *> zSorted;
    // First Pass: Render Opaque objects and sort transparent ones.
    auto &_stateManager = StateManager::get_instance();

    auto view = registry.view<RenderableElementComponent>();
    for (auto &entity : view) {
        auto &tag = registry.get<TagComponent>(entity);
        if (_stateManager.contains_entity(tag.tag)) {
            if (!_stateManager.should_render_entity(tag.tag)) {
                continue;
                ;
            }
        }

        auto &renderable = registry.get<RenderableElementComponent>(entity);
        if (!renderable.opaque) {
            auto &transform = registry.get<TransformComponent>(entity);
            float distance = glm::length(camera->position - transform.translation);
            zSorted[distance] = &entity;
            continue;
        }

        if (registry.has<ExpirableComponent>(entity))
            renderable.dynamic_show = !(registry.get<ExpirableComponent>(entity).expired());

        if (registry.has<DynamicDataComponent>(entity)) {
            std::lock_guard<std::mutex> lock(*registry.get<DynamicDataComponent>(entity).mutex);
            Renderer::render_meshes(renderable);
        } else {
            Renderer::render_meshes(renderable);
        }
    }

    // Second Pass: Render Z-Sorted Transparent Objects.
    for (std::map<float, const entt::entity *>::reverse_iterator it = zSorted.rbegin(); it != zSorted.rend(); ++it) {
        const entt::entity &entity = *it->second;
        auto &renderable = registry.get<RenderableElementComponent>(entity);

        if (registry.has<ExpirableComponent>(entity))
            renderable.dynamic_show = !(registry.get<ExpirableComponent>(entity).expired());

        auto &tag = registry.get<TagComponent>(entity);

        if (registry.has<DynamicDataComponent>(entity)) {
            std::lock_guard<std::mutex> lock(*registry.get<DynamicDataComponent>(entity).mutex);
            Renderer::render_meshes(renderable);
        } else {
            Renderer::render_meshes(renderable);
        }
    }

    view = registry.view<RenderableElementComponent>();
}

/*
* @brief Initilisation of the scene and the scriptable entities, framebuffer 
*/
void Scene::init(const unsigned int width, const unsigned int height) {
    registry.view<ScriptComponent>().each([=](auto entity, auto &sc) {
        // Initialize Scripts
        if (!sc.instance) {
            sc.instance = sc.instantiate_scriptable();
            sc.instance->m_Entity = Entity{entity, this};
            sc.instance->on_create();
        }
    });

    if (_vr_mode)
        _vr_system.create_vr_entities(this, _base_footprint);
    {
        auto view = registry.view<RenderableElementComponent>();
        for (auto entity : view) {
            auto &renderable = registry.get<RenderableElementComponent>(entity);
            if (registry.has<DynamicDataComponent>(entity)) {
                std::lock_guard<std::mutex> lock(*registry.get<DynamicDataComponent>(entity).mutex);
                Renderer::generate_meshes(renderable);
                Renderer::generate_textures(renderable);
            } else {
                Renderer::generate_meshes(renderable);
                Renderer::generate_textures(renderable);
            }
        }
    }
    // TODO(Simon): cleanup; handle size of window with framebuffer...
    {
        bool mainFramebufferSpecified{false};
        auto view = registry.view<FrameBufferComponent>();
        for (auto entity : view) {
            auto &framebuffer = registry.get<FrameBufferComponent>(entity);
            if (framebuffer.is_default_framebuffer) {
                framebuffer.render_width = width;
                framebuffer.render_height = height;
                mainFramebufferSpecified = true;
            }
            auto &camera = registry.get<CameraComponent>(framebuffer.camera_entity);
            CameraSystem::on_window_size_changed(camera, framebuffer.render_width, framebuffer.render_height);

            // TODO(Simon): check if renderable in entity
            if (!framebuffer.is_default_framebuffer) {
                auto &renderable = registry.get<RenderableElementComponent>(entity);
                Renderer::generate_frame_buffer(framebuffer, renderable);
            }
        }
        if (!mainFramebufferSpecified) {
            // TODO(Andi): reintroduce print
            // ROS_ERROR_STREAM("No default Framebuffer specified!");
        }
    }

    update_model();
}


void Scene::update_oribtal_point() {
    auto view = registry.view<CameraComponent>();
    auto orbitalPoint = find_entity_with_tag("orbit_point_visual");
    
    if (!orbitalPoint) return;

    for (auto entity : view) {
        auto& camera = registry.get<CameraComponent>(entity);
        if (camera.controllable) {
            auto& transform = orbitalPoint.get_component<TransformComponent>();
            transform.translation = camera.orbit_point.position;
            break;
        }
    }
}

void Scene::set_base_foot_print(const entt::entity &entity) {
    _base_footprint = entity;
}

Entity Scene::find_entity_with_tag(const std::string &tag) {
    if (_tag_entity_map.find(tag) == _tag_entity_map.end())
        return {};
    return _tag_entity_map[tag];
}

bool Scene::model_pose_changed() {
    if (_base_footprint == entt::null) {
        // TODO(Andi): reintroduce print
        // ROS_ERROR("does not have handle of base_footprint entity in render loop");
        return false;
    }
    return registry.get<DynamicDataComponent>(_base_footprint).has_new_data;
}
}  // namespace tod_gl
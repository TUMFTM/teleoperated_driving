/**
 * @file scene.hpp
 * @brief Defintion of the scene class that performs the rendering and management for the 3D scene  
 * @copyright 2024 TUMFTM
 * @details The Scene class serves as the central orchestrator for the 3D rendering environment,
 * managing entities, their components, and the rendering pipeline. It provides the framework
 * for entity creation, transformation hierarchy, camera management, and the render loop execution.
 * The class implements an Entity-Component-System (ECS) architecture using the EnTT library
 * for efficient component storage and querying.
 **/

#pragma once

#include <string>
#include <map>

#include "tod_gl/systems/vr_system.hpp"

#include "glad/glad.h"
#include "entt/entt.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"

namespace tod_gl {

class Entity;


/**
 * @class Scene
 * @brief Main controller class for the 3D scene, handling entity management and rendering
 * 
 * The Scene class manages the complete lifecycle of the 3D environment including:
 * - Entity creation and management
 * - Component hierarchy and transformations
 * - Camera system and view/projection matrices
 * - Rendering pipeline execution
 * - VR system integration (when enabled)
 * - Framebuffer management and rendering
 * 
 * The class follows a component-based architecture where entities are containers for various
 * components (Transform, Renderable, Camera, etc.) that define their behavior and appearance.
 * The render loop processes these components to generate the final rendered output.
 */
class Scene {
  public:
    Scene();
    ~Scene() {};
    /**
    * @brief Creates a new entity in the scene with the specified name
    * 
    * This method creates a new entity with the EnTT registry, adds a TransformComponent
    * and a TagComponent to it, and registers it in the tag-entity map for later lookup.
    * Each entity has a unique identifier and can be referenced through its tag.
    * 
    * @param name The name/tag to assign to the entity (defaults to "Entity" if empty)
    * @return Entity A wrapper object for the created entity
    */
    Entity create_entity(const std::string& name);

    /**
    * @brief Initializes the scene with the specified viewport dimensions
    * 
    * This method performs the following initialization steps:
    * - Initializes all scriptable entities by calling their on_create() methods
    * - Creates VR entities if VR mode is enabled
    * - Generates meshes and textures for all renderable entities
    * - Sets up framebuffers including the main framebuffer with the specified dimensions
    * - Updates the camera projection matrices based on the framebuffer dimensions
    * - Performs initial model transformations
    * 
    * @param width Width of the main viewport/framebuffer in pixels
    * @param height Height of the main viewport/framebuffer in pixels
    */
    void init(const unsigned int width, const unsigned int height);
    

    /**
    * @brief Main update and render loop for the scene
    * 
    * This method represents the main rendering pipeline executed each frame:
    * 1. Updates all scriptable entities via their on_update() methods
    * 2. Updates all cameras and their controllers
    * 3. Updates the orbital point for camera orbiting
    * 4. Updates model transformations based on entity hierarchies
    * 5. Uploads dynamic mesh data to the GPU
    * 6. Renders the scene to all active framebuffers
    * 7. Updates VR pose data if VR is enabled
    * 
    * The method is designed to be called at regular intervals, with timeStep
    * representing the elapsed time since the last update.
    * 
    * @param timeStep Elapsed time since the last update in seconds
    */
    void on_update(float timeStep);


    /*
    * @brief Performs the transformations based on the transform component hierachy system across the entites e.g. the car drives around and other objects are transformed accordingly
    */
    void update_model();

    /*
    * @brief Updates the camera's view and projection matrices from which the scene is gnerated 
    */
    void update_cameras(float timeStep);
    /*
    * @brief Uploads the @ref Mesh data to the GPU for rendering
    */
    void upload_data();
    int render_on_framebuffer();
    void update_projection_and_view();
    void set_base_foot_print(const entt::entity& entitiy);
    void update_oribtal_point();
    void render_mesh();

    /**
    * @brief Finds and returns an entity with the specified tag
    * 
    * This method looks up an entity in the tag-entity map and returns it.
    * If multiple entities have the same tag, only the last one created will be returned.
    * 
    * @param tag The tag to search for
    * @return Entity The entity with the specified tag, or an empty entity if not found
    */
    Entity find_entity_with_tag(const std::string& tag);

    entt::registry registry;
    glm::mat4 view;
    glm::mat4 projection;

  private:
    friend class Entity;
    entt::entity _base_footprint{entt::null};
    VRSystem _vr_system;
    std::map<std::string, Entity> _tag_entity_map;

    bool _vr_mode{false};

    bool model_pose_changed();
};

} // namespace tod_gl
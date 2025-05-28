/**
 * @file scene_layer.hpp
 * @brief Handles the creation, updates, and rendering of a 3D scene including vehicles, cameras, and other visual elements.
 * @copyright 2024 TUMFTM
**/

#pragma once

#include <map>
#include <memory>
#include <string>

#include "tod_gl/core/scene_layer.hpp"
#include "tod_gl/ros_interface/ros_interface.hpp"
#include "tod_gl/scene/scene.hpp"
#include "tod_gl/events/event.hpp"
#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/camera_controller.hpp"
#include "tod_gl/scene/components.hpp"

#include "tod_gl/events/application_event.hpp"
#include "tod_gl/events/key_event.hpp"
#include "tod_gl/events/mouse_events.hpp"
#include "tod_gl/core/model_loader.hpp"

#include "tod_core/param_set/CameraParameters.hpp"
#include "tod_core/param_set/VehicleParameters.hpp"
#include "tod_core/param_set/TransformParameters.hpp"

#include "glad/glad.h" // Glad must be included before glfw3
#include "GLFW/glfw3.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

static const float PI{3.14159f};

namespace tod_visual {
/**
 * @class VisualLayer
 * @brief A layer for rendering and updating a 3D scene with vehicle and camera models.
 * @copyright 2024 TUMFTM
 */
class VisualLayer : public tod_gl::SceneLayer {
  public:
      /**
     * @brief Constructs the VisualLayer with a given ROS interface and scene.
     * @param ros   Shared pointer to the ROS interface.
     * @param scene Shared pointer to the scene to be managed.
     */
    VisualLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene);

    /**
     * @brief Default destructor.
     */
    virtual ~VisualLayer() = default;
       /**
     * @brief Called when the layer is attached; used to initialize resources.
     */
    virtual void on_attach() override;
        /**
     * @brief Called when the layer is detached; used to release resources.
     */
    virtual void on_detach() override;
   /**
     * @brief Called every frame to update the scene.
     * @param ts The time step since the last update.
     */
    void on_update(float ts) override;
        /**
     * @brief Renders any ImGui-based debug or control UI elements.
     */
    virtual void on_im_gui_render() override;
     /**
     * @brief Handles events such as input and window resizing.
     * @param e The event to be handled.
     */
    void on_event(tod_gl::Event& e) override;

    std::mutex _mouse_position_mutex;
    geometry_msgs::msg::PointStamped _mouse_position;

  private:
    std::map<std::string, tod_gl::Entity> _coordinate_systems;

    std::unique_ptr<tod_core::param_set::Vehicle> _veh_params;
    std::unique_ptr<tod_core::param_set::Camera> _cam_params;
    std::unique_ptr<tod_core::param_set::Transform> _transform_params;

    tod_gl::CameraController& _cam_controller = tod_gl::CameraController::get_instance();

    float _width = 1280;
    float _height = 720;
    float _time_step = 0.f;

    void handle_window_resize_event(tod_gl::WindowResizeEvent& e);
    void handle_mouse_moved_event(tod_gl::MouseMovedEvent& e);
    void handle_mouse_button_pressed_event(tod_gl::MouseButtonPressedEvent& e);
    void handle_key_pressed_event(tod_gl::KeyPressedEvent& e);

    void handle_key_released_event(tod_gl::KeyReleasedEvent& e);
    void handle_mouse_button_released_event(tod_gl::MouseButtonReleasedEvent& e);

    tod_gl::FrameBufferComponent* get_default_framebuffer();

    void create_scene();
    void create_coodinate_system_entites();
    void create_display_entites();
    void create_vehicle_model_entites();
    void create_grid_and_floor_entites();
    void create_camera_and_framebuffer();
    void create_orbital_point();

        /**
     * @brief Creates a video renderer entity given a template video component.
     * @tparam VideoComp The video component type.
     * @param name    Name for the entity.
     * @param stateKey Key to fetch state or configuration.
     * @param isFisheye Whether the camera is fisheye or not.
     */
    template<typename VideoComp>
    void create_video_renderer(const std::string& name, const std::string& stateKey , const bool isFisheye );
    void create_video_renderers();



    void bind_scripts();
    // Helpers

    /**
     * @brief Creates a vehicle model entity by loading a model.
     * @param modelName   The name/path of the model.
     * @param translation A vec3 translation for positioning in the scene.
     * @param loader      Pointer to a model loader for loading the 3D model.
     * @return The created entity with the vehicle model.
     */
    tod_gl::Entity create_vehicle_model_entity(const std::string& modelName, const glm::vec3& translation,
                                            tod_gl::ModelLoader* loader);
};
}  // namespace tod_visual
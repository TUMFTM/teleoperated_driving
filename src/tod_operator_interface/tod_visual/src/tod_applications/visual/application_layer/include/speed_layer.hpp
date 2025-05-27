/**
 * @file speed_layer.hpp
 * @brief Renders a vertical speed bar for both current and desired velocities, providing
 *        a simple visualization of vehicle speed control. (Header File)
 * @copyright 2024 TUMFTM
 */

 #pragma once

 #include <memory>
 #include <string>
 
 #include "tod_gl/layers/docking_scene_layer.hpp"
 #include "tod_gl/ros_interface/ros_interface.hpp"
 #include "tod_gl/scene/scene.hpp"
 #include "tod_gl/events/event.hpp"
 #include "imgui/imgui.h"
 
 namespace tod_visual {
 
 /**
  * @class SpeedLayer
  * @brief Renders a vertical speed bar for both current and desired velocities.
  * @tparam T1 Component type providing desired speed information.
  * @tparam T2 Component type providing current speed information.
  */
 template <class T1, class T2>
 class SpeedLayer : public tod_gl::DockingSceneLayer {
   public:
     /**
      * @brief Constructs the SpeedLayer.
      * @param ros Shared pointer to the ROS interface.
      * @param scene Shared pointer to the scene.
      * @param split_dir The docking direction.
      * @param name The name of the layer.
      */
     SpeedLayer(std::shared_ptr<tod_gl::RosInterface> ros,
                std::shared_ptr<tod_gl::Scene> scene,
                ImGuiDir split_dir,
                std::string name);
 
     /**
      * @brief Default destructor.
      */
     ~SpeedLayer();
 
     /**
      * @brief Renders the vertical speed bar UI elements.
      */
     virtual void on_im_gui_render() override;
 
     /**
      * @brief Called when the layer is attached (unused here).
      */
     virtual void on_attach() override;
 
     /**
      * @brief Forwards event handling to the base ImGuiSceneLayer.
      * @param e The event reference to process.
      */
     virtual void on_event(tod_gl::Event &e) override;
 
     /**
      * @brief Updates speed data from the subscribed components.
      * @param ts The time step since the last update (unused).
      */
     virtual void on_update(float ts) override;
 
   private:
     float barWidth = 30.0f;       ///< The width of the vertical speed bar.
     float maxSpeed = 36.0f;       ///< Maximum speed displayed on the bar (in km/h).
     float desiredVelocity = 0.0f; ///< Desired speed in km/h.
     float currentVelocity = 0.0f; ///< Current speed in km/h.
 };
 
 }  // namespace tod_visual
 
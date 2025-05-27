/**
 * @file VideoStateLayer.hpp
 * @ingroup tod_visual_application
 * @brief Defines the Operator State Layer for the application.
 * 
 * This class provides the GUI and functionality for managing the video state,
 * including interaction with ROS components.
 * 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <cmath> 
#include <string>
#include <vector>
#include <filesystem> 

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "tod_gl/layers/imgui_layer.hpp"
#include "tod_config_msgs/srv/video_config.hpp"

#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"
#include "tod_gl/ros_interface/service_components/video_config_component.hpp"
#include "shared_cam.hpp"


#include "video_manager_docking_layer.hpp"

/**
 * @class VideoStateLayer
 * @brief Represents the operator state layer in the application.
 * 
 * This layer handles the GUI elements for displaying and managing the operator state,
 * including IP address selection, state updates, and interaction with ROS components.
 */
class VideoStateLayer : public tod_gl::VideoManagerDockingLayer {
  public:
    VideoStateLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<SharedCam> cam, ImGuiDir split_dir);
    ~VideoStateLayer() = default;

    virtual void on_attach() override;
    virtual void on_detach() override;
    virtual void on_im_gui_render() override;
    virtual void on_event(tod_gl::Event& e) override;
    virtual void on_update(float ts) override;
    void render_tab_content(const char* header);
    bool send_request();

    static int selected_video_rate;

      static rclcpp::Logger get_logger() {

      static auto logger = rclcpp::get_logger("VideoStateLayer");
      return logger;
   }







private:
  tod_gl::TodStatusComponent _status; 
  std::shared_ptr<SharedCam> _camera;
  tod_gl::VideoConfigComponent _videoConfig;
  int _bitrate;
  int _selected_scaling;
  int _height;
  int _width;
  int _width_offset;
  int _height_offset;
  int _bitrate_sum;
  bool _is_paused;


};

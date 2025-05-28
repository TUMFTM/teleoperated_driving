/**
 * @file video_manager.hpp
 * @brief Defines the main VideoManager class for the operator interface application.
 *
 * This file contains the declaration of the VideoManager class, which serves as the 
 * main entry point for managing the operator interface. The class is responsible for 
 * initializing and managing various layers of the application.
 * 
 * @copyright 2024 TUMFTM
 */
#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "tod_gl/core/application.hpp"
#include "tod_gl/core/window.hpp"
#include "tod_gl/layers/io_layer.hpp"
#include "tod_core/param_set/CameraParameters.hpp"
#include "tod_core/param_set/VideoParameters.hpp"

#include "tod_gl/layers/imgui_layer.hpp"
#include "imgui/imgui.h"

#include "video_state_layer.hpp"
#include "camera_position_layer.hpp"
#include "shared_cam.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "video_manager_docking_layer.hpp"



namespace tod_gl {


/**
 * @class VideoManager
 * @brief Main application manager for videointerface.
 *
 * The VideoManager class initializes and manages various layers of the video manager application.
 * It inherits from the Application base class and sets up the required layers for the video manager.
 */
 
class VideoManager : public Application {
  public:

    VideoManager(int argc, char** argv, const std::string& name);

    ~VideoManager() = default;


    static rclcpp::Logger get_logger() {

      static auto logger = rclcpp::get_logger("VideoManager");
      return logger;



    }
    private:
      std::shared_ptr<std::vector<std::shared_ptr<SharedCam>>> _shared_cams_list;
      ImGuiLayer* _imGui_layer;
      std::unique_ptr<tod_core::param_set::Camera> _cam_params;
      std::unique_ptr<tod_core::param_set::Video> _video_params;
      // TODO Check if remove
      uint8_t _video_rate_control_mode{tod_status_msgs::msg::Status::VIDEO_RATE_CONTROL_MODE_SINGLE};
      int _bitrate_sum{0};
      bool _connected{false};
      bool _adapt_video_params_enabled{false};
      int get_mapping(const std::string& name);

}; 



        struct NameAndStreamOnConnect { std::string name; bool streamOnConnect{true}; };
    } // namespace tod_gl

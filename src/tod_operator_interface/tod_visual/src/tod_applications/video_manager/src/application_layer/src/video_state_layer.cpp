/**
 * @file video_state_layer.cpp
 * @brief Implementation of the VideoStateLayer class for managing video stream configurations in the UI.
 * @copyright 2024 TUMFTM
 */

#include "video_state_layer.hpp"
#include <string>

std::string response = ""; ///< Global response string for feedback.
int VideoStateLayer::selected_video_rate = 0; ///< Static variable for storing the selected video rate.
static const char* scaling_options[] = {"1.0", "0.5", "0.75", "1.0"}; ///< Scaling options for video.
tod_status_msgs::msg::Status statusMsg; ///< Status message instance.

/**
 * @brief Constructs a VideoStateLayer instance.
 * @param ros Shared pointer to the ROS interface.
 * @param cam Shared pointer to the camera object.
 */
VideoStateLayer::VideoStateLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<SharedCam> cam,ImGuiDir split_dir)
    : VideoManagerDockingLayer(ros, split_dir),  _status(ros), _camera(cam), _videoConfig(ros), _is_paused(false) {

    _name = _camera->name.c_str();
    _bitrate = 0;
    _selected_scaling = 0;
    _width = 0;
    _height = 0;
    _width_offset = 0;
    _height_offset = 0;
    _bitrate_sum = 0;
    split_direction = split_dir;
}

/**
 * @brief Called when the layer is attached to the application.
 */
void VideoStateLayer::on_attach() {}

/**
 * @brief Called when the layer is detached from the application.
 */
void VideoStateLayer::on_detach() {}

/**
 * @brief Renders the ImGui elements for the video state layer.
 */
void VideoStateLayer::on_im_gui_render() {
    static bool request_in_progress = false; ///< Prevents overlapping requests.

    RCLCPP_DEBUG(VideoStateLayer::get_logger(), 
                 "on_im_gui_render: _is_paused=%d, is_active=%d, request_in_progress=%d", 
                 _is_paused, _camera->is_active, request_in_progress);

    if (!request_in_progress) {
        // Handle stream activation
        if (_camera->is_active && _is_paused) {
            request_in_progress = true;
            if (send_request()) {
                _is_paused = false;
                RCLCPP_INFO(VideoStateLayer::get_logger(), "%s Stream activated", _camera->name.c_str());
            } else {
                _camera->is_active = false;
                RCLCPP_ERROR(VideoStateLayer::get_logger(), "%s Activation failed", _camera->name.c_str());
            }
            request_in_progress = false;
        }

        // Handle stream pausing
        if (!_camera->is_active && !_is_paused) {
            request_in_progress = true;
            if (send_request()) {
                _is_paused = true;
                RCLCPP_INFO(VideoStateLayer::get_logger(), "%s Stream paused", _camera->name.c_str());
            } else {
                _camera->is_active = true;
                RCLCPP_ERROR(VideoStateLayer::get_logger(), "%s Pause failed", _camera->name.c_str());
            }
            request_in_progress = false;
        }
    }

    // Render UI components
    if (_camera->is_active) {
        ImGui::Begin(_dock_space_window_name.c_str());
        ImGui::SameLine();
        ImGui::Begin(_name.c_str(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        render_tab_content(_camera->name.c_str());
        ImGui::End();
        ImGui::End();
    }
}

/**
 * @brief Processes events for the layer.
 * @param e Event to process.
 */
void VideoStateLayer::on_event(tod_gl::Event& e) {
    (void)e;
}

/**
 * @brief Updates the layer.
 * @param ts Time step for the update.
 */
void VideoStateLayer::on_update(float ts) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

/**
 * @brief Renders the tab content for video stream reconfiguration.
 * @param header Header title for the tab content.
 */
void VideoStateLayer::render_tab_content(const char* header) {
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.7f), "Reconfigure Video Stream %s", _name.c_str());

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    float label_width = 120.0f;
    float spacing = 10.0f;  
    float input_width = 200.0f;

    ImGui::PushItemWidth(120.0f);

    // Input fields for various configurations
    ImGui::SetCursorPosX(spacing);
    ImGui::Text("kBit Rate:");
    ImGui::SameLine(label_width + spacing);
    ImGui::PushItemWidth(input_width);
    ImGui::InputInt("##_bitrate", &_bitrate, 200, 500);

    ImGui::SetCursorPosX(spacing);
    ImGui::Text("Scaling:");
    ImGui::SameLine(label_width + spacing);
    ImGui::PushItemWidth(input_width);
    ImGui::Combo("##scaling", &_selected_scaling, scaling_options, IM_ARRAYSIZE(scaling_options));

    ImGui::SetCursorPosX(spacing);
    ImGui::Text("Width:");
    ImGui::SameLine(label_width + spacing);
    ImGui::PushItemWidth(input_width);
    ImGui::InputInt("##width", &_width, 10, 50);

    ImGui::SetCursorPosX(spacing);
    ImGui::Text("Height:");
    ImGui::SameLine(label_width + spacing);
    ImGui::PushItemWidth(input_width);
    ImGui::InputInt("##height", &_height, 10, 50);

    ImGui::SetCursorPosX(spacing);
    ImGui::Text("Width Offset:");
    ImGui::SameLine(label_width + spacing);
    ImGui::PushItemWidth(input_width);
    ImGui::InputInt("##width_offset", &_width_offset, 10, 50);

    ImGui::SetCursorPosX(spacing);
    ImGui::Text("Height Offset:");
    ImGui::SameLine(label_width + spacing);
    ImGui::PushItemWidth(input_width);
    ImGui::InputInt("##height_offset", &_height_offset, 10, 50);

    ImGui::SetCursorPosX(spacing);
    ImGui::Text("Bitrate Sum:");
    ImGui::SameLine(label_width + spacing);
    ImGui::PushItemWidth(input_width);
    ImGui::InputInt("##bitrate_sum", &_bitrate_sum, 100, 500);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Buttons for sending and resetting settings
   // if (_status.get_tod_status() == 0) ImGui::BeginDisabled();

    if (ImGui::Button("Send Request", ImVec2(120, 50))) {
        send_request();
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset Settings", ImVec2(120, 50))) {
        _bitrate = 0;
        _selected_scaling = 0; 
        _width = 0;
        _height = 0;
        _width_offset = 0;
        _height_offset = 0;
        _bitrate_sum = 0;

        send_request();
        RCLCPP_INFO(VideoStateLayer::get_logger(), "Settings reset to default");
        response = "Settings reset to default.";
    }

   // if (_status.get_tod_status() == 0) ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::Text(response.c_str());

    ImGui::PopItemWidth();




// NOT FOR OPEN-SOURCE
//std::string imagePath = "TUM_Logo_blau_rgb_p.png";
//GLuint iconViewId = ImGuiLayer::load_texture(
//    (tod_gl::RosInterface::get_package_path() + "/resources/icons/" + imagePath).c_str());
//
//ImVec2 originalSize(740, 390);
//
//float scaleFactor = 0.1f;
//ImVec2 scaledSize(originalSize.x * scaleFactor, originalSize.y * scaleFactor);
//
//ImVec2 layerSize = ImGui::GetWindowSize();
//ImVec2 layerPos = ImGui::GetWindowPos();
//
//float xPosition = layerPos.x+10;  
//float yPosition = layerPos.y + layerSize.y - scaledSize.y;  
//
//
//ImGui::SetCursorScreenPos(ImVec2(xPosition, yPosition));
//
//ImGui::Image((void *)(intptr_t)iconViewId, scaledSize);
}

/**
 * @brief Sends a reconfiguration request for the video stream.
 * @return True if the request was successful, false otherwise.
 */
bool VideoStateLayer::send_request() {
    RCLCPP_INFO(VideoStateLayer::get_logger(), "send_request called: _is_paused=%d, is_active=%d", _is_paused, _camera->is_active);

    if (_status.get_tod_status() <= 0) {
        RCLCPP_INFO(VideoStateLayer::get_logger(), "Not connected. Request cannot be sent.");
        return false;
    }

    auto videoConfigRequest = std::make_shared<tod_config_msgs::srv::VideoConfig::Request>();
    if(_camera->is_active&&_is_paused){
        
    }

    // Populate the request fields
    videoConfigRequest->camera_name = _camera->name;
    videoConfigRequest->paused = !_camera->is_active;
    videoConfigRequest->actual_width = static_cast<int64_t>(round(_width)); 
    videoConfigRequest->actual_height = static_cast<int64_t>(round(_height));
    double factor = 1.0;
    if (scaling_options[_selected_scaling] == "Auto") {
    videoConfigRequest->scaling_factor = 1.0;
    RCLCPP_INFO(VideoStateLayer::get_logger(), "################################");
    RCLCPP_INFO(VideoStateLayer::get_logger(), std::to_string(1.0).c_str());
       
    }else{  
    std::string option_str = scaling_options[_selected_scaling];
    option_str.erase(std::remove(option_str.begin(), option_str.end(), '%'), option_str.end());
     factor = std::stod(option_str);
    videoConfigRequest->scaling_factor = std::to_string(factor);
    RCLCPP_INFO(VideoStateLayer::get_logger(), "################################");
    RCLCPP_INFO(VideoStateLayer::get_logger(), std::to_string(factor).c_str());

    }

    videoConfigRequest->width = static_cast<int64_t>(round(_width)); 
    videoConfigRequest->height = static_cast<int64_t>(round(_height)); 
    videoConfigRequest->offset_width = static_cast<int64_t>(round(_width_offset)); 
    videoConfigRequest->offset_height = static_cast<int64_t>(round(_height_offset)); 
    videoConfigRequest->bitrate = static_cast<int64_t>(round(_bitrate)); 


    bool success = _videoConfig.request_reconfigure(videoConfigRequest);


    if (success) {
        response = "Request sent successfully!";
        RCLCPP_INFO(VideoStateLayer::get_logger(), "Request sent successfully");
    } else {
        response = "Request failed!";
        RCLCPP_ERROR(VideoStateLayer::get_logger(), "Request failed");
    }

    return success; 

}



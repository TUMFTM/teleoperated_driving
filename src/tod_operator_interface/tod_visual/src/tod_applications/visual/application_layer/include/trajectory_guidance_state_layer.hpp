/**
 * @file trajectory_guidance_state_layer.hpp
 * @brief Displays the current trajectory guidance state, including the target velocity, state transitions, and events.
 *        Provides a visual overlay with color-coded status updates for better user awareness.
 * @copyright 2024 TUMFTM
 */

/**
 * @class TrajectoryGuidanceStateLayer
 * @brief Handles the visualization of trajectory guidance states in the UI.
 * @tparam TrajectoryGuidanceStateComp Component type providing trajectory state updates.
 */
#pragma once

#include <memory>
#include <string>

#include "ui_layer.hpp"
#include "view_port_layer.hpp"

#include "tod_gl/layers/docking_scene_layer.hpp"

#include "tod_gl/ros_interface/subscribing_components/trajectory_guidance_state_component.hpp"

#include "tod_trajectory_guidance_msgs/msg/trajectory_guidance_state.hpp"

namespace tod_visual {

template <class TrajectoryGuidanceStateComp>

class TrajectoryGuidanceStateLayer : public UILayer {
  public:
      /**
     * @brief Constructs the TrajectoryGuidanceStateLayer.
     * @param ros Shared pointer to the ROS interface.
     * @param scene Shared pointer to the scene.
     * @param view_port_layer Pointer to the associated ViewPortLayer.
     */
    TrajectoryGuidanceStateLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene,
                                 ViewPortLayer* view_port_layer)
        : UILayer(ros, scene, view_port_layer) {
        _name = "TrajectoryGuidanceStateLayer";
        initialize_fonts();
    }

    ~TrajectoryGuidanceStateLayer() = default;

    virtual void on_im_gui_render() override {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImVec2 window_pos = viewport->Pos;
        const float padding = 10.0f;
        // ImVec2 layerPos = ImVec2(windowPos.x + padding, padding);
        
        ImVec2 layer_pos = ImVec2(
            window_pos.x + padding, 
            window_pos.y + viewport->Size.y - ImGui::GetFrameHeight() - padding  
        );


        //ImGui::SetNextWindowPos(layerPos);
        
        ImGui::SetNextWindowPos(
            ImVec2(window_pos.x + padding, window_pos.y + viewport->WorkSize.y - padding),
            ImGuiCond_Always,
            ImVec2(0.0f, 1.0f)  // Bottom-left alignment
        );
        ImGuiWindowFlags window_flags = 
            ImGuiWindowFlags_NoTitleBar | 
            ImGuiWindowFlags_NoResize | 
            ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoNav |
            ImGuiWindowFlags_NoBackground;  

        ImGui::Begin("TrajectoryGuidanceStateLayer", nullptr, window_flags);
        
        render_state_info();

        ImGui::End();
    }

    virtual void on_update(float ts) override {
        tod_gl::Entity subscription_manager = _active_scene->find_entity_with_tag("SubscriptionManager");

        if (subscription_manager.has_component<TrajectoryGuidanceStateComp>()) {
            TrajectoryGuidanceStateComp &comp = subscription_manager.get_component<TrajectoryGuidanceStateComp>();
            current_state_ = comp.current_state_;
            last_event_ = comp.last_event_;
            current_state_text_ = comp.state_to_string(current_state_);
            last_event_text_ = comp.event_to_string(last_event_);
            target_velocity_ = comp.target_velocity_;
            
        }
    }

  private:
    void initialize_fonts() {
        ImGuiIO& io = ImGui::GetIO();
        DriveInfoFontSmall_ = io.Fonts->AddFontFromFileTTF(
            (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 16.0f);
        DriveInfoFontLarge_ = io.Fonts->AddFontFromFileTTF(
            (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 20.0f);
    }

    void render_state_info() {
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 current_pos = ImGui::GetCursorScreenPos();

        ImVec2 rect_min = current_pos;
        ImVec2 rect_max = ImVec2(rect_min.x + 220, rect_min.y + 110); 
        ImColor bg_color = get_state_color();
        draw_list->AddRectFilled(rect_min, rect_max, bg_color, 5.0f);

        // Draw text
        ImGui::PushFont(DriveInfoFontLarge_);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f,0.0f,0.0f,1.0f));
        ImGui::SetCursorScreenPos(ImVec2(current_pos.x + 10, current_pos.y + 5));
        ImGui::Text("Trajectory Guidance");
        ImGui::PopStyleColor();
        ImGui::PopFont();

        ImGui::PushFont(DriveInfoFontSmall_);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f,0.0f,0.0f,1.0f));
        ImGui::SetCursorScreenPos(ImVec2(current_pos.x + 10, current_pos.y + 25));
        ImGui::Text("Target Speed: %d", (int)(target_velocity_));
        ImGui::SetCursorScreenPos(ImVec2(current_pos.x + 10, current_pos.y + 40 ));
        ImGui::Text("State: %s", current_state_text_.c_str());
        ImGui::SetCursorScreenPos(ImVec2(current_pos.x + 10, current_pos.y + 55));
        ImGui::Text("Last Event: %s", last_event_text_.c_str());
        ImGui::PopStyleColor();
        ImGui::PopFont();

        render_state_icon(ImVec2(rect_max.x - 30, rect_min.y + 10));
    }

    ImColor get_state_color() {
        switch (current_state_) {
            case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY:
                return ImColor(255, 255, 0); // YELLOW
            case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATING_TRAJECTORY:
                return ImColor(255, 165, 0); // Orange
            case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_TRAJECTORY:
                return ImColor(0, 160, 0); // Green
            case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY:
                return ImColor(255, 0, 0); // Red
            default:
                return ImColor(128, 128, 128); // Gray
        }
    }

    void render_state_icon(const ImVec2& position) {
        std::string image_texture;
        if (current_state_ == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY) {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/clock.png";
        } else if (current_state_ == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATING_TRAJECTORY) {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/clock.png";
        } else if (current_state_ == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_TRAJECTORY) {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/wheel.png";
        } else if (current_state_ == tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY) {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/stop.png";
        } else {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/stop.png";
        }

        GLuint icon_texture_id = tod_gl::ImGuiSceneLayer::load_texture(image_texture.c_str());
        ImGui::SetCursorScreenPos(position);
        ImGui::Image((void*)(intptr_t)icon_texture_id, ImVec2(20, 20));
    }

    uint8_t last_event_ = 0;
    std::string last_event_text_ = "No new messages";
    uint8_t current_state_ = 0;
    std::string current_state_text_ = "No new messages";

    float target_velocity_{0.0f};


    ImFont* DriveInfoFontSmall_;
    ImFont* DriveInfoFontLarge_;
};
}  // namespace tod_visual
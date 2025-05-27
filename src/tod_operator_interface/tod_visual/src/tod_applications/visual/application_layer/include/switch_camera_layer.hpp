/**
 * @file switch_camera_layer.hpp
 * @brief Manages camera switching, system status display, and toggle controls in a UI overlay for trajectory guidance.
 *        Provides an interface to switch between different camera views and shows the current system status.
 *        It also displays the last event and trajectory guidance state from ROS components.
 * @copyright 2024 TUMFTM
 */

/**
 * @class SwitchCameraLayer
 * @brief A UI layer that handles camera switching, system status display, and event feedback for trajectory guidance.
 * @tparam TrajectoryGuidanceStateComp The component type to retrieve trajectory guidance states and events.
 */


#pragma once

#include "ui_layer.hpp"
#include "view_port_layer.hpp"

#include "tod_gl/layers/docking_scene_layer.hpp"

#include "tod_gl/ros_interface/subscribing_components/trajectory_guidance_state_component.hpp"

#include "tod_msgs/msg/trajectory_guidance_state.hpp"

namespace tod_visual {
template <class TrajectoryGuidanceStateComp>
class SwitchCameraLayer : public UILayer {
  public:
    SwitchCameraLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene,
                      ViewPortLayer* view_port_layer)
        : UILayer(ros, scene, view_port_layer) {
        _name = "SwitchCamera";
    }
    ~SwitchCameraLayer() = default;

    virtual void on_im_gui_render() override {
        ImVec2 current_display_size = io_.DisplaySize;
        float button_dimension = std::min(current_display_size.y * 0.05f, 50.0f);
        ImVec2 button_size = ImVec2(button_dimension, button_dimension);

        ImVec2 button_status_size = ImVec2(100, 45);
        ImVec2 unfold_status_size = ImVec2(current_display_size.x * 0.2f, current_display_size.y * 0.2f);
        ImVec2 current_status_size = isExpanded_ ? unfold_status_size : ImVec2(0, 0);

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImVec2 window_position = viewport->Pos;

        ImVec2 full_window_size = ImGui::GetContentRegionAvail();

        ImVec2 viewport_position = view_port_layer->getPos();
        ImVec2 viewport_size = view_port_layer->getSize();
        ImVec2 layer_pos =
            ImVec2(viewport_position.x + current_display_size.x - button_status_size.x - 40, window_position.y + 50);

        // Toggle Control Owner
        ImVec2 toggle_mode_pos = ImVec2(viewport_position.x + current_display_size.x - 320, window_position.y + 40);
        toggle_with_images(toggle_mode_pos);

        // Small sub message under system status
        ImColor system_status_color = ImColor(0, 160, 0);
        ImVec2 badge_position = ImVec2(viewport_position.x + current_display_size.x - button_status_size.x * 1.45f,
                                      window_position.y + button_status_size.y * 2.5f);
        draw_badge(badge_position, system_status_color);

        ImVec2 sidebar_position = ImVec2(layer_pos.x + 50, layer_pos.y + viewport_size.y * 0.3f);

        // Camera buttons shall only be overlaying the video layer
        current_display_size = viewport_size;

        ImGui::SetNextWindowPos(layer_pos);
        ImGui::SetNextWindowSize(ImVec2(button_status_size.x + 20, current_display_size.y + 10));

        ImGui::Begin("SwitchCamera", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoScrollbar);

        GLuint icon_inside_outside_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/inside-outside.png").c_str());
        GLuint icon_switch_camera_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/switch-camera.png").c_str());

        ImVec2 system_status_pos = ImVec2(0, 0);
        ImVec2 unfold_status_pos =
            ImVec2(window_position.x + current_display_size.x - unfold_status_size.x - button_status_size.x * 0.2f,
                   window_position.y + button_status_size.y * 3.5f);
        ImVec2 unfold_status_text_pos = ImVec2(unfold_status_pos.x + 10, unfold_status_pos.y + 10);

        ImVec2 sidebar_dimensions = ImVec2(60.0f, current_display_size.y * 0.4f);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 1.0f, 0.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 1.0f, 1.0f, 0.4f));

        // system status button
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 window_pos = ImGui::GetWindowPos();
        ImVec2 rect_min = ImVec2(window_pos.x + 5, window_pos.y);
        ImVec2 rect_max = ImVec2(window_pos.x + button_status_size.x + 10, window_pos.y + button_status_size.y + 10);
        const float rounding = 5.0f;
        const float padding = 5.0f;
        draw_list->AddRectFilled(rect_min, rect_max, system_status_color, rounding);

        ImGui::PushFont(DriveInfoFontSmall_);
        char status_text[32];
        sprintf(status_text, "%s", "SYSTEM \nSTATUS");
        ImVec2 system_status_text_size = ImGui::CalcTextSize(status_text);

        ImVec2 system_status_text_pos = ImVec2(window_pos.x + padding + 20, window_pos.y + padding);

        ImGui::SetCursorScreenPos(system_status_text_pos);
        sprintf(status_text, "%s", "SYSTEM \nSTATUS");
        ImGui::Text(status_text);
        ImGui::PopFont();

        ImGui::SetCursorPos(system_status_pos);
        ImVec2 tmp = ImVec2(button_status_size.x + 10, button_status_size.y + 10);
        if (ImGui::Button(("###hidden"), tmp)) {
            // TODO system status
            isExpanded_ = !isExpanded_;
        }

        // Placeholder for unfolded status background
        draw_list = ImGui::GetForegroundDrawList();
        ImU32 dark_gray_color = IM_COL32(64, 64, 64, 255);
        ImU32 white_color = IM_COL32(255, 255, 255, 255);
        tmp = ImVec2(unfold_status_pos.x + current_status_size.x, unfold_status_pos.y + current_status_size.y);
        draw_list->AddRectFilled(unfold_status_pos, tmp, dark_gray_color);

        const char* message = last_event_text.c_str();
        ImVec2 text_size = ImGui::CalcTextSize(message);

        ImVec2 text_pos = ImVec2(unfold_status_pos.x + 10.0f, unfold_status_pos.y + 10.0f);
        ImGui::SetCursorPos(text_pos);
        ImGui::PushFont(statusUnfoldedFont_);
        if (isExpanded_) {
            draw_list->AddText(text_pos, white_color, message);
        } else {
            draw_list->AddText(text_pos, white_color, "");
        }
        ImGui::PopFont();

        // Stach overall style so rounding change only affects camera buttons
        ImGuiStyle& style = ImGui::GetStyle();
        float old_frame_rounding = style.FrameRounding;
        style.FrameRounding = 25.0f;

        // Draw button for inside camera with sidebar
        ImVec2 inside_camera_size = ImVec2(button_size.x * 0.7f, button_size.y * 0.8f);
        ImVec2 inside_outside_pos =
            ImVec2(button_status_size.x - inside_camera_size.x + 5, (current_display_size.y - inside_camera_size.y) * 0.5f);
        draw_sidebar(sidebar_dimensions, sidebar_position, button_size);

        ImGui::SetCursorPos(inside_outside_pos);
        if (ImGui::ImageButton((void*)(intptr_t)icon_inside_outside_id, inside_camera_size)) {
            // NOTE: Switching camera not supported in setup
        }

        ImGui::PopStyleColor(2);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(26.0f / 255.0f, 27.0f / 255.0f, 28.0f / 255.0f, 180.0f / 255.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 1.0f, 1.0f, 0.4f));

        // Draw button for switching camera
        ImVec2 switch_camera_size = ImVec2(button_size.x * 0.8f, button_size.y * 0.65f);
        ImVec2 switch_camera_pos =
            ImVec2(button_status_size.x - switch_camera_size.x, current_display_size.y * 0.875f - button_size.y * 1.2f);
        ImGui::SetCursorPos(switch_camera_pos);
        if (ImGui::ImageButton((void*)(intptr_t)icon_switch_camera_id, switch_camera_size)) {
            // NOTE: Switching camera not supported in setup
        }

        style.FrameRounding = old_frame_rounding;

        ImGui::PopStyleColor(2);

        ImGui::End();
    }

    virtual void on_attach() override {}

    virtual void on_event(tod_gl::Event& e) override { ImGuiSceneLayer::on_event(e); }

    virtual void on_update(float ts) override {
        tod_gl::Entity subscription_manager = _active_scene->find_entity_with_tag("SubscriptionManager");

        if (subscription_manager.has_component<TrajectoryGuidanceStateComp>()) {
            TrajectoryGuidanceStateComp& comp = subscription_manager.get_component<TrajectoryGuidanceStateComp>();
            current_state_ = comp.current_state;
            last_event_ = comp.last_event;
            current_state_text = comp.state_to_string(current_state);
            last_event_text = comp.event_to_string(last_event);
        }
    }

    /**
     * @brief Draws a sidebar background with a rectangular panel and a button area.
     * @param dimensions The dimensions of the sidebar.
     * @param position The top-left position for the sidebar.
     * @param button_size The size for the button area in the sidebar.
     */
    void draw_sidebar(ImVec2 dimensions, ImVec2 position, ImVec2 button_size) {
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        ImU32 fill_color = IM_COL32(26, 27, 28, 180);

        // Position und Größe des Rechtecks
        float corner_radius = 10.0f;
        ImVec2 rect_tall_min = ImVec2(position.x + button_size.x * 0.5f + 15, position.y);
        ImVec2 rect_tall_max = ImVec2(position.x + 70, position.y + dimensions.y);
        ImVec2 rect_button_min = ImVec2(position.x + 15, position.y + (dimensions.y - button_size.y) * 0.5f);
        ImVec2 rect_button_max = ImVec2(rect_button_min.x + button_size.x + 20, rect_button_min.y + button_size.y + 10);

        // Zeichne das gefüllte Rechteck
        draw_list->AddRectFilled(rect_tall_min, rect_tall_max, fill_color, corner_radius);
        draw_list->AddRectFilled(rect_button_min, rect_button_max, fill_color, corner_radius);

        // Zeichne den Rahmen
        draw_list->AddRect(rect_tall_min, rect_tall_max, fill_color, corner_radius, 0, 2.0f);
        draw_list->AddRect(rect_button_min, rect_button_max, fill_color, corner_radius, 0, 2.0f);
    }
    
    /**
     * @brief Draws a small badge showing the current trajectory guidance state and an icon.
     * @param position The top-left position of the badge.
     * @param color The background color of the badge.
     */
    void draw_badge(const ImVec2& position, const ImColor& color) {
        std::string image_texture;
        char status_text[32];
        if (current_state == tod_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY) {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/clock.png";
            sprintf(statusText, "%s", current_state_text.c_str());
        } else if (current_state == tod_msgs::msg::TrajectoryGuidanceState::EXECUTING_TRAJECTORY) {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/wheel.png";
            sprintf(statusText, "%s", current_state_text.c_str());
        } else if (current_state == tod_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY) {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/stop.png";
            sprintf(statusText, "%s", current_state_text.c_str());
        } else {
            image_texture = tod_gl::RosInterface::get_package_path() + "/resources/icons/stop.png";
            sprintf(status_text, "%s", "No new messages");
        }

        GLuint icon_statusk_id = ImGuiSceneLayer::load_texture(image_texture.c_str());

        const float rounding = 5.0f;
        const ImVec2 padding = ImVec2(10.0f, 5.0f);

        ImGui::PushFont(DriveInfoFontSmall_);
        char tmp[32];
        sprintf(tmp, "%s", "Steering");
        ImVec2 text_size = ImGui::CalcTextSize(tmp);  // ensure that it's always the same lenght and long enough
        ImVec2 actual_text_size = ImGui::CalcTextSize(status_text);
        ImGui::PopFont();
        ImVec2 small_icon_size = ImVec2(20, 20);
        ImVec2 badge_size = ImVec2(text_size.x + small_icon_size.x + 3 * padding.x, text_size.y + 2 * padding.y);

        ImGui::SetNextWindowPos(position);
        ImGui::SetNextWindowSize(badge_size);
        ImGui::Begin("Badge", NULL,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 window_pos = ImGui::GetWindowPos();
        ImVec2 rect_min = ImVec2(window_pos.x + 10, window_pos.y + 2);
        ImVec2 rect_max = ImVec2(window_pos.x + badge_size.x - 5, window_pos.y + badge_size.y - 2);
        draw_list->AddRectFilled(rect_min, rect_max, color, rounding);

        ImVec2 text_pos =
            ImVec2(window_pos.x + padding.x + (text_size.x - actual_text_size.x) * 0.5f + 5, window_pos.y + padding.y);
        ImVec2 image_pos = ImVec2(window_pos.x + padding.x + text_size.x + padding.x, window_pos.y + padding.y);
        ImGui::SetCursorScreenPos(text_pos);
        ImGui::PushFont(DriveInfoFontSmall_);
        ImGui::Text(status_text);
        ImGui::PopFont();
        ImGui::SetCursorScreenPos(image_pos);
        ImGui::Image((void*)(intptr_t)icon_statusk_id, small_icon_size);

        ImGui::End();
    }

    void toggle_with_images(ImVec2 layer_pos) {
        ImGui::SetNextWindowPos(layer_pos);
        ImGui::SetNextWindowSize(ImVec2(180, 80));

        ImGui::Begin("drivingToggle", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoScrollbar);

        ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        float height = 30;  // ImGui::GetFrameHeight();
        float width = height * 1.55f;
        float padding = 10.0f;
        float image_size = height * 1.3f;

        GLuint icon_car_top_view_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/car_top_view.png").c_str());
        GLuint icon_person_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/person.png").c_str());

        float total_width = image_size * 2.3f + width + padding * 2.0f;

        draw_list->AddRectFilled(p, ImVec2(p.x + total_width + image_size * 0.1f, p.y + image_size * 1.45f),
                                 IM_COL32(26, 27, 28, 180), 5.0f);

        ImGui::SetCursorScreenPos(ImVec2(p.x + padding - 5, p.y + (image_size * 1.2f - height) / 2.0f));
        ImGui::Image((void*)(intptr_t)icon_car_top_view_id, ImVec2(image_size, image_size));

        ImGui::SetCursorScreenPos(
            ImVec2(p.x + image_size + padding, p.y + (image_size * 1.4f - height) * 0.5f + height * 0.2f));
        toggle_button("##toggle");

        ImGui::SetCursorScreenPos(
            ImVec2(p.x + image_size * 1.2f + width + padding * 2.0f, p.y + (image_size * 1.2f - height) / 2.0f));
        ImGui::Image((void*)(intptr_t)icon_person_id, ImVec2(image_size, image_size));

        ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + image_size));

        ImGui::End();
    }

    void toggle_button(const char* str_id) {
        // ImGui does not support toogle switches, so it will be created from scratch
        // https://github.com/ocornut/imgui/issues/1537
        ImVec2 regular_pos = ImGui::GetCursorScreenPos();
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        float regular_height = ImGui::GetFrameHeight();
        float height = regular_height * 0.8;
        ImVec2 p = regular_pos;
        float width = height * 2.5f;
        float radius = height * 0.4f;

        if (ImGui::InvisibleButton(str_id, ImVec2(width, height))) {
            controlOwnerToggle_ = !controlOwnerToggle_;
        }
        ImU32 col_bg;
        if (ImGui::IsItemHovered()) {
            col_bg = controlOwnerToggle_ ? IM_COL32(145 + 20, 211, 68 + 20, 255)
                                        : IM_COL32(218 - 20, 218 - 20, 218 - 20, 255);
        } else {
            col_bg = controlOwnerToggle_ ? IM_COL32(145, 211, 68, 255) : IM_COL32(218, 218, 218, 255);
}
        ImU32 white = IM_COL32(255, 255, 255, 255);

        draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height * 0.8f), white, height * 0.4f);
        draw_list->AddCircleFilled(ImVec2(controlOwnerToggle_ ? (p.x + width - radius) : (p.x + radius), p.y + radius),
                                   radius - 1.5f, IM_COL32(100, 156, 200, 255));
    }

  private:
    ImGuiIO& io_ = ImGui::GetIO();
    bool isExpanded_ = false;

    uint8_t last_event_ = 0;
    std::string last_event_text = "No new messages";
    uint8_t current_state_ = 0;
    std::string current_state_text = "No new messages";
    bool controlOwnerToggle_;
    float maxButtonSize_ = 40;
    ImFont* statusUnfoldedFont_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Regular.ttf").c_str(), 30.0f);
    ImFont* DriveInfoFontSmall_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 22.0f);
    ImFont* DriveInfoFontBoldSmall_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 20.0f);
};
}  // namespace tod_visual
/**
 * @file drive_info_layer.hpp
 * @brief Main UI Layer for the vehicle control
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "ui_layer.hpp"
#include "view_port_layer.hpp"

#include "tod_gl/events/key_codes.hpp"
#include "tod_gl/layers/docking_scene_layer.hpp"

#include "tod_gl/ros_interface/subscribing_components/joy_stick_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/network_metrics_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/primary_control_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/secondary_control_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/automation_status_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/primary_vehicle_state_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/secondary_vehicle_state_component.hpp"
#include "tod_gl/scene/camera_controller.hpp"

#include "tod_vehicle_msgs/VehicleEnums.h"
#include "tod_operator_msgs/joystickConfig.h"
#include "tod_operator_msgs/msg/key_press.hpp"

#include "sensor_msgs/msg/joy.hpp"

namespace tod_visual {
// TODO: What about this templating here? Does that do anything for us?
template <class PrimaryVehicleStateComp, class SecondaryVehicleStateComp, class ToDStatusComponent, class AutomationStatusComponent, class NetworkMetricsComponent, class JoyStickComp, class PrimaryControlComp, class SecondaryControlComp>

/**
 * @class DriveInfoLayer
 * @brief Provides a UI layer for displaying and controlling vehicle information such as speed, gear, indicators, etc.
 * @tparam PrimaryVehicleStateComp   The primary vehicle state component type.
 * @tparam SecondaryVehicleStateComp The secondary vehicle state component type.
 * @tparam ToDStatusComponent        The component type to retrieve ToD status.
 * @tparam AutomationStatusComponent The component type to retrieve automation status.
 * @tparam NetworkMetricsComponent   The component type to retrieve network metrics.
 * @tparam JoyStickComp              The component type to retrieve joystick state.
 * @tparam PrimaryControlComp        The component type to retrieve primary control commands.
 * @tparam SecondaryControlComp      The component type to retrieve secondary control state.
 * @copyright 2024 TUMFTM
 */
class DriveInfoLayer : public UILayer {
  public:
      /**
     * @brief Constructs a DriveInfoLayer.
     * @param ros Shared pointer to the RosInterface.
     * @param scene Shared pointer to the Scene.
     * @param view_port_layer Pointer to the associated ViewPortLayer.
     */
    DriveInfoLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene,
                   ViewPortLayer *view_port_layer)
        : UILayer(ros, scene, view_port_layer) {
        _name = "DriveInfoLayer";
    }
        /**
     * @brief Default destructor.
     */
    ~DriveInfoLayer() = default;
    /**
     * @brief Main ImGui render loop for the DriveInfoLayer UI elements.
     */
    virtual void on_im_gui_render() override {
        ImGuiIO &io = ImGui::GetIO();
        ImVec2 current_display_size = io.DisplaySize;
        ImVec2 current_window_size = ImGui::GetWindowSize();
        ImVec2 current_window_pos = ImGui::GetWindowPos();

        ImGuiViewport *main_viewport = ImGui::GetMainViewport();
        ImVec2 main_viewport_pos = main_viewport->Pos;

        ImVec2 window_pos = ImVec2(main_viewport_pos.x, main_viewport_pos.y + current_display_size.y * 0.85f);
        ImVec2 bottom_color_size = ImVec2(current_display_size.x, current_display_size.y * 0.35f);

        ImGui::SetNextWindowPos(window_pos);
        ImGui::SetNextWindowSize(ImVec2(current_display_size.x, current_display_size.y * 0.15f));

        ImGui::Begin("DriveInfo", nullptr,
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollWithMouse |
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoScrollbar);

        render_half_circle();

        render_color_gradient(bottom_color_size);

        draw_bottom_text();

        render_drive_info();

        ImGui::End();
    }
    /**
     * @brief Renders the half-circle visual at the bottom part of the UI.
     */
    void render_half_circle() {
        ImGuiIO &io = ImGui::GetIO();
        ImVec2 current_display_size = io.DisplaySize;
        ImVec2 current_window_size = ImGui::GetWindowSize();

        // Half circle for dock
        GLuint icon_half_circle_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/half-circle-full.png").c_str());
        float half_circle_height = std::min(current_display_size.y * halfCircleRatio_, 240.0f);
        ImVec2 half_circle_icon_size = ImVec2(current_display_size.x, half_circle_height);
        accountForSizeDifference_ = (current_display_size.y * halfCircleRatio_ > 240.0f)
                                       ? (current_display_size.y * halfCircleRatio_ - 240.0f)
                                       : 0.0f;
        ImVec2 half_circle_pos = ImVec2(0.0f, accountForSizeDifference_ - 10);
        ImGui::SetCursorPos(half_circle_pos);
        ImGui::Image((void *)(intptr_t)icon_half_circle_id, half_circle_icon_size);
    }
   /**
     * @brief Draws the bottom text and curved line graphic.
     */
    void draw_bottom_text() {
        ImGuiIO &io = ImGui::GetIO();
        ImVec2 current_display_size = io.DisplaySize;
        ImVec2 current_window_size = ImGui::GetWindowSize();

        // curved bottom line
        GLuint icon_half_circle_line_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/half-circle-line-slim.png").c_str());
        ImVec2 half_circle_line_icon_size = ImVec2(current_window_size.x * 0.7f, current_window_size.y * 0.17f);
        ImVec2 curved_line_pos =
            ImVec2((current_window_size.x - half_circle_line_icon_size.x) * 0.5f, current_window_size.y * 0.8f);
        ImGui::SetCursorPos(curved_line_pos);
        ImGui::Image((void *)(intptr_t)icon_half_circle_line_id, half_circle_line_icon_size);

        // bottom text "trajectory guidance"
        ImGui::PushFont(DriveInfoFont_);
        ImVec2 bottom_text_size = ImGui::CalcTextSize(vehicle_automation_state_.c_str());
        ImGui::PopFont();

        float padding = std::min(current_window_size.y * 0.05f, 15.0f);
        ImVec2 bottom_text_pos = ImVec2((current_window_size.x - bottom_text_size.x) * 0.5f,
                                      current_window_size.y * 0.95f - bottom_text_size.y + 10);
        ImGui::SetCursorPos(bottom_text_pos);
        ImGui::PushFont(DriveInfoFont_);
        ImGui::Text("%s", vehicle_automation_state_.c_str());
        ImGui::PopFont();
    }
   /**
     * @brief Renders the color gradient background depending on automation mode.
     * @param bottom_color_size The size of the gradient area to render.
     */
    void render_color_gradient(ImVec2 bottom_color_size) {
        ImGuiIO &io = ImGui::GetIO();
        ImVec2 current_display_size = io.DisplaySize;
        ImVec2 current_window_size = ImGui::GetWindowSize();
        ImVec2 color_gradient_pos = ImVec2(0.0f, 0.0f);

        // DEBUG // std::cout << "State " << vehicle_automation_state << std::endl;

        GLuint icon_bottom_color_unkown = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/white-bottom.png").c_str());
        GLuint icon_bottom_color_auto = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/green-bottom.png").c_str());
        GLuint icon_bottom_color_remote = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/blue-bottom.png").c_str());

        if (vehicle_automation_state_ == "AUTO") {
            ImGui::SetCursorPos(color_gradient_pos);
            ImGui::Image((void *)(intptr_t)icon_bottom_color_auto, bottom_color_size);
            return;
        }

        if (vehicle_automation_state_ == "REMOTE") {
            ImGui::SetCursorPos(color_gradient_pos);
            ImGui::Image((void *)(intptr_t)icon_bottom_color_remote, bottom_color_size);
            return;
        }

        ImGui::SetCursorPos(color_gradient_pos);
        ImGui::Image((void *)(intptr_t)icon_bottom_color_unkown, bottom_color_size);
    }
        /**
     * @brief Renders all UI elements related to drive information and control buttons.
     */

    void render_drive_info() {
        ImGuiIO &io = ImGui::GetIO();
        ImVec2 current_display_size = io.DisplaySize;
        ImVec2 current_window_size = ImGui::GetWindowSize();
        float available_width = current_display_size.x;
        float available_height = current_display_size.y / 7.5f;
        ImVec2 viewport_position = view_port_layer->getPos();

        float button_dimension = std::min(available_height * 0.3f, 70.0f);
        ImVec2 dynamic_button_size = ImVec2(button_dimension, button_dimension);
        // ImVec2 goButtonSize = ImVec2(dynamicButtonSize.x * 1.4f, dynamicButtonSize.x * 1.4f);
        interactiveButtons_ = dynamic_button_size;

        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        // Current speed + unit
        char velocity_text[32];
        sprintf(velocity_text, "%d", (int)(speed_));
        ImGui::PushFont(DriveInfoFontBig_);
        ImVec2 speed_size = ImGui::CalcTextSize(velocity_text);
        ImGui::PopFont();

        ImGui::PushFont(DriveInfoFont_);
        ImVec2 unit_size = ImGui::CalcTextSize("km/h");
        ImGui::PopFont();

        float text_total_height = speed_size.y + unit_size.y;
        float start_x = (available_width - speed_size.x) * 0.5f;
        float start_y = (available_height - text_total_height) * 0.5f;

        ImGui::SetCursorPos(ImVec2(start_x, start_y));
        ImGui::PushFont(DriveInfoFontBig_);
        ImGui::Text("%s", velocity_text);
        ImGui::PopFont();

        ImGui::SetCursorPos(ImVec2(start_x + (speed_size.x - unit_size.x) * 0.5f, start_y + speed_size.y * 0.75f));
        ImGui::PushFont(DriveInfoFont_);
        ImGui::Text("km/h");
        ImGui::PopFont();

        float speed_control_y = start_y + speed_size.y;

        // Get camera and cameraController for button functionality
        tod_gl::Entity vp_frame_buffer = _active_scene->find_entity_with_tag("ViewPortFramebuffer");
        tod_gl::FrameBufferComponent frame_buffer_component = vp_frame_buffer.get_component<tod_gl::FrameBufferComponent>();
        auto &camera = _active_scene->registry.get<tod_gl::CameraComponent>(frame_buffer_component.camera_entity);
        tod_gl::CameraController &controller = tod_gl::CameraController::get_instance();

        // set image paths
        std::string image_path = controller.get_top_view_on() ? "car_rear_view.png" : "car_top_view.png";
        GLuint icon_view_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/" + image_path).c_str());
        GLuint icon_target_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/target.png").c_str());
        GLuint icon_plus_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/plus.png").c_str());
        GLuint icon_minus_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/minus.png").c_str());
        GLuint icon_up_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/arrow_up.png").c_str());
        GLuint icon_sound_on_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/sound-on.png").c_str());
        GLuint icon_sound_mute_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/sound-mute.png").c_str());
        GLuint icon_current_sound_id = soundOn_ ? icon_sound_mute_id : icon_sound_on_id;
        std::string network_bars_path =
            tod_gl::RosInterface::get_package_path() + "/resources/icons/network-" + std::to_string(networkBars_) + ".png";
        GLuint icon_network_id = ImGuiSceneLayer::load_texture(network_bars_path.c_str());
        GLuint icon_mic_on_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/microphone-on.png").c_str());
        GLuint icon_mic_off_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/microphone-off.png").c_str());
        GLuint icon_current_mic_id = microphoneOn_ ? icon_mic_off_id : icon_mic_on_id;  // Show inverted values
        GLuint icon_down_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/arrow_down.png").c_str());
        // GLuint iconDeleteId = ImGuiSceneLayer::load_texture((tod_gl::RosInterface::get_package_path() +
        // "/resources/icons/close.png").c_str()); GLuint iconEditId =
        // ImGuiSceneLayer::load_texture((tod_gl::RosInterface::get_package_path() +
        // "/resources/icons/pencil.png").c_str()); GLuint iconStartId =
        // ImGuiSceneLayer::load_texture((tod_gl::RosInterface::get_package_path() + "/resources/icons/go.png").c_str());

        GLuint icon_light_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/light-blue.png").c_str());

        GLuint icon_arrow_left_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/arrow_left.png").c_str());
        GLuint icon_arrow_right_id = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/arrow_right.png").c_str());

        // GLuint iconArrowDownId = ImGuiSceneLayer::load_texture((tod_gl::RosInterface::get_package_path() +
        // "/resources/icons/arrow_down.png").c_str()); GLuint iconArrowUpId =
        // ImGuiSceneLayer::load_texture((tod_gl::RosInterface::get_package_path() +
        // "/resources/icons/arrow_up.png").c_str());

        std::string image_path_cam = controller.get_is_move_camera() ? "arrows.png" : "dotted-line.png";
        GLuint icon_mode = ImGuiSceneLayer::load_texture(
            (tod_gl::RosInterface::get_package_path() + "/resources/icons/" + image_path_cam).c_str());

        // set element positions and sizes
        float det_button_size = std::min(available_height - margin_, maxButtonSize_);
        ImVec2 button_size = ImVec2(det_button_size, det_button_size);
        float calc_zoom_button_size = det_button_size - det_button_size / 2 - zoomMargin_ / 2;
        ImVec2 zoom_button_size = ImVec2(calc_zoom_button_size, calc_zoom_button_size);
        float slider_height = style_.FramePadding.y * 2 + ImGui::GetTextLineHeightWithSpacing() - margin_;
        float offset_camera_buttons_from_circle = 22.0f;
        ImVec2 slider_pos = ImVec2(available_width - sliderLength_ - 50,
                                  (available_height - slider_height) * 0.8f + offset_camera_buttons_from_circle);
        ImVec2 up_pos = ImVec2(slider_pos.x - zoom_button_size.x - margin_,
                              (available_height - button_size.y) * 0.8f + offset_camera_buttons_from_circle);
        ImVec2 down_pos = ImVec2(slider_pos.x - zoom_button_size.x - margin_, up_pos.y + zoom_button_size.y + zoomMargin_);
        ImVec2 plus_pos = ImVec2(up_pos.x - zoom_button_size.x - margin_, up_pos.y);
        ImVec2 minus_pos = ImVec2(down_pos.x - zoom_button_size.x - margin_, down_pos.y);
        // ImVec2 editPos = ImVec2(plusPos.x - zoomButtonSize.x - margin, plusPos.y);
        // ImVec2 deletePos = ImVec2(minusPos.x - zoomButtonSize.x - margin, minusPos.y);
        ImVec2 target_pos = ImVec2(plus_pos.x - button_size.x - margin_, up_pos.y);
        ImVec2 perspective_pos = ImVec2(target_pos.x - button_size.x - margin_, up_pos.y);
        ImVec2 cam_pos = ImVec2(perspective_pos.x - button_size.x - margin_, up_pos.y);
        ImVec4 tint_color = ImVec4(0.14f, 0.59f, 0.75f, 1.0f);

        ImVec2 left_indicator_pos = ImVec2((available_width)*0.45f - (interactiveButtons_.x / 2.0f) - 5.0f,
                                         (available_height - interactiveButtons_.y) * 0.7f - 10.0f);
        ImVec2 right_indicator_pos = ImVec2((available_width)*0.55f - (interactiveButtons_.x / 2.0f) - 5.0f,
                                          (available_height - interactiveButtons_.y) * 0.7f - 10.0f);
        ImVec2 gear_pos = ImVec2(available_width * 0.65f, (available_height - interactiveButtons_.y) * 0.7f);
        ImVec2 sound_pos = ImVec2(available_width * 0.2f, (available_height - interactiveButtons_.y) * 0.75f);
        ImVec2 light_pos = ImVec2(available_width * 0.715f, (available_height - interactiveButtons_.y) * 0.6f);
        ImVec2 microphone_pos = ImVec2(available_width * 0.32f, (available_height - interactiveButtons_.y) * 0.65f);
        ImVec2 start_pos = ImVec2(available_width * 0.085f, (available_height - interactiveButtons_.y) * 0.9f);

        ImVec2 speed_buttons_pos = ImVec2(available_width * 0.56f, (available_height - interactiveButtons_.y) * 0.5f);

        ImVec2 network_pos = ImVec2(available_width * 0.78f, (available_height - interactiveButtons_.y) * 0.72f);

        ImVec2 increase_speed_pos =
            ImVec2((available_width - zoom_button_size.x) * 0.5f + 75, speed_control_y + zoom_button_size.y);
        ImVec2 decrease_speed_pos =
            ImVec2((available_width - zoom_button_size.x) * 0.5f - 75, speed_control_y + zoom_button_size.y);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 1.0f, 0.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 1.0f, 1.0f, 0.4f));
        ImGui::SetCursorPos(increase_speed_pos);
        if (ImGui::ImageButton((void *)(intptr_t)icon_up_id, zoom_button_size)) {
            // PublishIncreaseSpeed();
            std::cout << "Increasing Speed " << desired_speed_ << std::endl;
            if (desired_speed_ <= 3.7) {
                if (desired_speed_ == 0.0) {
                    desired_speed_ += 1.3;
                } else {
                    desired_speed_ += 0.1;
                }
                tod_operator_msgs::msg::KeyPress msg;
                msg.key = static_cast<int>(tod_gl::KeyCode::W);  // Send to the Path Creater and then logic handled in
                                                                 // TrajectoryGuidance Node Super ugly
                keypress_->publish(msg);
                std::cout << "Increasing Speed to " << desired_speed_ << std::endl;
            }
        }
        ImGui::SetCursorPos(decrease_speed_pos);
        if (ImGui::ImageButton((void *)(intptr_t)icon_down_id, zoom_button_size)) {
            // PublishDecreaseSpeed();
            std::cout << "Decreasing Speed " << desired_speed_ << std::endl;
            if (desired_speed_ > 0.0) {
                if (desired_speed_ <= 1.3) {
                    desired_speed_ = 0.0;
                } else {
                    desired_speed_ -= 0.1;
                }
                tod_operator_msgs::msg::KeyPress msg;
                msg.key = static_cast<int>(tod_gl::KeyCode::S);  // Send to the Path Creater and then logic handled in
                                                                 // TrajectoryGuidance Node Super ugly
                keypress_->publish(msg);
                std::cout << "Decreasing Speed to " << desired_speed_ << std::endl;
            }
        }
        ImGui::PopStyleColor(2);

        // Change between trajectory guidance and camera movement
        ImGui::SetCursorPos(cam_pos);
        if (render_image_button_with_tint(icon_mode, tint_color, button_size)) {
            controller.switch_is_move_camera();
            modeIsNotTrajectory_ = controller.isMoveCamera;
        }

        // latency + network quality
        int latency_value = static_cast<int>(std::round(latency_));
        char latency_text[32];
        sprintf(latency_text, "%d ms", latency_value);

        ImGui::PushFont(DriveInfoFont_);
        ImVec2 latency_text_size = ImGui::CalcTextSize(latency_text);
        ImVec2 latency_pos = ImVec2(network_pos.x, network_pos.y + interactiveButtons_.y * 0.8f);
        ImGui::PopFont();

        ImGui::SetCursorPos(latency_pos);
        ImGui::PushFont(DriveInfoFont_);
        ImGui::Text("%s", latency_text);
        ImGui::PopFont();
        ImGui::SetCursorPos(network_pos);
        ImVec2 network_size = ImVec2(interactiveButtons_.x * 0.8f, interactiveButtons_.y * 0.8f);
        ImGui::Image((void *)(intptr_t)icon_network_id, network_size);

        // perspective button
        ImGui::SetCursorPos(perspective_pos);
        if (ImGui::ImageButton((void *)(intptr_t)icon_view_id, button_size)) {
            controller.switch_view(camera);
        }

        // target button
        ImGui::SetCursorPos(target_pos);
        if (ImGui::ImageButton((void *)(intptr_t)icon_target_id, button_size)) {
            controller.back_to_car(camera);
            sliderValue_ = 0;
            lastSliderValue_ = 0;
        }

        // edit button
        // ImGui::SetCursorPos(editPos);
        // if (ImGui::ImageButton((void *)(intptr_t)iconEditId, zoomButtonSize))
        // {
        //     // Currently only placeholder
        // }

        // delete button
        // ImGui::SetCursorPos(deletePos);
        // if (ImGui::ImageButton((void *)(intptr_t)iconDeleteId, zoomButtonSize))
        // {
        //     tod_operator_msgs::msg::KeyPress msg;
        //     msg.key = static_cast<int>(KeyCode::Backslash); // 32
        //     _keypress->publish(msg);
        // }

        // angle buttons (some weird bug occurred where the buttons where not pressable, therefore this extensive
        // implementation)
        ImGui::SetCursorPos(up_pos);
        ImGui::PushID("up_button");
        ImGui::ImageButton((void *)(intptr_t)icon_up_id, zoom_button_size);

        if (ImGui::IsItemActive() && !upButtonActive_) {
            controller.move_up(1.0f); // artifical timestamp
            upButtonActive_ = true;
        } else if (!ImGui::IsItemActive() && upButtonActive_) {
            upButtonActive_ = false;
}

        ImGui::PopID();

        ImGui::SetCursorPos(down_pos);
        ImGui::PushID("down_button");
        ImGui::ImageButton((void *)(intptr_t)icon_down_id, zoom_button_size);

        if (ImGui::IsItemActive() && !downButtonActive_) {
            controller.move_down(1.0f); // artifical timestamp
            downButtonActive_ = true;
        } else if (!ImGui::IsItemActive() && downButtonActive_) {
            downButtonActive_ = false;
}

        ImGui::PopID();

        // zoom buttons
        ImGui::SetCursorPos(plus_pos);
        if (ImGui::ImageButton((void *)(intptr_t)icon_plus_id, zoom_button_size)) {
            controller.zoom_in(camera, 1.0f); // artifical timestamp
        }

        ImGui::SetCursorPos(minus_pos);
        if (ImGui::ImageButton((void *)(intptr_t)icon_minus_id, zoom_button_size)) {
            controller.zoom_out(camera, 1.0f); // artifical timestamp
        }

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 1.0f, 0.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 1.0f, 1.0f, 0.4f));

        bool is_blink_on = (blinkTimer_ / blinkInterval_) < 0.5f;
        ImVec4 yellow = ImVec4(1.0f, 186.0f / 255.0f, 0.0f, 1.0f);
        ImVec4 tint_color_indicator = is_blink_on ? yellow : ImVec4(1.0f, 1.0f, 1.0f, 0.5f);

        ImGui::SetCursorPos(left_indicator_pos);
        if (render_image_button_with_tint(
                icon_arrow_left_id, leftIndicatorOn_ ? tint_color_indicator : ImVec4(1.0f, 1.0f, 1.0f, 0.5f), button_size)) {
            leftIndicatorOn_ = !leftIndicatorOn_;
            publish_indicator_state();
            std::cout << " Indicator left" << std::endl;
        }

        ImGui::SetCursorPos(right_indicator_pos);
        if (render_image_button_with_tint(
                icon_arrow_right_id, rightIndicatorOn_ ? tint_color_indicator : ImVec4(1.0f, 1.0f, 1.0f, 0.5f), button_size)) {
            rightIndicatorOn_ = !rightIndicatorOn_;
            publish_indicator_state();
            std::cout << " Indicator right" << std::endl;
        }

        // gearDisplay button
        ImGui::SetCursorPos(gear_pos);
        ImGui::PushFont(DriveInfoFontBig_);
        // Gear changing shall not be available in GUI
        ImGui::InvisibleButton("##gear", interactiveButtons_);

        // Calculate the border position and size with padding
        ImVec2 gear_border_pos = ImGui::GetItemRectMin();
        ImVec2 gear_border_size = ImGui::GetItemRectSize();
        ImVec2 border_pos = ImVec2(gear_border_pos.x - 3, gear_border_pos.y - 3);
        ImVec2 border_size = ImVec2(gear_border_size.x + 6, gear_border_size.y + 6);

        // NOTE: the line has to be drawn at the end of this function

        // Render the current gearDisplay on the button (so it is centered)
        ImVec2 gear_button_size = ImGui::GetItemRectSize();
        ImVec2 gear_text_size = ImGui::CalcTextSize(gearDisplay_.c_str());
        ImVec2 gear_text_pos = ImGui::GetItemRectMin();
        gear_text_pos.x += (gear_button_size.x - gear_text_size.x) / 2.0f;
        gear_text_pos.y += (gear_button_size.y - gear_text_size.y) / 2.0f;
        ImGui::SetCursorScreenPos(gear_text_pos);
        ImGui::Text("%s", gearDisplay_.c_str());

        ImGui::PopFont();

        // sound button
        // ImGui::SetCursorPos(soundPos);
        // ImVec2 soundMicSize = ImVec2(interactiveButtons.x * 1.8f, interactiveButtons.y * 0.9f);
        // if (ImGui::ImageButton((void *)(intptr_t)iconCurrentSoundId, soundMicSize))
        // {
        //     soundOn = !soundOn;
        //     std::cout << "Pressed button " << std::endl;
        //     // Currently only placeholder
        // }

        // headlights button - don't show if light is off
        ImGui::SetCursorPos(light_pos);
        ImVec2 light_button_size = ImVec2(interactiveButtons_.x * 0.85f, interactiveButtons_.y * 0.6);
        if (lightStatus_ > 0) {
            ImGui::Image((void *)(intptr_t)icon_light_id, light_button_size);
        }

        // microphone button
        // ImGui::SetCursorPos(microphonePos);
        // if (ImGui::ImageButton((void *)(intptr_t)iconCurrentMicId, soundMicSize))
        // {
        //     microphoneOn = !microphoneOn;
        //     // Currently only placeholder
        // }

        // Start Button (hold pressed for GO, release for STOP)
        // ImGui::SetCursorPos(startPos);
        // ImGui::PushID("start_button");
        // ImGui::ImageButton((void *)(intptr_t)iconStartId, goButtonSize);

        // // check if pressed
        // if (ImGui::IsItemActive() && !startButtonActive)
        // {
        //     tod_operator_msgs::msg::KeyPress msg;
        //     msg.key = static_cast<int>(KeyCode::Enter); // 257
        //     _keypress->publish(msg);
        //     startButtonActive = true;
        //     std::cout << "Start has been pressed" << std::endl;
        // }

        // // check if released
        // if (!ImGui::IsItemActive() && startButtonActive && ImGui::IsItemDeactivated())
        // {
        //     tod_operator_msgs::msg::KeyPress msg;
        //     msg.key = static_cast<int>(KeyCode::Space); // 32 (stop command)
        //     _keypress->publish(msg);
        //     startButtonActive = false;
        //     std::cout << "Start has been released" << std::endl;
        // }

        // ImGui::PopID();

        ImGui::PopStyleColor(2);

        // slider
        ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(255, 255, 255, 0));
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, IM_COL32(255, 255, 255, 0));
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive, IM_COL32(255, 255, 255, 0));
        ImGui::PushStyleVar(ImGuiStyleVar_GrabMinSize, 2.0f);
        ImGui::PushItemWidth(sliderLength_);

        ImGui::SetCursorPos(slider_pos);
        ImGui::SliderFloat(" ", &sliderValue_, -1.0f, 1.0f, "");

        ImVec2 p = ImGui::GetItemRectMin();
        ImVec2 q = ImGui::GetItemRectMax();

        float line_y = (p.y + q.y) * 0.5f;
        draw_list->AddLine(ImVec2(p.x, line_y), ImVec2(p.x + sliderLength_, line_y), IM_COL32(255, 255, 255, 255), 1.0f);

        ImGui::PopItemWidth();
        ImGui::PopStyleVar(1);
        ImGui::PopStyleColor(3);

        // slider logic
        if (sliderValue_ != lastSliderValue_) {
            if (sliderValue_ > lastSliderValue_) {
                controller.rotate_camera_translation(1, camera, 1.0f);
            } else {
                controller.rotate_camera_translation(1, camera,-1.0f);
            }
            lastSliderValue_ = sliderValue_;
        }

        // Draw line around gearDisplay
        draw_list->AddRect(border_pos, ImVec2(border_pos.x + border_size.x, border_pos.y + border_size.y),
                           IM_COL32(255, 255, 255, 255), 10.0f, 0, 3.0f);
    }
    /**
     * @brief Called upon attaching the layer to the application.
     */
    virtual void on_attach() override {}
    /**
     * @brief Periodic update function to handle state changes, network metrics, and input.
     * @param ts The timestep since the last update.
     */
    virtual void on_update(float ts) override {
        update_network_bars();

        blinkTimer_ += ts;
        if (blinkTimer_ >= blinkInterval_) {
            blinkTimer_ -= blinkInterval_;
        }

        tod_gl::Entity subscription_manager = _active_scene->find_entity_with_tag("SubscriptionManager");

        if (subscription_manager.has_component<PrimaryControlComp>()) {
            PrimaryControlComp &comp = subscription_manager.get_component<PrimaryControlComp>();
            speed_ = 3.6f * comp.get_velocity();
        }

        if (subscription_manager.has_component<SecondaryVehicleStateComp>()) {
            SecondaryVehicleStateComp &comp = subscription_manager.get_component<SecondaryVehicleStateComp>();

            currentGear_ = comp.get_gear_position();

            lightStatus_ = comp.get_flash_light();

            switch (comp.get_indicator()) {
                case 0:
                    leftIndicatorOn_ = false;
                    rightIndicatorOn_ = false;
                    break;
                case 1:
                    leftIndicatorOn_ = true;
                    rightIndicatorOn_ = false;
                    break;
                case 2:
                    leftIndicatorOn_ = false;
                    rightIndicatorOn_ = true;
                    break;
                case 3:
                    leftIndicatorOn_ = true;
                    rightIndicatorOn_ = true;
                    break;
                default:
                    break;
            }
        }

        if (subscription_manager.has_component<ToDStatusComponent>()) {
            ToDStatusComponent &comp = subscription_manager.get_component<ToDStatusComponent>();
            drivingMode_ = comp.get_operator_control_mode_string();
        }

        if (subscription_manager.has_component<AutomationStatusComponent>()) {
            AutomationStatusComponent &comp = subscription_manager.get_component<AutomationStatusComponent>();
            vehicle_automation_state_ = comp.get_automation_status_string();
        }

        if (subscription_manager.has_component<SecondaryControlComp>()) {
            SecondaryControlComp &comp = subscription_manager.get_component<SecondaryControlComp>();
            gearDisplay_ = comp.get_gear_position_string();
        }

        if (subscription_manager.has_component<JoyStickComp>()) {
            JoyStickComp &comp = subscription_manager.get_component<JoyStickComp>();
            std::vector<int32_t> button_state = comp.get_buttons();

            if (!button_state.empty()) { //TODO: Add out_of_range checks before accessing the button states
                if (button_state[joystick::ButtonPos::INCREASE_SPEED] == 1 &&
                    prevButtonState_[joystick::ButtonPos::INCREASE_SPEED] == 0) {
                    if (desired_speed_ <= 37.) {
                        desired_speed_ += 1.;
                        tod_operator_msgs::msg::KeyPress msg;
                        msg.key = static_cast<int>(tod_gl::KeyCode::W);
                        keypress_->publish(msg);
                        std::cout << "Increasing Speed to " << desired_speed_ << std::endl;
                    }
                } else if (button_state[joystick::ButtonPos::DECREASE_SPEED] == 1 &&
                           prevButtonState_[joystick::ButtonPos::DECREASE_SPEED] == 0) {
                    if (desired_speed_ > 0.0) {
                        desired_speed_ -= 1.;
                        tod_operator_msgs::msg::KeyPress msg;
                        msg.key = static_cast<int>(tod_gl::KeyCode::S);
                        keypress_->publish(msg);
                        std::cout << "Decreasing Speed to " << desired_speed_ << std::endl;
                    }
                }
                prevButtonState_ = button_state;
            }
        }

        if (subscription_manager.has_component<NetworkMetricsComponent>()) {
            NetworkMetricsComponent &comp = subscription_manager.get_component<NetworkMetricsComponent>();
            link_quality_ = comp.get_link_quality();
            latency_ = comp.get_latency();
        }
    }
        /**
     * @brief Updates the number of network bars based on link quality.
     */

    void update_network_bars() {
        // TODO: Meaningful values ? were chosen arbitrary
        if (link_quality_ > 0.95) {
            networkBars_ = 4;
        } else if (link_quality_ > 0.9) {
            networkBars_ = 3;
        } else if (link_quality_ > 0.80) {
            networkBars_ = 2;
        } else if (link_quality_ > 0.70) {
            networkBars_ = 1;
        } else if (link_quality_ > 0.69) {
            networkBars_ = 5;  // one bar in red
        } else {
            networkBars_ = 0;
}
    }

  private:

    // TODO: Find a solution for Publisher to get the into the ros_interface and being able to access them in a scene and without
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr change_drive_info_ =
        _ros->create_publisher<sensor_msgs::msg::Joy>("output/joystick", 1);
    rclcpp::Publisher<tod_operator_msgs::msg::KeyPress>::SharedPtr keypress_ =
        _ros->create_publisher<tod_operator_msgs::msg::KeyPress>("output/key_press", 1);
    
    float sliderValue_ = 0;
    float lastSliderValue_ = sliderValue_;
    float margin_ = 12;
    float zoomMargin_ = 10;
    float sliderLength_ = 150;
    float maxButtonSize_ = 45;
    float interactiveButtonsSize_ = 26.0f;
    ImVec2 interactiveButtons_ = ImVec2(interactiveButtonsSize_, interactiveButtonsSize_);
    ImVec2 signSize_ = ImVec2(45.0f, 45.0f);
    double currentTime_ = ImGui::GetTime();

    bool modeIsNotTrajectory_ = true;
    bool controlOwnerToggle_ = false;
    bool startButtonActive_ = false;
    bool upButtonActive_ = false;
    bool downButtonActive_ = false;

    bool soundOn_ = true;
    bool microphoneOn_ = true;

    int8_t lightStatus_;
    int8_t indicator_;
    double link_quality_ = 0;
    int networkBars_ = 0;
    double latency_ = 0;

    std::string drivingMode_ = "TELEOPERATION";
    std::string vehicle_automation_state_ = "UNKNOWN";
    float speed_ = 0.0f;
    float desired_speed_ = 0.0;
    std::string gearDisplay_ = "D";
    int currentGearMsg_ = -1;
    int currentGear_{0};

    std::vector<int> prevButtonState_;

    bool leftIndicatorOn_ = false;
    bool rightIndicatorOn_ = false;
    float blinkTimer_ = 0.0f;
    const float blinkInterval_ = 0.5f;

    float halfCircleRatio_ = 0.15f;
    float accountForSizeDifference_ = 0.0f;
      /**
     * @brief Publishes the current indicator state via joystick-like message.
     */

    void publish_indicator_state() {
        sensor_msgs::msg::Joy msg;
        msg.buttons.resize(9);
        msg.buttons[joystick::ButtonPos::INDICATOR_LEFT] = leftIndicatorOn_ ? 1 : 0;
        msg.buttons[joystick::ButtonPos::INDICATOR_RIGHT] = rightIndicatorOn_ ? 1 : 0;
        change_drive_info_->publish(msg);
    }
    /**
     * @brief Publishes a "speed up" button state reset (not currently used).
     */
    void publish_increase_speed() {
        sensor_msgs::msg::Joy msg;
        msg.buttons.resize(9);
        msg.buttons[joystick::ButtonPos::INCREASE_SPEED] = 0;
        change_drive_info_->publish(msg);
    }

    /**
     * @brief Publishes a "slow down" button state reset (not currently used).
     */
    void publish_decrease_speed() {
        sensor_msgs::msg::Joy msg;
        msg.buttons.resize(9);
        msg.buttons[joystick::ButtonPos::DECREASE_SPEED] = 0;
        change_drive_info_->publish(msg);
    }

    ImGuiStyle &style_ = ImGui::GetStyle();
    ImGuiIO &io_ = ImGui::GetIO();
    ImFont *DriveInfoFontSmall_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 30.0f);
    ImFont *DriveInfoFont_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 40.0f);
    ImFont *DriveInfoFontMedium_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 60.0f);
    ImFont *DriveInfoFontBig_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 90.0f);
    ImFont *DriveInfoFontVeryBig_ = io_.Fonts->AddFontFromFileTTF(
        (tod_gl::RosInterface::get_package_path() + "/resources/fonts/opensans/OpenSans-Bold.ttf").c_str(), 120.0f);
    /**
     * @brief Helper for rendering an image with tint.
     * @param texture_id The texture ID to render.
     * @param tint_color The color tint.
     * @param size The size of the image.
     */
    void render_image_with_tint(GLuint texture_id, ImVec4 tint_color, ImVec2 size) {
        ImGui::Image((void *)(intptr_t)texture_id, size, ImVec2(0, 0), ImVec2(1, 1), tint_color);
    }
        /**
     * @brief Helper for rendering an image button with tint.
     * @param texture_id The texture ID to render.
     * @param tint_color The color tint.
     * @param size The size of the button.
     * @return True if the button was clicked, false otherwise.
     */

    bool render_image_button_with_tint(GLuint texture_id, ImVec4 tint_color, ImVec2 size) {
        return ImGui::ImageButton((void *)(intptr_t)texture_id, size, ImVec2(0, 0), ImVec2(1, 1), -1, ImVec4(0, 0, 0, 0),
                                  tint_color);
    }
};

}  // namespace tod_visual

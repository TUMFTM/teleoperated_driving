/**
 * @file speed_layer.cpp
 * @brief Implementation of the SpeedLayer class for visualizing current and desired speeds. (Source File)
 *        Splits the template implementation out of the header, with an example instantiation at the end
 *        to allow separate compilation for specific template types if desired.
 *
 * @copyright 2024 TUMFTM
 */

 #include "speed_layer.hpp"
 #include "imgui/imgui.h"
 
 namespace tod_visual {
 
 /**
  * @brief Constructor for the SpeedLayer class.
  * @param ros Shared pointer to the ROS interface.
  * @param scene Shared pointer to the scene.
  * @param split_dir The docking direction.
  * @param name The name of the layer.
  */
 template <class T1, class T2>
 SpeedLayer<T1, T2>::SpeedLayer(std::shared_ptr<tod_gl::RosInterface> ros,
                                std::shared_ptr<tod_gl::Scene> scene,
                                ImGuiDir split_dir,
                                std::string name)
     : DockingSceneLayer(ros, scene, split_dir) {
     _name = name;
 }
 
 /**
  * @brief Default destructor.
  */
 template <class T1, class T2>
 SpeedLayer<T1, T2>::~SpeedLayer() = default;
 
 /**
  * @brief Renders the vertical speed bar UI elements.
  */
 template <class T1, class T2>
 void SpeedLayer<T1, T2>::on_im_gui_render() {
     // Begin dock space window and layer window
     ImGui::Begin(_dock_space_window_name.c_str());
     ImGui::Begin(_name.c_str());
 
     ImVec2 windowSize = ImGui::GetContentRegionAvail();
     float adjustedY = windowSize.y - 20;
     ImVec2 pos = ImGui::GetCursorScreenPos();
 
     // Speed unit text
     const char *speedUnit = "km/h";
     ImVec2 text_size = ImGui::CalcTextSize(speedUnit);
     ImVec2 text_pos = ImVec2(windowSize.x / 2.0f - 5.0f, 0);
 
     ImGui::SetCursorPos(text_pos);
     ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.5f), speedUnit);
 
     // Calculate width of max speed label
     char largestLabel[10];
     sprintf(largestLabel, "%d", (int)maxSpeed);
     ImVec2 largestLabelSize = ImGui::CalcTextSize(largestLabel);
 
     // Margin based on largest number
     float margin = largestLabelSize.x + 3.0f;
 
     // Determine bar position (left or right)
     bool isLeft = _name == "Speed_L";
     ImVec2 barPos;
     if (isLeft) {
         barPos = ImVec2(pos.x + margin, pos.y + 20);
     } else {
         barPos = ImVec2(pos.x + windowSize.x - margin - barWidth, pos.y + 20);
     }
 
     // Compute fill heights
     float speedFraction = currentVelocity / maxSpeed;
     float filledHeight  = adjustedY * speedFraction;
     float desiredHeight = adjustedY * (desiredVelocity / maxSpeed);
 
     ImVec2 filledPos  = ImVec2(barPos.x, barPos.y + adjustedY - filledHeight);
     ImVec2 desiredPos = ImVec2(barPos.x, barPos.y + adjustedY - desiredHeight);
 
     ImDrawList *draw_list = ImGui::GetWindowDrawList();
 
     // Draw the background bar
     draw_list->AddRectFilled(
         barPos,
         ImVec2(barPos.x + barWidth, barPos.y + adjustedY),
         IM_COL32(50, 50, 50, 255));
 
     // Desired speed overlay (semi-transparent)
     draw_list->AddRectFilled(
         desiredPos,
         ImVec2(barPos.x + barWidth, barPos.y + adjustedY),
         IM_COL32(0, 175, 0, 50));
 
     // Current speed overlay (solid)
     draw_list->AddRectFilled(
         filledPos,
         ImVec2(barPos.x + barWidth, barPos.y + adjustedY),
         IM_COL32(0, 150, 0, 255));
 
     // Red line at max speed
     if (currentVelocity > (maxSpeed - 0.1f)) {
         draw_list->AddLine(
             ImVec2(barPos.x, barPos.y),
             ImVec2(barPos.x + barWidth + 1, barPos.y),
             IM_COL32(255, 0, 0, 255), 3.0f);
     }
 
     // Speed markers (every 5 km/h)
     for (int speed = 0; speed <= (int)maxSpeed; speed += 5) {
         float y = barPos.y + adjustedY - (adjustedY * speed / maxSpeed);
         char speedLabel[10];
         sprintf(speedLabel, "%d", speed);
         ImVec2 speedLabelSize = ImGui::CalcTextSize(speedLabel);
         ImVec2 textPos;
 
         if (isLeft) {
             textPos = ImVec2(barPos.x - margin, y - speedLabelSize.y / 2);
         } else {
             textPos = ImVec2(barPos.x + barWidth + 6, y - speedLabelSize.y / 2);
         }
 
         // Ticks
         draw_list->AddLine(
             ImVec2(barPos.x + barWidth / 2.0f, y),
             ImVec2(barPos.x + barWidth / 2.0f + (isLeft ? -10.0f : 10.0f), y),
             IM_COL32(255, 255, 255, 255));
 
         // Labels
         draw_list->AddText(textPos, IM_COL32(255, 255, 255, 255), speedLabel);
     }
 
     // End windows
     ImGui::End();
     ImGui::End();
 }
 
 /**
  * @brief Called when the layer is attached (unused).
  */
 template <class T1, class T2>
 void SpeedLayer<T1, T2>::on_attach() {
     // Additional setup if needed
 }
 
 /**
  * @brief Forwards events to the ImGuiSceneLayer for default handling.
  * @param e The event reference.
  */
 template <class T1, class T2>
 void SpeedLayer<T1, T2>::on_event(tod_gl::Event &e) {
     ImGuiSceneLayer::on_event(e);
 }
 
 /**
  * @brief Updates speed data from subscribed components.
  * @param ts The time step (unused).
  */
 template <class T1, class T2>
 void SpeedLayer<T1, T2>::on_update(float ts) {
     tod_gl::Entity SubscriptionManager = _active_scene->find_entity_with_tag("SubscriptionManager");
     if (SubscriptionManager.has_component<T1>()) {
         T1 &controlComp = SubscriptionManager.get_component<T1>();
         desiredVelocity = 3.6f * controlComp.Speed;  // Convert from m/s to km/h if needed
     }
     if (SubscriptionManager.has_component<T2>()) {
         T2 &dataComp = SubscriptionManager.get_component<T2>();
         currentVelocity = 3.6f * dataComp.Speed;
     }
 }
 
 /**
  * @brief Template destructor.
  */
 template <class T1, class T2>
 SpeedLayer<T1, T2>::~SpeedLayer() = default;
 
 }  // namespace tod_visual
 

/**
 * @file camera_position_layer.cpp
 * @brief Implementation of the CameraPositionLayer class to manage and display camera segments in the UI.
 * @copyright 2024 TUMFTM
 */

#include "camera_position_layer.hpp"
std::vector<bool> segment_states;
std::vector<bool> segment_disabled;
std::vector<bool> segment_mouse_states;

GLuint texture_id = 0;

// Constructor
/**
 * @brief Constructs a CameraPositionLayer instance.
 * @param ros Shared pointer to the ROS interface.
 * @param cams Shared pointer to the list of shared cameras.
 */
CameraPositionLayer::CameraPositionLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<std::vector<std::shared_ptr<SharedCam>>> cams,ImGuiDir split_dir)
: VideoManagerDockingLayer(ros, split_dir), _shared_cams_list(cams), _status(ros) {
    _name = "CameraPositionLayer";
    split_direction = split_dir;
    initialize_segments();



}





// Utility Function
/**
 * @brief Calculates a point on an ellipse for a given angle.
 * @param cx X-coordinate of the ellipse center.
 * @param cy Y-coordinate of the ellipse center.
 * @param rx Radius of the ellipse along the X-axis.
 * @param ry Radius of the ellipse along the Y-axis.
 * @param angle Angle in radians.
 * @return ImVec2 Point on the ellipse.
 */
ImVec2 CalculateEllipsePoint(float cx, float cy, float rx, float ry, float angle) {
    return ImVec2(cx + rx * cos(angle), cy + ry * sin(angle));
}

// Member Functions

/**
 * @brief Called when the layer is attached to the application.
 */
void CameraPositionLayer::on_attach() {

}

/**
 * @brief Called when the layer is detached from the application.
 */
void CameraPositionLayer::on_detach() {}

/**
 * @brief Initializes the segments with default states.
 */
void CameraPositionLayer::initialize_segments() {
    segment_states.resize(6, false);
    segment_disabled.resize(6, true);
    segment_mouse_states.resize(6, false);
}

/**
 * @brief Renders the ImGui elements for the layer.
 */
void CameraPositionLayer::on_im_gui_render() {


        ImGui::SameLine();
        ImGui::Begin(_dock_space_window_name.c_str());
        ImGui::SameLine();
        ImGui::Begin(_name.c_str());

    // Manage disabled states based on TOD status
    if (_status.get_tod_status() == 0) {
      //ro  ImGui::BeginDisabled();
        for (size_t i = 0; i < segment_disabled.size(); ++i) {
            segment_disabled[i] = true;
        }
    } else {
        for (size_t i = 0; i < segment_disabled.size(); ++i) {
            segment_disabled[i] = false;
        }
    }

    // Synchronize segment states with camera states
    for (size_t i = 0; i < segment_states.size(); ++i) {
        for (const auto& cam : *_shared_cams_list) {
            if (cam->mapping == static_cast<int>(i + 1)) {
                segment_states[i] = cam->is_active;
                break;
            }
        }
    }

    // Render the segments
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
      float padding = std::min(canvas_size.x, canvas_size.y) * 0.10f;

    // Reduce available size by the padding
    float available_width = canvas_size.x - 2 * padding;
    float available_height = canvas_size.y - 2 * padding;


    // Center position for the segments and the icon
    float cx = canvas_pos.x + padding + available_width / 2;
    float cy = canvas_pos.y + padding + available_height / 2;

    // Dynamic radii for the segments
    float rx = available_width / 2 - padding;  // Dynamic X-radius
    float ry = available_height / 2 - padding;  // Dynamic Y-radius

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const int num_segments = 6;
    float angle_step = 2.0f * M_PI / num_segments;

    for (int i = 0; i < num_segments; i++) {
        // Check if segment has mapping
        bool has_mapping = false;
        for (const auto& cam : *_shared_cams_list) {
            if (cam->mapping == i + 1) {
                has_mapping = true;
                break;
            }
        }
        if (!has_mapping) continue;

        // Calculate segment points
        float start_angle = i * angle_step;
        float end_angle = (i + 1) * angle_step;
        const int arc_points = 30;
        std::vector<ImVec2> segment_points;
        segment_points.push_back(ImVec2(cx, cy));

        for (int j = 0; j <= arc_points; j++) {
            float t = start_angle + j * (end_angle - start_angle) / arc_points;
            segment_points.push_back(ImVec2(cx + rx * cos(t), cy + ry * sin(t)));
        }

        // Determine hover and fill colors
        ImVec2 mouse_pos = ImGui::GetMousePos();
        bool is_hovered = !segment_disabled[i] && is_point_in_polygon(mouse_pos, segment_points);

        ImU32 fill_color;
        if (segment_disabled[i]) {
            fill_color = IM_COL32(128, 128, 128, 255);
        } else if (is_hovered) {
            fill_color = IM_COL32(0, 82, 147, 255);
        } else {
            fill_color = segment_states[i] ? IM_COL32(152, 198, 234, 255) : IM_COL32(0, 51, 89, 255);
        }

        draw_list->AddConvexPolyFilled(segment_points.data(), segment_points.size(), fill_color);
        draw_list->AddPolyline(segment_points.data(), segment_points.size(), IM_COL32(255, 255, 255, 255), true, 2.0f);

        // Handle mouse click
        if (!segment_disabled[i]) {
            bool is_mouse_down = ImGui::IsMouseClicked(ImGuiMouseButton_Left);
            if (!segment_mouse_states[i] && is_mouse_down && is_point_in_polygon(mouse_pos, segment_points)) {
                segment_states[i] = !segment_states[i];
                std::cout << "Segment " << (i + 1) << " clicked!" << std::endl;

                int mapping = i + 1;
                for (const auto& cam : *_shared_cams_list) {
                    if (cam->mapping == mapping) {
                        cam->is_active = !cam->is_active;
                        break;
                    }
                }
            }
            segment_mouse_states[i] = is_mouse_down;
        }
    }

    // Render car image at center
     std::string imagePath = "Edgar_TopView.png";
GLuint edgarIcon = ImGuiLayer::load_texture(
    (tod_gl::RosInterface::get_package_path() + "/resources/icons/" + imagePath).c_str());

        float image_size_x = (rx+ry)/2*0.150f*5;
        float image_size_y = (rx+ry)/2*0.190f*5;
        ImVec2 image_pos = ImVec2(cx - image_size_x / 2, cy - image_size_y / 2);

        draw_list->AddImage((void*)(intptr_t)edgarIcon, image_pos, ImVec2(image_pos.x + image_size_x, image_pos.y + image_size_y));
    

    render_buttons_missing_cams();
    //if (_status.get_tod_status() == 0) ImGui::EndDisabled();
    ImGui::End();
    ImGui::End();
}

/**
 * @brief Checks if a point is inside a polygon.
 * @param point The point to check.
 * @param polygon The polygon defined by a vector of points.
 * @return True if the point is inside, false otherwise.
 */
bool CameraPositionLayer::is_point_in_polygon(const ImVec2& point, const std::vector<ImVec2>& polygon) {
    int num_vertices = polygon.size();
    bool inside = false;

    for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
        const ImVec2& vi = polygon[i];
        const ImVec2& vj = polygon[j];

        if (((vi.y > point.y) != (vj.y > point.y)) &&
            (point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x)) {
            inside = !inside;
        }
    }

    return inside;
}


/**
 * @brief Renders buttons for cameras without mapping.
 */
void CameraPositionLayer::render_buttons_missing_cams() {
    for (const auto& cam : *_shared_cams_list) {
        if (cam->mapping == 0) {
            bool isChecked = cam->is_active;
            std::string checkbox_label = cam->name;
            ImGui::SameLine();
            if (ImGui::Checkbox(checkbox_label.c_str(), &isChecked)) {
                cam->is_active = isChecked;
            }
        }
    }
}

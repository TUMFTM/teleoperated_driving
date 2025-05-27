/**
 * @file view_port_layer.cpp
 * @brief The ViewportLayer turns the scenes framebuffer into a renderable imGui texture and manages the mouse clicks
 * @copyright 2024 TUMFTM
**/

#include "view_port_layer.hpp"

#include "tod_gl/core/cursor_position.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/camera_system.hpp"

namespace tod_visual {

ViewPortLayer::ViewPortLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene,
                             ImGuiDir split_dir)
    : DockingSceneLayer(ros, scene, split_dir) {
    _name = "ViewPortLayer";
}

void ViewPortLayer::on_im_gui_render() {
    ImGui::Begin(_dock_space_window_name.c_str());

    ImGui::Begin(_name.c_str());
    tod_gl::Entity vp_frame_buffer = _active_scene->find_entity_with_tag("ViewPortFramebuffer");

    unsigned int texture_id =
        vp_frame_buffer.get_component<tod_gl::RenderableElementComponent>().meshes.front().textures.front().id;
    tod_gl::FrameBufferComponent frame_buffer_component = vp_frame_buffer.get_component<tod_gl::FrameBufferComponent>();

    auto &cursor_position = tod_gl::CursorPosition::get();

    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (avail.x != frame_buffer_component.render_width || avail.y != frame_buffer_component.render_height) {
        frame_buffer_component.render_width = avail.x;
        frame_buffer_component.render_height = avail.y;
        auto &camera = _active_scene->registry.get<tod_gl::CameraComponent>(frame_buffer_component.camera_entity);
        tod_gl::CameraSystem::on_window_size_changed(camera, avail.x, avail.y);
        cursor_position.set_viewport(avail.x, avail.y);
    }

    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 cursor_screen_pos = ImGui::GetCursorScreenPos();

    cursor_position.isOverViewport = mouse_pos.x >= window_pos.x && mouse_pos.x < window_pos.x + avail.x &&
                                    mouse_pos.y >= window_pos.y && mouse_pos.y < window_pos.y + avail.y;

    ImVec2 viewport_pos = ImGui::GetWindowViewport()->Pos;

    if (cursor_position.isOverViewport) {
        auto other_mouse_pos_x = mouse_pos.x - cursor_screen_pos.x;
        auto other_mouse_pos_y = mouse_pos.y - cursor_screen_pos.y;
        cursor_position.set_mouse_position(other_mouse_pos_x, other_mouse_pos_y, true);
    } else {
        cursor_position.set_mouse_position(mouse_pos.x - window_pos.x, mouse_pos.y - window_pos.y, false);
    }

    _block_events = !ImGui::IsWindowHovered();
    ImGui::Image(reinterpret_cast<void *>(texture_id),
                 ImVec2{static_cast<float>(frame_buffer_component.render_width),
                        static_cast<float>(frame_buffer_component.render_height)},
                 ImVec2{0, 1}, ImVec2{1, 0});

    ImGui::End();  // ViewPort

    ImGui::End();  // dockspace
}

void ViewPortLayer::on_attach() {
    _shader = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video.frag").c_str());

    SetFrameBuffer();
}

void ViewPortLayer::SetFrameBuffer() {
    tod_gl::Entity view_port_frame_buffer = _active_scene->create_entity("ViewPortFramebuffer");
    auto &view_port_fb_component = view_port_frame_buffer.add_component<tod_gl::FrameBufferComponent>(false);

    // Set higher resolution values
    int high_res_width = 3840;   // 4K 16:9
    int high_res_height = 2160;  // 4K 16:9
    view_port_fb_component.render_width = high_res_width;
    view_port_fb_component.render_height = high_res_height;

    unsigned int shader_program = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/shader.frag").c_str());
    tod_gl::Mesh mesh = tod_gl::Mesh::non_empty_mesh();
    mesh.textures.emplace_back(view_port_fb_component.render_width, view_port_fb_component.render_height, "mainTexture",
                               GL_TEXTURE_2D, GL_RGB, GL_RGB);

    auto &renderable =
        view_port_frame_buffer.add_component<tod_gl::RenderableElementComponent>(shader_program, mesh, GL_TRIANGLES);

    auto &main_frame_bf =
        _active_scene->find_entity_with_tag("MainFramebuffer").get_component<tod_gl::FrameBufferComponent>();

    view_port_fb_component.camera_entity = main_frame_bf.camera_entity;
    // don't render on mainFrameBuffer anymore since we have a viewport now.
    main_frame_bf.should_render = false;
}

void ViewPortLayer::on_event(tod_gl::Event &e) {
    ImGuiSceneLayer::on_event(e);
}

ImVec2 ViewPortLayer::getPos() {
    return _window_position;
}

ImVec2 ViewPortLayer::getSize() {
    return _window_size;
}

ImVec2 ViewPortLayer::getAvail() {
    return _avail;
}
}  // namespace tod_visual
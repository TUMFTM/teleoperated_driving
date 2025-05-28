/**
 * @file video_layer_new.cpp
 * @brief Implementation of VideoLayer template class
 * @copyright 2024 TUMFTM
**/

#include "video_layer.hpp"

#pragma once

namespace tod_visual {

template <class ImageComp>
VideoLayer<ImageComp>::VideoLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene, 
                                  ImGuiDir split_dir, std::string name)
    : tod_gl::DockingSceneLayer(ros, scene, split_dir) {
    _name = name;    
}

template <class ImageComp>
VideoLayer<ImageComp>::~VideoLayer() {
    tod_gl::Renderer::delete_texture(texture_);
}

template <class ImageComp>
bool VideoLayer<ImageComp>::is_video_layer() const {
    return true;
}

template <class ImageComp>
void VideoLayer<ImageComp>::on_im_gui_render() {
    if (!show_videos) {
        return;
    }
    ImGui::SameLine();
    ImGui::Begin(_dock_space_window_name.c_str());
    ImGui::SameLine();
    ImGui::Begin(_name.c_str());

    if (ImGui::IsWindowCollapsed()) {
        ImGui::End();
        ImGui::End();
        return;
    }

    handle_keyboard_input();
    render_video_texture();

    ImGui::End();  // Video
    ImGui::End();  // Dockspace
}

template <class ImageComp>
void VideoLayer<ImageComp>::on_attach() {
    shader_ = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video_layer.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video_rgb.frag").c_str());

    front_buffer_ = tod_gl::Buffer(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW);
    back_buffer_ = tod_gl::Buffer(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW);

    // Initialize texture
    texture_ = tod_gl::Texture(width_, height_, "video_texture", GL_TEXTURE_2D, GL_RGB8, GL_RGB);
    tod_gl::Renderer::generate_texture(texture_, nullptr, shader_, video_layer_texture_unit);

    // Initialize pixel buffer
    tod_gl::Renderer::create_buffer(front_buffer_, nullptr, width_ * height_ * 3);
    tod_gl::Renderer::create_buffer(back_buffer_, nullptr, width_ * height_ * 3);
}

template <class ImageComp>
void VideoLayer<ImageComp>::on_event(tod_gl::Event &e) {
    ImGuiSceneLayer::on_event(e);
}

template <class ImageComp>
void VideoLayer<ImageComp>::on_update(float ts) {
    tod_gl::Entity subscription_manager = _active_scene->find_entity_with_tag("SubscriptionManager");
    if (subscription_manager.has_component<ImageComp>()) {
        const sensor_msgs::msg::Image &image = subscription_manager.get_component<ImageComp>().image;
        
        if (!image.data.empty()) {
            width_ = image.width;
            height_ = image.height;
            update_video_texture(image);
        }
    }
}

template <class ImageComp>
void VideoLayer<ImageComp>::handle_keyboard_input() {
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
            offset_y_ = std::max(0.0f, offset_y_ - offset_increment_);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
            offset_y_ += offset_increment_;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
            should_video_fit_to_window_ = false;
            custom_scaling_ = std::max(0.5f, custom_scaling_ - scaling_increment_);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
            custom_scaling_ += scaling_increment_;
            should_video_fit_to_window_ = false;
        }
    }
}

template <class ImageComp>
void VideoLayer<ImageComp>::update_video_texture(const sensor_msgs::msg::Image& image) {
    // Resize buffer if needed
    if (width_ != texture_.width || height_ != texture_.height) {
        texture_.width = width_;
        texture_.height = height_;
        tod_gl::Renderer::delete_texture(texture_);
        tod_gl::Renderer::generate_texture(texture_, nullptr, shader_, video_layer_texture_unit);
        tod_gl::Renderer::create_buffer(front_buffer_, nullptr, width_ * height_ * 3);
    }

    tod_gl::ShaderSystem::use_shader_program(shader_);
    
    tod_gl::Buffer& current_buffer = using_front_buffer_ ? back_buffer_ : front_buffer_;

    // Update texture using buffer
    tod_gl::Renderer::update_texture(
        texture_,
        video_layer_texture_unit,
        current_buffer,
        0, 0,
        width_, height_,
        (void*)image.data.data()
    );
    
    tod_gl::ShaderSystem::use_shader_program(0);

    using_front_buffer_ = !using_front_buffer_;
}

template <class ImageComp>
void VideoLayer<ImageComp>::render_video_texture() {
    ImVec2 avail = ImGui::GetContentRegionAvail();
    calculate_display_dimensions(avail);

    float off_x = ((avail.x - width_) * 0.5f);
    off_x = std::max(0.0f, off_x);
    
    ImVec2 cursor_pos = ImGui::GetCursorPos();
    ImGui::SetCursorPos(ImVec2(cursor_pos.x + off_x, cursor_pos.y + offset_y_));

    if (texture_.id != 0) {
        tod_gl::ShaderSystem::use_shader_program(shader_);
        tod_gl::RenderCommand::ForTexture::active_and_bind(texture_, video_layer_texture_unit);
        
        ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(texture_.id)),
                    ImVec2(width_, height_));
        
        tod_gl::RenderCommand::ForTexture::unbind(texture_);
        tod_gl::ShaderSystem::use_shader_program(0);
    }
}

template <class ImageComp>
void VideoLayer<ImageComp>::calculate_display_dimensions(const ImVec2& available_space) {
    if (should_video_fit_to_window_) {
        float ratio = width_ / height_;
        width_ = available_space.y * ratio;
        height_ = available_space.y;
    } else {
        float ratio = width_ / height_;
        width_ = available_space.y * ratio * custom_scaling_;
        height_ = available_space.y * custom_scaling_;
    }
}




} // namespace tod_visual
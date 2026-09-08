/**
 * @file video_layer_new.hpp
 * @brief Handles the rendering of video streams in the UI. Manages video textures, scaling, and rendering logic.
 *        Supports multiple video sources and ensures efficient texture updates.
 * @copyright 2024 TUMFTM
**/


#pragma once

#include <algorithm>

#include "glad/glad.h"

#include "tod_gl/layers/docking_scene_layer.hpp"

#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/renderer/renderer.hpp"
#include "tod_gl/renderer/renderer_command.hpp"
#include "tod_gl/ros_interface/subscribing_components/image_component.hpp"
#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/scene/components.hpp"


namespace tod_visual {

/**
 * @class VideoLayer
 * @brief Manages video streaming and rendering within the UI.
 * @tparam ImageComp The type of image component used for video rendering.
 */
template <class ImageComp>
class VideoLayer : public tod_gl::DockingSceneLayer {
  public:
      /**
     * @brief Constructs a VideoLayer instance.
     * @param ros Shared pointer to the ROS interface.
     * @param scene Shared pointer to the scene.
     * @param split_dir ImGui docking direction.
     * @param name The name of the video layer.
     */
    VideoLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene, ImGuiDir split_dir,
               std::string name);
    //~VideoLayer() = default;
    ~VideoLayer();
    virtual bool is_video_layer() const override;
    virtual void on_im_gui_render() override;
    virtual void on_attach() override;
    virtual void on_event(tod_gl::Event &e) override;
    virtual void on_update(float ts) override;

  private:
    static const GLuint video_layer_texture_unit = 15;
    const std::string video_window_ = "Video";
    unsigned int shader_;
    tod_gl::Texture texture_;
    tod_gl::Buffer front_buffer_;
    tod_gl::Buffer back_buffer_;
    bool using_front_buffer_{false};

    unsigned int width_ = 960;
    unsigned int height_ = 600;
    float display_width_ = 960.0f;
    float display_height_ = 600.0f;
    float offset_y_ = 0.0f;
    float custom_scaling_ = 1.0f;
    const float offset_increment_ = 10.0f;
    const float scaling_increment_ = 0.1f;
    bool should_video_fit_to_window_ = false;

    // offset image if arrow keys are pressed
    //  Check if the window is focused
    void handle_keyboard_input();
    void update_video_texture(const sensor_msgs::msg::Image& image);
    void render_video_texture();
    void calculate_display_dimensions(const ImVec2& available_space);


};
}  // namespace tod_visual

template class tod_visual::VideoLayer<tod_gl::ImageComponentFrontCenter>;
template class tod_visual::VideoLayer<tod_gl::ImageComponentFrontLeft>;
template class tod_visual::VideoLayer<tod_gl::ImageComponentFrontRight>;
template class tod_visual::VideoLayer<tod_gl::ImageComponentRearLeft>;
template class tod_visual::VideoLayer<tod_gl::ImageComponentRearRight>;
template class tod_visual::VideoLayer<tod_gl::ImageComponentRearCenter>;


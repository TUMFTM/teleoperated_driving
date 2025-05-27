/**
 * @file opengl_renderer_api.hpp
 * @brief OpenGL abstraction for the rendering of the scene
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 **/

#pragma once

#include "tod_gl/renderer/renderer_api.hpp"

namespace tod_gl {

/*
* @brief OpenGL Implemenation of the renderer API of the graphics library. Currently OpenGL is the only supported backend
*/
class OpenGLRendererAPI : public RendererAPI {
  public:
    void init() override;
    void enable(const uint32_t capability) override;
    void set_blend_func(const uint32_t sfactor, const uint32_t dfactor) override;
    void set_viewport(const uint32_t x, const uint32_t y, const uint32_t width, const uint32_t height) override;

    void set_clear_color(const glm::vec4 &color) override;
    void clear() override;
    void set_line_width(const float linewidth) override;
    void set_point_size(const float pointsize) override;
    void set_pixel_storage_mode(const uint32_t pname, const int32_t param) override;
    void draw_elements(const uint32_t mode, const uint32_t count, const uint32_t type, const void *data) override;
    void draw_arrays(const uint32_t mode, const uint32_t first, const uint32_t count) override;

    void generate_buffer(uint32_t *id) override;
    void generate_vertex_array(uint32_t *id) override;
    void generate_texture(uint32_t *id) override;
    void delete_texture(uint32_t *id) override;
    void generate_framebuffer(uint32_t *id) override;
    void bind_framebuffer(const uint32_t target, const uint32_t id) override;
    void generate_renderbuffer(uint32_t *id) override;
    void bind_renderbuffer(const uint32_t target, const uint32_t id) override;
    void attach_texture_to_framebuffer(const uint32_t target, const uint32_t attachment, const uint32_t textarget,
                                    const unsigned int texture, int level) override;
    void allocate_render_buffer_memory(const uint32_t format, int samples, int width, int height) override;
    void attach_renderbuffer_to_framebuffer(unsigned int renderbuffer, uint32_t attachment) override;
    void bind_buffer(const uint32_t target, const uint32_t id) override;
    void bind_vertex_array(const uint32_t id) override;
    void bind_texture(const uint32_t type, const uint32_t id) override;
    void active_texture(const uint32_t number) override;
    void bind_image_to_texture(const uint32_t target, const int32_t level, const int32_t internalFormat,
                            const uint32_t width, const uint32_t height, const int32_t border, const uint32_t format,
                            const uint32_t type, const void *data) override;
    void bind_sub_image_to_texture(const uint32_t target, const int32_t level, const int32_t xOffset, const int32_t yOffset,
                               const uint32_t width, const uint32_t height, const uint32_t format, const uint32_t type,
                               const void *data) override;
    void set_texture_parameter(const uint32_t target, const uint32_t pname, const uint32_t param) override;
    void upload_to_buffer(const uint32_t target, const void *data, const uint32_t size, const uint32_t usage) override;
    void unbind_buffer(const uint32_t target) override;
    void unbind_vertex_array() override;
    void unbind_render_buffer() override;
    void unbind_framebuffer() override;
    void unbind_texture(const uint32_t type) override;
    bool check_framebuffer_status() override;
    void set_vertex_attribute_ptr(const uint32_t index, const int32_t size, const uint32_t type, const bool normalized,
                               const uint32_t stride, const void *pointer) override;
    void enable_vertex_attribute_ptr(const uint32_t index) override;
};

}  // namespace tod_gl
/**
 * @file renderer_api.hpp
 * @brief OpenGL abstraction for the rendering of the scene
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <memory>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>

namespace tod_gl {

class RendererAPI {
  public:
    virtual ~RendererAPI() = default;
    virtual void init() = 0;
    virtual void enable(const uint32_t capability) = 0;
    virtual void set_blend_func(const uint32_t sfactor, const uint32_t dfactor) = 0;
    virtual void set_viewport(const uint32_t x, const uint32_t y, const uint32_t width, const uint32_t height) = 0;
    virtual void set_clear_color(const glm::vec4& color) = 0;
    virtual void clear() = 0;
    virtual void set_line_width(const float linewidth) = 0;
    virtual void set_point_size(const float pointsize) = 0;
    virtual void set_pixel_storage_mode(const uint32_t pname, const int32_t param) = 0;
    virtual void draw_elements(const uint32_t mode, const uint32_t count, const uint32_t type, const void* data) = 0;
    virtual void draw_arrays(const uint32_t mode, const uint32_t first, const uint32_t count) = 0;
    virtual void generate_buffer(uint32_t* id) = 0;
    virtual void generate_vertex_array(uint32_t* id) = 0;
    virtual void generate_texture(uint32_t* id) = 0;
    virtual void delete_texture(uint32_t* id) = 0;
    virtual void generate_framebuffer(uint32_t* id) = 0;
    virtual void generate_renderbuffer(uint32_t* id) = 0;
    virtual void bind_buffer(const uint32_t target, const uint32_t id) = 0;
    virtual void bind_framebuffer(const uint32_t target, const uint32_t id) = 0;
    virtual void bind_renderbuffer(const uint32_t target, const uint32_t id) = 0;
    virtual void allocate_render_buffer_memory(const uint32_t format, int samples, int width, int height) = 0;
    virtual void attach_renderbuffer_to_framebuffer(unsigned int renderbuffer, uint32_t attachment) = 0;
    virtual void attach_texture_to_framebuffer(const uint32_t target, const uint32_t attachment, const uint32_t textarget,
                                            const unsigned int texture, int level) = 0;
    virtual void bind_vertex_array(const uint32_t id) = 0;
    virtual void bind_texture(const uint32_t type, const uint32_t id) = 0;
    virtual void active_texture(const uint32_t number) = 0;
    virtual void bind_image_to_texture(const uint32_t target, const int32_t level, const int32_t internalformat,
                                    const uint32_t width, const uint32_t height, const int32_t border,
                                    const uint32_t format, const uint32_t type, const void* data) = 0;
    virtual void bind_sub_image_to_texture(const uint32_t target, const int32_t level, const int32_t xOffset,
                                       const int32_t yOffset, const uint32_t width, const uint32_t height,
                                       const uint32_t format, const uint32_t type, const void* data) = 0;
    virtual void set_texture_parameter(const uint32_t target, const uint32_t pname, const uint32_t param) = 0;
    virtual void upload_to_buffer(const uint32_t target, const void* data, const uint32_t size, const uint32_t usage) = 0;
    virtual void unbind_buffer(const uint32_t target) = 0;
    virtual void unbind_render_buffer() = 0;
    virtual void unbind_framebuffer() = 0;
    virtual void unbind_vertex_array() = 0;
    virtual void unbind_texture(const uint32_t type) = 0;
    virtual bool check_framebuffer_status() = 0;
    virtual void set_vertex_attribute_ptr(const uint32_t index, const int32_t size, const uint32_t type,
                                       const bool normalized, const uint32_t stride, const void* pointer) = 0;
    virtual void enable_vertex_attribute_ptr(const uint32_t index) = 0;
    static std::unique_ptr<RendererAPI> create();
};

}  // namespace tod_gl
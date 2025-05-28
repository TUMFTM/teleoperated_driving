/**
 * @file renderer_command.hpp
 * @brief OpenGL abstraction of renderer commands for the rendering of the scene
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <memory>

#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/renderer/renderer_api.hpp"

namespace tod_gl {



/**
 * @brief RenderCommand System Overview
 *
 * The RenderCommand class organizes OpenGL operations into logical groups using
 * nested classes, each focused on a specific aspect of rendering:
 *
 * 1. ForBuffer - Manages vertex and index buffer operations
 *   - Creates, binds, and uploads data to GPU buffers
 *   - Provides combined operations like generate_bind_and_upload() for convenience
 *
 * 2. ForTexture - Handles texture creation and manipulation
 *   - Manages texture generation, binding, and parameter configuration
 *   - Supports texture data uploads and updates for dynamic content
 *
 * 3. ForVertexArray - Controls vertex array objects (VAOs)
 *   - Creates and binds VAOs which store vertex attribute configurations
 *
 * 4. ForVertexAttributePtr - Sets up vertex attribute formats
 *   - Configures how vertex data (positions, texcoords, colors) is interpreted
 *
 * 5. ForFramebuffer - Manages framebuffer objects for off-screen rendering
 *   - Creates framebuffers and attaches textures/renderbuffers as render targets
 *
 * 6. ForRenderbuffer - Handles renderbuffer objects for depth/stencil attachments
 *   - Creates and configures renderbuffers for framebuffers
 *
 * 7. Draw - Executes drawing operations
 *   - Issues commands to render geometry using either indexed or array drawing
 *
 * Each operation is delegated to the underlying RendererAPI implementation,
 * maintaining the separation between the command interface and the actual OpenGL calls.
 */
class RenderCommand {
  public:
    static void init() { static_renderer_api->init(); }

    static void enable(const uint32_t capability) { static_renderer_api->enable(capability); }

    static void set_viewport(const uint32_t x, const uint32_t y, const uint32_t width, const uint32_t height) {
        static_renderer_api->set_viewport(x, y, width, height);
    }

    static void set_clear_color(const glm::vec4 &color) { static_renderer_api->set_clear_color(color); }

    static void clear() { static_renderer_api->clear(); }

    static void set_line_width(const float linewidth) { static_renderer_api->set_line_width(linewidth); }

    static void set_point_size(const float pointsize) { static_renderer_api->set_point_size(pointsize); }

    static void set_pixel_storage_mode(const uint32_t pname, const int32_t param) {
        static_renderer_api->set_pixel_storage_mode(pname, param);
    }

    class Draw {
      public:
        static void Elements(const uint32_t mode, const uint32_t count, const uint32_t type, const void *data) {
            static_renderer_api->draw_elements(mode, count, type, data);
        }

        static void Arrays(const uint32_t mode, const uint32_t first, const uint32_t count) {
            static_renderer_api->draw_arrays(mode, first, count);
        }
    };

    class ForBuffer {
      public:
        static void generate(Buffer &buffer) { static_renderer_api->generate_buffer(&buffer.id); }

        static void bind(const Buffer &buffer) { static_renderer_api->bind_buffer(buffer.target, buffer.id); }

        static void upload(const Buffer &buffer, const void *data, const uint32_t size) {
            static_renderer_api->upload_to_buffer(buffer.target, data, size, buffer.usage);
        }

        static void unbind(const Buffer &buffer) { static_renderer_api->unbind_buffer(buffer.target); }

        static void generate_and_bind(Buffer &buffer) {
            generate(buffer);
            bind(buffer);
        }

        static void bind_and_upload(const Buffer &buffer, const void *data, const uint32_t size) {
            bind(buffer);
            upload(buffer, data, size);
        }

        static void generate_bind_and_upload(Buffer &buffer, const void *data, const uint32_t size) {
            generate(buffer);
            bind_and_upload(buffer, data, size);
        }
    };

    class ForTexture {
      public:
        static void generate(Texture &texture) { static_renderer_api->generate_texture(&texture.id); }

        static void delete_texture(Texture &texture) { static_renderer_api->delete_texture(&texture.id); }

        static void bind(const Texture &texture) { static_renderer_api->bind_texture(texture.type, texture.id); }

        static void active(const uint32_t textureNumber) { static_renderer_api->active_texture(textureNumber); }

        static void unbind(const Texture &texture) { static_renderer_api->unbind_texture(texture.type); }

        static void bind_image(const Texture &texture, const void *data) {
            static_renderer_api->bind_image_to_texture(texture.type, 0, texture.internal_format, texture.width, texture.height, 0,
                                              texture.format, GL_UNSIGNED_BYTE, data);
        }

        static void bind_sub_image(const Texture &texture, const int32_t xOffset, const int32_t yOffset,
                                 const uint32_t width, const uint32_t height, const void *data) {
            static_renderer_api->bind_sub_image_to_texture(texture.type, 0, xOffset, yOffset, width, height, texture.format,
                                                 GL_UNSIGNED_BYTE, data);
        }

        static void set_parameter(const Texture &texture, const uint32_t pname, const uint32_t param) {
            static_renderer_api->set_texture_parameter(texture.type, pname, param);
        }

        static void active_and_bind(const Texture &texture, const uint32_t number) {
            active(number);
            bind(texture);
        }

        static void generate_active_and_bind(Texture &texture, const uint32_t number) {
            generate(texture);
            active_and_bind(texture, number);
        }

        static void generate_and_bind(Texture &texture, const uint32_t number) {
            generate(texture);
            bind(texture);
        }
    };

    class ForVertexArray {
      public:
        static void generate(VertexArray &vao) { static_renderer_api->generate_vertex_array(&vao.id); }

        static void bind(const VertexArray &vao) { static_renderer_api->bind_vertex_array(vao.id); }

        static void unbind(const VertexArray &vao) { static_renderer_api->unbind_vertex_array(); }

        static void generate_and_bind(VertexArray &vao) {
            generate(vao);
            bind(vao);
        }
    };

    class ForVertexAttributePtr {
      public:
        static void set(const uint32_t index, const int32_t size, const uint32_t type, const bool normalized,
                        const uint32_t stride, const void *pointer) {
            static_renderer_api->set_vertex_attribute_ptr(index, size, type, normalized, stride, pointer);
        }

        static void enable(const uint32_t index) { static_renderer_api->enable_vertex_attribute_ptr(index); }

        static void set_and_enable(const uint32_t index, const int32_t size, const uint32_t type, const bool normalized,
                                 const uint32_t stride, const void *pointer) {
            set(index, size, type, normalized, stride, pointer);
            enable(index);
        }
    };

    class ForFramebuffer {
      public:
        static void bind(const uint32_t id) { static_renderer_api->bind_framebuffer(GL_FRAMEBUFFER, id); }

        static void generate(uint32_t *fbo) { static_renderer_api->generate_framebuffer(fbo); }

        static void generate_and_bind(uint32_t *fbo) {
            generate(fbo);
            bind(*fbo);
        }

        static void unbind() { static_renderer_api->unbind_framebuffer(); }

        static void attach_texture(const uint32_t target, const uint32_t attachment, const uint32_t textarget,
                                  const unsigned int texture) {
            static_renderer_api->attach_texture_to_framebuffer(target, attachment, textarget, texture, 0);
        }

        static void attach_renderbuffer(unsigned int renderbuffer, uint32_t attachement) {
            static_renderer_api->attach_renderbuffer_to_framebuffer(renderbuffer, attachement);
        }

        static void check() {
            if (static_renderer_api->check_framebuffer_status())
                std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
        }

        static void check_and_unbind() {
            check();
            unbind();
        }
    };

    class ForRenderbuffer {
      public:
        static void generate(uint32_t *rbo) { static_renderer_api->generate_renderbuffer(rbo); }

        static void bind(const uint32_t id) { static_renderer_api->bind_renderbuffer(GL_RENDERBUFFER, id); }

        static void allocate_memory(const uint32_t format, int samples, int width, int height) {
            static_renderer_api->allocate_render_buffer_memory(format, samples, width, height);
        }

        static void allocate_memory(const uint32_t format, int width, int height) {
            static_renderer_api->allocate_render_buffer_memory(format, 0, width, height);
        }

        static void unbind() { static_renderer_api->unbind_render_buffer(); }

        static void create(uint32_t *rbo, const uint32_t format, int samples, int width, int height) {
            generate(rbo);
            bind(*rbo);
            allocate_memory(format, samples, width, height);
            unbind();
        }

        static void create(uint32_t *rbo, const uint32_t format, int width, int height) {
            create(rbo, format, 0, width, height);
        }
    };

  private:
    static std::unique_ptr<RendererAPI> static_renderer_api;
};

}  // namespace tod_gl
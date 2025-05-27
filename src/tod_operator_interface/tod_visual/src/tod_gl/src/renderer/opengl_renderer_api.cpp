/**
 * @file opengl_renderer_api.cpp
 * @brief OpenGL Abstraction for rendering of the scene
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 **/

#include "tod_gl/renderer/opengl_renderer_api.hpp"

#include "tod_gl/renderer/renderer.hpp"

#include "glad/glad.h"

#define PrintGLError() Renderer::PrintGLError(__FILE__, __LINE__)
namespace tod_gl {
void OpenGLRendererAPI::init() {
    enable(GL_BLEND);
    set_blend_func(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    enable(GL_DEPTH_TEST);
}

void OpenGLRendererAPI::enable(const uint32_t capability) {
    glEnable(capability);
    PrintGLError();
}

void OpenGLRendererAPI::set_blend_func(const uint32_t sfactor, const uint32_t dfactor) {
    glBlendFunc(sfactor, dfactor);
}

void OpenGLRendererAPI::set_viewport(const uint32_t x, const uint32_t y, const uint32_t width, const uint32_t height) {
    glViewport(x, y, width, height);
    PrintGLError();
}

void OpenGLRendererAPI::set_clear_color(const glm::vec4 &color) {
    glClearColor(color.r, color.g, color.b, color.a);
    PrintGLError();
}

void OpenGLRendererAPI::clear() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRendererAPI::set_line_width(const float linewidth) {
    glLineWidth(linewidth);
    PrintGLError();
}

void OpenGLRendererAPI::set_point_size(const float pointsize) {
    glPointSize(pointsize);
    PrintGLError();
}

void OpenGLRendererAPI::set_pixel_storage_mode(const uint32_t pname, const int32_t param) {
    glPixelStorei(pname, param);
    PrintGLError();
}

void OpenGLRendererAPI::draw_elements(const uint32_t mode, const uint32_t count, const uint32_t type, const void *data) {
    glDrawElements(mode, count, type, data);
    PrintGLError();
}

void OpenGLRendererAPI::draw_arrays(const uint32_t mode, const uint32_t first, const uint32_t count) {
    glDrawArrays(mode, first, count);
    PrintGLError();
}

void OpenGLRendererAPI::generate_buffer(uint32_t *id) {
    glGenBuffers(1, id);
    PrintGLError();
}

void OpenGLRendererAPI::generate_vertex_array(uint32_t *id) {
    glGenVertexArrays(1, id);
    PrintGLError();
}

void OpenGLRendererAPI::generate_texture(uint32_t *id) {
    glGenTextures(1, id);
    PrintGLError();
}

void OpenGLRendererAPI::delete_texture(uint32_t *id) {
    glDeleteTextures(1, id);
    PrintGLError();
}

void OpenGLRendererAPI::generate_framebuffer(uint32_t *id) {
    glGenFramebuffers(1, id);
    PrintGLError();
}

void OpenGLRendererAPI::bind_renderbuffer(const uint32_t target, const uint32_t id) {
    glBindRenderbuffer(target, id);
    PrintGLError();
}

void OpenGLRendererAPI::generate_renderbuffer(uint32_t *id) {
    glGenRenderbuffers(1, id);
    PrintGLError();
}

void OpenGLRendererAPI::bind_framebuffer(const uint32_t target, const uint32_t id) {
    glBindFramebuffer(target, id);
    PrintGLError();
}
void OpenGLRendererAPI::bind_buffer(const uint32_t target, const uint32_t id) {
    glBindBuffer(target, id);
    PrintGLError();
}

void OpenGLRendererAPI::attach_texture_to_framebuffer(const uint32_t target, const uint32_t attachment,
                                                   const uint32_t textarget, const unsigned int texture, int level) {
    glFramebufferTexture2D(target, attachment, textarget, texture, level);
    PrintGLError();
}

void OpenGLRendererAPI::allocate_render_buffer_memory(const uint32_t format, int samples, int width, int height) {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, format, width, height);
    PrintGLError();
}

void OpenGLRendererAPI::attach_renderbuffer_to_framebuffer(unsigned int renderbuffer, uint32_t attachment) {
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, renderbuffer);
    PrintGLError();
}

void OpenGLRendererAPI::bind_vertex_array(const uint32_t id) {
    glBindVertexArray(id);
    PrintGLError();
}

void OpenGLRendererAPI::bind_texture(const uint32_t type, const uint32_t id) {
    glBindTexture(type, id);
    PrintGLError();
}

void OpenGLRendererAPI::active_texture(const uint32_t number) {
    glActiveTexture(GL_TEXTURE0 + number);
    PrintGLError();
}

void OpenGLRendererAPI::bind_image_to_texture(const uint32_t target, const int32_t level, const int32_t internalFormat,
                                           const uint32_t width, const uint32_t height, const int32_t border,
                                           const uint32_t format, const uint32_t type, const void *data) {
    glTexImage2D(target, level, internalFormat, width, height, border, format, type, data);
    PrintGLError();
}

void OpenGLRendererAPI::bind_sub_image_to_texture(const uint32_t target, const int32_t level, const int32_t xOffset,
                                              const int32_t yOffset, const uint32_t width, const uint32_t height,
                                              const uint32_t format, const uint32_t type, const void *data) {
    glTexSubImage2D(target, level, xOffset, yOffset, width, height, format, type, data);
    PrintGLError();
}

void OpenGLRendererAPI::set_texture_parameter(const uint32_t target, const uint32_t pname, const uint32_t param) {
    glTexParameteri(target, pname, param);
    PrintGLError();
}

void OpenGLRendererAPI::upload_to_buffer(const uint32_t target, const void *data, const uint32_t size,
                                       const uint32_t usage) {
    glBufferData(target, size, data, usage);
    PrintGLError();
}

void OpenGLRendererAPI::unbind_buffer(const uint32_t target) {
    glBindBuffer(target, 0);
    PrintGLError();
}

void OpenGLRendererAPI::unbind_vertex_array() {
    glBindVertexArray(0);
    PrintGLError();
}

void OpenGLRendererAPI::unbind_render_buffer() {
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    PrintGLError();
}

void OpenGLRendererAPI::unbind_framebuffer() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    PrintGLError();
}

void OpenGLRendererAPI::unbind_texture(const uint32_t type) {
    glBindTexture(type, 0);
    PrintGLError();
}

bool OpenGLRendererAPI::check_framebuffer_status() {
    return glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE;
}

void OpenGLRendererAPI::set_vertex_attribute_ptr(const uint32_t index, const int32_t size, const uint32_t type,
                                              const bool normalized, const uint32_t stride, const void *pointer) {
    glVertexAttribPointer(index, size, type, normalized, stride, pointer);
    PrintGLError();
}

void OpenGLRendererAPI::enable_vertex_attribute_ptr(const uint32_t index) {
    glEnableVertexAttribArray(index);
    PrintGLError();
}

}  // namespace tod_gl
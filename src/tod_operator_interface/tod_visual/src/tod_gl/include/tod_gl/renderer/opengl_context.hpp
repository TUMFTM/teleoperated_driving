/**
 * @file opengl_context.hpp
 * @brief Implementation of the OpenGl context @link https://www.khronos.org/opengl/wiki/OpenGL_Context
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#pragma once

#include <iostream>

#include "tod_gl/core/graphics_context.hpp"

#include "tod_gl/renderer/glfw_window.hpp"

namespace tod_gl {

class OpenGLContext : public GraphicsContext {
  public:
    explicit OpenGLContext(GLFWwindow *windowHandle);
    void init() override;
    void swap_buffers() override;

  private:
    GLFWwindow *_window_handle;
};

}  // namespace tod_gl
/**
 * @file graphics_context.cpp
 * @brief TODO: Add brief
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/core/graphics_context.hpp"

#include "tod_gl/renderer/opengl_context.hpp"
#include "tod_gl/renderer/glfw_window.hpp"

#include "imgui/imgui_impl_glfw.h"

namespace tod_gl {

std::unique_ptr<GraphicsContext> GraphicsContext::create(void *window) {
    return std::make_unique<OpenGLContext>(static_cast<GLFWwindow *>(window));
}

}  // namespace tod_gl

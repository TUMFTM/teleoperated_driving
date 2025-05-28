/**
 * @file window.cpp
 * @brief TODO: Add brief
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/core/window.hpp"

#include "tod_gl/renderer/glfw_window.hpp"

namespace tod_gl {
std::unique_ptr<Window> Window::create(const WindowProps& props) {
    return std::make_unique<GLFWWindow>(props);
}
}  // namespace tod_gl
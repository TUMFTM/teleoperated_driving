/**
 * @file opengl_context.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/renderer/opengl_context.hpp"

#include <iostream>

#include "glad/glad.h" // Glad must be included before glfw!
#include <GLFW/glfw3.h>

namespace tod_gl {

OpenGLContext::OpenGLContext(GLFWwindow *windowHandle) : _window_handle(windowHandle) {}

void OpenGLContext::init() {
    glfwMakeContextCurrent(_window_handle);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
    }
}

void OpenGLContext::swap_buffers() {
    glfwSwapBuffers(_window_handle);
}

}  // namespace tod_gl
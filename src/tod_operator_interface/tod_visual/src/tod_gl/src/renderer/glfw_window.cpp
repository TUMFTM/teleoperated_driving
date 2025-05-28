/**
 * @file glfw_window.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/renderer/glfw_window.hpp"

#include <iostream>
#include <memory>
#include <string>

#include "tod_gl/events/application_event.hpp"
#include "tod_gl/events/key_event.hpp"
#include "tod_gl/events/key_codes.hpp"
#include "tod_gl/events/mouse_events.hpp"
#include "tod_gl/events/mouse_codes.hpp"

#include "glad/glad.h" // Glad must be included before glfw!
#include <GLFW/glfw3.h>

namespace tod_gl {

static uint8_t s_GLFWWindowCount = 0;

void APIENTRY glDebugOutput(GLenum source, GLenum type, unsigned int id, GLenum severity, GLsizei length,
                            const char *message, const void *userParam) {
    // ignore non-significant error/warning codes
    if (id == 131169 || id == 131185 || id == 131218 || id == 131204)
        return;

    std::cout << "---------------" << std::endl;
    std::cout << "Debug message (" << id << "): " << message << std::endl;

    switch (source) {
        case GL_DEBUG_SOURCE_API:
            std::cout << "Source: API";
            break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
            std::cout << "Source: Window System";
            break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER:
            std::cout << "Source: Shader Compiler";
            break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:
            std::cout << "Source: Third Party";
            break;
        case GL_DEBUG_SOURCE_APPLICATION:
            std::cout << "Source: Application";
            break;
        case GL_DEBUG_SOURCE_OTHER:
            std::cout << "Source: Other";
            break;
    }
    std::cout << std::endl;

    switch (type) {
        case GL_DEBUG_TYPE_ERROR:
            std::cout << "Type: Error";
            break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            std::cout << "Type: Deprecated Behaviour";
            break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            std::cout << "Type: Undefined Behaviour";
            break;
        case GL_DEBUG_TYPE_PORTABILITY:
            std::cout << "Type: Portability";
            break;
        case GL_DEBUG_TYPE_PERFORMANCE:
            std::cout << "Type: Performance";
            break;
        case GL_DEBUG_TYPE_MARKER:
            std::cout << "Type: Marker";
            break;
        case GL_DEBUG_TYPE_PUSH_GROUP:
            std::cout << "Type: Push Group";
            break;
        case GL_DEBUG_TYPE_POP_GROUP:
            std::cout << "Type: Pop Group";
            break;
        case GL_DEBUG_TYPE_OTHER:
            std::cout << "Type: Other";
            break;
    }
    std::cout << std::endl;

    switch (severity) {
        case GL_DEBUG_SEVERITY_HIGH:
            std::cout << "Severity: high";
            break;
        case GL_DEBUG_SEVERITY_MEDIUM:
            std::cout << "Severity: medium";
            break;
        case GL_DEBUG_SEVERITY_LOW:
            std::cout << "Severity: low";
            break;
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            std::cout << "Severity: notification";
            break;
    }
    std::cout << std::endl << std::endl;
}

static void GLFWErrorCallback(int error, const char *description) {
    std::cout << "GLFW Error Callback" << std::endl;
}

GLFWWindow::GLFWWindow(const WindowProps &props) {
    init(props);
}

GLFWWindow::~GLFWWindow() {
    shutdown();
}

void GLFWWindow::init(const WindowProps &props) {
    _data.title = props.title;
    _data.width = props.width;
    _data.height = props.height;

    if (s_GLFWWindowCount == 0) {
        int success = glfwInit();
        glfwSetErrorCallback(GLFWErrorCallback);
    }
    // added for debugging
    bool turnDebuggingOn{false};
    if (turnDebuggingOn) {
        std::cout << "\033[1;31mDEBUGGING IN OPENGL ENABLED - "
                     "TURN OFF FOR MORE FPS\033[0m"
                  << "\n";
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
    }

    _window = glfwCreateWindow((int)props.width, (int)props.height, _data.title.c_str(), nullptr, nullptr);

    if (_window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        // return -1;
    }
    ++s_GLFWWindowCount;

    _graphics_context = GraphicsContext::create(_window);
    _graphics_context->init();

    glfwSetWindowUserPointer(_window, &_data);

    glfwSetWindowCloseCallback(_window, [](GLFWwindow *window) {
        WindowData &data = *(WindowData *)glfwGetWindowUserPointer(window);
        WindowCloseEvent event;
        data.EventCallback(event);
    });

    glfwSetScrollCallback(_window, [](GLFWwindow *window, double xOffset, double yOffset) {
        WindowData &data = *(WindowData *)glfwGetWindowUserPointer(window);

        MouseScrolledEvent event((float)xOffset, (float)yOffset);
        data.EventCallback(event);
    });

    glfwSetKeyCallback(_window, [](GLFWwindow *window, int key, int scancode, int action, int mods) {
        WindowData &data = *(WindowData *)glfwGetWindowUserPointer(window);
        switch (action) {
            case GLFW_PRESS: {
                KeyPressedEvent event(static_cast<KeyCode>(key), 0);
                data.EventCallback(event);
                break;
            }
            case GLFW_RELEASE: {
                KeyReleasedEvent event(static_cast<KeyCode>(key));
                data.EventCallback(event);
                break;
            }
            case GLFW_REPEAT: {
                KeyPressedEvent event(static_cast<KeyCode>(key), 1);
                data.EventCallback(event);
                break;
            }
        }
    });

    glfwSetMouseButtonCallback(_window, [](GLFWwindow *window, int button, int action, int mods) {
        WindowData &data = *(WindowData *)glfwGetWindowUserPointer(window);
        switch (action) {
            case GLFW_PRESS: {
                MouseButtonPressedEvent event(static_cast<MouseCode>(button));
                data.EventCallback(event);
                break;
            }
            case GLFW_RELEASE: {
                MouseButtonReleasedEvent event(static_cast<MouseCode>(button));
                data.EventCallback(event);
                break;
            }
        }
    });

    glfwSetCursorPosCallback(_window, [](GLFWwindow *window, double xpos, double ypos) {
        WindowData &data = *(WindowData *)glfwGetWindowUserPointer(window);

        MouseMovedEvent event((float)xpos, (float)ypos, data.height);
        data.EventCallback(event);
    });

    glfwSetFramebufferSizeCallback(_window,
                                   [](GLFWwindow *window, int width, int height) { glViewport(0, 0, width, height); });

    // Set GLFW callbacks
    glfwSetWindowSizeCallback(_window, [](GLFWwindow *window, int width, int height) {
        WindowData &data = *(WindowData *)glfwGetWindowUserPointer(window);
        data.width = width;
        data.height = height;

        WindowResizeEvent event(width, height);
        data.EventCallback(event);
    });
    int flags;
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    if (flags & GL_CONTEXT_FLAG_DEBUG_BIT) {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glDebugOutput, nullptr);
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
    }
    glfwSwapInterval(0);
}

void GLFWWindow::shutdown() {
    glfwDestroyWindow(_window);
    --s_GLFWWindowCount;
    if (s_GLFWWindowCount == 0)
        glfwTerminate();
}

void GLFWWindow::on_update() {
    glfwPollEvents();
    _graphics_context->swap_buffers();
}

}  // namespace tod_gl
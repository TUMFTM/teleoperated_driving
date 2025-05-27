/**
 * @file graphics_context.hpp
 * @brief Creates the opengl graphics context 
 * @copyright 2024 TUMFTM based on Cherno's Hazel
 */

#pragma once

#include <memory>

#include "imgui/imgui_impl_glfw.h"

struct GLFWwindow;

namespace tod_gl {

class GraphicsContext {
  public:
    virtual ~GraphicsContext() = default;
    virtual void init() = 0;
    virtual void swap_buffers() = 0;
    static std::unique_ptr<GraphicsContext> create(void *window);
};

}  // namespace tod_gl
/**
 * @file renderer_api.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/renderer/renderer_api.hpp"

#include "tod_gl/renderer/opengl_renderer_api.hpp"

namespace tod_gl {

std::unique_ptr<RendererAPI> RendererAPI::create() {
    return std::make_unique<OpenGLRendererAPI>();
}

}  // namespace tod_gl
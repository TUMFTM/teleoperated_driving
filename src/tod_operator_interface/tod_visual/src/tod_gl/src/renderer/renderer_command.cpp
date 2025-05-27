/**
 * @file renderer_command.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM 
 **/

#include "tod_gl/renderer/renderer_command.hpp"

namespace tod_gl {

std::unique_ptr<RendererAPI> RenderCommand::static_renderer_api = RendererAPI::create();

}  // namespace tod_gl
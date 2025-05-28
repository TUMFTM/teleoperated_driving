/**
 * @file io_layer.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/layers/io_layer.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/scene/entity.hpp"

namespace tod_gl {

IOLayer::IOLayer(std::shared_ptr<RosInterface> ros) : Layer(ros, "IOLayer") {}

void IOLayer::on_attach() {}

}  // namespace tod_gl
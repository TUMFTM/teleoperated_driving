/**
 * @file layer.cpp
 * @brief TODO: Add brief
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/core/layer.hpp"

namespace tod_gl {

Layer::Layer(const std::string& debugName) 
    : _name(debugName) {}

Layer::Layer(std::shared_ptr<RosInterface> ros, const std::string& name) 
    : _name(name), _ros(ros) {}

}  // namespace tod_gl
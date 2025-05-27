/**
 * @file scene_layer.cpp
 * @brief Scene abstraction of the layer, i.e. if you have a scene application @ref SceneApplication then the layers used will have access to the 3D scenes content
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/core/scene_layer.hpp"

namespace tod_gl {
SceneLayer::SceneLayer(const std::string& debugName) : _name(debugName) {}

SceneLayer::SceneLayer(std::shared_ptr<RosInterface> ros, std::shared_ptr<Scene> scene, const std::string& name)
    : _name(name), _ros(ros), _active_scene(scene) {}
}  // namespace tod_gl
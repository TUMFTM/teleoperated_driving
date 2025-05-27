/**
 * @file scene_layer_stack.hpp
 * @brief Manages the dearImGui layers for the render loop. ImGui layers will be rendered in the order they are pushed. Containes access to the 3D scene within the layers
 * @copyright 2024 TUMFTM based on Cherno's Hazel
 */

#pragma once

#include <vector>

#include "tod_gl/core/scene_layer.hpp"

namespace tod_gl {

class SceneLayerStack {
  public:
    SceneLayerStack() = default;
    ~SceneLayerStack();

    void push_layer(SceneLayer* SceneLayer);
    void push_overlay(SceneLayer* overlay);
    void pop_layer(SceneLayer* SceneLayer);
    void pop_overlay(SceneLayer* overlay);

    std::vector<SceneLayer*>::iterator begin() { return _layers.begin(); }
    std::vector<SceneLayer*>::iterator end() { return _layers.end(); }
    std::vector<SceneLayer*>::reverse_iterator rbegin() { return _layers.rbegin(); }
    std::vector<SceneLayer*>::reverse_iterator rend() { return _layers.rend(); }

    std::vector<SceneLayer*>::const_iterator begin() const { return _layers.begin(); }
    std::vector<SceneLayer*>::const_iterator end() const { return _layers.end(); }
    std::vector<SceneLayer*>::const_reverse_iterator rbegin() const { return _layers.rbegin(); }
    std::vector<SceneLayer*>::const_reverse_iterator rend() const { return _layers.rend(); }

  private:
    std::vector<SceneLayer*> _layers;
    unsigned int _layer_insert_index = 0;
};

}  // namespace tod_gl
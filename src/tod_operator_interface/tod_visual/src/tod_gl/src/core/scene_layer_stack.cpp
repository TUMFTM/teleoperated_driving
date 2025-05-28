/**
 * @file scene_layer_stack.cpp
 * @brief TODO: Add brief
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/core/scene_layer_stack.hpp"

#include <algorithm>

namespace tod_gl {

SceneLayerStack::~SceneLayerStack() {
    for (SceneLayer* layer : _layers) {
        layer->on_detach();
        delete layer;
    }
}

void SceneLayerStack::push_layer(SceneLayer* layer) {
    _layers.emplace(_layers.begin() + _layer_insert_index, layer);
    _layer_insert_index++;
    layer->on_attach();
}

void SceneLayerStack::push_overlay(SceneLayer* overlay) {
    _layers.emplace_back(overlay);
    overlay->on_attach();
}

void SceneLayerStack::pop_layer(SceneLayer* layer) {
    auto it = std::find(_layers.begin(), _layers.begin() + _layer_insert_index, layer);
    if (it != _layers.begin() + _layer_insert_index) {
        layer->on_detach();
        _layers.erase(it);
        _layer_insert_index--;
    }
}

void SceneLayerStack::pop_overlay(SceneLayer* overlay) {
    auto it = std::find(_layers.begin() + _layer_insert_index, _layers.end(), overlay);
    if (it != _layers.end()) {
        overlay->on_detach();
        _layers.erase(it);
    }
}

}  // namespace tod_gl
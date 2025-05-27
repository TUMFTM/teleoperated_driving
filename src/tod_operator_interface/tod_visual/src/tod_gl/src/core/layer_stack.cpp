/**
 * @file layer_stack.cpp
 * @brief TODO: Add brief
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/core/layer_stack.hpp"

#include <algorithm>

namespace tod_gl {

LayerStack::~LayerStack() {
    for (Layer* layer : _layers) {
        layer->on_detach();
        delete layer;
    }
}

void LayerStack::push_layer(Layer* layer) {
    _layers.emplace(_layers.begin() + _layer_insert_index, layer);
    _layer_insert_index++;
    layer->on_attach();
}

void LayerStack::push_overlay(Layer* overlay) {
    _layers.emplace_back(overlay);
    overlay->on_attach();
}

void LayerStack::pop_layer(Layer* layer) {
    auto it = std::find(_layers.begin(), _layers.begin() + _layer_insert_index, layer);
    if (it != _layers.begin() + _layer_insert_index) {
        layer->on_detach();
        _layers.erase(it);
        _layer_insert_index--;
    }
}

void LayerStack::pop_overlay(Layer* overlay) {
    auto it = std::find(_layers.begin() + _layer_insert_index, _layers.end(), overlay);
    if (it != _layers.end()) {
        overlay->on_detach();
        _layers.erase(it);
    }
}

}  // namespace tod_gl
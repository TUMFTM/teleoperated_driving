/**
 * @file layer_stack.hpp
 * @brief Manages the dearImGui layers for the render loop. ImGui layers will be rendered in the order they are pushed  
 * @copyright 2024 TUMFTM based on Cherno's Hazel
 */

#pragma once

#include <vector>

#include "tod_gl/core/layer.hpp"

namespace tod_gl {

class LayerStack {
  public:
    LayerStack() = default;
    ~LayerStack();

    void push_layer(Layer* layer);
    void push_overlay(Layer* overlay);
    void pop_layer(Layer* layer);
    void pop_overlay(Layer* overlay);

    std::vector<Layer*>::iterator begin() { return _layers.begin(); }
    std::vector<Layer*>::iterator end() { return _layers.end(); }
    std::vector<Layer*>::reverse_iterator rbegin() { return _layers.rbegin(); }
    std::vector<Layer*>::reverse_iterator rend() { return _layers.rend(); }

    std::vector<Layer*>::const_iterator begin() const { return _layers.begin(); }
    std::vector<Layer*>::const_iterator end() const { return _layers.end(); }
    std::vector<Layer*>::const_reverse_iterator rbegin() const { return _layers.rbegin(); }
    std::vector<Layer*>::const_reverse_iterator rend() const { return _layers.rend(); }

  private:
    std::vector<Layer*> _layers;
    unsigned int _layer_insert_index = 0;
};

}  // namespace tod_gl
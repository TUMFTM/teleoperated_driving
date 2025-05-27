/**
 * @file io_layer.hpp
 * @brief Base layer for non-visible operations such as manageing the state of subscriptions and publisher components within the application
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <memory>

#include "tod_gl/core/layer.hpp"

/**
 * All the Subscribing/Publishing components should be handled under this layer
 */
namespace tod_gl {

class IOLayer : public Layer {
  public:
    IOLayer(std::shared_ptr<RosInterface> ros);
    ~IOLayer() = default;

    virtual void on_attach() override;
};

}  // namespace tod_gl
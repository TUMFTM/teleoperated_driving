/**
 * @file scene_layer_event.hpp
 * @brief Event abstraction for the dearImGui based scene layers within the application
 * @copyright 2024 TUMFTM
 */

#pragma once

#include "tod_gl/core/scene_layer.hpp"
#include "tod_gl/events/event.hpp"

namespace tod_gl {

class SceneLayerStackChangedEvent : public Event {
  public:
    SceneLayerStackChangedEvent(SceneLayer* layer, bool deleted) : _layer(layer), _deleted(deleted){};

    static EventType get_static_type() { return EventType::LayerStackChanged; }
    EventType get_event_type() const override { return get_static_type(); }
    const char* get_name() const override { return "SceneLayerStackChanged"; }
    int get_category_flags() const override { return (EventCategoryApplication); }

    inline SceneLayer* get_layer() const { return _layer; }
    inline bool is_deleted() const { return _deleted; }

  private:
    SceneLayer* _layer;
    bool _deleted;
};

}  // namespace tod_gl
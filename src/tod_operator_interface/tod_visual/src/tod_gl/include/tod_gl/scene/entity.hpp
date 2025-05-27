/**
 * @file entity.cpp
 * @brief Entity Abstraction for Entt and utilities for the entities registry that manages all of the scenes entities
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 **/

#pragma once

#include <utility>

#include "tod_gl/scene/scene.hpp"

#include "entt/entt.hpp"

namespace tod_gl {


/*
* @brief Entity is the base abstraction of the tod_gl library to interact with the ENTT-Library
*/
class Entity {
  public:
    Entity() = default;
    Entity(const Entity &entity) = default;
    Entity(const entt::entity &handle, Scene *scene) : _entity_handle(handle), _scene(scene) {}

    entt::entity &get_handle() { return _entity_handle; }

    template <typename T, typename... Args>
    T &add_component(Args &&...args) {
        return _scene->registry.emplace<T>(_entity_handle, std::forward<Args>(args)...);
    }

    template <typename T>
    T &get_component() {
        return _scene->registry.get<T>(_entity_handle);
    }

    template <typename T>
    bool has_component() {
        return _scene->registry.has<T>(_entity_handle);
    }

    template <typename T>
    void remove_component() {
        _scene->registry.remove<T>(_entity_handle);
    }

    Scene& get_bounded_scene() { 
        if (!_scene) {
            throw std::runtime_error("Bounded scene is null");
        }
        return *_scene; 
    }

    operator bool() const { return _entity_handle != entt::null; }

  private:
    entt::entity _entity_handle{entt::null};
    Scene *_scene{nullptr};
};

} // namespace tod_gl
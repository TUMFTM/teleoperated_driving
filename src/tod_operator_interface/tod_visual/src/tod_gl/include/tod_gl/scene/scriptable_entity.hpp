
/**
 * @file scriptable_entity.hpp
 * @brief Scriptable Entities are entities that have their own update loop within the scene i.e. dynamic enities 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/entity.hpp"
#include "tod_gl/scene/scene.hpp"

namespace tod_gl {

class ScriptableEntity {
  public:
    virtual ~ScriptableEntity() {};
    ScriptableEntity() = default;

    template <typename T>
    T& get_component() {
        return m_Entity.get_component<T>();
    }

    template <typename T, typename... Args>
    T& add_component(Args&&... args) {
        return m_Entity.add_component<T>(std::forward<Args>(args)...);
    }

    template <typename T>
    void remove_component() {
        m_Entity.remove_component<T>();
    }

    Scene& get_bounded_scene() { return m_Entity.get_bounded_scene(); }

  protected:
    virtual void on_create() {}
    virtual void on_destroy() {}
    virtual void on_update(float) {}

  private:
    Entity m_Entity;
    friend class Scene;
};

} // namespace tod_gl
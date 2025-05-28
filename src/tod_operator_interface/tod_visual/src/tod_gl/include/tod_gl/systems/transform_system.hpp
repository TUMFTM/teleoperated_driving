/**
 * @file transform_system.hpp
 * @brief Transformation System for the scene's entities using the @ref TransformComponent transform information
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <memory>
#include "tod_gl/scene/components.hpp"

#include "glm/glm.hpp"

namespace tod_gl {

class TransformSystem {
  public:
    TransformSystem(const TransformSystem& obj) = delete;
    TransformSystem& operator=(const TransformSystem&) = delete;

    static TransformSystem* get_instance() {
        if (instance_ == nullptr) {
            instance_ = new TransformSystem();
        }
        return instance_;
    }

    glm::mat4 local_to_world(TransformComponent& transform);
    glm::mat4 local_to_world_rotation(TransformComponent& transform);
    glm::mat4 get_transform_between_entities( TransformComponent& source,  TransformComponent& targetEntity);
    glm::vec3 extract_rotation_euler( glm::mat4 &transform);
    glm::vec3 extract_translation( glm::mat4 &transform);
    glm::vec3 to_world_coordinates(const glm::vec3& gamePosition);
    glm::vec3 to_game_coordinates(const glm::vec3& worldPosition);
    void set_world_offset(const glm::vec3& initialPosition);
    glm::vec3 get_world_offset() const { return world_offset_; };
    bool is_world_offset_set() const { return is_world_offset_set_; };
    
  private:
    TransformSystem(){};
    ~TransformSystem() = default;

    static TransformSystem* instance_;
  
    bool initialized;
    glm::vec3 world_offset_{0.0f, 0.0f, 0.0f};
    bool is_world_offset_set_{false};

    glm::mat4 local_to_parent(TransformComponent& transform);
    glm::mat4 local_to_parent_rotation(TransformComponent& transform);
};

inline glm::vec3 TransformSystem::to_world_coordinates(const glm::vec3& gamePosition) {
    return gamePosition + world_offset_;
}

inline glm::vec3 TransformSystem::to_game_coordinates(const glm::vec3& worldPosition) {
    return worldPosition - world_offset_;
}

inline void TransformSystem::set_world_offset(const glm::vec3& initialPosition) {
    if (!is_world_offset_set_){
        world_offset_ = initialPosition;
        is_world_offset_set_ = true;
    }
}

} // namespace tod_gl
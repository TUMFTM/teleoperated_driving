/**
 * @file vr_system.cpp
 * @ingroup tod_gl_systems
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <string>

#include "openvr/openvr.h"
#include "entt/entt.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

namespace tod_gl {

struct CameraComponent;
struct VRComponent;
struct TransformComponent;
struct RenderableElementComponent;
class Scene;
class Entity;

class VRSystem {
  private:
    void handle_vr_error(vr::EVRInitError err);
    glm::mat4 _headPose;
    glm::mat4 convert_steam_vr_matrix_to_glm(const vr::HmdMatrix34_t& matPose);
    glm::mat4 convert_steam_vr_matrix_to_glm(const vr::HmdMatrix44_t& matPose);

  public:
    VRSystem() = default;
    ~VRSystem();

    bool vrInit();
    bool init_compositor();
    vr::IVRSystem* _pHMD{NULL};
    void calc_projection_matrix(CameraComponent& camera, VRComponent& vr);
    void calc_view_matrix(CameraComponent& camera, VRComponent& vr, TransformComponent& transform);
    void update_vr_pose();
    void create_vr_entities(Scene* activeScene, entt::entity parent);
    void create_vr_entity(Scene* activeScene, Entity parent, vr::EVREye eye);
    static void submit_texture(RenderableElementComponent& renderable, VRComponent& vr);
};

} // namespace tod_gl
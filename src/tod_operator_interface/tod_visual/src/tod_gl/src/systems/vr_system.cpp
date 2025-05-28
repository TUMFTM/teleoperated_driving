/**
 * @file vr_system.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/systems/vr_system.hpp"

#include "tod_gl/systems/shader_system.hpp"
#include "tod_gl/systems/transform_system.hpp"

#include "glm/gtc/type_ptr.hpp"

namespace tod_gl {

VRSystem::~VRSystem() {
    if (_pHMD) {
        vr::VR_Shutdown();
        _pHMD = NULL;
    }
}

bool VRSystem::vrInit() {
    _pHMD = NULL;

    if (!vr::VR_IsHmdPresent()) {
        // TODO: (Andi): reintroduce print
        // ROS_INFO("HMD is not connected");
        return false;
    }
    if (!vr::VR_IsRuntimeInstalled()) {
        // TODO: (Andi): reintroduce print
        // ROS_ERROR("OpenVR Runtime is not installed");
        return false;
    }
    vr::EVRInitError err = vr::VRInitError_None;
    _pHMD = vr::VR_Init(&err, vr::VRApplication_Scene);
    if (err != vr::VRInitError_None) {
        // TODO: (Andi): reintroduce print
        // ROS_ERROR("Check if SteamVR is started!");
        _pHMD = NULL;
        return false;
    }
    return init_compositor();
}

bool VRSystem::init_compositor() {
    if (!_pHMD)
        return false;
    vr::EVRInitError peError = vr::VRInitError_None;
    if (!vr::VRCompositor()) {
        printf("Compositor initialization failed. See log file for details\n");
        return false;
    }
    return true;
}

void VRSystem::update_vr_pose() {
    static char devClassChar[vr::k_unMaxTrackedDeviceCount];
    static vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    static glm::mat4 devicePose[vr::k_unMaxTrackedDeviceCount];
    static int validPoseCount;
    static std::string poseClasses;

    if (!_pHMD)
        return;

    vr::VRCompositor()->WaitGetPoses(trackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
    validPoseCount = 0;
    poseClasses = "";
    for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice) {
        if (trackedDevicePose[nDevice].bPoseIsValid) {
            validPoseCount++;
            devicePose[nDevice] = convert_steam_vr_matrix_to_glm(trackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
            if (devClassChar[nDevice] == 0) {
                switch (_pHMD->GetTrackedDeviceClass(nDevice)) {
                    case vr::TrackedDeviceClass_Controller:
                        devClassChar[nDevice] = 'C';
                        break;
                    case vr::TrackedDeviceClass_HMD:
                        devClassChar[nDevice] = 'H';
                        break;
                    case vr::TrackedDeviceClass_Invalid:
                        devClassChar[nDevice] = 'I';
                        break;
                    case vr::TrackedDeviceClass_GenericTracker:
                        devClassChar[nDevice] = 'G';
                        break;
                    case vr::TrackedDeviceClass_TrackingReference:
                        devClassChar[nDevice] = 'T';
                        break;
                    default:
                        devClassChar[nDevice] = '?';
                        break;
                }
            }
            poseClasses += devClassChar[nDevice];
        }
    }
    if (trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
        _headPose = devicePose[vr::k_unTrackedDeviceIndex_Hmd];
        _headPose = glm::inverse(_headPose);
    }
}

void VRSystem::submit_texture(RenderableElementComponent& renderable, VRComponent& vr) {
    vr::Texture_t eyeTexture = {(void*)(uintptr_t)renderable.meshes.at(0).textures.at(0).id, vr::TextureType_OpenGL,
                                vr::ColorSpace_Gamma};
    vr::VRCompositor()->Submit(vr.eye, &eyeTexture);
}

void VRSystem::calc_projection_matrix(CameraComponent& camera, VRComponent& vr) {
    if (!_pHMD) {
        camera.projection = glm::mat4(1.0f);
    } else {
        vr::HmdMatrix44_t mat = _pHMD->GetProjectionMatrix(vr.eye, camera.near_plane, camera.far_plane);
        camera.projection = convert_steam_vr_matrix_to_glm(mat);
    }
}

void VRSystem::calc_view_matrix(CameraComponent& camera, VRComponent& vr, TransformComponent& transform) {
    if (!_pHMD) {
        camera.view = glm::mat4(1.0f);
    } else {
        vr::HmdMatrix34_t eyeToHeadTransform = _pHMD->GetEyeToHeadTransform(vr.eye);
        // ToDo: use Entity Transform Tree: baseFootprint-> HMD -> VRCamera(Left/Right)
        camera.view = convert_steam_vr_matrix_to_glm(eyeToHeadTransform) * _headPose *
                      glm::inverse(tod_gl::TransformSystem::get_instance()->local_to_world(transform));
    }
}

void VRSystem::handle_vr_error(vr::EVRInitError err) {
    throw std::runtime_error(vr::VR_GetVRInitErrorAsEnglishDescription(err));
}

glm::mat4 VRSystem::convert_steam_vr_matrix_to_glm(const vr::HmdMatrix44_t& matPose) {
    glm::mat4 matrixObj(matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], matPose.m[3][0], matPose.m[0][1],
                        matPose.m[1][1], matPose.m[2][1], matPose.m[3][1], matPose.m[0][2], matPose.m[1][2],
                        matPose.m[2][2], matPose.m[3][2], matPose.m[0][3], matPose.m[1][3], matPose.m[2][3],
                        matPose.m[3][3]);
    return matrixObj;
}

glm::mat4 VRSystem::convert_steam_vr_matrix_to_glm(const vr::HmdMatrix34_t& matPose) {
    glm::mat4 matrixObj(matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0, matPose.m[0][1], matPose.m[1][1],
                        matPose.m[2][1], 0.0, matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0, matPose.m[0][3],
                        matPose.m[1][3], matPose.m[2][3], 1.0f);
    return matrixObj;
}

void VRSystem::create_vr_entities(Scene* activeScene, entt::entity par) {
    Entity HMD = activeScene->create_entity("HMD");
    auto& transform = HMD.get_component<TransformComponent>();
    transform.set_parent(Entity(par, activeScene));
    transform.set_rotation(glm::vec3(glm::radians(90.0f), 0.0f, glm::radians(-90.0f)));
    transform.set_translation(glm::vec3(0.0f, 0.0f, 0.0f));

    create_vr_entity(activeScene, HMD, vr::Eye_Left);
    create_vr_entity(activeScene, HMD, vr::Eye_Right);
}

void VRSystem::create_vr_entity(Scene* activeScene, Entity parent, vr::EVREye eye) {
    Entity vrCameraEntity = activeScene->create_entity("VRCamera" + eye);
    vrCameraEntity.add_component<CameraComponent>();
    vrCameraEntity.get_component<TransformComponent>().set_parent(parent);

    Entity vrFramebufferEntity = activeScene->create_entity("VREntity" + eye);
    vrFramebufferEntity.add_component<VRComponent>(eye);
    auto& framebufferLeft = vrFramebufferEntity.add_component<FrameBufferComponent>();
    _pHMD->GetRecommendedRenderTargetSize(&framebufferLeft.render_width, &framebufferLeft.render_height);

    // Give framebuffer a camera entity to specify view to render
    framebufferLeft.camera_entity = vrCameraEntity.get_handle();
    // Add mesh to specify texture for Framebuffer to render to
    Mesh mesh = Mesh::non_empty_mesh();
    mesh.textures.push_back(
        Texture(framebufferLeft.render_width, framebufferLeft.render_height, "", GL_TEXTURE_2D, GL_RGBA, GL_RGBA));
    auto& renderable = vrFramebufferEntity.add_component<RenderableElementComponent>(0, mesh);
    renderable.static_show = false;
}

} // namespace tod_gl
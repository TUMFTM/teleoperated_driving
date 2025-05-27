/**
 * @file video_renderer_new.cpp
 * @brief VideoRenderer renders a incoming video stream in multiple modes, for more information we refer to the readme 
 * @copyright 2024 TUMFTM
**/
#include "tod_dynamic_entities/video_renderer.hpp"
#include <glm/gtx/string_cast.hpp>

namespace TodDynamicEntities {

template<typename VideoComp>
void VideoRenderer<VideoComp>::on_create() {
    unsigned int videoShader = tod_gl::ShaderSystem::create_shader_program(
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video.vert").c_str(),
        (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video_rgb_rect.frag").c_str());
    
    
    auto &videoCmp = this->add_component<tod_gl::VideoComponent>(_cam_name, _is_fisheye);
    
    videoCmp.pixel_buffers.emplace_back(2048, 2048, "RGBtex");

    auto mesh = tod_gl::Mesh::non_empty_mesh();
    for (auto &pxBuf : videoCmp.pixel_buffers) {
        auto tex =
            tod_gl::Texture(pxBuf.width, pxBuf.height, pxBuf.name, GL_TEXTURE_RECTANGLE, GL_RGB8, GL_RGB);
        mesh.textures.emplace_back(tex);

        tod_gl::Renderer::create_buffer(pxBuf.buf, 0, 
                        pxBuf.width * pxBuf.height);
    }    
    this->get_component<tod_gl::TransformComponent>().set_parent(get_bounded_scene().find_entity_with_tag("base_footprint"));

    // TODO Move to function
    auto& bfTransform = get_bounded_scene().find_entity_with_tag("base_footprint").template get_component<tod_gl::TransformComponent>();
    auto& camTranform = get_bounded_scene().find_entity_with_tag("Cosys"+_cam_name).template get_component<tod_gl::TransformComponent>();

    glm::mat4 t_cam_base = tod_gl::TransformSystem::get_instance()->get_transform_between_entities(bfTransform, camTranform);
    glm::vec3 translation = tod_gl::TransformSystem::get_instance()->extract_translation(t_cam_base);
    glm::quat glm_quat = glm::quat_cast(glm::mat3(t_cam_base));
    
    _camera_transformation.header.frame_id = "base_footprint";
    _camera_transformation.child_frame_id = _cam_name;

    _camera_transformation.transform.translation.x = translation.x;
    _camera_transformation.transform.translation.y = translation.y;
    _camera_transformation.transform.translation.z = translation.z;

    _camera_transformation.transform.rotation.x = glm_quat.x;
    _camera_transformation.transform.rotation.y = glm_quat.y;
    _camera_transformation.transform.rotation.z = glm_quat.z;
    _camera_transformation.transform.rotation.w = glm_quat.w;


    this->add_component<tod_gl::RenderableElementComponent>(videoShader, mesh);
    this->add_component<tod_gl::ExpirableComponent>(1000);
    this->add_component<tod_gl::DynamicDataComponent>();


    // Init tod_gl::TransformComponent
    this->get_component<tod_gl::TransformComponent>().set_parent(get_bounded_scene().find_entity_with_tag("base_footprint"));

    auto tag = this->get_component<tod_gl::TagComponent>().tag;

    auto serializable = tod_gl::serialization::deserialize_video_component(_config_path, _vehicle_ID, _cam_name, this->get_component<tod_gl::VideoComponent>(), this->get_component<tod_gl::TransformComponent>());
    //auto serializable{true};

    if (serializable) {
        if (videoCmp.is_fisheye) {
            _o_cam_model = std::make_unique<OcamModel>(videoCmp.camera_name, _vehicle_ID, _cam_config_path);
            videoCmp.width_raw = _o_cam_model->width_raw;
            videoCmp.height_raw = _o_cam_model->height_raw;
            init_mesh<OcamModel>( *_o_cam_model);
            initMesh = true;
        } else {
            //const PinholeModel camMdl(videoCmp.camera_name, _vehicle_ID, _cam_config_path);
            _pinhole_cam_model = std::make_unique<PinholeModel>(videoCmp.camera_name, _vehicle_ID, _cam_config_path);
            videoCmp.width_raw = _pinhole_cam_model->width_raw;
            videoCmp.height_raw = _pinhole_cam_model->height_raw;
            init_mesh<PinholeModel>(*_pinhole_cam_model);
            initMesh = true;
        }
    } else {
        std::cerr << "Could not find configs for vehicle " << _vehicle_ID << std::endl;
    }
}

template<typename VideoComp>
void VideoRenderer<VideoComp>::on_destroy() {}

template<typename VideoComp>
void VideoRenderer<VideoComp>::on_update(float) {

    auto &_stateManager = tod_gl::StateManager::get_instance();
    auto &tag = this->get_component<tod_gl::TagComponent>();


    auto &dynamic = this->get_component<tod_gl::DynamicDataComponent>();
    auto &renderable = this->get_component<tod_gl::RenderableElementComponent>();
    auto &mesh = renderable.meshes.front();

    if (_stateManager.contains_entity(_scene_manager_key)) {
        if (!_stateManager.should_render_entity(_scene_manager_key)) {
            auto &dynamic = this->get_component<tod_gl::DynamicDataComponent>();
            std::lock_guard<std::mutex> lock(*dynamic.mutex);
            dynamic.has_new_data = true;
            mesh.vertices.clear();
            mesh.indices.clear();
            initMesh = false;
            return;
        }
    }

    auto &videoCmp = this->get_component<tod_gl::VideoComponent>();    


    if (!initMesh) {
        videoCmp.scaling_x = 1.0;
        videoCmp.scaling_y = 1.0;
        if (videoCmp.is_fisheye) {
            init_mesh<OcamModel>( *_o_cam_model);
        } else {
            init_mesh<PinholeModel>(*_pinhole_cam_model);
        }
        initMesh = true;
    }

    std::lock_guard<std::mutex> lock(*dynamic.mutex);
    this->get_component<tod_gl::ExpirableComponent>().restamp();

    auto subManager = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!subManager.template has_component<VideoComp>()) {
        return;
    }

    auto &imageComp = subManager.template get_component<VideoComp>();
    if ((imageComp.image.data.empty())) {
        return;
    }


    videoCmp.last_image_msg = std::make_shared<sensor_msgs::msg::Image>(imageComp.image);

    if (videoCmp.pixel_buffers.at(0).width != videoCmp.last_image_msg->width ||
                videoCmp.pixel_buffers.at(0).height != videoCmp.last_image_msg->height) {
                update_resolution();
    }

    auto &pxBuf = videoCmp.pixel_buffers.at(0);
    tod_gl::Renderer::update_texture(
        mesh.textures.at(0),
        0,
        pxBuf.buf,
        0, 0,
        pxBuf.width,
        pxBuf.height,
        (void*)videoCmp.last_image_msg->data.data());


    this->get_component<tod_gl::DynamicDataComponent>().has_new_data = true;
    // Maybe?
    //curImage.reset();

    // TODO: update_texture Intro  ´/ Fix scene Render Loop


}
}  // namespace TodDynamicEntities
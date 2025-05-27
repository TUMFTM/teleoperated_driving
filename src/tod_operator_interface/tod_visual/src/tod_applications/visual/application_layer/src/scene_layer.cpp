/**
 * @file visual_layer.cpp
 * @brief VisualLayer creating the content of the @ref Scene of the Application
 * @copyright 2024 TUMFTM
**/

#include "scene_layer.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <future>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "tod_gl/core/cursor_position.hpp"
#include "tod_gl/core/scene_application.hpp"
#include "tod_gl/scene/scene_serialization.hpp"
#include "tod_gl/systems/camera_system.hpp"

#include "tod_static_entities/camera.hpp"
#include "tod_static_entities/coordinate_system.hpp"
#include "tod_static_entities/floor.hpp"
#include "tod_static_entities/grid.hpp"
#include "tod_static_entities/orbital_point.hpp"


#include "tod_dynamic_entities/point_cloud_renderer.hpp"
#include "tod_dynamic_entities/driving_lane_renderer.hpp"
#include "tod_dynamic_entities/position_controller.hpp"
#include "tod_dynamic_entities/wheel_controller.hpp"
#include "tod_dynamic_entities/object_list_renderer.hpp"
#include "tod_dynamic_entities/path_control_points_renderer.hpp"
#include "tod_dynamic_entities/lanelet_map_renderer.hpp"
#include "tod_dynamic_entities/path_renderer.hpp"
#include "tod_dynamic_entities/video_renderer.hpp"
#include "tod_dynamic_entities/trajectory_renderer.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "tod_operator_msgs/msg/key_press.hpp"

#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/path.hpp"

namespace tod_visual {

VisualLayer::VisualLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene)
    : SceneLayer(ros, scene, "InterfaceLayer") {
    std::string desired_path= _ros.get()->get_config_path() + "/vehicle_config/";
    _veh_params = std::make_unique<tod_core::param_set::Vehicle>(_ros.get(), desired_path);
    _cam_params = std::make_unique<tod_core::param_set::Camera>(_ros.get(), desired_path);
    _transform_params = std::make_unique<tod_core::param_set::Transform>(_ros.get(), desired_path);
}

void VisualLayer::on_attach() {
    create_scene();
}

void VisualLayer::on_detach() {}

void VisualLayer::on_update(float ts) {
    _active_scene->on_update(ts);
    _time_step = ts;
}

void VisualLayer::on_im_gui_render() {}

void VisualLayer::on_event(tod_gl::Event& e) {
    tod_gl::EventDispatcher dispatcher(e);

    dispatcher.dispatch<tod_gl::WindowResizeEvent>(
        [this](auto&&... args) { return this->handle_window_resize_event(std::forward<decltype(args)>(args)...); });
    dispatcher.dispatch<tod_gl::MouseMovedEvent>(
        [this](auto&&... args) { return this->handle_mouse_moved_event(std::forward<decltype(args)>(args)...); });
    dispatcher.dispatch<tod_gl::MouseButtonPressedEvent>(
        [this](auto&&... args) { return this->handle_mouse_button_pressed_event(std::forward<decltype(args)>(args)...); });
    dispatcher.dispatch<tod_gl::KeyPressedEvent>(
        [this](auto&&... args) { return this->handle_key_pressed_event(std::forward<decltype(args)>(args)...); });

    dispatcher.dispatch<tod_gl::MouseButtonReleasedEvent>(
        [this](auto&&... args) { return this->handle_mouse_button_released_event(std::forward<decltype(args)>(args)...); });
    dispatcher.dispatch<tod_gl::KeyReleasedEvent>(
        [this](auto&&... args) { return this->handle_key_released_event(std::forward<decltype(args)>(args)...); });

    auto view = _active_scene->registry.view<tod_gl::CameraComponent>();
    auto bf = _active_scene->find_entity_with_tag("base_footprint");
    for (auto entity : view) {
        auto& camera = _active_scene->registry.get<tod_gl::CameraComponent>(entity);
        if (camera.controllable) {
            _cam_controller.on_event(e, camera, _time_step);
        }
    }
}

tod_gl::FrameBufferComponent* VisualLayer::get_default_framebuffer() {
    auto view = _active_scene->registry.view<tod_gl::FrameBufferComponent>();


    for (auto entity : view) {
        tod_gl::FrameBufferComponent& framebuffer = _active_scene->registry.get<tod_gl::FrameBufferComponent>(entity);
        if (framebuffer.is_default_framebuffer) {
            return &framebuffer;
        }
    }
    return nullptr;
}

void VisualLayer::handle_window_resize_event(tod_gl::WindowResizeEvent& e) {
    tod_gl::FrameBufferComponent* framebuffer = get_default_framebuffer();

    if (framebuffer) {
        framebuffer->render_width = e.get_width();
        framebuffer->render_height = e.get_height();
        auto& camera = _active_scene->registry.get<tod_gl::CameraComponent>(framebuffer->camera_entity);
        tod_gl::CameraSystem::on_window_size_changed(camera, e.get_width(), e.get_height());
    }
}

void VisualLayer::handle_mouse_moved_event(tod_gl::MouseMovedEvent& e) {
    if (!_cam_controller.get_mouse_pressed() || !_cam_controller.get_is_move_camera()) {
        auto& cursorPosition = tod_gl::CursorPosition::get();

        if (cursorPosition.is_position_valid()) {
            auto [x, y] = cursorPosition.get_mouse_position();
            auto [width, height] = cursorPosition.get_viewport_dimensions();

            auto floorTransform = _active_scene->find_entity_with_tag("floor").get_component<tod_gl::TransformComponent>();

            _mouse_position = cursorPosition.get_real_world_coordinates(_active_scene->view, _active_scene->projection, x,
                                                                    y, width, height, floorTransform);

            std::lock_guard<std::mutex> lock(_mouse_position_mutex);
            _ros->set_mouse_moved_for_publish(_mouse_position);

            // std::cout << "handle_mouse_button_pressed_event xr: " << _mouse_position.point.x << ", yr: " <<
            // _mouse_position.point.y  << std::endl;
        }
    }
}

void VisualLayer::handle_mouse_button_pressed_event(tod_gl::MouseButtonPressedEvent& e) {
    if (!_cam_controller.get_is_move_camera()) {
        std::lock_guard<std::mutex> lock(_mouse_position_mutex);

        _mouse_position.header.frame_id = "map";
        _ros->set_mouse_click_for_publish(_mouse_position);
    }
}

void VisualLayer::handle_mouse_button_released_event(tod_gl::MouseButtonReleasedEvent& e) {
    if (!_cam_controller.get_is_move_camera()) {
        std::lock_guard<std::mutex> lock(_mouse_position_mutex);

        _mouse_position.header.frame_id = "map";
        _ros->set_mouse_release_for_publish(_mouse_position);
    }
}

void VisualLayer::handle_key_pressed_event(tod_gl::KeyPressedEvent& e) {
    tod_operator_msgs::msg::KeyPress msg;
    msg.key = static_cast<int>(e.get_key_code());
    _ros->set_key_press_for_publish(msg);
}

void VisualLayer::handle_key_released_event(tod_gl::KeyReleasedEvent& e) {
    tod_operator_msgs::msg::KeyPress msg;
    msg.key = static_cast<int>(e.get_key_code());
    _ros->set_key_release_for_publish(msg);
}

void VisualLayer::create_scene() {
    rclcpp::Rate r(10);
    while (_cam_params->get_current_id() == "") {
        r.sleep();
        RCLCPP_INFO_THROTTLE(_ros->get_logger(), *_ros->get_clock(), 1000, "waiting for id");
    }

    // Load parameter default values after vehicleID was set
    _veh_params->load_parameters();
    _cam_params->load_parameters();
    _transform_params->load_parameters();

    create_coodinate_system_entites();
    create_camera_and_framebuffer();

    // create_display_entites();
    create_vehicle_model_entites();
    create_grid_and_floor_entites();

    bind_scripts();

    create_video_renderers();
    create_orbital_point();
    // check that all initialized entities have a parent except for map entity
    auto view = _active_scene->registry.view<tod_gl::TransformComponent>();
    for (auto entity : view) {
        tod_gl::TransformComponent& myTf = _active_scene->registry.get<tod_gl::TransformComponent>(entity);
        const tod_gl::TagComponent& myTag = _active_scene->registry.get<tod_gl::TagComponent>(entity);
        if (myTf.parent_entity.get_handle() == entt::null) {
            if (!myTf.is_map_frame) {
                myTf.set_parent(_coordinate_systems.at("base_footprint"));
                const auto& parentTag =
                    _active_scene->registry.get<tod_gl::TagComponent>(myTf.parent_entity.get_handle());
                RCLCPP_WARN(_ros->get_logger(), "Entity %s does not have a parent, set to %s", myTag.tag.c_str(),
                            parentTag.tag.c_str());
            }
        }
    } 
    std::cout << "VisualLayer created" << std::endl;
}

void VisualLayer::create_coodinate_system_entites() {
    tod_gl::Entity map = TodStaticEntities::CoordinateSystem::create(_active_scene, "map");
    _coordinate_systems.emplace(map.get_component<tod_gl::TagComponent>().tag, map);

    map.get_component<tod_gl::TransformComponent>().is_map_frame = true;


    tod_gl::Entity bf =
        TodStaticEntities::CoordinateSystem::create(_active_scene, "base_footprint");
    tod_gl::DynamicDataComponent& dyn = bf.add_component<tod_gl::DynamicDataComponent>();
    dyn.has_new_data = true;  // has new data for first render loop iteration
    _active_scene->set_base_foot_print(bf.get_handle());

    _coordinate_systems.emplace(bf.get_component<tod_gl::TagComponent>().tag, bf);
    bf.get_component<tod_gl::TransformComponent>().set_parent(map);

    std::string parentTag = bf.get_component<tod_gl::TagComponent>().tag;
    for (const auto& tf : _transform_params->get_transforms()) {
        const std::string& child = tf.child_frame_id;
        tod_gl::Entity newCosys = TodStaticEntities::CoordinateSystem::create(_active_scene, "Cosys" + child);

        glm::vec3 translation(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
        glm::quat q(tf.transform.rotation.w, tf.transform.rotation.x,
                    tf.transform.rotation.y, tf.transform.rotation.z);
        // Check if correct
        glm::mat4 transformMat = glm::translate(glm::mat4(1.0f), translation) * glm::mat4(q);

        glm::vec3 extractedRotation = tod_gl::TransformSystem::get_instance()->extract_rotation_euler(transformMat);
        auto& tfCmp = newCosys.get_component<tod_gl::TransformComponent>();
        tfCmp.set_translation(translation);
        tfCmp.set_rotation(extractedRotation);
        tfCmp.set_parent(bf);

        newCosys.get_component<tod_gl::RenderableElementComponent>().static_show = false;
        _coordinate_systems.emplace(child, newCosys);
    }
}

// TODO Niklas: Do we remove it? it has interesting code in ti
void VisualLayer::create_display_entites() {
    // float displayHeight{ 0.20f };
    // float displayX{ 1.75f };
    //
    // tod_gl::Topics::Topic topicList[] = {tod_gl::Topics::VelocityDisplay, tod_gl::Topics::DesiredVelocityDisplay,
    // tod_gl::Topics::GearDisplay, tod_gl::Topics::DesiredGearDisplay}; float displayYs[] = {-.45f, 0.f, .3f, .5f};
    //
    // for (uint8_t ix = 0; ix < 4 ; ix++){
    //     tod_gl::Topics::Topic& topic = topicList[ix];
    //     tod_gl::Entity displayEntity = TodStaticEntities::Display::create(
    //     _active_scene, topic.entityName, _coordinate_systems.at(Topics::BaseFootPrint.entityName),
    //     tod_gl::RosInterface::get_package_path());
    //
    //     displayEntity.get_component<tod_gl::TransformComponent>().set_translation(glm::vec3(displayX, displayYs[ix],
    //     displayHeight)); std::function<void(const tod_vehicle_msgs::msg::VehicleData::ConstSharedPtr &msg)> cb_updater =
    //         std::bind(TodStaticEntities::Display::onSpeedUpdate, _1, displayEntity);
    //     _ros->add_subscriber<tod_vehicle_msgs::msg::VehicleData>(topic.topicName, cb_updater);
    // }
}

void VisualLayer::create_vehicle_model_entites() {
    std::string modelPath = _ros->get_config_path() + "/vehicle_config/" + _veh_params->get_current_id() + "/model-mesh/";
    tod_gl::ModelLoader* loader =
        new tod_gl::ModelLoader(_active_scene, modelPath);
    float zPos = 0.3f;

    tod_gl::Entity bf = _coordinate_systems.at("base_footprint");

    tod_gl::Entity base = create_vehicle_model_entity("model_chassis", glm::vec3(0.0f, 0.0f, 0.0f), loader);

    tod_gl::Entity steeringWheel =
        create_vehicle_model_entity("model_steeringWheel", glm::vec3(0.3f, 0.3f, zPos), loader); // TODO: Change the naming with new configs
    steeringWheel.get_component<tod_gl::TransformComponent>().set_parent(base);

    glm::vec3 frontLeft = glm::vec3(_veh_params->get_distance_front_axle(), _veh_params->get_track_width() / 2.0, zPos);
    tod_gl::Entity wheel = create_vehicle_model_entity("model_wheel", frontLeft, loader);
    wheel.get_component<tod_gl::TransformComponent>().set_parent(bf);
    wheel.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::FrontWheelController>();

    glm::vec3 frontRight = glm::vec3(_veh_params->get_distance_front_axle(), -_veh_params->get_track_width() / 2.0, zPos);
    wheel = create_vehicle_model_entity("model_wheel", frontRight, loader);
    wheel.get_component<tod_gl::TransformComponent>().set_parent(bf);
    wheel.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::FrontWheelController>();

    glm::vec3 rearLeft = glm::vec3(-_veh_params->get_distance_rear_axle(), _veh_params->get_track_width() / 2.0, zPos);
    wheel = create_vehicle_model_entity("model_wheel", rearLeft, loader);
    wheel.get_component<tod_gl::TransformComponent>().set_parent(bf);
    wheel.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::WheelController>();

    glm::vec3 rearRight = glm::vec3(-_veh_params->get_distance_rear_axle(), -_veh_params->get_track_width() / 2.0, zPos);
    wheel = create_vehicle_model_entity("model_wheel", rearRight, loader);
    wheel.get_component<tod_gl::TransformComponent>().set_parent(bf);
    wheel.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::WheelController>();
}

tod_gl::Entity VisualLayer::create_vehicle_model_entity(const std::string& modelName, const glm::vec3& translation,
                                                     tod_gl::ModelLoader* loader) {
    tod_gl::Entity model = loader->load_model(modelName, _coordinate_systems.at("base_footprint"),
                                             glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(glm::radians(0.0f), 0.0f, 0.0f),
                                             tod_gl::RosInterface::get_package_path());
    model.get_component<tod_gl::TransformComponent>().set_translation(translation);
    return model;
}

void VisualLayer::create_grid_and_floor_entites() {
    tod_gl::Entity grid = TodStaticEntities::Grid::create(
       _active_scene, "Grid", tod_gl::RosInterface::get_package_path());
    grid.get_component<tod_gl::TransformComponent>().is_map_frame = true;
    
    tod_gl::Entity floor =
        TodStaticEntities::Floor::create(_active_scene, "floor", tod_gl::RosInterface::get_package_path(),
                                         _coordinate_systems.at("base_footprint"));
}

void VisualLayer::create_camera_and_framebuffer() {
    tod_gl::Entity camera = TodStaticEntities::Camera::create(
        _active_scene, "VehicleFollowCamera", _coordinate_systems.at("base_footprint"));

    tod_gl::Entity mainFramebuffer = _active_scene->create_entity("MainFramebuffer");
    mainFramebuffer.get_component<tod_gl::TransformComponent>().is_map_frame = true;
    auto& framebuffers = mainFramebuffer.add_component<tod_gl::FrameBufferComponent>(true);
    framebuffers.camera_entity = camera.get_handle();
    // TODO
    // callback switches camera position to lock backwards (gear rear, InvertSteeringInGearReverse in
    // tod_command_creation.launch)
    //  _ros->add_subscriber<tod_vehicle_msgs::VehicleData>("/Operator/VehicleBridge/vehicle_data",
    //                                             TodStaticEntities::Camera::onGearUpdate, camera);
}


void VisualLayer::create_video_renderers() {
    //TODO Niklas Maybe there is a better way? e.g. have a templated ImageComponent and then use that one
    //TODO Change Name to match real name after config change
    //TODO Niklas: Do we use Fisheyes? RC-Car?
    create_video_renderer<tod_gl::ImageComponentFrontCenter>("frontcenter","frontcenter", false);
    // create_video_renderer<tod_gl::ImageComponentFrontLeft>("frontleft","frontleft",false); 
    // create_video_renderer<tod_gl::ImageComponentFrontRight>("frontright","frontright",false);
    // create_video_renderer<tod_gl::ImageComponentRearCenter>("rearcenter","rearcenter",false);
    // create_video_renderer<tod_gl::ImageComponentRearLeft>("rearleft","rearleft",false); 
    // create_video_renderer<tod_gl::ImageComponentRearRight>("rearright","rearright",false);
}


template<typename VideoComp>
void VisualLayer::create_video_renderer(const std::string& name, const std::string& stateKey , const bool isFisheye ) {

    const auto vehicleID = _ros->get_parameter("vehicleID").as_string();
    const auto cam_config_path = _cam_params->get_path_to_config_files(); 
    const auto config_path = _ros->get_config_path();
    tod_gl::Entity VideoRendererEnt = _active_scene->create_entity(name + "Renderer");
    VideoRendererEnt.add_component<tod_gl::ScriptComponent>()
        .bind_with_params<TodDynamicEntities::VideoRenderer<VideoComp>>(name,stateKey,cam_config_path,vehicleID,config_path,isFisheye);
}

void VisualLayer::bind_scripts() {
    tod_gl::Entity bf = _coordinate_systems.at("base_footprint");
    bf.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::PositionController>();

    tod_gl::Entity ObjectRenderer = _active_scene->create_entity("ObjectRenderer");
    ObjectRenderer.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::ObjectListRenderer>();
    ObjectRenderer.get_component<tod_gl::TransformComponent>().set_parent(_active_scene->find_entity_with_tag("map"));

    tod_gl::Entity TrajectoryRenderEnt = _active_scene->create_entity("TrajectoryRenderer");
    TrajectoryRenderEnt.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::TrajectoryRenderer>();
    TrajectoryRenderEnt.get_component<tod_gl::TransformComponent>().set_parent(_active_scene->find_entity_with_tag("map"));

    tod_gl::Entity DrivingLaneRendererEnt = _active_scene->create_entity("DrivingLaneRenderer");
    DrivingLaneRendererEnt.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::DrivingLaneRenderer>();

    tod_gl::Entity PointCloudRender = _active_scene->create_entity("PointCloudRenderer");
    PointCloudRender.add_component<tod_gl::ScriptComponent>()
        .bind<TodDynamicEntities::PointCloudRenderer>();

    tod_gl::Entity PathControlPoints = _active_scene->create_entity("PathControlPointsRenderer");
    PathControlPoints.add_component<tod_gl::ScriptComponent>().bind<TodDynamicEntities::PathControlPointsRenderer>();
    PathControlPoints.get_component<tod_gl::TransformComponent>().set_parent(_active_scene->find_entity_with_tag("map"));

    tod_gl::Entity pathRendererEnt = _active_scene->create_entity("PathRenderer");
    pathRendererEnt.add_component<tod_gl::ScriptComponent>().bind_with_params<TodDynamicEntities::PathRenderer<tod_trajectory_guidance_msgs::msg::Path>>(
        "PathRenderer", 
        glm::vec3(1.0f, 1.0f, 1.0f),
        2.7f,
        0.05f);
    pathRendererEnt.get_component<tod_gl::TransformComponent>().set_parent(_active_scene->find_entity_with_tag("map"));

    tod_gl::Entity validationPathRendererEnt = _active_scene->create_entity("ValidationPathRenderer");
    validationPathRendererEnt.add_component<tod_gl::ScriptComponent>().bind_with_params<TodDynamicEntities::PathRenderer<tod_trajectory_guidance_msgs::msg::Trajectory>>(
        "ValidationPathRenderer", 
        glm::vec3(1.0f, .87f, .13f),
        2.7f,
        0.07);
    validationPathRendererEnt.get_component<tod_gl::TransformComponent>().set_parent(_active_scene->find_entity_with_tag("map"));

    std::string map_name;
    std::vector<double> map_origin;
    _ros->get_param<std::string>("map", map_name);
    _ros->get_param<std::vector<double>>("map_origin", map_origin);
    std::string map_path = _ros->get_package_path() + "/resources/maps/" + map_name;
    tod_gl::Entity LaneletRenderer = _active_scene->create_entity("Lanelet");
    LaneletRenderer.add_component<tod_gl::ScriptComponent>().bind_with_params<TodDynamicEntities::LaneletMapRenderer>(map_path, map_origin);
    LaneletRenderer.get_component<tod_gl::TransformComponent>().set_parent(_active_scene->find_entity_with_tag("map"));



}

void VisualLayer::create_orbital_point() {

  auto cam_view = _active_scene->registry.view<tod_gl::CameraComponent>();
  for (auto entity : cam_view) {
    auto &camera = _active_scene->registry.get<tod_gl::CameraComponent>(entity); 
    tod_gl::Entity orbitPointVisual = TodStaticEntities::OrbitalPoint::create(
        _active_scene, 
        "orbit_point_visual", 
        0.2f, 
        tod_gl::RosInterface::get_package_path(),
        _coordinate_systems.at("base_footprint"),
        camera
    );
  }
}
}  // namespace tod_visual
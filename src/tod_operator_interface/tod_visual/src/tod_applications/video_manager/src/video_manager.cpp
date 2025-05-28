#include "video_manager.hpp"



namespace tod_gl {

VideoManager::VideoManager(int argc, char** argv, const std::string& name)
    : Application(argc, argv, name){ 
    push_overlay(new VideoManagerDockingLayer(_ros,ImGuiDir_None));
    _shared_cams_list = std::make_shared<std::vector<std::shared_ptr<SharedCam>>>();
    // Lade Kamera-Parameter
    std::string desired_path= _ros.get()->get_config_path() + "/vehicle_config/";
    _cam_params = std::make_unique<tod_core::param_set::Camera>(_ros.get(), desired_path);
    _cam_params->load_parameters();
    // Erhalte die Sensorliste
    auto sensors = _cam_params->get_sensors();
    for (const auto& cam : sensors) {
        int mapping = get_mapping(cam.name);
   
        auto newCam = std::make_shared<SharedCam>(cam.name, cam.stream_on_connect, mapping);
       _shared_cams_list->emplace_back(newCam);
       push_overlay(new VideoStateLayer(_ros, newCam, ImGuiDir_Left));
        RCLCPP_INFO(get_logger(), "Added camera: Name = %s, Active = %d, Mapping = %d",
                    cam.name.c_str(), cam.stream_on_connect, mapping);
    }
    push_overlay(new CameraPositionLayer(_ros, _shared_cams_list,ImGuiDir_Right));
}

int VideoManager::get_mapping(const std::string& name) {
        if (name.find("rearright") != std::string::npos) {
            return 1;
        } else if (name.find("rearcenter") != std::string::npos) {
            return 2;
        } else if (name.find("rearleft") != std::string::npos) {
            return 3;
        } else if (name.find("frontleft") != std::string::npos) {
            return 4;
        } else if (name.find("frontcenter") != std::string::npos) {
            return 5;
        } else if (name.find("frontright") != std::string::npos) {
            return 6;
        }
        return 0; 
    }
}
 

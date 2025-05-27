/**
 * @file input_device_controller.cpp
 * @brief Integration of the @ref MyInputDevice with the ROS Interface   
 * @copyright 2025 TUMFTM
 **/

#include "tod_input_devices/input_device_controller.hpp"
#include <stdlib.h>
#include <functional>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace tod_input_device {

InputDeviceController::InputDeviceController(int argc, char **argv) {
    _ros = std::make_unique<RosInterface>(argc, argv, this);
    
    _ros->init();
    _ros->services.push_back(_ros->node->create_service<tod_operator_msgs::srv::InputDevice>(
                            "InputDevice/change_input_device",
                            std::bind(&InputDeviceController::callback_change_device_request, this, _1, _2)));

    try {
        _ros->node->declare_parameter<bool>("ConfigMode", _configurationMode);  
        _configurationMode = _ros->node->get_parameter("ConfigMode").as_bool();
        if (_configurationMode) {
            RCLCPP_INFO(_ros->node->get_logger(),
                        "Parameter 'ConfigMode' set to true");
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(_ros->node->get_logger(),
                    "Failed to retrieve 'ConfigMode' parameter. Using default value: %s. Error: %s",
                    _configurationMode ? "true" : "false", e.what());
    }

    try {
        _ros->node->declare_parameter<bool>("debug", _debug); 
        _debug = _ros->node->get_parameter("debug").as_bool();
        if (_debug) {
            RCLCPP_INFO(_ros->node->get_logger(),
                        "Parameter 'debug' set to true");
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(_ros->node->get_logger(),
                    "Failed to retrieve 'debug' parameter. Using default value: false. Error: %s",
                    _debug ? "true" : "false", e.what());
    }
    
    if (_debug) {
        _ros->set_debug_mode();
    }

    auto axis_callback = std::bind(
        &InputDeviceController::callback_axis_changed, this, std::placeholders::_1, std::placeholders::_2);
    auto button_callback = std::bind(
        &InputDeviceController::callback_button_changed, this, std::placeholders::_1, std::placeholders::_2);
    auto error_callback = std::bind(
        &InputDeviceController::callback_error, this, std::placeholders::_1);

    _input_devices["Virtual"] = std::make_shared<VirtualInputDevice>(axis_callback, button_callback);
    _input_devices["Usb"] = std::make_shared<UsbInputDevice>(axis_callback, button_callback, error_callback);
    #if SENSO_MACRO 
        _input_devices["Senso"] = std::make_unique<SensoInputDevice>(axis_callback, button_callback);
    #endif
    change_input_device(); // Default Parameter-File loaded in launch-File
}

InputDeviceController::~InputDeviceController() {
    _ros->terminate();
}

void InputDeviceController::terminate() {
    for (auto &device : _input_devices) { // Deactivate others
        RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << ": Deactivating Input Device: " << device.first);
        device.second->terminate();
    }
}

void InputDeviceController::callback_axis_changed(const int axis, const double value) {
    if (_configurationMode) {
        RCLCPP_INFO_STREAM(_ros->get_logger(), "Axis:" << axis << ", Value: " << value << ", inverted:" << _axis_mapping.at(axis).invert_axes);
    }

    float sign;
    if (_axis_mapping.count(axis) != 0) {
        sign = _axis_mapping.at(axis).invert_axes ? -1.0 : 1.0;
        _ros->set_axis(_axis_mapping.at(axis).position, sign * value); // sets Button at MsgPos with HW Button as input
    }
}

void InputDeviceController::callback_button_changed(const int button, const int state) {
    if (_configurationMode)
        RCLCPP_INFO_STREAM(_ros->get_logger(), "Button:" << button << ", State: " << state);

    if (_button_mapping.count(button) != 0)
        _ros->set_button(_button_mapping.at(button), state); // sets Button at MsgPos with HW Button as input
}

void InputDeviceController::callback_error(const std::string &errorMsg) {
    RCLCPP_ERROR_STREAM_ONCE(_ros->get_logger(), errorMsg);
    _ros->terminate();
}

bool InputDeviceController::callback_change_device_request(
    const std::shared_ptr<tod_operator_msgs::srv::InputDevice::Request> req,
    const std::shared_ptr<tod_operator_msgs::srv::InputDevice::Response> res)
{
    std::string cfg_file_path = req->input_device_directory;
    std::string load_command = "ros2 param load " + std::string(_ros->get_namespace()) + "/"
            + _ros->get_node_name()  + " " + cfg_file_path;

    namespace fsys = std::filesystem;

    if (!fsys::exists(cfg_file_path) || !(fsys::is_regular_file(cfg_file_path) || fsys::is_symlink(cfg_file_path))) {
        RCLCPP_DEBUG_STREAM(_ros->get_logger(), 
            _ros->get_node_name() << ": Could not find or open config file " << cfg_file_path);
        res->successfully_changed = false;
        return false;
    } else {
        RCLCPP_DEBUG_STREAM(_ros->get_logger(), 
            _ros->get_node_name() << ": Executing command: " << load_command);
        
        uint result = 0; // std::system(load_command.c_str());
        std::vector<rclcpp::Parameter> params = Yayp_te(cfg_file_path);
        for (rclcpp::Parameter param : params) {
            try {
                result += !_ros->set_parameter(param).successful;
            } catch (const std::exception &e) {
                RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << " Exception when changing parameters: " << e.what());
            }
        }
        
        RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << " result: " << result);
        if (result != 0) {
            res->successfully_changed = false;
            RCLCPP_DEBUG_STREAM(_ros->get_logger(), 
                _ros->get_node_name() << ": Could not load config file " << cfg_file_path);
            return false;
        }
    }
    change_input_device();
    res->successfully_changed = true;
    return true;
}

void InputDeviceController::change_input_device() {
    std::string inputDeviceType = _ros->get_parameter("type").get_parameter_value().get<std::string>();

    RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << ": Try changing to Type: " << inputDeviceType);

    // Deactivate all input devices
    for (auto &device : _input_devices) {
        RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << ": Input Device " << device.first 
                            << (device.second->running ? " running" : " not running"));
        if (device.second->running) { // if running
            RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << ": Deactivating Input Device: " << device.first);
            device.second->deactivate();
        }
    }
    // Clear
    _ros->clear_joystick_msg();

    // Load new parameters
    update_mapping_from_param_workspace();

    std::string _correction = _ros->get_parameter("correction").get_parameter_value().get<std::string>();
    _input_devices[inputDeviceType]->set_correction(_correction);

    // Start desired input device
    if (!_input_devices[inputDeviceType]->running) {
        RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << ": Activating Input Device: " << inputDeviceType);
        _input_devices[inputDeviceType]->activate();
        RCLCPP_DEBUG_STREAM(_ros->get_logger(), _ros->get_node_name() << ": Input Device: " << inputDeviceType << " activated");
    }
    if (_configurationMode) {
        RCLCPP_INFO_STREAM(_ros->get_logger(), "Number Of Axes:" << _input_devices[inputDeviceType]->get_number_of_axes());
        RCLCPP_INFO_STREAM(_ros->get_logger(), "Number Of buttons:" << _input_devices[inputDeviceType]->get_number_of_buttons());
    }
}

void InputDeviceController::update_mapping_from_param_workspace() {
    _button_mapping.clear();
    _axis_mapping.clear();

    int par_indicator_left = _ros->get_parameter("button_config.IndicatorLeft").get_parameter_value().get<int>();
    int par_indicator_right = _ros->get_parameter("button_config.IndicatorRight").get_parameter_value().get<int>();
    int par_flashlight = _ros->get_parameter("button_config.FlashLight").get_parameter_value().get<int>();
    int par_frontlight = _ros->get_parameter("button_config.FrontLight").get_parameter_value().get<int>();
    int par_honk = _ros->get_parameter("button_config.Honk").get_parameter_value().get<int>();
    int par_increase_speed = _ros->get_parameter("button_config.IncreaseSpeed").get_parameter_value().get<int>();
    int par_decrease_speed = _ros->get_parameter("button_config.DecreaseSpeed").get_parameter_value().get<int>();
    int par_increase_gear = _ros->get_parameter("button_config.IncreaseGear").get_parameter_value().get<int>();
    int par_decrease_gear = _ros->get_parameter("button_config.DecreaseGear").get_parameter_value().get<int>();
    int par_axis_steering = _ros->get_parameter("axis_config.Steering").get_parameter_value().get<int>();
    int par_axis_throttle = _ros->get_parameter("axis_config.Throttle").get_parameter_value().get<int>();
    int par_axis_brake = _ros->get_parameter("axis_config.Brake").get_parameter_value().get<int>();
    bool par_invert_axis_steering = _ros->get_parameter("invert_axis.Steering").get_parameter_value().get<bool>();
    bool par_invert_axis_throttle = _ros->get_parameter("invert_axis.Throttle").get_parameter_value().get<bool>();
    bool par_invert_axis_brake = _ros->get_parameter("invert_axis.Brake").get_parameter_value().get<bool>();

    _button_mapping[par_indicator_left] = joystick::ButtonPos::INDICATOR_LEFT;
    _button_mapping[par_indicator_right] = joystick::ButtonPos::INDICATOR_RIGHT;
    _button_mapping[par_flashlight] = joystick::ButtonPos::FLASHLIGHT;
    _button_mapping[par_frontlight] = joystick::ButtonPos::FRONTLIGHT;
    _button_mapping[par_honk] = joystick::ButtonPos::HONK;
    _button_mapping[par_increase_speed] = joystick::ButtonPos::INCREASE_SPEED;
    _button_mapping[par_decrease_speed] = joystick::ButtonPos::DECREASE_SPEED;
    _button_mapping[par_increase_gear] = joystick::ButtonPos::INCREASE_GEAR;
    _button_mapping[par_decrease_gear] = joystick::ButtonPos::DECREASE_GEAR;

    _axis_mapping.insert(std::make_pair(par_axis_steering, AxisItem(joystick::AxesPos::STEERING, 
                        par_invert_axis_steering)));
    _axis_mapping.insert(std::make_pair(par_axis_throttle, AxisItem(joystick::AxesPos::THROTTLE,
                        par_invert_axis_throttle)));
    _axis_mapping.insert(std::make_pair(par_axis_brake, AxisItem(joystick::AxesPos::BRAKE,
                        par_invert_axis_brake)));
}

std::vector<rclcpp::Parameter> InputDeviceController::Yayp_te(std::string path_to_yaml) {
    /** Yet another yaml parser - tod edition! Only supports one layer of parameter namespaces! **/
    std::vector<rclcpp::Parameter> parameters;

    std::string parameter_namespace = "";
    int parameter_namespace_layer = -1;
    std::string node = "";
    std::fstream yaml_file;
    int ros_parameters_layer = -1;
    
    yaml_file.open(path_to_yaml, std::ios::in);
    if (yaml_file.is_open()){
        std::string line = "";
        std::string key = "";
        std::string value = "";

        bool value_to_push = false;
        bool concatenate_lines = false;
        
        while(std::getline(yaml_file, line)) { // Read data from file object into string.
            line = line.substr(0, line.find("#")); // Remove comments from line
            bool not_empty_line = (line.find(":") != std::string::npos) || concatenate_lines;

            if(not_empty_line) {
                std::string::size_type current_layer = 0;
                while (line.find(" ", current_layer + 1) == current_layer + 1)
                    current_layer++;
                current_layer = (current_layer == std::string::npos) ? 0 : current_layer;
                bool new_concatenate_lines = (line.find("\\") != std::string::npos);
                line = line.substr(current_layer + 1, line.find("\\") - (current_layer + 1));

                if(concatenate_lines) {
                    value += line;
                }
                else if(line.find("ros__parameters:") != std::string::npos) { // Line indicates ros_parameters
                    parameter_namespace_layer = current_layer + 2;
                    ros_parameters_layer = current_layer;
                }
                else if(ros_parameters_layer < 0) {
                    node = line.substr(0, line.find(":"));
                }
                else if((ros_parameters_layer > 0) && (current_layer > static_cast<std::string::size_type>(ros_parameters_layer))) { // parameter or parameter string layer
                    key = line.substr(0, line.find(":"));
                    while(key.back() == ' ')
                        key.pop_back();
                    value = line.substr(line.find(":") + 1, std::string::npos);
                    std::string only_spaces(value.length(), ' ');

                    bool is_parameter_layer = (value.empty() || !value.compare(only_spaces));
                    if(is_parameter_layer) {
                        parameter_namespace = line.substr(0, line.find(":"));
                        parameter_namespace_layer = current_layer;
                    }
                    else if(current_layer > static_cast<std::string::size_type>(ros_parameters_layer)) {
                        value_to_push = true;
                        if(current_layer > static_cast<std::string::size_type>(parameter_namespace_layer)) {
                            key = "" + parameter_namespace + "." + key;
                        }
                    }
                }
                if(value_to_push && !new_concatenate_lines) {
                    value_to_push = false;
                    try {
                        int i_value = std::stoi(value);
                        parameters.push_back(rclcpp::Parameter(key, i_value));
                    }
                    catch(...) {
                        if(value.find('"') != std::string::npos) {
                            value = value.substr(value.find('"') + 1, std::string::npos);
                            value = value.substr(0, value.find('"'));
                            parameters.push_back(rclcpp::Parameter(key, value));
                        }
                        else {
                            if(value.find("true") != std::string::npos) {
                                parameters.push_back(rclcpp::Parameter(key, true));
                            }
                            else if(value.find("false") != std::string::npos) {
                                parameters.push_back(rclcpp::Parameter(key, false));
                            }
                        }
                    }
                }
                concatenate_lines = new_concatenate_lines;
            }
        }
        yaml_file.close(); // Close the file object.
    }
    return parameters;
}

} // namespace tod_input_device

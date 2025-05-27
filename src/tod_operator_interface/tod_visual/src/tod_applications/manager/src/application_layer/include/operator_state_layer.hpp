/**
 * @file operator_state_layer.hpp
 * @ingroup tod_visual_application
 * @brief Defines the Operator State Layer for the application.
 * 
 * This class provides the GUI and functionality for managing the operator state,
 * including IP address selection, status updates, and interaction with ROS components.
 * 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <cmath> 
#include <string>
#include <vector>
#include <filesystem> 

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "ipv4_validity_checker.h"

#include "tod_gl/layers/imgui_layer.hpp"
#include "tinyfiledialogs.h"

#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"
#include "tod_gl/ros_interface/publishing_components/manager_button_status_component.hpp"
#include "tod_gl/ros_interface/service_components/input_device_component.hpp"
#include "tod_gl/ros_interface/service_components/packet_capture_component.hpp"
#include "tod_gl/ros_interface/service_components/network_monitor_component.hpp"

#include "operator_manager_docking_layer.hpp"

/**
 * @enum GUIState
 * @brief Represents the various states of the operator GUI for enabling and disabling elements depending on the state.
 */
enum GUIState {
    DISCONNECTED,
    IS_CONNECTING,
    CONNECTED,
    IS_STARTING,
    STARTED,
    IS_STOPPING,
    STOPPED,
    IS_DISCONNECTING
};

    inline std::string PRIO1 { "10.100.80." }; // NOT FOR OPEN SOURCE  (EDGAR VPN)
    inline std::string PRIO2 { "10.100.199." }; // NOT FOR OPEN SOURCE (TOF VPN)
    inline std::string PRIO3 { "10.100.100." }; // NOT FOR OPEN SOURCE (Lehrstuhl VPN)
    inline std::string PRIO4 { "127.0.0." };
    inline std::string PRIO5 { "192.168." };

    
    inline std::vector<std::string> listWithPriorities;

/**
 * @class operator_state_layer
 * @brief Represents the operator state layer in the application.
 * 
 * This layer handles the GUI elements for displaying and managing the operator state,
 * including IP address selection, state updates, and interaction with ROS components.
 */
class OperatorStateLayer : public tod_gl::OperatorManagerDockingLayer{
  public:
    OperatorStateLayer(std::shared_ptr<tod_gl::RosInterface> _ros, ImGuiDir split_dir);
    ~OperatorStateLayer() = default;

    void on_attach() override;
    void on_detach() override;
    void on_im_gui_render() override;
    void on_event(tod_gl::Event& e) override;
    void on_update(float ts) override;
    char* browseFile();
    static int selectedConcept;
    static int selected_video_rate;
    static int selectedPacketCapture;


    void readAndStoreOwnIpAddresses(); 
    void UpdateState() ;
    private:
    std::vector<std::string> ipOperatorOptions; 
    int selectedOperatorIP = 0;

private:
  tod_gl::TodStatusComponent _status; 
  tod_gl::ManagerButtonStatusComponent _buttonStatus;
  tod_gl::InputDeviceComponent _inputDevice;
  tod_gl::PacketCaptureComponent _packetCapture;
  tod_gl::NetworkMonitorComponent _networkMonitor;
};

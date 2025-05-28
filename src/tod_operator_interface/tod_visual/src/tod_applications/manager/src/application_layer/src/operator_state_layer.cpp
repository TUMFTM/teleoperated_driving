/**
 * @file operator_state_layer.cpp
 * @brief Implementation of the OperatorStateLayer class for managing operator-specific states and UI interactions.
 * 
 * This file defines the OperatorStateLayer class, responsible for handling operator-related functionality 
 * in the teleoperation system, such as IP address management, packet capture mode, and control mode selection. 
 * It provides dynamic and interactive UI elements to manage connections, configurations, and operational states.
 * 
 * The implementation leverages the ImGui framework for rendering UI components and integrates with ROS for
 * system communication and status updates.
 * 
 * Key Features:
 * - Dynamic resizing of UI elements based on window dimensions.
 * - IP address validation and selection for both operator and vehicle configurations.
 * - Packet capture mode selection with visual feedback.
 * - Control mode selection for teleoperation management.
 * - File browsing and configuration selection.
 * - Real-time connection and status handling.
 * 
 * @copyright 2024 
 * TUMFTM. All rights reserved.
 */

#include "operator_state_layer.hpp"


int OperatorStateLayer::selectedConcept = 0;
int OperatorStateLayer::selectedPacketCapture = 0;
tod_status_msgs::msg::Status statusMsg;

std::string selectedFile = "";
std::string displayedFileName = "No file selected"; 
IPv4ValidityChecker ipValidator;

GUIState currentState = DISCONNECTED;

std::chrono::steady_clock::time_point timerStart;
bool isTimerActive = false;
GUIState previousState;
const int timeoutDuration = 5;



OperatorStateLayer::OperatorStateLayer(std::shared_ptr<tod_gl::RosInterface> _ros, ImGuiDir split_dir)
     : OperatorManagerDockingLayer(_ros, split_dir), 
     _status(_ros), _buttonStatus(_ros), _inputDevice(_ros), _packetCapture(_ros), _networkMonitor(_ros){
    _name = "operator_state_layer";
        split_direction = split_dir;
    readAndStoreOwnIpAddresses(); 

}


void OperatorStateLayer::on_attach() {}

void OperatorStateLayer::on_detach() {}

void OperatorStateLayer::on_im_gui_render() {

    UpdateState();
    ImGui::SameLine();
    ImGui::Begin(_dock_space_window_name.c_str());
    ImGui::SameLine();
    ImGui::Begin(_name.c_str(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);


    // Get dynamic window dimensions
    ImVec2 windowSize = ImGui::GetWindowSize();
    float dynamicWidth = windowSize.x;  // Use full width of the window
    float dynamicHeight = 400; //decided to use static height for usability

    

float scale = 1.5f;

// Calculate dynamic sizes
float buttonWidth = dynamicWidth * 0.2f * scale;   // Buttons take 20% of the width
float buttonHeight = dynamicHeight * 0.05f * scale;  // Buttons take 5% of the height
float spacing = dynamicWidth * 0.02f * scale;      // Spacing takes 2% of the width

// Calculate combo width to be 3x the width of buttons placed side-by-side
float comboWidth = (3 * buttonWidth) + (2 * spacing);  // 3 buttons + 2 gaps between them
    // Spacing takes 2% of the width
    if (currentState!=DISCONNECTED) ImGui::BeginDisabled();


    // IP Address Section
    static char inputBuffer[16] = "10.0.0.10"; 
    ImGui::Text("IP Address Vehicle:");
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, (buttonHeight - ImGui::CalcTextSize("Example").y) * 0.5f));

    ImGui::SetNextItemWidth(comboWidth);
    if (ImGui::BeginCombo("##ipVehicle", inputBuffer)) {

    static const char* ipVehicleOptions[] = { "10.0.0.10", "11.0.0.10" ,"127.0.0.1"};
    for (int i = 0; i < IM_ARRAYSIZE(ipVehicleOptions); i++) {
        if (ImGui::Selectable(ipVehicleOptions[i])) {
            strncpy(inputBuffer, ipVehicleOptions[i], sizeof(inputBuffer));
        }
    }

    ImGui::Separator(); 
    ImGui::InputText("Custom IP", inputBuffer, sizeof(inputBuffer));

    ImGui::EndCombo();

}


    ImGui::Text("IP Address Operator:");
  
    ImGui::SetNextItemWidth(comboWidth-buttonWidth-spacing);
   if (ImGui::BeginCombo("##ipOperator", ipOperatorOptions[selectedOperatorIP].c_str())) {
        

        
    for (size_t i = 0; i < ipOperatorOptions.size(); ++i) {
        bool isSelected = (selectedOperatorIP == i);
        if (ImGui::Selectable(ipOperatorOptions[i].c_str(), isSelected)) {
            if (ipValidator.validate(ipOperatorOptions[i])) {
                selectedOperatorIP = i; 
            } else {
                RCLCPP_INFO(_ros->get_logger(),"Invalid IP Address: %s",ipOperatorOptions[i].c_str() );
            }
        }
        if (isSelected) {
            ImGui::SetItemDefaultFocus(); 
        }
    }

    ImGui::EndCombo();
    }
        ImGui::PopStyleVar();

    ImGui::SameLine(0.0f, spacing);

    // Update-Button
    if (ImGui::Button("Update", ImVec2(buttonWidth, buttonHeight))) {
        ipOperatorOptions.clear(); 
        readAndStoreOwnIpAddresses(); 
        selectedOperatorIP = 0; 
    }

    ImGui::Spacing(); 
    ImGui::Spacing(); 
        // Packet Capture Mode Buttons
    ImGui::Text("Packet Capture Mode:");

    const char* packetCaptureModes[] = { "Disabled", "Operator Only", "Both" };

    for (int i = 0; i < IM_ARRAYSIZE(packetCaptureModes); i++) {
        ImVec4 buttonColor = (selectedPacketCapture == i)
                             ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f)
                             : ImGui::GetStyleColorVec4(ImGuiCol_Button);

        ImVec4 hoverColor = (selectedPacketCapture == i)
                            ? ImVec4(0.0f, 0.8f, 0.0f, 1.0f)
                            : ImGui::GetStyleColorVec4(ImGuiCol_ButtonHovered);

        ImGui::PushStyleColor(ImGuiCol_Button, buttonColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hoverColor);

        if (ImGui::Button(packetCaptureModes[i], ImVec2(buttonWidth, buttonHeight))) {
            selectedPacketCapture = i;
       
        }

        

        ImGui::PopStyleColor(2);

        if (i < IM_ARRAYSIZE(packetCaptureModes) - 1) ImGui::SameLine(0.0f, spacing);
    }

    ImGui::Spacing(); 
    if (currentState!=DISCONNECTED) ImGui::EndDisabled();

// Connect/Disconnect Button
    if (currentState==DISCONNECTED || currentState== IS_CONNECTING) {
        if (ImGui::Button(currentState==IS_CONNECTING ? "Connecting..." : "Connect", ImVec2(comboWidth, buttonHeight))) {
            previousState = currentState;
            isTimerActive = true;
            timerStart = std::chrono::steady_clock::now();
            currentState=IS_CONNECTING;
            tod_status_msgs::msg::ManagerButtonStatus buttonStatusMsg;
            buttonStatusMsg.operator_ip_address = ipOperatorOptions[selectedOperatorIP];
            buttonStatusMsg.vehicle_ip_address = std::string(inputBuffer);
            buttonStatusMsg.clicked_button = tod_status_msgs::msg::ManagerButtonStatus::CONNECT_BUTTON;
            buttonStatusMsg.vehicle_control_mode= selectedConcept;
            _buttonStatus.publish_button_status(buttonStatusMsg);

           

            RCLCPP_INFO(_ros->get_logger(), "Connecting to Vehicle IP: %s", inputBuffer);
            if(selectedPacketCapture==0){
                _packetCapture.publish_packet_capture_operator(false);
                _packetCapture.publish_packet_capture_vehicle(false);
            }
            if(selectedPacketCapture==1){
                _packetCapture.publish_packet_capture_operator(true);
                _packetCapture.publish_packet_capture_vehicle(false);
            }
            if(selectedPacketCapture==2){
                _packetCapture.publish_packet_capture_operator(true);
                _packetCapture.publish_packet_capture_vehicle(true);
            }
        }
    } else {
        if (currentState==STARTED||currentState==IS_STOPPING||currentState==IS_STARTING) ImGui::BeginDisabled();
        if (ImGui::Button(currentState==IS_DISCONNECTING ? "Disconnecting..." : "Disconnect", ImVec2(comboWidth, buttonHeight))) {
            previousState = currentState;
            isTimerActive = true;
            timerStart = std::chrono::steady_clock::now();
            currentState=IS_DISCONNECTING;
            tod_status_msgs::msg::ManagerButtonStatus buttonStatusMsg;
            buttonStatusMsg.clicked_button = tod_status_msgs::msg::ManagerButtonStatus::DISCONNECT_BUTTON;
                        buttonStatusMsg.vehicle_control_mode= selectedConcept;
            _buttonStatus.publish_button_status(buttonStatusMsg);
            RCLCPP_INFO(_ros->get_logger(), "Disconnecting to Vehicle IP: %s", inputBuffer);
        }
        if (currentState==STARTED||currentState==IS_STOPPING||currentState==IS_STARTING) ImGui::EndDisabled();
    }

    ImGui::Spacing(); 

    if (currentState==STARTED||currentState==IS_STOPPING||currentState==DISCONNECTED||currentState==IS_CONNECTING) ImGui::BeginDisabled();

   // Control Mode Buttons
    ImGui::Text("Control Mode:");
    const char* controlModes[] = { "Direct Control", "Shared Control","Waypoint Control", "Trajectory Guidance", "Safe Corridor","Perception Modification"
                                     };

    for (int i = 0; i < IM_ARRAYSIZE(controlModes); i++) {

        ImVec4 buttonColor = (selectedConcept == i) 
                             ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f) 
                             : ImGui::GetStyleColorVec4(ImGuiCol_Button);

        ImVec4 hoverColor = (selectedConcept == i) 
                            ? ImVec4(0.0f, 0.8f, 0.0f, 1.0f)
                            : ImGui::GetStyleColorVec4(ImGuiCol_ButtonHovered);


        ImGui::PushStyleColor(ImGuiCol_Button, buttonColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hoverColor);

        if (ImGui::Button(controlModes[i], ImVec2(buttonWidth, buttonHeight))) {
            selectedConcept = i; 
            tod_status_msgs::msg::ManagerButtonStatus buttonStatusMsg;
            buttonStatusMsg.vehicle_control_mode = i;
            buttonStatusMsg.clicked_button= tod_status_msgs::msg::ManagerButtonStatus::NONE_BUTTON;
            _buttonStatus.publish_button_status(buttonStatusMsg);
        }

        ImGui::PopStyleColor(2);

          if (i == 2) {
        ImGui::NewLine();
    } else if (i < IM_ARRAYSIZE(controlModes) - 1) {
        ImGui::SameLine(0.0f, spacing);
    }
    }

  
    ImGui::Spacing(); 


    ImGui::Spacing(); 


    ImGui::Text("Select Configuration File:");


    if (ImGui::Button("Browse", ImVec2(buttonWidth, buttonHeight))) {

        char* filePath= browseFile();
        if (filePath) {
            std::filesystem::path selectedPath(filePath);
            selectedFile = selectedPath.string();
            displayedFileName = selectedPath.filename().string(); 
            free(filePath); 
        } else{
            displayedFileName = "No file selected";

        }
    }


    ImGui::SameLine(0.0f, spacing);

    ImGui::TextWrapped(displayedFileName.c_str());

    ImGui::Spacing(); 




    // Start/Stop Button
    if(currentState!=STARTED && currentState!=IS_STOPPING){
        if (ImGui::Button( currentState!=IS_STARTING ? "Start" : "Starting...", ImVec2(buttonWidth, buttonHeight))) {
            previousState = currentState;
            isTimerActive = true;
            timerStart = std::chrono::steady_clock::now();
                currentState=IS_STARTING;
                tod_status_msgs::msg::ManagerButtonStatus buttonStatusMsg;
                _inputDevice.publish_input_device(selectedFile.c_str());
                buttonStatusMsg.clicked_button = tod_status_msgs::msg::ManagerButtonStatus::START_BUTTON;
                buttonStatusMsg.vehicle_control_mode= selectedConcept;
                _buttonStatus.publish_button_status(buttonStatusMsg);
               


        }
         if (currentState==DISCONNECTED||currentState==IS_CONNECTING) ImGui::EndDisabled();
     }else{
         if (currentState==STARTED||currentState==IS_STOPPING) ImGui::EndDisabled();
        if (ImGui::Button(currentState!=IS_STOPPING ? "Stop" : "Stopping..", ImVec2(buttonWidth, buttonHeight))) {
           
            previousState = currentState;
            isTimerActive = true;
            timerStart = std::chrono::steady_clock::now();
            currentState=IS_STOPPING;
            tod_status_msgs::msg::ManagerButtonStatus buttonStatusMsg;
            buttonStatusMsg.clicked_button = tod_status_msgs::msg::ManagerButtonStatus::STOP_BUTTON;
            buttonStatusMsg.vehicle_control_mode= selectedConcept;
            _buttonStatus.publish_button_status(buttonStatusMsg);
            }
           
        }
        




// NOT FOR OPEN-SOURCE
// std::string imagePath = "TUM_Logo_blau_rgb_p.png";
// GLuint iconViewId = ImGuiLayer::load_texture(
//     (tod_gl::RosInterface::get_package_path() + "/resources/icons/" + imagePath).c_str());
// 
// 
// ImVec2 originalSize(740, 390);
// 
// 
// float scaleFactor = 0.1f;
// ImVec2 scaledSize(originalSize.x * scaleFactor, originalSize.y * scaleFactor);
// 
// 
// ImVec2 layerSize = ImGui::GetWindowSize();
// ImVec2 layerPos = ImGui::GetWindowPos();
// 
// 
// float xPosition = layerPos.x+10; 
// float yPosition = layerPos.y + layerSize.y - scaledSize.y; 
// 
// 
// ImGui::SetCursorScreenPos(ImVec2(xPosition, yPosition));
// 
// ImGui::Image((void *)(intptr_t)iconViewId, scaledSize);
ImGui::End();
ImGui::End();

}

void OperatorStateLayer::on_event(tod_gl::Event& e) {
    (void)e;
}

void OperatorStateLayer::on_update(float ts) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    if (isTimerActive) {
        auto now = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(now - timerStart).count();

        if (elapsedTime >= timeoutDuration) {
            RCLCPP_WARN(_ros->get_logger(), "Transition timed out. Reverting to previous state: %d", previousState);
            currentState = previousState;
            isTimerActive = false;
        }
    }
}

char* OperatorStateLayer::browseFile() {
     std::string package_share_directory = ament_index_cpp::get_package_share_directory("tod_input_devices");
    std::filesystem::path configPath = package_share_directory + "/config/fanatec.yaml";


    if (!std::filesystem::exists(configPath)) {
        std::cerr << "Error: Directory does not exist: " << configPath << std::endl;
        return nullptr; 
    }


    const char* filePath = tinyfd_openFileDialog(
        "Select Configuration File",  
        configPath.string().c_str(),  
        0,                            
        nullptr,                      
        nullptr,                    
        0                            
    );

    if (filePath) {
        std::cout << "Selected file: " << filePath << std::endl;
        return strdup(filePath);
    } else {
        std::cout << "No file selected." << std::endl;
        return nullptr; 
    }
}
void OperatorStateLayer::readAndStoreOwnIpAddresses() {
    struct ifaddrs* ifAddrStruct = nullptr;    
    getifaddrs(&ifAddrStruct);

    for (struct ifaddrs* ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;

        if (ifa->ifa_addr->sa_family == AF_INET) { // IPv4
            struct sockaddr_in* sa = (struct sockaddr_in*)ifa->ifa_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(sa->sin_addr), addressBuffer, INET_ADDRSTRLEN);
            ipOperatorOptions.push_back(addressBuffer);
        }
    }

    if (ifAddrStruct != nullptr) {
        freeifaddrs(ifAddrStruct);
    }

    std::sort(ipOperatorOptions.begin(), ipOperatorOptions.end(), [](const std::string& a, const std::string& b) {
        auto getPriority = [](const std::string& ip) {
            if (ip.find(PRIO1) == 0) return 1;
            if (ip.find(PRIO2) == 0) return 2;
            if (ip.find(PRIO3) == 0) return 3;
            if (ip.find(PRIO4) == 0) return 4;
            if (ip.find(PRIO5) == 0) return 5;
            return 6; 
        };

        int priorityA = getPriority(a);
        int priorityB = getPriority(b);

        if (priorityA == priorityB) {
            return a < b; 
        }
        return priorityA < priorityB; 
    });
}

void OperatorStateLayer::UpdateState() {
    int todStatus = _status.get_tod_status();

    switch (currentState) {
        case IS_CONNECTING:
            if (todStatus == 1) {
                RCLCPP_INFO(_ros->get_logger(), "Transition to CONNECTED");
                currentState = CONNECTED;
                isTimerActive = false;
                _networkMonitor.SetMonitorStatus(ipOperatorOptions[selectedOperatorIP],true);
            }
            break;

        case IS_STARTING:
            if (todStatus == 2) {
                RCLCPP_INFO(_ros->get_logger(), "Transition to STARTED");
                currentState = STARTED;
                isTimerActive = false;
            }
            break;

        case IS_STOPPING:
            if (todStatus == 1) {
                RCLCPP_INFO(_ros->get_logger(), "Transition to CONNECTED");
                currentState = CONNECTED;
                isTimerActive = false;
            }
            break;

        case IS_DISCONNECTING:
            if (todStatus == 0) {
                RCLCPP_INFO(_ros->get_logger(), "Transition to DISCONNECTED");
                currentState = DISCONNECTED;
                _networkMonitor.SetMonitorStatus(ipOperatorOptions[selectedOperatorIP],false);
                isTimerActive = false;
            }
            break;

        default:
            break;
    }
            
    
}


/**
 * @file rtsp_server.cpp
 * @brief This file defines the `RtspServer` class for managing the setup and operation of an RTSP server in a ROS2 environment. The `RtspServer` class is responsible for initializing camera streams, handling RTSP video pipelines, and managing camera parameters. It also provides functionality for reconfiguring video settings dynamically and publishing video data streams.
 * 
 * The `RtspServer` class integrates with ROS2 to receive image data, manage video configurations, and send video streams to connected clients over RTSP. It supports dynamic reconfiguration of video parameters and camera settings, and provides mechanisms for setting bitrate, cropping, and scaling video streams. The class uses GStreamer for video pipeline management and subscription to camera data topics.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */
#include "rtsp_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace tod_rtsp;

RtspServer::RtspServer() : Node("VehicleRtspServer"){
    //declare parameters
    this->declare_parameter<int>("rtsp_port", 8554); // < parameter to set the path to the stream settings
    this->declare_parameter<std::string>("camera_params_path", std::string(ament_index_cpp::get_package_share_directory("tod_rtsp")) + "/config"); // < parameter to set the path to the router settings
    this->declare_parameter<std::string>("vehicleID", "edgar");
    this->declare_parameter<std::string>("router_settings_path", std::string(ament_index_cpp::get_package_share_directory("tod_rtsp")) + "/config/package_config/tod_rtsp"); // < parameter to set the path to the router settings
    this->declare_parameter<std::string>("stream_settings_path", std::string(ament_index_cpp::get_package_share_directory("tod_rtsp")) + "/config/package_config/tod_rtsp"); // < parameter to set the path to the stream settings
    this->declare_parameter<std::string>("image_output_format", "RGB");
    this->declare_parameter<double>("inactivity_timeout", 3.0);

    this->inactivity_timeout_ = this->get_parameter("inactivity_timeout").as_double();
    this->port_ = "" + std::to_string(this->get_parameter("rtsp_port").as_int()); // later on only string is necessary, however we want to make shure port is a int
    
    //  load camera parameters, the vehicle_id is not updated dynamically at runtime because the vehicle should not change at runtime
    std::string camera_param_path = this->get_parameter("camera_params_path").as_string() + "/vehicle_config/"; 
    //         + "/" + this->get_parameter("vehicle_id").as_string()
    //         + "/sensors-camera.yaml";
    std::cout << "camera params path:" << camera_param_path << " | vehicle id: " << this->get_parameter("vehicleID").as_string() << std::endl;
    cam_params_ = std::make_unique<tod_core::param_set::Camera>(this, camera_param_path);
    cam_params_->load_parameters();

    // load stream settings for h264
    std::string stream_settings_path = this->get_parameter("stream_settings_path").as_string()  + "/";
    stream_settings_ = std::make_unique<tod_core::param_set::Video>(this, stream_settings_path, "h264");
    stream_settings_->load_parameters();

    // load ip config file (router_settings.yml)
    std::string router_settings_path = this->get_parameter("router_settings_path").as_string() + "/router_settings.yml";
    YAML::Node router_settings = YAML::LoadFile(router_settings_path);

    for (const auto &ip : router_settings["ips"]) {
        _ips.push_back(ip.as<std::string>());
    }
    if (_ips.empty()) {
        _ips.push_back("0.0.0.0");
    }
    for (const auto &ip : _ips) {// printing the ip adresses
        RCLCPP_INFO(RtspServer::get_logger(), "Loaded IP: %s", ip.c_str());
    }

    std::string topicNamespace = cam_params_->get_camera_topics_namespace();
    std::string imageName = cam_params_->get_camera_image_name();

    this->declare_parameter<int>("bitrate_sum", stream_settings_->get_bitrate());
    this->bitrate_sum_ = this->get_parameter("bitrate_sum").as_int();
    size_t numCams = cam_params_->get_sensors().size();
    this->base_bitrate_ = this->bitrate_sum_/ numCams;

    load_cam_params_init_streams(topicNamespace, imageName);
    
    subs_status_ = this->create_subscription<tod_status_msgs::msg::Status>("input/vehicle_status", 5, std::bind(&RtspServer::callback_status, this, std::placeholders::_1));
    reconfig_service_ = this->create_service<tod_config_msgs::srv::VideoConfig>("VehicleRtspServer/set_video_config", std::bind(&RtspServer::callback_video_reconfig, this, std::placeholders::_1, std::placeholders::_2));
    // initialize the GStreamer library
    gst_init(nullptr, nullptr);
}

bool RtspServer::set_bitrate_for_streams(int bitrate_sum) {
    int new_bitrate = bitrate_sum / static_cast<int>(cameras_.size());
    bool success = true;
    for(auto cam : cameras_){
        for (auto stream : cam->_streams) {
            stream->set_bitrate(new_bitrate);
        }
    }
    return success;
}

bool RtspServer::load_cam_params_init_streams(const std::string &topicNamespace, const std::string &imageName) {
    RCLCPP_INFO(this->get_logger(), "load_cam_params_init_streams");
    for (const auto &cam : cam_params_->get_sensors()) {
        CameraObject* camObj = new CameraObject();
        camObj->camera = cam.name;
        cameras_.push_back(std::shared_ptr<CameraObject>(camObj));
        for (size_t i = 0; i < _ips.size(); ++i) {
            try {
                // setting new video config based on the videoParams
                videoConfig new_config;                
                new_config.height = stream_settings_->get_height();
                new_config.scaling_factor = stream_settings_->get_scaling_factor();
                new_config.width = stream_settings_->get_width();
                new_config.offset_width = stream_settings_->get_offset_width();
                new_config.offset_height = stream_settings_->get_offset_height();

                auto stream = camObj->_streams.emplace_back(std::make_shared<RtspStream>(
                    camObj->camera, 
                    _ips[i],
                    this->port_,
                    this->base_bitrate_,
                    new_config,
                    this->inactivity_timeout_,
                    std::make_shared<rclcpp::Logger>(RtspServer::get_logger())));
                stream->update_activity(!cam.stream_on_connect);
                std::string topic{topicNamespace + cam.name + imageName};
                RCLCPP_INFO(this->get_logger(), "Topic: %s", topic.c_str());
                stream->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(topic, 1, 
                    std::bind(&RtspStream::ros2_image_callback, stream, std::placeholders::_1)
                );            
            }catch (const std::exception &e) {
                //std::cerr <<  "Error in RtspServer: parameter " << e.what() << std::endl;
                RCLCPP_ERROR(RtspServer::get_logger(), "Error in RtspServer: parameter %s", e.what());
                return false;
            }
        }
    }
    return true;
}

void RtspServer::run(){
    RCLCPP_INFO(RtspServer::get_logger(), "Starting RtspServer");
    // create the server object
    GstRTSPServer *gstServer = gst_rtsp_server_new();
    // attach the server to the default maincontext
    gst_rtsp_server_attach(gstServer, nullptr);
    // the mount point manages the mapping from a request url to a specific stream and its configuration
    GstRTSPMountPoints *gstMounts = gst_rtsp_server_get_mount_points(gstServer);
    // create the main loop
    GMainLoop *gstLoop = g_main_loop_new(nullptr, FALSE);
    std::thread gstThread([gstLoop](){ g_main_loop_run(gstLoop); });
    rclcpp::Rate r(1000);
    static bool operatorConnectedPrevious{false};
    while (rclcpp::ok())
    {
        r.sleep();
        rclcpp::spin_some(this->shared_from_this());       
        for(auto cam : cameras_){
            for (auto stream : cam->_streams) {
                stream->refresh(gstMounts);
        }}
        if (operatorConnectedPrevious && !_operatorConnected)
        {
            for(auto cam : cameras_){
                for (auto stream : cam->_streams) {
                    stream->set_bitrate(this->base_bitrate_);
                    stream->reset();
                }
            }}
        operatorConnectedPrevious = _operatorConnected;
    }
    g_main_loop_quit(gstLoop);
    gstThread.join();
    g_object_unref(gstMounts);
    g_object_unref(gstServer);
    g_main_loop_unref(gstLoop);
}

void RtspServer::callback_status(const tod_status_msgs::msg::Status::ConstPtr &msg){
    _operatorConnected = (msg->tod_status != tod_status_msgs::msg::Status::TOD_STATUS_IDLE);
}


void RtspServer::callback_video_reconfig(const std::shared_ptr<tod_config_msgs::srv::VideoConfig::Request> request,
                                          std::shared_ptr<tod_config_msgs::srv::VideoConfig::Response> response){
    bool success = true;
    int total_cams_found = 0;
    response->empty = 1;  // indicate that response won't be empty
    std::string message{""};
    for (auto cam : cameras_){
        for (auto stream : cam->_streams){
            videoConfig new_config;
            new_config.scaling_factor = request->scaling_factor;
            new_config.width = request->width;
            new_config.height = request->height;
            new_config.offset_width = request->offset_width;
            new_config.offset_height = request->offset_height;

            if (stream->is_stream(request->camera_name)){
                total_cams_found++;
                bool resize_success = stream->update_config(new_config);
                if (resize_success) RCLCPP_INFO(RtspServer::get_logger(), "Cropping and Scaling run successfully");
                else RCLCPP_INFO(RtspServer::get_logger(), "Cropping and Scaling did not run successfully");

                bool bitrate_success = stream->set_bitrate(request->bitrate);
                if (bitrate_success) RCLCPP_INFO(RtspServer::get_logger(), "Failed to set bitate for %s", request->camera_name.c_str());
                else RCLCPP_INFO(RtspServer::get_logger(), "Set bitrate successfully %s", message.c_str());

                bool active_success = stream->update_activity(request->paused);
                std::string scaling_factor_str = request->scaling_factor;


                // configure message output: TODO: add ip address to the message#
                message += resize_success ?
                    " Reconfigured " + request->camera_name + " with scaling " + scaling_factor_str + " and resolution " + std::to_string(request->width) + "x" + std::to_string(request->height) 
                    : " Failed to reconfigure " + request->camera_name + " with scaling " + scaling_factor_str + " and resolution " + std::to_string(request->width) + "x" + std::to_string(request->height);
                message += bitrate_success ? 
                    "Set bitrate = " + std::to_string(request->bitrate) + " for " + request->camera_name
                    : "Failed to set bitrate for " + request->camera_name;
                std::string paused = request->paused ? "Paused" : "Unpaused";
                message += paused + " stream " + request->camera_name;

                RCLCPP_INFO(RtspServer::get_logger(), "%s stream %s successfully ", paused.c_str(),request->camera_name.c_str());
                // evaluating general success. logic is: for one stream all configs must be set successfully. Also all streams must be set successfully
                bool local_success = resize_success && bitrate_success & active_success;
                success = success && local_success;                    
            }
    }}
    response->message = message; 
    RCLCPP_INFO(RtspServer::get_logger(),"callback  reconfig response %s", message.c_str());
    return;

}


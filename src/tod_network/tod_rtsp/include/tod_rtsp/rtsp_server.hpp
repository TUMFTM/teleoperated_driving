/**
 * @file rtsp_server.hpp
 * @brief This file defines the `RtspServer` class for managing the setup and operation of an RTSP server in a ROS2 environment. The `RtspServer` class is responsible for initializing camera streams, handling RTSP video pipelines, and managing camera parameters. It also provides functionality for reconfiguring video settings dynamically and publishing video data streams.
 * 
 * The `RtspServer` class integrates with ROS2 to receive image data, manage video configurations, and send video streams to connected clients over RTSP. It supports dynamic reconfiguration of video parameters and camera settings, and provides mechanisms for setting bitrate, cropping, and scaling video streams. The class uses GStreamer for video pipeline management and subscription to camera data topics.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */
#pragma once
#include "rtsp_stream.hpp" 

// general includes
#include <mutex>
#include <algorithm>
#include <memory>
#include <string>
#include <thread>
#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>


//gstreamer includes
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtp/gstrtpbuffer.h>

// #include <dynamic_reconfigure/server.h>
// dynamic reconfigure
// rclcpp includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/parameter_events_filter.hpp> 
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter_events_filter.hpp>


#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

//tod includes
#include <tod_config_msgs/srv/video_config.hpp>
#include <tod_status_msgs/msg/status.hpp>
#include <tod_vehicle_msgs/msg/primary_vehicle_state.hpp>
#include <tod_core/param_set/CameraParameters.hpp>
#include <tod_core/param_set/VideoParameters.hpp>
// #include <tod_vehicle_msgs/include/VehicleEnums.h> TODO: include the correct file

namespace tod_rtsp {


struct CameraObject {
    std::string camera;
    std::vector<std::shared_ptr<RtspStream>> _streams;
};  

class RtspServer : public rclcpp::Node
{
    // struct CameraObject;
    // struct RtspStream;
public:
    explicit RtspServer();
    ~RtspServer() {}
    void run();


    static rclcpp::Logger get_logger() {

        static auto logger = rclcpp::get_logger("RtspServer");
        return logger;

    }
private:
    //ros2 parameters
    std::vector<std::string> _ips; //all vehicle IPs
    std::string port_{"8554"};
    
    //class attributes
    std::string _nodeName{""};
    float inactivity_timeout_{3.0};
    int base_bitrate_{100};
    int bitrate_sum_;
    std::string vehicleID_{"edgar"};
    bool _operatorConnected{false};
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr _auto_param_callback_handle; // parameter callback for what parameter?
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr subs_status_; // to check if connected? (isn't the stream always ready?)
    rclcpp::Service<tod_config_msgs::srv::VideoConfig>::SharedPtr reconfig_service_; 
    std::unique_ptr<tod_core::param_set::Camera> cam_params_;
    std::unique_ptr<tod_core::param_set::Video> stream_settings_; // are they currently used?
    std::vector<std::shared_ptr<CameraObject>> cameras_; // these are the pipelines

    //private functions
    
    // loads camera parameters and subscribes to camera topic 
    bool load_cam_params_init_streams(const std::string &topicNamespace, const std::string &imageName);
    bool set_cropping_and_scaling(const std::shared_ptr<tod_config_msgs::srv::VideoConfig::Request> request, std::shared_ptr<RtspStream> stream2reconfigure);
    bool set_bitrate_for_stream(int bitrate, std::shared_ptr<RtspStream> stream2reconfigure);
    bool reset_settings();

    void factory_gst_video_pipeline(std::shared_ptr<RtspStream> stream, GstRTSPMountPoints *_gstMounts);
    void push_data(std::shared_ptr<RtspStream> stream);
    void callback_raw_image(const sensor_msgs::msg::Image::ConstPtr &msg, std::shared_ptr<RtspStream> stream);
    void callback_status(const tod_status_msgs::msg::Status::ConstPtr &msg);
    
    void callback_video_reconfig(const std::shared_ptr<tod_config_msgs::srv::VideoConfig::Request> request,
                                  std::shared_ptr<tod_config_msgs::srv::VideoConfig::Response> response); // service callback TODO: replace with messages
    bool set_bitrate_for_streams(int bitrateSum);
};     
} // namespace tod_rtsp

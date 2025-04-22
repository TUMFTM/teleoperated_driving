/**
 * @file rtsp_stream.hpp
 * @brief This file defines the `RtspStream` class for managing video streaming over RTSP using GStreamer. The class provides functionality to set up an RTSP stream, handle video configurations, and manage video encoding, including bitrate adjustments, cropping, and scaling. It also supports receiving image data via ROS2 messages and dynamically reconfiguring stream settings.
 * 
 * The `RtspStream` class integrates with ROS2 to receive image data, configure GStreamer pipelines, and publish the video stream over RTSP. The class supports dynamic reconfiguration of video settings, including bitrate, resolution, and cropping, and includes mechanisms for managing video data through callbacks and maintaining stream activity. It ensures that stream settings are applied properly and provides methods to check and update stream configurations.
 * 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */
 #pragma once
#include <string>
#include <map>
#include <mutex>
#include <atomic>
#include <algorithm>

//ros2 includes
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>


//GSTREAMER INCLUDE
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtp/gstrtpbuffer.h>


namespace tod_rtsp{

struct PipelineConfig {
    std::string encoderType;
    std::map<std::string, std::string> encoderSettingsMaps;
    std::string videoConvertSettings;
    // std::string colorSpaceEncoding{"i420"};
    int bitrate;   
};

// struct BandwidthSetting {
//     std::vector<int> transitionBitrates;
//     std::vector<std::string> scalings;
//     int bitrateDemand{0};
//     int bitrateAllocated{0};
//     int optimalResolutionIndex{-1};
// };


struct videoConfig{
    int offset_width;
    int offset_height;
    int width;
    int height;
    int raw_step{-1};
    std::string scaling_factor;
}; 


class RtspStream {

public:
    RtspStream(const std::string name, std::string ip, std::string port, int bitrate, videoConfig video_config, const float inactivity_timeout, std::shared_ptr<rclcpp::Logger> logger);

    //necessary for constructing stream
    void factory_gst_video_pipeline(GstRTSPMountPoints* _gstMounts);

    void refresh(GstRTSPMountPoints *gstMounts);
    void reset();

    void ros2_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg); //TODO: move from rtsp_stream

    bool set_bitrate(int bitrate);
    int get_bitrate();
    bool is_stream(std::string camera_name);
    bool update_config(videoConfig config);
    bool update_activity(bool paused);
        
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;


private:
    std::shared_ptr<videoConfig>  video_config_;
    PipelineConfig pipeline_config_; //former gstSettings
    std::shared_ptr<sensor_msgs::msg::Image> latest_image_;
    std::shared_ptr<rclcpp::Logger> logger_;
    std::mutex mutex_;
    const std::string name_;
    const std::string ip_;
    const std::string port_;
    const float inactivity_timeout_;

    std::atomic<bool> gst_data_request_; ///< flag indicates if need-data has been called by gstreamer, atomic for multi thread safety
    std::atomic<bool> ros2_new_data_;
    std::atomic<bool> is_active_;
    std::chrono::time_point<std::chrono::system_clock> gst_last_request_;

    void gst_media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media);
    static void static_gst_media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, RtspStream *stream);
    void gst_need_data(GstElement *appSrc, guint unused);
    //forwards the call to this class
    static void static_gst_need_data(GstElement* appSrc, guint unused, RtspStream* stream);
    void push_data();
    static void static_push_data(RtspStream *stream);
    bool is_inactive();
    //Method for adding the timestamp of the ROS message image to the RTP header
    static GstPadProbeReturn add_rtp_timestamp_probe(GstPad *pad, GstPadProbeInfo *info, gpointer timestamp);
    
    //color_encoding_lookup
    std::string get_gst_encoding(std::string image_encoding);

    const std::unordered_map<std::string, std::string> color_encoding_map_ = {
        {sensor_msgs::image_encodings::MONO16, "UYVY"},
        {sensor_msgs::image_encodings::YUV422, "UYVY"},
        {sensor_msgs::image_encodings::BGR8, "BGR"},
        {sensor_msgs::image_encodings::BGRA8, "BGRA"},
        {sensor_msgs::image_encodings::MONO8, "GRAY8"},
        {sensor_msgs::image_encodings::RGB8, "RGB"}
    };

    //gstreamer objects:
    GstRTSPMediaFactory *factory{nullptr}; // TODO: if i understand it correctly this entry only exists to check if it exists and then to build it in a function. But after the function, it is not used again? Maybe to not loose scope?
    GstElement *appsrc_{nullptr};
    GstElement *encoder_{nullptr};
    GstElement *videoconvert_{nullptr};
    GstElement *videocrop_{nullptr};
    GstElement *scalingFilter_{nullptr};


// HERE OLD CODE STARTS ____________________________________________________--

    
    // BandwidthSetting bandwidthSettings;
    // bool newDataAvailable{false}; //for what?
    // // bool needData{false}; // for what?
    // // bool is_front_facing{false}; // for dynamic reconfiguration?
    // // bool isActive{false}; // for what?
    // bool driving_bitrate_updated{false}; // for what?
    // bool last_driving_forward{false}; // for what?
    //  lastNeedDataStamp = std::chrono::system_clock::now(); // ToDo Flo
    // int rawHeight{-1}, rawWidth{-1}, rawStep{-1};
    // //std::string encoding{""};
    // std::vector<uint8_t> imgData;
    // std::string name{""};
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subsImg;
    // //rclcpp::Subscription<tod_msgs::msg::Status>::SharedPtr subsStatus; // for what? not even initalized

    // std::mutex mutex;
    // // std::shared_ptr<videoConfig> currentConfig; TODO: added above
    // PipelineConfig gstSettings;
    // bool paused;
    // // rclcpp::Time lastTimestamp; //Timestamp of last sent Image/Buffer
    // std::string ip; //IP of the router to publish

    // RtspStream(const std::string &myName, const int defaultBitrateForStream ): name(myName),
    // currentConfig(std::make_shared<videoConfig>())
    // {
    //     gstSettings.bitrate = defaultBitrateForStream;
    //     currentConfig->offset_width = 0; 
    //     currentConfig->offset_height = 0;
    //     currentConfig->scaling_factor = "1p000";

    // }

    // void reset(const int defaultBitrateForStream)
    // {
    //     gstSettings.bitrate = defaultBitrateForStream;
    //     rawWidth = currentConfig->width;
    //     rawHeight = currentConfig->height;
    //     rawStep = currentConfig->raw_step;
    //     currentConfig->offset_width = 0;
    //     currentConfig->offset_height = 0;
    //     currentConfig->scaling_factor = "1p000";
    //     needData = false;
    //     newDataAvailable = false;
    // } 
};
} // namespace tod_rtsp
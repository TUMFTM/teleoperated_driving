/**
 * @file rtsp_clients.hpp
 * @author Nils Gehrke
 * @brief This file contains the implementation for managing network connections related to RTSP streams. It configures and manages multiple camera streams, handles communication with the operator, and interfaces with ROS2 to publish image data. The `RtspClients` class handles the setup of the RTSP server, manages client connections, and continuously monitors and updates video stream parameters such as bitrate and framerate.
 * 
 * The class is designed to dynamically manage RTSP streams, adjusting parameters such as image output format, bitrate, and resolution based on real-time configuration changes. The client connects to the server, monitors video stream performance, and handles any necessary adjustments in real-time. It also integrates with ROS2 to publish information related to the video streams.
 * 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */


#pragma once
#include <string>
#include <thread>
#include <memory>
#include <vector>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/rtp/gstrtpbuffer.h>

#include "tod_status_msgs/msg/status.hpp"
#include "tod_network_msgs/msg/paket_info.hpp"
#include "tod_vehicle_msgs/msg/video_info.hpp"
#include "tod_core/param_set/CameraParameters.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace tod_rtsp {

    struct RtspStream;


    struct CameraObject {
        std::string camera;
        rclcpp::Time timestamp;
        std::mutex timestampMutex;
        std::vector<std::shared_ptr<RtspStream>> _streams;
    };


    struct RtspStream 
    {
            std::string name;
            std::string imageOutputFormat;
            rclcpp::Time lastTimestamp;
            std::string ip;
            CameraObject* camera;

            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImage;
            rclcpp::Publisher<tod_vehicle_msgs::msg::VideoInfo>::SharedPtr pubVideoInfo;
            rclcpp::Publisher<tod_network_msgs::msg::PaketInfo>::SharedPtr pubPaketInfo;

            std::mutex mutex;
            GstElement *pipeline{nullptr};
            GstRTPBuffer rtpPaket = GST_RTP_BUFFER_INIT;
            int rtpPaketCount{0}, frameCount{0}, pktSizeSum_bit{0};
            int bitrate_kbit{0}, framerate{0}, imgHeight_px{0}, imgWidth_px{0};
            std::chrono::time_point<std::chrono::system_clock>  lastVideoInfoCalc;

            
            
            bool isJpeg{false};

            RtspStream( CameraObject* camera, bool jpg,std::string streamNumber,  const std::string &outputFormat) :
                name{camera->camera},
                imageOutputFormat{outputFormat},
                isJpeg{jpg} {
                // support for camera names with dots: ros topic names with 'DOT', uris with '.'
                std::string str2find = "DOT";
                std::size_t found;
                this->camera=camera;
                while ((found = name.find(str2find)) != std::string::npos) {
                    name.replace(found, str2find.length(), ".");
                }
            }


            // Methode zum Abrufen des Loggers
            rclcpp::Logger get_logger() const {
                return rclcpp::get_logger(ip);
            }
    };

    class RtspClients : public rclcpp::Node
    {
        public:
            /**
            * @brief Constructor.
            *
            * This function constructs the RtspClients according to the specified parameter files via the node parameters
            *
            */
            RtspClients();
            /**
            * @brief starts the rtsp client and waits for connect and disconnect events.
            *
            * This function is running continously until the ros2 node is terminated. It triggers the connectVideoClient and disconnectVideoClient calls.
            * In case of connect or disconnect, the funtion loops over all streams and triggers the connect individually
            *
            */
            void run();
            /**
            * @brief returns static rclcpp logger.
            *
            */
            static rclcpp::Logger get_logger() {
                static auto logger = rclcpp::get_logger("RtspClients");
                return logger;
            }

            /**
            * @brief Handles GStreamer messages for the RTSP stream and checks if the stream ended.
            *
            * This function processes messages from the GStreamer bus associated with the RTSP stream.
            * Currently, it handles End-of-Stream (EOS) messages and logs a warning when the stream ends.
            *
            * @param bus Pointer to the GStreamer bus receiving the message.
            * @param message Pointer to the GStreamer message to be handled.
            * @param stream Pointer to the RtspStream object associated with the message.
            * @return gboolean TRUE to indicate that the message was handled.
            */
            static gboolean on_gst_message(GstBus *bus, GstMessage *message, RtspStream *stream);
        private:
            // Subscriber
            rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _status_sub; // <subscription of the status message to recieve streams on connect
            // Vehicle IPs
            std::vector<std::string> _ips; 
            // Params
            std::unique_ptr<tod_core::param_set::Camera> _camera_param_handler;
            std::string router_config_path_;
            std::string camera_param_path_;
            std::string image_output_format_{"yuv422"};
            std::string rtsp_port_;

            // Variables
            std::string _nn{""};
            std::vector<std::shared_ptr<CameraObject>> _cameras;
            int _latency{500};
            bool _connected{false};
            std::map<std::string, std::string> _color_format = {
                {"i420", "I420"},
                {sensor_msgs::image_encodings::RGB8, "RGB"},
                {sensor_msgs::image_encodings::YUV422, "UYVY"},
                {sensor_msgs::image_encodings::BGR8, "BGR"},
                {sensor_msgs::image_encodings::BGRA8, "BGRA"},
                {sensor_msgs::image_encodings::MONO8, "GRAY8"}
            };

            /**
            * @brief Handles new status messages.
            *
            * This function processes the incoming status messages. Based on a connect or disconnect, it constructs or 
            * deconstructs the gstreamer pipelines for receiving the video streams
            *
            * @param msg new status message
            */
            void callback_status_msg(const tod_status_msgs::msg::Status &msg); 


            // Class functions
            /**
            * @brief Handles connection with vehicle and starts the stream receiver for the specified stream
            *
            * Constructs the gstreamer pipeline for receiving the stream at the ip adress and the data given in the stream objekt.
            * Registers the callbacks newRtpPacket and newImageSample
            *
            * @param stream stream object pointer
            * @param vehicleIp ip adress of vehicle
            */
            void connect_video_client(std::shared_ptr<RtspStream> stream, const std::string &vehicleIp);
            /**
            * @brief Handles disconnect with vehicle and shuts down the stream receiver for the specified stream
            *
            * shuts down and unreferences the gstreamer pipeline when disconnecting
            *
            * @param stream stream object pointer to shut down
            */
            void disconnect_video_client(std::shared_ptr<RtspStream> stream);
            /**
            * @brief Handles new received rtp packet from vehicle to publish the metadata
            *
            * constructs and publishes the PaketInfo message based on the received rtp packet
            *
            * @param identity identity of the gstreamer element, not used but necessary for callback
            * @param buffer data buffer containing the rtp message
            * @param stream stream object pointer to publish on the right topic and evaluate rtp package actuality
            */
            static void new_rtp_packet(GstElement *identity, GstBuffer *buffer, RtspStream *stream);
            /**
            * @brief Handles new received image from the rtsp stream
            *
            * constructs and publishes Image message if new image is received. 
            * Evaluates the actuality of the provided image via a timestamp.
            *
            * @param appSink sink to pull the image from
            * @param stream stream object with the image publisher and the last timestamp
            */
            static void new_image_sample(GstAppSink* appSink, RtspStream *stream);
    };
} //namespace tod_rtsp
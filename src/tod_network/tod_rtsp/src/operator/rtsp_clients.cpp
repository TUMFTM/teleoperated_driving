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

#include "rtsp_clients.hpp"

namespace tod_rtsp{

    using std::placeholders::_1;
    using namespace std::chrono_literals;

    RtspClients::RtspClients() : Node("OperatorRtspClients")
    {
        this->declare_parameter<int>("rtsp_port", 8554); // < parameter to set the path to the stream settings
        this->declare_parameter<std::string>("router_config_path", std::string(ament_index_cpp::get_package_share_directory("tod_rtsp")) + "/config/package_config/tod_rtsp"); // < parameter to set the path to the router settings
        this->declare_parameter<std::string>("camera_params_path", std::string(ament_index_cpp::get_package_share_directory("tod_rtsp")) + "/config"); // < parameter to set the path to the stream settings
        this->declare_parameter<std::string>("image_output_format", "rgb8");    
        this->declare_parameter<std::string>("vehicleID", "edgar");
        
        // load the ip config file and push back the ips
        this->router_config_path_ = this->get_parameter("router_config_path").as_string();
        std::string config_file_path = this->router_config_path_ + "/router_settings.yml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        for (const auto &ip : config["ips"]) {
            _ips.push_back(ip.as<std::string>());
        }
        for (const auto &ip : _ips) { // printing IP Adresses in the logger
            RCLCPP_INFO(RtspClients::get_logger(), "Loaded IP: %s", ip.c_str());
        }
        // loading the parameters for the camera streams
        this->camera_param_path_ = this->get_parameter("camera_params_path").as_string()  + "/vehicle_config/";
        _camera_param_handler = std::make_unique<tod_core::param_set::Camera>(this, this->camera_param_path_);
        _camera_param_handler->load_parameters();
        // setting output format
        this->image_output_format_ = this->get_parameter("image_output_format").as_string();
        RCLCPP_INFO_STREAM(this->get_logger(), image_output_format_);
        // setting rtsp_port
        this->rtsp_port_ = std::to_string(this->get_parameter("rtsp_port").as_int());

        //Subscriber of the status message to start requesting and receiving streams when connection has been established
        _status_sub = this->create_subscription<tod_status_msgs::msg::Status>(
            "input/operator_status",
            1,
            std::bind(&RtspClients::callback_status_msg, this, _1));

        // Variables
        _nn = this->get_name(); // node name

        // loading the stream parameters
        for (const auto &cam : _camera_param_handler->get_sensors()) 
        {
            CameraObject* camObj = new CameraObject();
            camObj->camera = cam.name;
            camObj->timestamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
            _cameras.push_back(std::shared_ptr<CameraObject>(camObj));
            for (size_t i = 0; i < _ips.size()+1; ++i) {
                camObj->_streams.emplace_back(std::make_shared<RtspStream>(camObj, cam.is_jpeg, std::to_string(i),image_output_format_));
                auto stream = camObj->_streams.back();
                std::string ns(&cam.name.at(1), cam.name.size()-1); // name without '/'
                stream->pubImage = this->create_publisher<sensor_msgs::msg::Image>(ns + "/image",  rclcpp::SensorDataQoS());
                stream->pubVideoInfo = this->create_publisher<tod_vehicle_msgs::msg::VideoInfo>(ns + "/video_info", 5); 
                stream->pubPaketInfo = this->create_publisher<tod_network_msgs::msg::PaketInfo>(ns + "/paket_info", 100);
            }
        }
    }

    void RtspClients::run() 
    {
        GMainLoop* gstLoop = g_main_loop_new(nullptr, FALSE);
        std::thread gstThread([gstLoop]() {
            g_main_loop_run(gstLoop); // blocking
        });

        rclcpp::Rate r(10);
        while (rclcpp::ok()) {
            
            rclcpp::spin_some(this->get_node_base_interface());
            for(auto cam : _cameras){
                for (auto stream : cam->_streams) {
                    if (std::chrono::system_clock::now() >= stream->lastVideoInfoCalc + std::chrono::duration<double>(1.0)) {
                        // calculate and set current video info
                        std::lock_guard<std::mutex> lock(stream->mutex);
                        stream->bitrate_kbit = stream->pktSizeSum_bit / 1024;
                        stream->framerate = stream->frameCount;
                        stream->rtpPaketCount = stream->pktSizeSum_bit = stream->frameCount = 0;
                        stream->lastVideoInfoCalc = std::chrono::system_clock::now();
                        stream->lastTimestamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
                    }
                    // publish video info
                    tod_vehicle_msgs::msg::VideoInfo msg;
                    msg.header.stamp = this->now();
                    msg.header.frame_id = stream->name;
                    msg.kbitrate = uint32_t(stream->bitrate_kbit);
                    msg.framerate = uint32_t(stream->framerate);
                    msg.image_height = uint32_t(stream->imgHeight_px);
                    msg.image_width = uint32_t(stream->imgWidth_px);
                    msg.image_nof_pixel = uint32_t(stream->imgHeight_px * stream->imgWidth_px);

                    stream->pubVideoInfo->publish(msg);
                }
            }
            r.sleep();
        }

        g_main_loop_quit(gstLoop);
        gstThread.join();
        g_main_loop_unref(gstLoop);
    }

    GstElement* findRtspsrcElement(GstElement *pipeline) {
        GstIterator *it = gst_bin_iterate_elements(GST_BIN(pipeline));
        GstElement *rtspsrc_element = NULL;
        GValue item = G_VALUE_INIT;

        gboolean done = FALSE;

        while (!done) {
            switch (gst_iterator_next(it, &item)) {
                case GST_ITERATOR_OK: {
                    GstElement *element = GST_ELEMENT(g_value_get_object(&item));
                    const gchar *element_type_name = G_OBJECT_TYPE_NAME(element);

                    if (g_strcmp0(element_type_name, "GstRTSPSrc") == 0) {
                        rtspsrc_element = element;
                        g_print("Found RTSP source element: %s of type %s\n", GST_ELEMENT_NAME(element), element_type_name);
                        done = TRUE;
                    }

                    g_value_reset(&item);
                    break;
                }
                case GST_ITERATOR_RESYNC:
                    gst_iterator_resync(it);
                    break;
                case GST_ITERATOR_DONE:
                    done = TRUE;
                    break;
                case GST_ITERATOR_ERROR:
                    done = TRUE;
                    break;
            }
        }

        g_value_unset(&item);
        gst_iterator_free(it);

        if (rtspsrc_element) {
            gst_object_ref(rtspsrc_element);  // Referenz zählen, damit das Element nach der Rückgabe noch gültig ist
        }

        return rtspsrc_element;
    }

    static GstPadProbeReturn extractRtpTimestampProbe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        RtspStream *stream = static_cast<RtspStream *>(user_data);  
        std::lock_guard lock(stream->camera->timestampMutex);

        if (buffer && (info->type & GST_PAD_PROBE_TYPE_BUFFER)) {
            GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;

            if (gst_rtp_buffer_map(buffer, GST_MAP_READ, &rtp_buffer)) {
                guint8 ext_id = 1;  // ID of extension 1-14
                guint8 appbits;  // application sepecific bits (3 bits)
                guint nth = 0;  // We read the first (0-based index) extension
                guint64 custom_timestamp = 0;
                guint size = 0;

                gpointer data = nullptr;
                if (gst_rtp_buffer_get_extension_twobytes_header(&rtp_buffer, &appbits, ext_id, nth, &data, &size)) {
                    if (size == sizeof(custom_timestamp)) {
                        memcpy(&custom_timestamp, data, size);
                        //g_print("Stream  Received custom timestamp: %u ms\n", stream->name.c_str(), custom_timestamp);
                        uint64_t timestamp_ns = static_cast<uint64_t>(custom_timestamp);
                        stream->lastTimestamp = rclcpp::Time(timestamp_ns);
                        // RCLCPP_ERROR(RtspClients::get_logger(), "Updating lastTimestamp: %llu ns", timestamp_ns);
                    } else {
                        g_warning("Received RTP header extension with unexpected size: %u bytes\n", size);
                    }
                } else {
                //  g_print("RTP header extension not found or error reading it.\n");
                }

                gst_rtp_buffer_unmap(&rtp_buffer);
            }
        }
        

        return GST_PAD_PROBE_OK;
    }

    static void onPadAdded(GstElement *element, GstPad *pad, gpointer data) {
        gchar *pad_name = gst_pad_get_name(pad);
        g_print("New pad %s added to element %s\n", pad_name, GST_ELEMENT_NAME(element));

            GstPad *src_pad = gst_element_get_static_pad(element, pad_name);
            if (src_pad) {
                gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, (GstPadProbeCallback)extractRtpTimestampProbe, data, NULL);
                gst_object_unref(src_pad);
            } else {
                g_print("Failed to get 'src' pad after it was added.\n");
            }
        

        g_free(pad_name);
    }


    void RtspClients::callback_status_msg(const tod_status_msgs::msg::Status &msg) 
    {
        bool connected = (msg.tod_status != tod_status_msgs::msg::Status::TOD_STATUS_IDLE);
        if (connected && !_connected) {
            // on connect
            for (auto cam : _cameras){
                for (size_t i = 0; i < cam->_streams.size(); ++i) {
                    auto stream = cam->_streams[i];
                    if(i==0){
                        connect_video_client(stream, msg.vehicle_ip_address);
                    }else{
                        if(_ips.size() > 0 && (_ips[i-1] != msg.vehicle_ip_address)){
                            connect_video_client(stream, _ips[i-1]);
                        }
                    }
                }
            }
        }
        if (!connected && _connected) {
            // on disconnect
            for (auto cam : _cameras){
                for (auto stream : cam->_streams) {
                    disconnect_video_client(stream);
                }
            }
        }
        _connected = connected;
    }

    gboolean RtspClients::on_gst_message(GstBus *bus, GstMessage *message, RtspStream *stream) {
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_EOS:
                RCLCPP_WARN(RtspClients::get_logger(), "End of stream reached for %s. If end not intended, restart vehicle and operator docker to continue.", stream->name.c_str());
                break;
            default:
                break;
        }
        return TRUE;
    }

    void RtspClients::connect_video_client(std::shared_ptr<RtspStream> stream, const std::string &vehicleIp) 
    {
        // Check if the vehicle IP or stream name is empty
        if (vehicleIp == "" || stream->name == "") {
            RCLCPP_ERROR(this->get_logger(), "%s: vehicleIp \"%s\" or camera name \"%s\" are not set - abort connecting client",
                        _nn.c_str(), vehicleIp.c_str(), stream->name.c_str());
            return;
        }

        // Initialize GStreamer if not already initialized
        if (!gst_is_initialized()) gst_init(0, 0);

        // Determine the GStreamer image format using the node parameter and the lookup table
        std::string gstImageFormat{"rgb8"};
        if (_color_format.find(stream->imageOutputFormat) != _color_format.end()) { 
            gstImageFormat = _color_format[stream->imageOutputFormat];
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "%s: unknown image format %s - setting output format to %s",
                _nn.c_str(), stream->imageOutputFormat.c_str(), gstImageFormat.c_str());
        }

        // Construct the RTSP URI using the vehicle IP and stream name
        std::string uri = "rtsp://" + vehicleIp + ":" + this->rtsp_port_ + stream->name;

        // Create GStreamer pipeline descriptions for H264 and JPEG streams
        gchar *pipe_desc_h264 = g_strdup_printf("rtspsrc location=%s latency=%d drop-on-latency=true !"
                                                " identity name=myIdentity signal-handoffs=true !"
                                                " rtph264depay ! queue !" 
                                                " h264parse !"
                                                " avdec_h264 output-corrupt=true !"
                                                " videoconvert ! queue !"
                                                " capsfilter caps=video/x-raw,format=%s !"
                                                " appsink name=mySink sync=false emit-signals=true",
                                                uri.c_str(), _latency, gstImageFormat.c_str());
        gchar *pipe_desc_jpeg = g_strdup_printf("rtspsrc location=%s latency=%d drop-on-latency=true !"
                                                " identity name=myIdentity signal-handoffs=true !"
                                                " rtpjpegdepay !"
                                                " jpegparse !"
                                                " jpegdec !"
                                                " videoconvert !"
                                                " capsfilter caps=video/x-raw,format=%s !"
                                                " appsink name=mySink sync=false emit-signals=true",
                                                uri.c_str(), _latency, gstImageFormat.c_str());

        // Parse the appropriate pipeline based on the stream type (H264 or JPEG)
        GError *error{NULL};
        if (stream->isJpeg)
            stream->pipeline = gst_parse_launch(pipe_desc_jpeg, &error);
        else
            stream->pipeline = gst_parse_launch(pipe_desc_h264, &error);

        // Check for errors in pipeline parsing
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "%s: Something went wrong parsing launch string - Abort!", _nn.c_str());
            return;
        }

        // Free the pipeline description strings
        g_free(pipe_desc_h264);
        g_free(pipe_desc_jpeg);

        // Get the identity element from the pipeline
        GstElement *theIdentity = gst_bin_get_by_name(GST_BIN(stream->pipeline), "myIdentity");
        if (!theIdentity) {
            RCLCPP_ERROR(this->get_logger(), "%s: Could not get identity element from pipeline - Abort!", _nn.c_str());
            return;
        }
        // Connect the identity element's handoff signal to the callback
        g_signal_connect(theIdentity, "handoff", G_CALLBACK(new_rtp_packet), stream.get());

        // Get the appsink element from the pipeline
        GstElement *theAppSink = gst_bin_get_by_name(GST_BIN(stream->pipeline), "mySink");
        if (!theAppSink) {
            RCLCPP_ERROR(this->get_logger(), "%s: Could not get appsink from pipeline - Abort!", _nn.c_str());
            return;
        }
        // Connect the appsink element's new-sample signal to the callback
        g_signal_connect(theAppSink, "new-sample", G_CALLBACK(new_image_sample), stream.get());

        // Log the successful start of the stream
        RCLCPP_INFO(this->get_logger(), "%s: Started streaming %s from uri %s", _nn.c_str(), stream->name.c_str(), uri.c_str());

        // Set the pipeline to the PLAYING state
        gst_element_set_state(stream->pipeline, GST_STATE_PLAYING);

        // Find the rtspsrc element in the pipeline
        GstElement *rtpsrc = findRtspsrcElement(stream->pipeline);

        if (rtpsrc) {
            // Connect the pad-added signal of the rtspsrc element to the callback
            g_signal_connect(rtpsrc, "pad-added", G_CALLBACK(onPadAdded), stream.get());
        }

        // Add a message watch to the bus for handling GStreamer messages
        GstBus *bus = gst_element_get_bus(stream->pipeline);
        gst_bus_add_watch(bus, (GstBusFunc)on_gst_message, stream.get());
    }


    void RtspClients::disconnect_video_client(std::shared_ptr<RtspStream> stream) 
    {
        gst_element_send_event(stream->pipeline, gst_event_new_eos());
        gst_element_set_state(stream->pipeline, GST_STATE_NULL);
        gst_object_unref(stream->pipeline);
        RCLCPP_INFO(this->get_logger(), "%s: Stopped streaming %s", _nn.c_str(), stream->name.c_str());
    }

    void RtspClients::new_rtp_packet(GstElement *identity, GstBuffer *buffer, RtspStream *stream) 
    {
        if(stream->camera->timestamp < stream->lastTimestamp){ 
            std::lock_guard<std::mutex> lock(stream->mutex);
            RCLCPP_DEBUG(RtspClients::get_logger(), "%s: new rtp Package sent", stream->name.c_str());
            // store and publish packet info
            tod_network_msgs::msg::PaketInfo pktInfoMsg;
            
            auto time = std::chrono::system_clock::now();

            std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch());
            std::chrono::nanoseconds nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch());

            pktInfoMsg.header.stamp.sec = (int32_t)sec.count();
            pktInfoMsg.header.stamp.nanosec = (int32_t)(nanosec.count() % 1000000000);
            
            pktInfoMsg.size_bit = int32_t(8 * gst_buffer_peek_memory(buffer, 0)->size);
            gst_rtp_buffer_map(buffer, GST_MAP_READ, &stream->rtpPaket);
            pktInfoMsg.seq_num = gst_rtp_buffer_get_seq(&stream->rtpPaket);
            gst_rtp_buffer_unmap(&stream->rtpPaket);
            stream->pubPaketInfo->publish(pktInfoMsg);

            ++stream->rtpPaketCount;
            stream->pktSizeSum_bit += pktInfoMsg.size_bit;
        }
    }

    void RtspClients::new_image_sample(GstAppSink* appSink, RtspStream* stream) 
    {
        // RCLCPP_ERROR(stream->get_logger(), "Entering newImageSample for stream %s", stream->name.c_str());
        GstSample* sample = gst_app_sink_pull_sample(appSink);
        if (!sample) {
            RCLCPP_ERROR(stream->get_logger(), "No sample received from GStreamer pipeline.");
            return;
        }
        GstCaps* caps = gst_sample_get_caps(sample);
        if (!caps) {
            RCLCPP_ERROR(RtspClients::get_logger(), "%s Client: could not get image info from filter caps", stream->name.c_str());
            return;
        }
        GstStructure* s = gst_caps_get_structure(caps, 0);
        int width{0}, height{0};
        if (!(gst_structure_get_int(s, "width", &width)
            && gst_structure_get_int(s, "height", &height))) {
            RCLCPP_ERROR(RtspClients::get_logger(), "%s Client: Could not get image width and height from filter caps", stream->name.c_str());
            return;
        }
        GstBuffer* buffer = gst_sample_get_buffer(sample);    
        GstMemory* mem = gst_buffer_get_all_memory(buffer);
        GstMapInfo info;
        if (gst_memory_map(mem, &info, GST_MAP_READ)) {
            sensor_msgs::msg::Image::SharedPtr new_image_msg = std::make_shared<sensor_msgs::msg::Image>();
            if (buffer) {
                std::lock_guard lock(stream->camera->timestampMutex);           
                if(stream->camera->timestamp < stream->lastTimestamp){ 
                    new_image_msg->header.stamp = stream->lastTimestamp;
                    new_image_msg->data = std::vector<u_char>(info.data, info.data + info.size);
                    new_image_msg->width = width;
                    new_image_msg->height = height;
                    new_image_msg->step = uint(info.size / height);
                    new_image_msg->encoding = stream->imageOutputFormat;
                    stream->imgHeight_px = height;
                    stream->imgWidth_px = width;
                    stream->pubImage->publish(*new_image_msg);
                    
                    // RCLCPP_ERROR(stream->get_logger(), 
                    //                     "New Image arrived %s at time: %.3f seconds. Last Stamp: %.3f", 
                    //                     stream->name.c_str(), 
                    //                     stream->lastTimestamp.seconds(),
                    //                     stream->camera->timestamp.seconds());
                    stream->camera->timestamp = stream->lastTimestamp;                   

                }else{
                    //  RCLCPP_ERROR(stream->get_logger(), "Image was too late  %s at time: %.3f seconds. Last Stamp: %.3f", stream->name.c_str(), stream->lastTimestamp.seconds(),stream->camera->timestamp.seconds());
                }
            }             
            stream->imgHeight_px = height;
            stream->imgWidth_px = width;
        }

        gst_sample_unref(sample);
        gst_memory_unref(mem);
        gst_memory_unmap(mem, &info);
        std::lock_guard<std::mutex> lock(stream->mutex);
        ++stream->frameCount; // used to publish current frame rate from ros main loop
    }
} //namespace tod_rtsp
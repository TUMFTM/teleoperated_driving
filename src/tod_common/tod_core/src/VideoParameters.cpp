// Copyright Niklas Krauss 2024
#include "tod_core/param_set/VideoParameters.hpp"

#include <cassert>

namespace tod_core
{
namespace param_set
{
Video::Video(rclcpp::Node * node_ptr, const std::string& config_path, const std::string& encoderType)
    : ParameterHandler(node_ptr, "vehicleID"), _desiredPath(config_path), _encoderType(encoderType)
{
    // Normalize the encoderType to lowercase to match YAML values (if necessary)
    std::transform(_encoderType.begin(), _encoderType.end(), _encoderType.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Initialize the appropriate encoder based on encoderType
    // This logic could potentially be moved into a separate method if it grows complex
    if (_encoderType == "h264") {
        _encoder = H264Encoder();
    } else if (_encoderType == "h265") {
        _encoder = H265Encoder();
    } else if (_encoderType == "vp9") {
        _encoder = VP9Encoder();
    } else if (_encoderType == "av1") {
        _encoder = AV1Encoder();
    } else {
        std::cerr << "Unsupported encoder type: " << encoderType << std::endl;
        // Handle unsupported encoder type (throw exception, set error state, etc.)
    }

}

bool Video::load_parameters() {
    YamlLoader loader;
    if (!loader.load_from_path(get_path_to_config_files() + "/" + get_yaml_file())) {
        return false;
    }

    // Load common video settings
    _videoSettings.width = loader.get_param<int>("video_settings", "width");
    _videoSettings.height = loader.get_param<int>("video_settings", "height");
    _videoSettings.offsetWidth = loader.get_param<int>("video_settings", "offset_width");
    _videoSettings.offsetHeight = loader.get_param<int>("video_settings", "offset_height");
    _videoSettings.actualWidth = loader.get_param<int>("video_settings", "actual_width");
    _videoSettings.actualHeight = loader.get_param<int>("video_settings", "actual_height");
    _videoSettings.scalingFactor = loader.get_param<std::string>("video_settings", "scaling_factor");
    _videoSettings.bitrate = loader.get_param<int>("video_settings", "bitrate");

    // Load encoder-specific settings based on _encoderType
    auto encoderSettings = loader.get_settings_for_key_value<std::string>("encoders", "encoder_type", _encoderType);
    if (!encoderSettings) {
        std::cerr << "Error: No settings found for encoder type " << _encoderType << std::endl;
        return false;
    }
    _encoderSettings = *encoderSettings;

    if (_encoderType == "h264") {
        H264Encoder h264encoder;

        // Assuming your H264Encoder class has corresponding fields
        // Directly assign values from _encoderSettings to h264encoder's fields
        for (const auto& setting : _encoderSettings) {
            if (setting.first == "tune") h264encoder.tune = setting.second;
            else if (setting.first == "speedPreset") h264encoder.speedPreset = setting.second;
            else if (setting.first == "slicedThreads") h264encoder.slicedThreads = setting.second == "true";
            else if (setting.first == "byteStream") h264encoder.byteStream = setting.second == "true";
            else if (setting.first == "intraRefresh") h264encoder.intraRefresh = setting.second == "true";
            else if (setting.first == "threads") h264encoder.threads = std::stoi(setting.second);
            else if (setting.first == "keyIntMax") h264encoder.keyIntMax = std::stoi(setting.second);
            else if (setting.first == "nvidiaBased") h264encoder.nvidiaBased = setting.second == "true";
            // Add more fields as necessary
        }

        _encoder = h264encoder;
    }
  else if (_encoderType == "h265"){
    //tood 
  }
  else if (_encoderType == "vp9"){
    //tood 
  }
  else if (_encoderType == "av1"){
    //tood 
  }
  
  return true;
}
}  // namespace param_set
}  // namespace tod_core



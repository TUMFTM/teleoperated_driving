// Copyright 2021 Krauss
#ifndef TOD_CORE__PARAM_SET__VIDEOPARAMETERS_HPP_
#define TOD_CORE__PARAM_SET__VIDEOPARAMETERS_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tod_core/YamlLoader.hpp"
#include "tod_core/param_set/ParameterHandler.hpp"
namespace tod_core
{
namespace param_set
{

struct BaseEncoder {
    std::map<std::string, std::string> settingsDict; // For dynamic access to any setting
};

struct H264Encoder : public BaseEncoder {
    std::string encoderType{"h264"};
    std::string tune{""};
    std::string speedPreset{""};
    bool slicedThreads{true};
    bool byteStream{true};
    bool intraRefresh{true};
    int threads{1};
    int keyIntMax{15};
    bool nvidiaBased{false};
};

struct H265Encoder : public BaseEncoder {
    std::string encoderType{"h265"};
    std::string tune{""};
    std::string speedPreset{""};
    bool slicedThreads{true};
    bool byteStream{true};
    bool intraRefresh{true};
    int threads{1};
    int keyIntMax{15};
    bool nvidiaBased{false};
};

struct VP9Encoder : public BaseEncoder {
    std::string encoderType{"vp9"};
    std::string speedPreset{""};
    bool nvidiaBased{false};
    // add other paramters from yaml file here
};

struct AV1Encoder : public BaseEncoder {
    std::string encoderType{"av1"};
    std::string speedPreset{""};
    bool nvidiaBased{false};
    // add other paramters from yaml file here
};

using Encoder = std::variant<H264Encoder, H265Encoder, VP9Encoder, AV1Encoder>;


struct VideoSettings {
    int bitrate{10000};
    int actualWidth;
    int actualHeight;
    std::string scalingFactor{"1p000"};
    int width;
    int height;
    int offsetWidth;
    int offsetHeight;
  };

class Video : public ParameterHandler
{
public:

  Video(rclcpp::Node * node_ptr, const std::string& config_path, const std::string& encoderType);
  bool load_parameters() override;
  std::string get_yaml_file() override { return "stream_settings.yml"; }
  std::string get_path_to_config_files() override { return _desiredPath; }
  std::map<std::string, std::string> get_encoder_settings() const { return _encoderSettings; }
  Encoder get_encoder() const { return _encoder; }

  std::shared_ptr<VideoSettings> get_video_settings() const {
    return std::make_shared<VideoSettings>(_videoSettings);
  }   
  
  int get_bitrate() const { return _videoSettings.bitrate; }
  std::string get_encoder_type() const { return _encoderType; }
  int get_actual_width() const { return _videoSettings.actualWidth; }
  int get_actual_height() const { return _videoSettings.actualHeight; }
  std::string get_scaling_factor() const { return _videoSettings.scalingFactor; }
  int get_width() const { return _videoSettings.width; }
  int get_height() const { return _videoSettings.height; }
  int get_offset_width() const { return _videoSettings.offsetWidth; }
  int get_offset_height() const { return _videoSettings.offsetHeight; }


private:
  std::string _desiredPath{""};
  std::string _encoderType{""};
  std::map<std::string, std::string> _encoderSettings;
  Encoder _encoder;     

  VideoSettings  _videoSettings;

};
}  // namespace param_set
}  // namespace tod_core
#endif  // TOD_CORE__PARAM_SET__VIDEOPARAMETERS_HPP_
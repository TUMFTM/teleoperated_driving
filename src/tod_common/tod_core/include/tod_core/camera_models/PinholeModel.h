// Copyright 2021 Schimpe
#ifndef TOD_HELPER__CAMERA_MODELS__PINHOLEMODEL_H_
#define TOD_HELPER__CAMERA_MODELS__PINHOLEMODEL_H_

#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <tod_core/YamlLoader.hpp>
struct PinholeModel
{
public:
  int width_raw{0}, height_raw{0};
  float center_x{0.0f}, center_y{0.0f};
  float focal_x{0.0f}, focal_y{0.0f};

public:
  PinholeModel(const std::string & cameraName, const std::string & vehicleID,
    const std::string& config_path)
  {
    // get path to yaml with params and load
    std::string pathToYaml =
            config_path + vehicleID + "/camera-calibration/" + cameraName + ".yaml";

    YamlLoader loader;
    if (!loader.load_from_path(pathToYaml)) {
        printf("Could not load PinholeModel config for %s from %s.\n",
               cameraName.c_str(), vehicleID.c_str());
        return;
    }
    width_raw = loader.get_param<int>("image_width");
    height_raw = loader.get_param<int>("image_height");
    std::vector<float> calibData = loader.get_param<std::vector<float>>("camera_matrix", "data");
    focal_x = calibData.at(0);
    focal_y = calibData.at(4);
    center_x = calibData.at(2);
    center_y = calibData.at(5);
  }
  bool point_on_image(
    const geometry_msgs::msg::Point & pt, int & x_px, int & y_px, const double scaling_x = 1.0,
    const double scaling_y = 1.0) const
  {
    x_px = static_cast<int>(scaling_x * focal_x * pt.x / pt.z + scaling_x * center_x);
    y_px = static_cast<int>(scaling_y * focal_y * pt.y / pt.z + scaling_y * center_y);
    return (
      1 <= x_px && x_px <= scaling_x * width_raw && 1 <= y_px && y_px <= scaling_y * height_raw);
  }
};

#endif   // TOD_HELPER__CAMERA_MODELS__PINHOLEMODEL_H_

// Copyright 2021 Schimpe
#ifndef TOD_HELPER__CAMERA_MODELS__OCAMMODEL_H_
#define TOD_HELPER__CAMERA_MODELS__OCAMMODEL_H_

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "external/ocam_functions.cpp"
struct OcamModel
{
public:
  int width_raw{0}, height_raw{0};
  float center_x{0.0f}, center_y{0.0f};
  struct ocam_model o;
  OcamModel(const std::string & cameraName, const std::string & vehicleID,
    const std::string & config_path)
  {
    // get path to calibration file and load ocam model
    std::string pathToCalib =
            config_path + vehicleID + "/camera-calibration/" + cameraName + ".txt";

    get_ocam_model(&o, pathToCalib.data());
    width_raw = o.width;
    height_raw = o.height;
    center_x = static_cast<float>(o.xc);
    center_y = static_cast<float>(o.yc);
  }
  bool point_on_image(
    const geometry_msgs::msg::Point & pt, int & x_px, int & y_px, const double scaling_x = 1.0,
    const double scaling_y = 1.0) const
  {
    double point3D[3]{pt.y, pt.x, -pt.z};
    double point2D[2]{0.0, 0.0};
    world2cam(point2D, point3D, &o);
    x_px = static_cast<int>(point2D[1]);
    y_px = static_cast<int>(point2D[0]);
    return (
      1 <= x_px && x_px <= scaling_x * width_raw && 1 <= y_px && y_px <= scaling_y * height_raw);
  }
};
#endif  // TOD_HELPER__CAMERA_MODELS__OCAMMODEL_H_

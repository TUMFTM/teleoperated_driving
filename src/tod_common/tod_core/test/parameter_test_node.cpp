// Copyright 2021 Hoffmann
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tod_core/param_set/param_sets.hpp"
#include "tod_core/camera_models/OcamModel.h"
#include "tod_core/camera_models/PinholeModel.h"

using namespace std::chrono_literals;
class ParametersClass : public rclcpp::Node
{
public:
  ParametersClass() : Node("test_node")
  {
    std::string desiredPath =
      ament_index_cpp::get_package_share_directory("tod_core") + "/test_config/";
    //std::string desiredPath =
    //  ament_index_cpp::get_package_share_directory("tod_vehicle_config") + "/vehicle_config/";
    timer_ = this->create_wall_timer(1000ms, std::bind(&ParametersClass::main_loop, this));
    vehicleParamHandler_ = std::make_unique<tod_core::param_set::Vehicle>(this, desiredPath);
    camParamHandler_ = std::make_unique<tod_core::param_set::Camera>(this, desiredPath);
    lidarParamHandler_ = std::make_unique<tod_core::param_set::Lidar>(this, desiredPath);
    vehInfoParamHandler_ = std::make_unique<tod_core::param_set::VehicleInformation>(this, desiredPath);
    transformParamHandler_ = std::make_unique<tod_core::param_set::Transform>(this, desiredPath);
    vehicleParamHandler_->set_id_changed_cb(std::bind(&ParametersClass::on_vehicle_id_changed, this, std::placeholders::_1));
  }
  void main_loop()
  {
    std::cout << "Mass: " << vehicleParamHandler_->get_mass() << std::endl;
    std::cout << "CameraImageName: " << camParamHandler_->get_camera_image_name() << std::endl;
    std::cout << "PointCloudName: " << lidarParamHandler_->get_pointcloud_name() << std::endl;
    std::cout << "VehicleType: " << vehInfoParamHandler_->get_type() << std::endl;
    if (transformParamHandler_->get_transforms().size() > 0) {
      std::cout << "ChildFrameId at 0: "
                << transformParamHandler_->get_transforms().at(0).child_frame_id << std::endl;
    }
  }

  void on_vehicle_id_changed(const std::string& veh_id){
    std::cout << "Callback called: " << veh_id << std::endl;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tod_core::param_set::Vehicle> vehicleParamHandler_;
  std::unique_ptr<tod_core::param_set::Camera> camParamHandler_;
  std::unique_ptr<tod_core::param_set::Lidar> lidarParamHandler_;
  std::unique_ptr<tod_core::param_set::VehicleInformation> vehInfoParamHandler_;
  std::unique_ptr<tod_core::param_set::Transform> transformParamHandler_;
};
int main(int argc, char ** argv)
{
  std::string desiredPath =
    ament_index_cpp::get_package_share_directory("tod_core") + "/test_config/";
  std::string vehicleId = "tum-q7";
  std::string ocamCamName = "CameraTopViewFront";
  std::string pinholeCamName = "CameraFrontCenter";
  OcamModel mdl(ocamCamName, vehicleId, desiredPath);
  PinholeModel mdl2(pinholeCamName, vehicleId, desiredPath);
  printf("From %s:\n", vehicleId.c_str());
  printf("Loaded Ocam Camera Mdl for %s with cx=%.2f, cy=%.2f, w=%d, h=%d\n",
         ocamCamName.c_str(),
         mdl.center_x, mdl.center_y, mdl.width_raw, mdl.height_raw);
  printf("Loaded Pinhole Camera Mdl for %s with cx=%.2f, cy=%.2f, fx=%.2f, fy=%.2f, w=%d, h=%d\n\n",
         pinholeCamName.c_str(), mdl2.center_x, mdl2.center_y,
         mdl2.focal_x, mdl2.focal_y, mdl2.width_raw, mdl2.height_raw);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
// Todo(Simon): more generic interface()
// Todo(Simon): check if node_ptr not nullptr

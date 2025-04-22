/**
 * @file edgarautoware_sensing_interface.hpp
 * @brief Sensing interface for the research vehicle EDGAR using Autoware as AV stack.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgarautoware_interface
 */

#pragma once

#include "tod_generic_interface/sensing_interface.hpp"

// #include "tod_core/param_set/LidarParameters.hpp"
// #include "tod_core/param_set/CameraParameters.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace tod_edgarautoware_interface {
/**
 * @ingroup tod_edgarautoware_interface
 * @brief Interfaces for the research vehicle EDGAR using Autoware as AV stack.
 */

/**
 * @brief Sensing interface for the research vehicle EDGAR using Autoware as AV stack.
 */
class SensingInterface : public rclcpp::Node
{
    public:
        SensingInterface();
        void run();
    private:    
        // std::shared_ptr<tod_core::param_set::Camera> cam_params_;
        // std::shared_ptr<tod_core::param_set::Lidar> lidar_params_;
        std::shared_ptr<tod_generic_interface::SensingInterface> generic_sensing_interface_;
};

} // namespace tod_edgarautoware_interface
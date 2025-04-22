/**
 * @file edgar_sensing_interface.hpp
 * @brief Sensing interface for the research vehicle EDGAR without AV capabilities.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgar_interface
 */

#pragma once

#include "tod_generic_interface/sensing_interface.hpp"

#include "tod_core/param_set/LidarParameters.hpp"
#include "tod_core/param_set/CameraParameters.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace tod_edgar_interface {
/**
 * @ingroup tod_edgar_interface
 * @brief Interfaces for the research vehicle EDGAR.
 */

/**
 * @brief Sensing interface for the research vehicle EDGAR.
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

} // namespace tod_edgar_interface
/**
 * @file rc-car_sensing_interface.hpp
 * @brief RC-Car sensing interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_rc-car_interface
 */

#pragma once

#include "tod_generic_interface/sensing_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace tod_rccar_interface {
/**
 * @ingroup tod_rccar_interface
 * @brief Interfaces for the F1TENTH RC-Cars.
 */

/**
 * @brief Sensing interface for the F1TENTH RC-Cars.
 */
class SensingInterface : public rclcpp::Node
{
    public:
        SensingInterface();
        void run();
    private:    
        std::shared_ptr<tod_generic_interface::SensingInterface> generic_sensing_interface_;
};

} // namespace tod_rccar_interface
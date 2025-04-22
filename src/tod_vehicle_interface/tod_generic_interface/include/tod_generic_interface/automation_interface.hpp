/**
 * @file automation_interface.hpp
 * @brief Generic automation interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#pragma once

#include "tod_generic_interface/base_interface.hpp"

namespace tod_generic_interface {
/**
 * @ingroup tod_generic_interface
 * @brief Generic interfaces between vehicle platforms and the TUM Teleoperation software.
 */

/**
 * @brief Generic automation interface.
 */
class AutomationInterface : public BaseInterface 
{
    public:
        AutomationInterface(rclcpp::Node::SharedPtr node);
        virtual ~AutomationInterface() = default;
};

} // namespace tod_generic_interface
/**
 * @file actuation_interface.hpp
 * @brief Generic actuation interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#pragma once

#include "tod_generic_interface/base_interface.hpp"

namespace tod_generic_interface {
/**
 * @defgroup tod_generic_interface
 * @ingroup tod_generic_interface
 * @brief Generic interfaces between vehicle platforms and the TUM Teleoperation software.
 */

/**
 * @brief Generic actuation interface.
 */
class ActuationInterface : public BaseInterface 
{
    public:
        ActuationInterface(rclcpp::Node::SharedPtr node);
        virtual ~ActuationInterface() = default;
};

} // namespace tod_generic_interface



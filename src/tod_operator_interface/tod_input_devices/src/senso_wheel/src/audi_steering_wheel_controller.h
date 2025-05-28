// Copyright TUM-FTM

/**
 * @file audi_steering_wheel_controller.hpp
 * @brief Header file for the AudiSteeringWheelController class, which interfaces with Audi steering wheel buttons and controls.
 *
 * This file defines the `AudiSteeringWheelController` class, which processes CAN bus messages from an Audi steering wheel
 * and invokes button state callbacks. It inherits from `CanInterface` to communicate with the CAN bus.
 *
 * Key features:
 * - Decodes button and lever states from CAN bus messages.
 * - Provides callback functionality to notify external components of button state changes.
 * - Supports multiple button and lever configurations, including blinkers, wipers, and wheels.
 */

#pragma once

#include "can_interface.h"
#include <iostream>
#include <vector>
#include <functional>

 /**
  * @class AudiSteeringWheelController
  * @brief Interfaces with Audi steering wheel controls through the CAN bus.
  *
  * The `AudiSteeringWheelController` class listens for specific CAN bus messages,
  * interprets button and lever states, and provides callbacks to notify changes.
  */
class AudiSteeringWheelController : public CanInterface {
public:
    /**
     * @brief Constructs an AudiSteeringWheelController instance.
     *
     * Initializes the CAN interface and prepares to decode button states.
     */
    AudiSteeringWheelController();

    /**
     * @brief Destructor for the AudiSteeringWheelController class.
     */
    ~AudiSteeringWheelController();

    /**
     * @brief Sets the callback function for button state changes.
     * @param f A function that takes a vector of boolean button states.
     *
     * The callback is invoked whenever button states change due to incoming CAN bus messages.
     */
    void set_button_callback(std::function<void(const std::vector<bool>)> f);

private:
    /// Vector holding the current state of all buttons.
    std::vector<bool> buttonState;
    
    /// Callback function for button state changes.
    std::function<void(const std::vector<bool>)> _button_callback;

    /**
     * @brief Handles incoming CAN bus replies and decodes button states.
     * @param Identifier The CAN message identifier.
     * @param Length The length of the CAN message data.
     * @param Data Pointer to the CAN message data.
     *
     * This method decodes various button and lever states from CAN messages and updates the `buttonState` vector.
     * The callback function is invoked with the updated button states.
     */
    void HandleReply(unsigned int Identifier, unsigned char Length, const BYTE* Data) override;
};


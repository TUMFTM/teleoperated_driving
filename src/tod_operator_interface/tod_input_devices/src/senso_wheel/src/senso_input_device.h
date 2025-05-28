
/**
 * @file senso_input_device.hpp
 * @brief Header file for the SensoInputDevice class, which integrates SensoWheel and Audi steering wheel inputs.
 * @copyright TUMFTM 2024
 *
 * This file defines the `SensoInputDevice` class, which processes input from the SensoWheel and Audi steering wheel
 * controllers, providing axis and button callbacks for user-defined behavior.
 *
 * Key features:
 * - Integration of steering wheel and pedal inputs from the SensoWheel.
 * - Integration of button inputs from the Audi steering wheel.
 * - Support for cyclic events and multithreaded data processing.
 */

#pragma once
#include "my_input_device.h" 
#include "audi_steering_wheel_controller.h"
#include <iostream>
#include "senso_controller.h"
#include <thread>
#include <vector>

/**
 * @class SensoInputDevice
 * @brief Combines SensoWheel and Audi steering wheel input handling into a single device interface.
 *
 * The `SensoInputDevice` class processes inputs from the SensoWheel (steering, pedals) and Audi steering wheel
 * (buttons) controllers. It manages multithreaded communication and provides user-defined callbacks for
 * axis and button state changes.
 */
class SensoInputDevice : public MyInputDevice {
public:
    /**
     * @brief Constructs a SensoInputDevice instance.
     * @param axisCb Callback function for axis value changes.
     * @param buttonCb Callback function for button state changes.
     *
     * Initializes the SensoWheel and Audi steering wheel controllers with appropriate callbacks.
     */
    SensoInputDevice(std::function<void(const int, const double)> axisCb,
        std::function<void(const int, const int)> buttonCb);

    /**
     * @brief Destructor for the SensoInputDevice class.
     *
     * Cleans up resources and deactivates the device.
     */
    ~SensoInputDevice();

    /**
     * @brief Activates the SensoInputDevice.
     * @return `true` if activation is successful.
     *
     * Initializes the controllers, creates threads for data processing, and prepares the device for input.
     */
    bool activate() override;

    /**
     * @brief Deactivates the SensoInputDevice.
     * @return `true` if deactivation is successful.
     *
     * Stops data processing, shuts down controllers, and cleans up threads.
     */
    bool deactivate() override;

private:
    /**
     * @brief Handles steering wheel position messages.
     * @param steeringPosition The current steering wheel position in increments.
     *
     * Processes the steering position, applies scaling, and invokes the axis callback if there is significant change.
     */
    void onSteeringWheelMsgReceived(const int steeringPosition);

    /**
     * @brief Handles pedal position messages.
     * @param accPedalPos The current accelerator pedal position.
     * @param breakPedalPos The current brake pedal position.
     * @param clutchPedalPos The current clutch pedal position.
     *
     * Processes pedal positions, applies scaling, and invokes the axis callback for significant changes.
     */
    void onPedalMsgReceived(const int accPedalPos, const int breakPedalPos, const int clutchPedalPos);

    /**
     * @brief Handles button state messages.
     * @param buttonState A vector of boolean states for all buttons.
     *
     * Compares the current button states with the previous states and invokes the button callback for changes.
     */
    void onButtonCallback(const std::vector<bool>& buttonState);

    /**
     * @brief Runs cyclic events for the SensoWheel.
     *
     * Handles periodic events such as requesting updates or sending commands to the SensoWheel device.
     */
    void DoCyclicEvents();

    SensoController _sensoController;               ///< Controller for the SensoWheel device.
    AudiSteeringWheelController _audiController;    ///< Controller for the Audi steering wheel.
    std::thread _sensoRecvThread;                   ///< Thread for receiving SensoWheel data.
    std::thread _sensoCyclThread;                   ///< Thread for handling cyclic events.
    std::thread _audiRecvThread;                    ///< Thread for receiving Audi steering wheel data.
};


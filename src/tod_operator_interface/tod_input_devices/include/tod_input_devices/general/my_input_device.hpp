/**
 * @file my_input_device.cpp
 * @brief Abstraction for input devices with necessary function callback 
 * @copyright 2025 TUMFTM
 **/
#pragma once
#include <functional>
#include <stdlib.h>
#include <string>

#define MIN_JOY_RANGE -1.0      ///< Minimum joystick value.
#define MAX_JOY_RANGE 1.0       ///< Maximum joystick value.

namespace tod_input_device {

/**
 * @class MyInputDevice
 * @brief Abstract base class for input devices, providing functionality for handling input events and device state.
 *
 * The `MyInputDevice` class provides a common interface for managing input devices, including buttons and axes.
 * Derived classes must implement the activation and deactivation methods.
 */
class MyInputDevice {
public:
    /**
     * @brief Constructs a MyInputDevice with axis and button callbacks.
     * @param axisCb Callback function for axis value changes.
     * @param buttonCb Callback function for button state changes.
     */
    explicit MyInputDevice(std::function<void(const int, const double)> axisCb,
                           std::function<void(const int, const int)> buttonCb);

    /**
     * @brief Constructs a MyInputDevice with axis, button, and error callbacks.
     * @param axisCb Callback function for axis value changes.
     * @param buttonCb Callback function for button state changes.
     * @param errorCb Callback function for error reporting.
     */
    explicit MyInputDevice(std::function<void(const int, const double)> axisCb,
                           std::function<void(const int, const int)> buttonCb,
                           std::function<void(const std::string&)> errorCb);

    /**
     * @brief Virtual destructor.
     */
    virtual ~MyInputDevice() = default;

    void set_axis_callback(std::function<void(const int, const double)> f);
    void set_button_callback(std::function<void(const int, const int)> f);
    void set_correction(const std::string& calibration);
    int get_number_of_axes();
    int get_number_of_buttons();

    /**
     * @brief Activates the device.
     * @return `true` if the device was successfully activated, `false` otherwise.
     */
    virtual bool activate() = 0;

    /**
     * @brief Deactivates the device.
     * @return `true` if the device was successfully deactivated, `false` otherwise.
     */
    virtual bool deactivate() = 0;

    /**
     * @brief Terminates the device.
     *
     * Calls the `deactivate` method to safely shut down the device.
     */
    virtual void terminate();
    bool running{false};

protected:
    int _numberOfButtons{-1};           ///< Number of buttons supported by the device.
    int _numberOfAxes{-1};              ///< Number of axes supported by the device.
    std::function<void(const int, const double)> axis_callback;  ///< Callback for axis changes.
    std::function<void(const int, const int)> button_callback;   ///< Callback for button changes.
    std::function<void(const std::string&)> error_callback;      ///< Callback for error reporting.
    std::string _correction{""};        ///< Correction string for device calibration.

    /**
     * @brief Scales a value within the range [MIN_JOY_RANGE, MAX_JOY_RANGE].
     * @param nValue The value to scale.
     * @param nMinInput The minimum input range.
     * @param nMaxInput The maximum input range.
     * @param dMinOutput The minimum output range (default: MIN_JOY_RANGE).
     * @param dMaxOutput The maximum output range (default: MAX_JOY_RANGE).
     * @return The scaled value within the output range.
     */
    double scale_value(int nValue, int nMinInput, int nMaxInput,
                       double dMinOutput = MIN_JOY_RANGE, double dMaxOutput = MAX_JOY_RANGE);
};

} // namespace tod_input_device

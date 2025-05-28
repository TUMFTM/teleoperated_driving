
/**
 * @file usb_event_device.hpp
 * @brief Header file for the UsbEventDevice class, which interfaces with USB event devices and manages force feedback.
 * @author Andreas Schimpe based on Oliver Hamann's ffcfstress
 * @copyright Copyright 2024 TUMFTM
 *
 * This file defines the `UsbEventDevice` class, which provides an interface to interact with USB input devices that
 * support force feedback. It allows for configuring axes, managing force feedback effects, and resetting device states.
 * 
 * Key features:
 * - Detects and opens USB event devices.
 * - Configures force feedback for constant force effects.
 * - Manages device properties such as axis range and autocenter settings.
 * - Supports setting and resetting force feedback dynamically.
 * 
 * Dependencies:
 * - Requires Linux input subsystem (`<linux/input.h>`).
 * - Assumes force feedback support in the underlying device.
 */


#pragma once
#include <linux/input.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include "bitmaskros.h"

#define DEFAULT_AXIS_INDEX          0   ///< Default axis index for device configuration. 
#define DEFAULT_AXIS_CODE       ABS_X   ///< Default axis code for device configuration.

/**
 * @class UsbEventDevice
 * @brief Provides an interface for managing USB event devices with force feedback.
 * 
 * The `UsbEventDevice` class allows interaction with USB input devices that support force feedback. It
 * initializes the device, configures axes and ranges, and provides methods for applying constant force effects.
 */
class UsbEventDevice {
public:
    /**
     * @brief Constructs a UsbEventDevice object.
     * @param deviceNamespace Namespace to search for USB event devices.
     * 
     * Initializes the device by detecting available USB event devices in the specified namespace and
     * configuring properties such as axis range and force feedback.
     */
    explicit UsbEventDevice(const std::string &deviceNamespace);

    /**
     * @brief Destructor for the UsbEventDevice class.
     * 
     * Cleans up by deleting any active force feedback effects and closing the device.
     */
    ~UsbEventDevice() { if (ok()) delete_effect(); }

    /**
     * @brief Checks if the device was successfully initialized.
     * @return `true` if the device is initialized and ready, `false` otherwise.
     */
    bool ok() const { return _ok; }

    /**
     * @brief Sets the force feedback level.
     * @param ffValue The force feedback value, clamped between -1.0 and 1.0.
     * 
     * Updates the constant force effect with the specified force level and applies it to the device.
     */
    void set_force_feedback(const double ffValue);
    
    /**
     * @brief Resets the device by reinitializing properties and deleting active effects.
     */
    void reset();

private:
    /// Indicates whether the device was successfully initialized.
    bool _ok{false}; 
    
    /// Human-readable axis names.
    const std::vector<std::string> axis_names{ "X", "Y", "Z", "RX", "RY", "RZ", "WHEEL" };
    
    /// Axis codes.
    const std::vector<int> axis_codes{ ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_WHEEL };

        int _axis_index{DEFAULT_AXIS_INDEX};    ///< Current axis index being used.
    int _axis_code{DEFAULT_AXIS_CODE};          ///< Current axis code being used.
    bool _autocenter_off;                       ///< Indicates whether autocentering is disabled.
    int _device_handle;                         ///< File descriptor for the USB device.
    int _axis_min, _axis_max;                   ///< Minimum and maximum values for the current axis.
    struct ff_effect _effect;                   ///< Force feedback effect structure.

    /**
     * @brief Opens a USB event device in the specified namespace.
     * @param deviceNamespace Namespace to search for devices.
     * @return `true` if a device was successfully opened, `false` otherwise.
     */
    bool open_device(const std::string &deviceNamespace);

    /**
     * @brief Initializes the device by configuring axes, ranges, and force feedback properties.
     */
    void initialize_device();

    /**
     * @brief Creates a force feedback event with the specified force level.
     * @param force The force level, clamped between -0.8 and 0.8.
     */
    void create_event(const double force);
    
    /**
     * @brief Deletes the currently active force feedback effect.
     */
    void delete_effect();
};

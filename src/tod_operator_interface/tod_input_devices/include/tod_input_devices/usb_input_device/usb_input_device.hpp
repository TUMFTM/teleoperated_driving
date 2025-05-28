/**
 * @file usb_event_handler.hpp
 * @brief Converts USB event into Joy Messages
 * @copyright 2024 TUMFTM
 **/
#pragma once
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <linux/joystick.h>
#include <thread>
#include <cstring>
#include <string>
#include "tod_input_devices/general/my_input_device.hpp"
#include "tod_input_devices/usb_input_device/correction.hpp"

#define MIN_POSITION_INCREMENTS_USB -32767  ///< Minimum USB joystick position increment.
#define MAX_POSITION_INCREMENTS_USB 32767   ///< Maximum USB joystick position increment.

namespace tod_input_device {

/**
 * @class UsbInputDevice
 * @brief Represents a USB-based input device such as a USB steering wheel, joystick, or game controller.
 *
 * This class handles reading input events from a USB device and provides callbacks for axis and button inputs.
 * It also supports setting corrections for input scaling and operates asynchronously using a thread.
 */
class UsbInputDevice : public MyInputDevice {
public:
    /**
     * @brief Constructs a UsbInputDevice instance.
     * @param axisCb Callback function for axis value changes.
     * @param buttonCb Callback function for button state changes.
     * @param errorCb Callback function for error reporting.
     */
    UsbInputDevice(std::function<void(const int, const double)> axisCb,
                   std::function<void(const int, const int)> buttonCb,
                   std::function<void(const std::string&)> errorCb);

    /**
     * @brief Destructor for the UsbInputDevice class.
     *
     * Deactivates the device and cleans up resources.
     */
    ~UsbInputDevice();

    /**
     * @brief Activates the USB input device.
     * @return `true` if activation is successful.
     *
     * Opens the device config, initializes input dimensions, and starts the input reading thread.
     */
    bool activate() override;

    /**
     * @brief Deactivates the USB input device.
     * @return `true` if deactivation is successful.
     *
     * Stops the input reading thread and closes the device file.
     */
    bool deactivate() override;

private:
    struct js_event _jsevent;       ///< Joystick event structure.
    const char *_device;            ///< Path to the USB input device.
    int _js;                        ///< File descriptor for the USB device.
    std::thread _readUsbThread;     ///< Thread for reading input events.

    /**
     * @brief Initializes the size of axes and buttons based on device properties.
     */
    void init_size();

    /**
     * @brief Reads an input event from the USB device.
     * @param fd The file descriptor of the USB device.
     * @param event Pointer to a js_event structure to store the read event.
     * @return `0` if successful, `-1` otherwise.
     */
    int read_event(int fd, struct js_event *event);

    /**
     * @brief Runs the input event reading loop.
     *
     * Continuously reads events from the device and invokes the appropriate callbacks for axis and button inputs.
     */
    void run();
};

} // namespace tod_input_device

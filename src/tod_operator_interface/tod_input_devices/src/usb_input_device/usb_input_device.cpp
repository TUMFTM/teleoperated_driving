/**
 * @file usb_input_device.hpp
 * @brief Defines a usb input device using linux events to convert into ros messages
 * @copyright 2025 TUMFTM
 **/

#include "tod_input_devices/usb_input_device/usb_input_device.hpp"
#include "sys/statvfs.h"

namespace tod_input_device {

UsbInputDevice::UsbInputDevice(std::function<void(const int, const double)> axisCb,
        std::function<void(const int, const int)> buttonCb, std::function<void(const std::string&)> errorCb)
         : MyInputDevice(axisCb, buttonCb, errorCb) {
    // specify device
    _device = "/dev/input/js0";
}

UsbInputDevice::~UsbInputDevice() {
    deactivate();
}

bool UsbInputDevice::activate() {
    _js = open(_device, O_RDONLY);
    if (_js == -1)
        std::cout << "Could not open joystick" << std::endl;

    init_size();
    if (!Correction::set_correction(_correction.c_str(), _js))
        std::cout << "USB Input Device: Could not set correction, using default" << std::endl;

    running = true;
    _readUsbThread = std::thread(&UsbInputDevice::run, this);
    return true;
}

bool UsbInputDevice::deactivate() {
    if (running) {
        running = false;
        if (_readUsbThread.joinable()) { _readUsbThread.join(); }
        close(_js);
    }
    return true;
}

void UsbInputDevice::run() {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    struct timeval timeout;

    while (running) {
        timeout.tv_sec = 0.2; // set timeout to 0.2 sec
        FD_SET(_js, &read_fds);
        if (select(_js + 1, &read_fds, NULL, NULL, &timeout) == 1) {
            if (read_event(_js, &_jsevent) == -1) {
                error_callback("USB Input Device - Check Connection and restart!");
            }
            switch (_jsevent.type) {
            case JS_EVENT_BUTTON:
                button_callback(_jsevent.number, _jsevent.value);
                break;
            case JS_EVENT_AXIS:
                axis_callback(_jsevent.number, scale_value(_jsevent.value,
                    MIN_POSITION_INCREMENTS_USB, MAX_POSITION_INCREMENTS_USB));
                break;
            case (JS_EVENT_AXIS | JS_EVENT_INIT): // Handle initial events similar to normal events
                axis_callback(_jsevent.number, scale_value(_jsevent.value,
                    MIN_POSITION_INCREMENTS_USB, MAX_POSITION_INCREMENTS_USB));
                break;
            case (JS_EVENT_BUTTON | JS_EVENT_INIT):
                button_callback(_jsevent.number, _jsevent.value);
                break;
            default:
                break;
            }
            fflush(stdout);
        }
    }
}

int UsbInputDevice::read_event(int fd, struct js_event *event) {
    ssize_t bytes;
    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    return -1;
}

void UsbInputDevice::init_size() {
    __u8 axes;
    __u8 buttons;
    if (ioctl(_js, JSIOCGAXES, &axes) == -1) {
        _numberOfAxes = 0;
    } else {
        _numberOfAxes = (int)axes;
    }
    if (ioctl(_js, JSIOCGBUTTONS, &buttons) == -1) {
        _numberOfButtons = 0;
    } else {
        _numberOfButtons = (int)buttons;
    }
}

} // namespace tod_input_device

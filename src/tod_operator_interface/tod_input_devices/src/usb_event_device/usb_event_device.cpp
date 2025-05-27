
/**
 * @file usb_event_device.cpp
 * @brief Interface of the usb input device with the usb event functionality
 * @copyright 2025 TUMFTM based on Oliver Hamann's ffcfstress
 **/

#include "tod_input_devices/usb_event_device/usb_event_device.hpp"

UsbEventDevice::UsbEventDevice(const std::string &deviceNamespace)
    : _ok{open_device(deviceNamespace)}, _autocenter_off{false} {
    if (_ok) {
        initialize_device();
    }
}

void UsbEventDevice::set_force_feedback(const double ffValue) {
    create_event(ffValue);
}

void UsbEventDevice::reset() {
    delete_effect();
    initialize_device();
}

bool UsbEventDevice::open_device(const std::string &deviceNamespace) {
    // iterate through event numbers until device could be opened with write permission
    for (int i=0; i < 30; ++i) {
        std::string deviceName = deviceNamespace + std::to_string(i);
        _device_handle = open(deviceName.c_str(), O_RDWR|O_NONBLOCK);
        if (_device_handle >= 0) {
            break;
        }
    }
    if (_device_handle < 0) {
        fprintf(stderr, "ERROR: could not open a device in namespace %s - [%s:%d]\n",
                deviceNamespace.c_str(), __FILE__, __LINE__);
    }
    return (_device_handle >= 0);
}

void UsbEventDevice::initialize_device() {
    if (!_ok) {
        return;
    }

    // Which buttons has the device?
    unsigned char key_bits[1 + KEY_MAX/8/sizeof(unsigned char)];
    memset(key_bits, 0, sizeof(key_bits));
    if (ioctl(_device_handle, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0) {
        fprintf(stderr, "ERROR: can not get key bits (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }

    // Which axes has the device?
    unsigned char abs_bits[1 + ABS_MAX/8/sizeof(unsigned char)];
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        fprintf(stderr, "ERROR: can not get abs bits (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }
    // Check if selected axis is available
    if (!testBit(_axis_code, abs_bits)) {
        fprintf(stderr, "ERROR: selected axis %s not available [%s:%d] (see available ones with fftest)\n",
                axis_names[_axis_index].c_str(), __FILE__, __LINE__);
        exit(1);
    }
    // get axis value range
    struct input_absinfo absinfo;
    if (ioctl(_device_handle, EVIOCGABS(_axis_code), &absinfo) < 0) {
        fprintf(stderr, "ERROR: can not get axis value range (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }
    _axis_min = absinfo.minimum;
    _axis_max = absinfo.maximum;
    if (_axis_min >= _axis_max) {
        fprintf(stderr, "ERROR: bad axis value range (%d,%d) [%s:%d]\n",
                _axis_min, _axis_max, __FILE__, __LINE__);
        exit(1);
    }

    // Switch off auto centering
    struct input_event event;
    if (_autocenter_off) {
        memset(&event, 0, sizeof(event));
        event.type = EV_FF;
        event.code = FF_AUTOCENTER;
        event.value = 0;
        if (write(_device_handle, &event, sizeof(event)) != sizeof(event)) {
            fprintf(stderr, "ERROR: failed to disable auto centering (%s) [%s:%d]\n",
                    strerror(errno), __FILE__, __LINE__);
            exit(1);
        }
    }

    // Now get some information about force feedback
    unsigned char ff_bits[1 + FF_MAX/8/sizeof(unsigned char)];
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        fprintf(stderr, "ERROR: can not get ff bits (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }
    // check if force feedback supported
    if (!testBit(FF_CONSTANT, ff_bits)) {
        fprintf(stderr, "ERROR: device (or driver) has no constant force feedback support [%s:%d]\n",
                __FILE__, __LINE__);
        exit(1);
    }

    // initialize constant force effect
    memset(&_effect, 0, sizeof(_effect));
    _effect.type = FF_CONSTANT;
    _effect.id = -1;
    _effect.trigger.button = 0;
    _effect.trigger.interval = 0;
    _effect.replay.length = 0xffff;
    _effect.replay.delay = 0;
    _effect.u.constant.level = 0;
    _effect.direction = 0xC000;
    _effect.u.constant.envelope.attack_length = 0;
    _effect.u.constant.envelope.attack_level = 0;
    _effect.u.constant.envelope.fade_length = 0;
    _effect.u.constant.envelope.fade_level = 0;
    // upload
    if (ioctl(_device_handle, EVIOCSFF, &_effect) < 0) {
        fprintf(stderr, "ERROR: uploading effect failed (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }
    // start
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = _effect.id;
    event.value = 1;
    if (write(_device_handle, &event, sizeof(event)) != sizeof(event)) {
        fprintf(stderr, "ERROR: starting effect failed (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }
}

// update the device: set force and query joystick position
void UsbEventDevice::create_event(const double force) {
    // set force and upload effect
    double force2set = std::clamp(force, -0.8, 0.8); // -1.0, 1.0);
    _effect.u.constant.level = (int16_t) (force2set*32767.0);
    _effect.direction = 0xC000;
    _effect.u.constant.envelope.attack_level = (int16_t) (force*32767.0); // this one counts!
    _effect.u.constant.envelope.fade_level = (int16_t)(force*32767.0); // only to be safe
    if (ioctl(_device_handle, EVIOCSFF, &_effect) < 0) {
        perror("upload effect");
        // We do not exit here. Indeed, too frequent updates may be refused,
        // but that is not a fatal error
    }
}

void UsbEventDevice::delete_effect() {
    // Delete effect
    if (_effect.id != -1) {
        if (ioctl(_device_handle, EVIOCRMFF, _effect.id) < 0) {
            fprintf(stderr, "ERROR: removing effect failed (%s) [%s:%d]\n",
                    strerror(errno), __FILE__, __LINE__);
            // exit(1);
        }
        _effect.id = -1;
    }
}

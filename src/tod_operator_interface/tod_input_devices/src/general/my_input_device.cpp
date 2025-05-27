
/**
 * @file my_inputs_device.cpp
 * @brief Interface of the usb input device 
 * @copyright 2025 TUMFTM based on Oliver Hamann's ffcfstress
 **/

#include "tod_input_devices/general/my_input_device.hpp"

namespace tod_input_device {

MyInputDevice::MyInputDevice(
    std::function<void(const int, const double)> axisCb, std::function<void(const int, const int)> buttonCb) {
    axis_callback = axisCb;
    button_callback = buttonCb;
}

MyInputDevice::MyInputDevice(
    std::function<void(const int, const double)> axisCb, std::function<void(const int, const int)> buttonCb,
    std::function<void(const std::string&)> errorCb) {
    error_callback = errorCb;
    axis_callback = axisCb;
    button_callback = buttonCb;
}

void MyInputDevice::set_axis_callback(std::function<void(const int, const double)> f) {
    axis_callback = f;
}

void MyInputDevice::set_button_callback(std::function<void(const int, const int)> f) {
    button_callback = f;
}

void MyInputDevice::terminate() {
    deactivate();
}

void MyInputDevice::set_correction(const std::string& correction) {
    _correction = correction;
}

double MyInputDevice::scale_value(int nValue, int nMinInput, int nMaxInput, double dMinOutput, double dMaxOutput) {
    double dValue;
    dValue = dMinOutput + (double)(nValue - nMinInput) * (dMaxOutput - dMinOutput) / (double)(nMaxInput - nMinInput);
    if (dValue > dMaxOutput)
        return dMaxOutput;
    else if (dValue < dMinOutput)
        return dMinOutput;
    else
        return dValue;
}

int MyInputDevice::get_number_of_axes() {
    return _numberOfAxes;
}

int MyInputDevice::get_number_of_buttons() {
    return _numberOfButtons;
}

} // namespace tod_input_device

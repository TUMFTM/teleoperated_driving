// Copyright 2021 Hoffmann
#pragma once
#include <cmath>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <functional>

namespace tod_helper::Files {

inline bool iterate_file(std::string fileName, std::function<void (const std::string & )> callback) { //TODO: Move
    std::ifstream in(fileName.c_str());
    if(!in) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("tod_helper::Files"), "Cannot open the File : " << fileName);
        return false;
    }
    std::string str;
    while (std::getline(in, str)) {
        callback(str);
    }
    in.close();
    return true;
}
}; // namespace tod_helper::Files

/**
 * @file secondary_vehicle_state_component.cpp
 * @brief SecondaryVehicleState component that manages the subscription and the data for SecondaryControl topics for Secondary vehicle state coming from the vehicle 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/secondary_vehicle_state_component.hpp"

namespace tod_gl {

void SecondaryVehicleStateComponent::cb_message(const tod_vehicle_msgs::msg::SecondaryVehicleState::SharedPtr msg) {
    indicator_ = msg->indicator;
    gear_postion_ = msg->gear_position;
    honk_ = msg->honk;
    wiper_ = msg->wiper;
    head_light_ = msg->head_light;
    flash_light_ = msg->flash_light;
}

const std::string& SecondaryVehicleStateComponent::map_states_to_string(const std::unordered_map<int8_t, std::string>& map, 
                                                                        const int8_t& state) const {
    auto iter = map.find(state);
    if (iter != map.end()) {
        return iter->second;
    }
    else {
        return default_string_;
    }
}

}  //  namespace tod_gl
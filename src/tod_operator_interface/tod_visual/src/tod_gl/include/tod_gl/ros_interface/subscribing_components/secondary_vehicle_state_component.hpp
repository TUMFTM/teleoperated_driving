/**
 * @file secondary_vehicle_state_component.hpp
 * @brief SecondaryVehicleState component that manages the subscription and the data for SecondaryControl topics for Secondary vehicle state coming from the vehicle 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include <glm/glm.hpp>

#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"
#include "tod_vehicle_msgs/VehicleEnums.h"

namespace tod_gl {
  
class SecondaryVehicleStateComponent : public SubscribingComponent<tod_vehicle_msgs::msg::SecondaryVehicleState> {
  public:
    explicit SecondaryVehicleStateComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/secondary_vehicle_state"),
        default_string_("UNKOWN"),
        indicator_(eIndicator::INDICATOR_OFF),
        gear_postion_(eGearPosition::GEARPOSITION_PARK),
        honk_(eHonk::HONK_OFF),
        wiper_(eWiper::WIPER_OFF),
        head_light_(eHeadLight::HEADLIGHT_OFF),
        flash_light_(eFlashLight::FLASHLIGHT_OFF)
    {}
    
    int8_t get_indicator() const { return indicator_; };
    int8_t get_gear_position() const { return gear_postion_; };
    int8_t get_honk() const { return honk_; };
    int8_t get_wiper() const { return wiper_; };
    int8_t get_head_light() const { return head_light_; };
    int8_t get_flash_light() const { return flash_light_; };
    
    std::string get_gear_position_string() const { return map_states_to_string(gear_position_map_, gear_postion_); };

  private:
    void cb_message(const tod_vehicle_msgs::msg::SecondaryVehicleState::SharedPtr msg) override;
    const std::string& map_states_to_string (const std::unordered_map<int8_t, std::string>& map, const int8_t& state) const;

    std::unordered_map<int8_t, std::string> gear_position_map_{
        {eGearPosition::GEARPOSITION_PARK,    "P"},
        {eGearPosition::GEARPOSITION_REVERSE, "R"},
        {eGearPosition::GEARPOSITION_NEUTRAL, "N"},
        {eGearPosition::GEARPOSITION_DRIVE,   "D"},
        {eGearPosition::GEARPOSITION_SPORT,   "S"},
        {eGearPosition::GEARPOSITION_HAUL,    "H"}
    };

    std::string default_string_;
    
    int8_t indicator_;
    int8_t gear_postion_;
    int8_t honk_;
    int8_t wiper_;
    int8_t head_light_;
    int8_t flash_light_; 
};

}  //  namespace tod_gl
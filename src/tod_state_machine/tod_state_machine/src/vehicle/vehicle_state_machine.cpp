/**
 * @file vehicle_state_machine.cpp
 * @brief this file defines methods for the state machine
 * @copyright TUM-FTM
 */

#include "tod_state_machine/vehicle/vehicle_state_machine.hpp"
#include "tod_state_machine/vehicle/vehicle_state_machine_structure.hpp"

using namespace tod_vehicle_state_machine;

struct VehicleStateMachine::vehicle_state_machine_ : public boost::sml::sm<tod_vehicle_state_machine::TodVehicleStateMachine>
{
    explicit vehicle_state_machine_(VehicleStateMachine* tod_veh_sm) : boost::sml::sm<tod_vehicle_state_machine::TodVehicleStateMachine>(static_cast<VehicleStateMachine*>(tod_veh_sm)) {};
};

VehicleStateMachine::VehicleStateMachine() : tod_vehicle_state_machine_(std::make_shared<vehicle_state_machine_>(this)) {}

bool VehicleStateMachine::process_connection_requested()
{
    return tod_vehicle_state_machine_->process_event(tod_vehicle_state_machine::events::CONNECT_APPROVED{});
}

bool VehicleStateMachine::process_disconnection_triggered()
{
    return tod_vehicle_state_machine_->process_event(tod_vehicle_state_machine::events::DISCONNECT_CLICKED{});
}
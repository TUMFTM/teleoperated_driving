/**
 * @file operator_state_machine.cpp
 * @brief this file defines methods for the statemachine's event-trigger 
 * @copyright TUM-FTM
 */

#include "tod_state_machine/operator/operator_state_machine.hpp"
#include "tod_state_machine/operator/operator_state_machine_structure.hpp"

using namespace tod_operator_state_machine;

struct StateMachine::operator_state_machine_ : public boost::sml::sm<tod_operator_state_machine::TodOperatorStateMachine> 
{
    explicit operator_state_machine_(StateMachine* tod_sm) : boost::sml::sm<tod_operator_state_machine::TodOperatorStateMachine>(static_cast<StateMachine*>(tod_sm)) {};
};

StateMachine::StateMachine() : tod_operator_state_machine_(std::make_shared<operator_state_machine_>(this)) {}

bool StateMachine::process_connect_clicked()
{
    return tod_operator_state_machine_->process_event(tod_operator_state_machine::events::CONNECT_CLICKED{});
}

bool StateMachine::process_disconnect_clicked()
{
    return tod_operator_state_machine_->process_event(tod_operator_state_machine::events::DISCONNECT_CLICKED{});
}

bool StateMachine::process_start_clicked()
{
    return tod_operator_state_machine_->process_event(tod_operator_state_machine::events::START_CLICKED{});
}

bool StateMachine::process_stop_clicked()
{
    return tod_operator_state_machine_->process_event(tod_operator_state_machine::events::STOP_CLICKED{});
}
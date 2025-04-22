/**
 * @file vehicle_operator_state_machine.hpp
 * @brief this file defines methods used for the state machine
 * @copyright TUM-FTM
 */


#pragma once
#include <memory>
#include <iostream>
#include <vector>
#include <math.h>

namespace tod_vehicle_state_machine {

class VehicleStateMachine
{
    public:
        VehicleStateMachine();

        /**
         * @brief This function is called, as soon as a connection request is done on operator's side
         *        The information is delivered by changing the tod_status to "connected".
         * @param None This function does not take parameters
         * @return bool This function returns the approval for the state transition
         */
        bool process_connection_requested();

        /**
         * @brief This function is called, if the OperatorManager's 'disconnect'-button is clicked to establish the 
         *        connection. The information is delivered by changing the tod_status to "disconnected".
         * @param None This function does not take parameters
         * @return bool This function returns the approval for the state transition 
         */
        bool process_disconnection_triggered();

    private:
        struct vehicle_state_machine_;
        std::shared_ptr<vehicle_state_machine_> tod_vehicle_state_machine_;
};

} // namespace tod_vehicle_state_machine 
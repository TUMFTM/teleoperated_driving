/**
 * @file state_machine.hpp
 * @brief this file defines methods and variables used for the state machine
 * @copyright TUM-FTM
 */

#pragma once
#include <memory>
#include <iostream>
#include <vector>
#include <math.h>

namespace tod_operator_state_machine {

class StateMachine
{
    public:
        StateMachine();

        /**
         * @brief This function is called, if the OperatorManager's 'connect'-button is clicked to establish the 
         *        connection. The information is delivered by the OperatorManagerButton message. If the click-event
         *        is detected, the statemachine's transition is triggered.
         * @param None This function does not take parameters
         * @return bool This function returns the approval for the state transition
         */
        bool process_connect_clicked();

        /**
         * @brief This function is called, if the OperatorManager's 'disconnect'-button is clicked to establish the 
         *        connection. The information is delivered by the OperatorManagerButton message. If the click-event
         *        is detected, the statemachine's transition is triggered.
         * @param None This function does not take parameters
         * @return bool This function returns the approval for the state transition
         */
        bool process_disconnect_clicked();
        
        /**
         * @brief This function is called, if the OperatorManager's 'start'-button is clicked to establish the 
         *        connection. The information is delivered by the OperatorManagerButton message. If the click-event
         *        is detected, the statemachine's transition is triggered.
         * @param None This function does not take parameters
         * @return bool This function returns the approval for the state transition
         */
        bool process_start_clicked();
        
        /**
         * @brief This function is called, if the OperatorManager's 'stop'-button is clicked to establish the 
         *        connection. The information is delivered by the OperatorManagerButton message. If the click-event
         *        is detected, the statemachine's transition is triggered.
         * @param None This function does not take parameters
         * @return bool This function returns the approval for the state transition
         */
        bool process_stop_clicked();

    private:
        struct operator_state_machine_;
        std::shared_ptr<operator_state_machine_> tod_operator_state_machine_;
};

} // namespace tod_operator_state_machine
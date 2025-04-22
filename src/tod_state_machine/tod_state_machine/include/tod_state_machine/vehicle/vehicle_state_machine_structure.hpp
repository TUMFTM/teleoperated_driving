/**
 * @file state_machine_structure.hpp
 * @brief this file initiate the structure of the state machine with boost-sml
 * @copyright TUM-FTM
 */
#include "tod_state_machine/boost/sml.hpp"

namespace sml = boost::sml;

namespace tod_vehicle_state_machine {
    
    // States
    struct states {
        struct Idle{};
        struct Uplink{};
    };

    // Events
    struct events {
        struct CONNECT_APPROVED {};
        struct DISCONNECT_CLICKED {};
    };

    // Actions

    // State Machine
    struct TodVehicleStateMachine {
        auto operator ()() const
        {
            using namespace sml;
            return sml::make_transition_table(
                *state<states::Idle> + event<events::CONNECT_APPROVED> = state<states::Uplink>,
                state<states::Uplink> + event<events::DISCONNECT_CLICKED> = state<states::Idle>
            );
        };
    };
} // namespace tod_vehicle_state_machine
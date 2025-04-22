/**
 * @file state_machine_structure.hpp
 * @brief this file initiate the structure of the state machine with boost-sml
 * @copyright TUM-FTM
 */
#include "tod_state_machine/boost/sml.hpp"

namespace sml = boost::sml;

namespace tod_operator_state_machine {
    
    struct states {
        struct Idle{};
        struct Uplink{};
        struct TeleoperationStarted{};
    };

    struct events {
        struct CONNECT_CLICKED {};
        struct DISCONNECT_CLICKED {};
        struct START_CLICKED {};
        struct STOP_CLICKED {};
    };

    struct TodOperatorStateMachine {
        auto operator ()() const
        {
            using namespace sml;
            return sml::make_transition_table(
                *state<states::Idle> + event<events::CONNECT_CLICKED> = state<states::Uplink>,
                state<states::Uplink> + event<events::START_CLICKED> = state<states::TeleoperationStarted>,
                state<states::Uplink> + event<events::DISCONNECT_CLICKED> = state<states::Idle>,
                state<states::TeleoperationStarted> + event<events::STOP_CLICKED> = state<states::Uplink>,
                state<states::TeleoperationStarted> + event<events::DISCONNECT_CLICKED> = state<states::Idle>
            );
        };
    };

} // namespace tod_operator_state_machine
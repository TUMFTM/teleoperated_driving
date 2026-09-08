# Peanut01 G923 R/N/D Gear Selection Design

## Context

The Peanut01 operator uses a Logitech G923. Its right and left paddles are
mapped to the common `INCREASE_GEAR` and `DECREASE_GEAR` actions. The command
creator currently initializes the gear to `P` and steps through the contiguous
TOD range `P, R, N, D, S`. Peanut01 has no physical shifter and its chassis
protocol supports only `N`, `D`, and `R`, so exposing `P` and `S` on this
operator is misleading.

## Goals

- Initialize the Peanut01 operator gear to `N`.
- Limit G923 paddle selection to `R`, `N`, and `D`.
- Keep the selection at `R` or `D` when the corresponding boundary is reached.
- Preserve the existing rule that gear changes are accepted only while the
  commanded velocity is below `0.01 m/s`.
- Make the allowed range configurable so later changes require a node restart,
  not another rebuild.

## Non-Goals

- Do not change the shared TOD gear message or remove `P` and `S` enum values;
  other supported vehicles may still use them.
- Do not change G923 steering, throttle, brake, lighting, or horn mappings.
- Do not change the F710 configuration.
- Do not enable physical actuation.
- Do not infer a physical `P` state from MCU enable or electronic-brake bits.
  Vehicle feedback semantics are a separate change.

## Design

`CommandCreator` will declare three startup parameters:

| Parameter | Peanut01 value | Meaning |
| --- | ---: | --- |
| `minGearPosition` | `1` | `R` lower boundary |
| `maxGearPosition` | `3` | `D` upper boundary |
| `defaultGearPosition` | `2` | `N` startup and reset state |

The declarations retain the legacy defaults `0`, `4`, and `0` so an existing
non-Peanut01 deployment without the new YAML keys keeps its current behavior.
The Peanut01 deployment explicitly supplies `1`, `3`, and `2`.

The values use the existing TOD enum and therefore require no message or
network-protocol change. `set_gear()` will use member values loaded from these
parameters instead of its current hard-coded `0..4` range. A right-paddle edge
increments the current value up to the configured maximum; a left-paddle edge
decrements it down to the configured minimum. There is no wraparound.

`init_control_messages()` will initialize the gear from
`defaultGearPosition`. It will be called after the parameters are loaded so the
first command published after startup is `N`, not the message's zero-initialized
`P`. The same initialization is used when leaving teleoperation.

At startup, the node will validate that the three values are known TOD gear
values and satisfy:

```text
minGearPosition <= defaultGearPosition <= maxGearPosition
```

Invalid configuration will emit a fatal error and prevent `CommandCreator` from
starting, so an uncertain range can never publish a command. Before processing
a paddle edge, an out-of-range current value will be normalized to the
configured default. This prevents an old or zero-initialized `P` value from
leaking into the next command.

The Peanut01 command-creation YAML will carry the three values. The G923 input
mapping remains unchanged: button `4` is increase gear and button `5` is
decrease gear. Because the command stream can only contain values `1..3`, the
operator command display will only show `R`, `N`, or `D`; generic display
support for other vehicles remains intact.

## Data Flow

```text
G923 right paddle (button 4) -> INCREASE_GEAR -> R -> N -> D -> D
G923 left paddle  (button 5) -> DECREASE_GEAR -> D -> N -> R -> R
CommandCreator startup/reset  -> defaultGearPosition -> N
```

All resulting values continue through the existing secondary-control command,
network receiver, safety gate, and Peanut01 bridge without changing the TOD
wire representation.

## Safety Behavior

- A gear change remains blocked while commanded velocity is at least
  `0.01 m/s`.
- Invalid gear parameters fail closed by preventing command generation.
- The implementation does not turn on `enable_actuation`; deployment and topic
  verification occur with physical output disabled.
- `P` and `S` received from any unrelated or stale external source are outside
  this G923 generator's configured range and are normalized before the next
  paddle-driven selection.

## Verification

Automated tests will cover:

- startup and reset publish `N`;
- right paddle selects `N`, then `D`, and remains at `D`;
- left paddle selects `N`, then `R`, and remains at `R`;
- paddle input is ignored while commanded velocity is at least `0.01 m/s`;
- invalid parameter combinations prevent node startup;
- generated gear values never include `P` or `S` under Peanut01 configuration.

After deployment to the operator, ROS topic verification will confirm that the
secondary-control command starts at `gear_position: 2` and only reports
`1`, `2`, or `3` while the paddles are exercised. The vehicle bridge remains
disabled during this verification.

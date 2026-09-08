# Peanut01 Switchable Control Bridge Design

**Date:** 2026-08-10

## Goal

Connect the validated TOD vehicle command path in ROS domain 7 to the existing
Peanut01 Autoware vehicle interface in ROS domain 0. The bridge is deployable
without enabling physical actuation and can later be armed explicitly under
controlled test conditions.

## Current State

The remote command path is complete through the vehicle safety gate:

```text
operator -> network receiver -> command forwarder -> safety gate
         -> Peanut01DryRunInterface -> debug topic
```

The vehicle already runs `peanut_autoware_vehicle_interface` in domain 0. It is
the sole CAN writer on `can_mingnuo`, uses a 300 ms command timeout, checks MCU
and EPS feedback, merges emergency inputs, and gives the vehicle-local F710
override priority over Autoware commands.

The F710 connected to the vehicle is treated as the local safety-operator
override. The remote G923 is the TOD operator input. These sources are never
blended.

## Architecture

Add a Python dual-domain bridge process to the Peanut01 vehicle launch. It owns
one ROS context in domain 7 and one in domain 0.

The domain 7 side subscribes to:

- `/vehicle/safety/output/primary_control_cmd`
- `/vehicle/safety/output/secondary_control_cmd`
- `/vehicle/statemachine/output/vehicle_status`

The domain 0 side observes:

- `/vehicle/status/velocity_status`
- `/vehicle/status/control_mode`
- `/minguo/emergency_stop`
- `/minguo/teleop_override`

It always publishes converted messages to type-specific debug topics under
`/debug/tod_peanut01/autoware_*`. It creates publishers for
`/control/command/*` only after actuation is explicitly enabled and all arming
conditions pass.

The bridge does not access SocketCAN and does not send CAN frames directly.

## Command Conversion

The TOD primary velocity is a non-negative magnitude. Autoware velocity is
signed, so the bridge combines primary velocity with the latest TOD gear:

- `DRIVE`, `SPORT`, or `HAUL`: positive velocity and Autoware `DRIVE`.
- `REVERSE`: negative velocity and Autoware `REVERSE`.
- `PARK` or `NEUTRAL`: zero velocity and the matching Autoware gear.
- Unknown gear or non-finite input: a stop command and a conversion fault.

`CommandCreator` populates steering-wheel angle, not steering-tire angle. The
bridge divides steering-wheel angle by the configured positive steering ratio
(`16.0` by default) to produce Autoware steering-tire angle. It leaves optional
steering-rate, acceleration, and jerk fields undefined.

TOD indicators map as follows:

- Off: turn indicators disabled, hazard lights disabled.
- Left or right: matching turn indicator, hazard lights disabled.
- Both: turn indicators disabled, hazard lights enabled.

Unsupported horn, wiper, and lighting fields are reported in diagnostics but
are not sent to unrelated interfaces.

## Actuation State Machine

The bridge has four states: `DISABLED`, `ARMING`, `ACTIVE`, and `FAULT`.

`enable_actuation` defaults to `false`. A process or container restart always
starts in `DISABLED`; a persisted value must not restore `ACTIVE` automatically.
While disabled, no real command publishers exist and no control-mode request is
made.

Changing `enable_actuation` to `true` enters `ARMING`. The bridge requires all
of these conditions continuously for one second:

- TOD status is `TELEOPERATION`.
- Primary, secondary, and status messages are no older than 300 ms.
- Vehicle speed is below 0.02 m/s.
- TOD gear is PARK or NEUTRAL and requested speed is zero.
- All command values are finite and within configured conversion bounds.
- `/minguo/emergency_stop` is false.
- `/minguo/teleop_override` is false.
- Domain 0 vehicle feedback is fresh.

During arming, the bridge uses current steering feedback as the initial target
to avoid a steering step. Once the conditions are stable, it requests
Autoware `AUTONOMOUS` mode. Only a successful service response permits the
transition to `ACTIVE`; the existing vehicle interface rejects this request if
its chassis feedback is not ready.

In `ACTIVE`, the bridge publishes converted control, gear, turn-indicator, and
hazard-light commands at a fixed rate. The local F710 override remains higher
priority inside `peanut_autoware_vehicle_interface`.

Command timeout, emergency, stale feedback, invalid data, or a rejected control
mode causes a stop command and a latched `FAULT`. A fault cannot recover
automatically. The operator must set `enable_actuation` to `false`, verify that
the vehicle is stopped, and arm again.

Changing `enable_actuation` to `false` while active publishes a bounded stop
sequence, requests MANUAL mode, destroys all real command publishers, and
returns to `DISABLED`.

## Input Arbitration

The vehicle-local F710 override always has priority. Remote commands are not
mixed with `/cmd_vel`. When the F710 override becomes active, the bridge exits
`ACTIVE` and latches `FAULT`. Releasing the F710 does not automatically restore
remote control.

The remote G923 remains the selected TOD input source. Adding a second remote
input device would require a separate operator-side arbiter; automatic
last-input-wins switching is explicitly excluded.

## Configuration

The initial deployment uses:

```yaml
enable_actuation: false
source_domain_id: 7
target_domain_id: 0
command_timeout_ms: 300
feedback_timeout_ms: 300
arming_duration_ms: 1000
stopped_velocity_mps: 0.02
steering_ratio: 16.0
publish_rate_hz: 20.0
```

The deployed YAML must keep `enable_actuation: false`. Initial verification is
limited to debug topics and state transitions that cannot reach real command
topics.

## Testing

Pure mapping and state-machine tests cover forward, reverse, park, neutral,
indicators, hazards, non-finite values, stale messages, emergency, local F710
override, rejected autonomous-mode requests, disable, and fault reset.

Deployment tests require the default-disabled parameter and verify that the
bridge launch is present. A domain-level integration test confirms that debug
messages are produced while `/control/command/*` has no bridge publisher.

No physical motion test is part of this phase. Enabling actuation requires a
separate review and a vehicle that is lifted or in a closed test area with an
on-site safety operator and a physical emergency stop.

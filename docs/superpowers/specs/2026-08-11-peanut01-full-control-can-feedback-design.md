# Peanut01 Full Control CAN Feedback Design

**Date:** 2026-08-11

## Goal

Enable the remote Logitech G923 to control Peanut01 steering and longitudinal
motion through the existing TOD-to-Autoware bridge. Limit requested speed to
0.05 m/s on the operator, retain R/N/D selection, preserve the vehicle-local
F710 as the higher-priority controller, and verify actuation through live
MCU/EPS CAN feedback.

Implementation and deployment must keep enable_actuation false. Physical
actuation requires separate explicit authorization after software-only
verification and must occur with the vehicle lifted or in a controlled test
area with an on-site emergency-stop operator.

## Current Architecture

The operator-side path is:

    G923 -> InputDevice -> CommandCreator -> TOD network sender

The vehicle-side path is:

    TOD receiver -> SafetyGate -> Peanut01 ControlBridge
      -> Autoware command topics -> peanut_autoware_vehicle_interface
      -> MCU/EPS CAN

The vehicle-local F710 publishes /cmd_vel, /minguo/teleop_override, and
/minguo/emergency_stop. peanut_autoware_vehicle_interface is the sole CAN
writer and prioritizes emergency stop, then F710, then Autoware commands, then
an idle stop.

The current bridge observes velocity, steering, control mode, emergency stop,
and F710 override, but does not parse execution feedback from CAN.
SafetyDriverStatus has no publisher, so the TOD manager currently reports
emergency release, lateral approval, and longitudinal approval as false.

## Scope

This change:

- changes Peanut01 operator maximum velocity from 0.8 m/s to 0.05 m/s;
- does not add a vehicle-side velocity clamp;
- preserves the existing G923 steering mapping and vehicle boundaries;
- parses read-only MCU/EPS frames from /vehicle/can/raw;
- publishes TOD SafetyDriverStatus from verified vehicle feedback;
- adds arming and active-state execution checks to the control bridge;
- keeps F710 B-button override higher priority than remote G923 control;
- keeps VCU 0x18FA0121 feedback optional because it is not currently present.

This change does not modify SocketCAN, make the bridge a CAN writer, add an
automatic return from F710 to G923, or enable physical actuation during
deployment.

## CAN Feedback Module

Add a focused Python module that accepts the JSON payload carried by
std_msgs/msg/String on /vehicle/can/raw. It recognizes:

| CAN ID | Purpose |
| --- | --- |
| 0x1A2 | MCU power, enable, direction, gear, electromagnetic brake |
| 0x1A3 | MCU voltage, current, and motor RPM |
| 0x1A4 | MCU error codes and manual override |
| 0x401 | EPS mode, torque, errors, angle, and centering state |

The parser validates message shape, DLC, CAN ID, MCU checksum, and EPS XOR.
Malformed frames and invalid checksums do not refresh feedback timestamps.
Every status group has an independent monotonic receipt timestamp.

Protocol values follow the deployed DBC:

- MCU feedback gear 0 is N, 1 is D, and 2 is R;
- MCU direction 0 is invalid, 1 is forward, and 2 is reverse;
- EPS modes 0x20 and 0x23 are accepted angle-control modes;
- EPS centering states 0x55 and 0xEE are accepted;
- any non-zero EPS or MCU error field is a fault.

The bridge subscribes to the read-only ROS topic. It continues to have no
SocketCAN access and never sends a CAN frame directly.

## Safety Status Publisher

The domain 7 bridge node publishes
/vehicle/interface/actuation/from_actuation/safety_driver_status using
tod_vehicle_msgs/msg/SafetyDriverStatus.

The fields mean:

    vehicle_emergency_stop_released =
      /minguo/emergency_stop is fresh and false

    vehicle_lat_approved =
      EPS feedback is fresh
      and EPS mode is angle control
      and EPS centering state is valid
      and EPS error fields are zero

    vehicle_long_approved =
      MCU 0x1A2, 0x1A3, and 0x1A4 feedback is fresh
      and MCU reports power-up
      and MCU error fields are zero
      and MCU does not report manual override

MCU enable and electromagnetic-brake state are not pre-arm approvals. A
stopped vehicle intentionally reports MCU disabled and brake locked, so
requiring enabled and unlocked before arming would deadlock. Those values are
checked after commands are issued.

If any required input is stale or invalid, the corresponding approval becomes
false. The absent VCU frame is not used for approval in this phase.

## Operator Speed Limit

For Peanut01, CommandCreator.maxVelocity is 0.05 m/s. Forward commands are in
the range 0 to 0.05 m/s. Reverse commands use a non-negative TOD magnitude that
the vehicle bridge converts to -0.05 to 0 m/s.

The vehicle bridge checks values for finiteness and valid gear conversion but
does not clamp velocity. This is an operational limit, not a vehicle-side
independent safety limit. A stale or incorrectly configured operator can
therefore request a larger value, which is an accepted risk for this phase.

## Arming

Changing enable_actuation from false to true enters ARMING. The following
conditions must remain true for one second:

- TOD is in TELEOPERATION;
- primary, secondary, status, and vehicle feedback are fresh;
- the vehicle is stopped within 0.02 m/s;
- requested speed is zero and requested gear is N;
- command values are finite and convertible;
- /minguo/emergency_stop is fresh and false;
- /minguo/teleop_override is fresh and false;
- lateral and longitudinal approvals are true;
- MCU reports disabled;
- the electromagnetic brake reports locked.

After one stable second, the bridge requests Autoware AUTONOMOUS. A rejected
request latches FAULT. Real publishers are created only after the request is
accepted.

## Active Execution Checks

The bridge compares requested longitudinal state with MCU feedback.

For D or R with a non-zero requested speed:

- MCU enable must become true;
- the electromagnetic brake must become released;
- D requires MCU direction 1 and feedback gear 1;
- R requires MCU direction 2 and feedback gear 2.

For N or a zero requested speed:

- MCU enable must become false;
- the electromagnetic brake must become locked.
- requested N requires feedback gear 0.

A state transition starts a 500 ms confirmation window. Mismatch is tolerated
only inside that window. Missing, stale, or mismatched feedback after 500 ms
latches FAULT.

EPS feedback must remain fresh, in angle-control mode, centered, and free of
reported errors throughout ACTIVE. Emergency stop, invalid commands, stale TOD
data, stale vehicle feedback, leaving autonomous mode, or any MCU/EPS fault
also latches FAULT.

On a fault, the bridge publishes a bounded stop sequence, requests MANUAL,
destroys real publishers, and continues reporting the latched fault. Recovery
requires setting enable_actuation false, confirming the vehicle is stopped,
and arming again.

## F710 Arbitration

The F710 remains independent from the G923. Pressing F710 B makes
/minguo/teleop_override true. peanut_autoware_vehicle_interface immediately
selects F710 /cmd_vel, while the bridge detects the override and latches FAULT.

Releasing B starts the existing F710 smooth-stop behavior and leaves the
vehicle in MANUAL. The bridge never automatically returns control to the G923.
Restoring G923 control requires an explicit disable and re-arm cycle.

## Configuration

The bridge adds:

    can_feedback_topic: /vehicle/can/raw
    can_feedback_timeout_ms: 300
    execution_confirmation_timeout_ms: 500

The deployed vehicle configuration retains:

    enable_actuation: false

The Peanut01 operator configuration uses:

    maxVelocity: 0.05
    minGearPosition: 1
    maxGearPosition: 3
    defaultGearPosition: 2

## Diagnostics

Bridge diagnostics include:

- supervisor state and fault reason;
- emergency, lateral, and longitudinal approval values;
- MCU power, enable, gear, direction, brake, error, and feedback age;
- EPS mode, centering, errors, and feedback age;
- expected execution state and remaining confirmation time;
- F710 override state;
- unsupported secondary controls.

Diagnostics never substitute for the direct feedback checks used by the
supervisor.

## Testing

Pure tests cover:

- valid decoding of captured 0x1A2, 0x1A3, 0x1A4, and 0x401 frames;
- malformed JSON, wrong DLC, checksum or XOR failure, and stale feedback;
- lateral and longitudinal approval derivation;
- one-second arming with MCU disabled and brake locked;
- D and R execution confirmation within 500 ms;
- fault on enable, brake, direction, gear, EPS, or timeout mismatch;
- safe N and zero-speed confirmation;
- F710 override fault and no automatic G923 recovery;
- Peanut01 operator maxVelocity 0.05;
- deployed enable_actuation false.

Deployment verification remains non-actuating:

1. Build and run all tests.
2. Deploy operator and vehicle images with actuation disabled.
3. Confirm G923 commands are limited to 0.05 m/s.
4. Confirm CAN feedback diagnostics match read-only candump samples.
5. Confirm TOD safety status follows MCU, EPS, and emergency feedback.
6. Confirm no real bridge publisher exists on /control/command topics.

Physical motion testing is a separate operation requiring explicit
authorization, a lifted vehicle or controlled test area, and an on-site
emergency-stop operator.

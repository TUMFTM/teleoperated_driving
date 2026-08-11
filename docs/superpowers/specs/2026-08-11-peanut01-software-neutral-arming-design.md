# Peanut01 Software Neutral Arming Design

**Date:** 2026-08-11

## Goal

Allow Peanut01 to pass the disabled-to-arming safety gate when the chassis is
provably receiving a neutral stop command even though MCU status retains the
last D or R direction. Do not relabel or overwrite the real MCU gear feedback.

Deployment and verification keep `enable_actuation: false`. Physical motion
testing remains a separately authorized operation.

## Observed Behavior

With F710 released and `/cmd_vel` equal to zero, the sole CAN writer sends:

    0x1A1#29000000000000D6

This command means MCU disabled, command gear N, electromagnetic brake locked,
and target RPM zero. At the same time the MCU status frame can remain:

    0x1A2#855A80050004E0B7

The status frame reports MCU disabled and brake locked but feedback gear D.
The same behavior can retain R after reverse operation. Therefore an explicit
F710 N button would not solve the arming deadlock: the neutral command is
already being sent, but status does not transition to feedback gear 0.

## Architecture

Extend the existing read-only CAN feedback module to decode the outgoing
`0x1A1` command observed on `/vehicle/can/raw`. The bridge remains a ROS-only
observer and never writes SocketCAN.

The parser validates the standard-frame JSON envelope, DLC, MCU checksum, and
command fields. It records an independent monotonic timestamp for the latest
valid command frame and exposes:

- command enable;
- command mode;
- command gear;
- command electromagnetic-brake mode;
- command RPM;
- command-frame age.

A software-neutral observation is true only when a fresh command frame has:

    enable = false
    mode = 1
    gear = 2
    electromagnetic brake = locked
    RPM = 0

## Arming Rules

Changing `enable_actuation` from false to true still enters ARMING. The
existing one-second stable arming period remains unchanged.

During that full second, all existing checks remain required except the strict
`MCU feedback gear == N` check. It is replaced by a fresh software-neutral
observation. The complete longitudinal pre-arm evidence is:

- operator requests N and zero speed;
- vehicle speed is within the existing stopped threshold;
- F710 override is fresh and false;
- emergency, lateral, and longitudinal approvals are true;
- MCU status is fresh, powered, disabled, and brake locked;
- the outgoing `0x1A1` command is fresh and is the exact neutral-stop semantic;
- MCU and EPS errors remain clear.

Feedback gear remains diagnostic truth. A retained D or R value is accepted
only during disabled ARMING while all software-neutral evidence is present.
It is never published as N and is never accepted as active execution proof.

## Active Rules

ACTIVE behavior remains strict:

- D motion requires MCU enabled, brake released, direction forward, and
  feedback gear D;
- R motion requires MCU enabled, brake released, direction reverse, and
  feedback gear R;
- zero-speed stop requires MCU disabled and brake locked;
- requested N additionally requires a fresh neutral-stop `0x1A1` command.

The active N state no longer requires MCU feedback gear 0 because the observed
chassis does not provide it. Any stale or non-neutral command frame starts the
existing execution confirmation window and then faults if not corrected.

## Failure Handling

Malformed, stale, checksum-invalid, or semantically non-neutral `0x1A1` frames
cannot satisfy software neutral. A failed condition resets the one-second
arming timer. In ACTIVE it follows the existing 500 ms transition window and
then latches FAULT.

F710 B override, emergency stop, stale MCU/EPS status, motion while requesting
N, or loss of brake lock continue to fault immediately under existing rules.

## Diagnostics

Bridge diagnostics add command enable, mode, gear, brake, RPM, command age, and
`software_neutral` fields. They continue to show the independent MCU feedback
gear so operators can see values such as:

    software_neutral=true
    command_gear=2
    feedback_gear=1

This is an explicit command-versus-feedback distinction, not fabricated gear
feedback.

## Testing

Tests cover:

- valid decoding and checksum validation of the captured neutral `0x1A1` frame;
- rejection of malformed, stale, moving, enabled, released-brake, and non-N
  command frames;
- one-second arming with software neutral and retained D or R feedback;
- refusal to arm without a fresh neutral command;
- active N confirmation from command plus disabled/brake-locked status;
- unchanged strict D/R active execution checks;
- diagnostics exposing command and feedback gear separately;
- deployed `enable_actuation: false`.

## Deployment

Build and deploy only the vehicle image because the change is in the vehicle
bridge. Inspect the candidate image, recreate only `tod_vehicle`, and verify
with actuation disabled that software-neutral diagnostics follow the live
`0x1A1` frame. Do not set `enable_actuation` true or perform movement testing
as part of this change.

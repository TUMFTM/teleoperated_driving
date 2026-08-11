# Peanut01 Execution Confirmation Timeout Design

## Goal

Increase the Peanut01 control bridge execution-feedback confirmation timeout
from 500 ms to 1000 ms. This gives the chassis more time to complete drive,
reverse, and stop transitions before the supervisor latches a feedback mismatch
fault.

## Scope

- Set `execution_confirmation_timeout_ms` to `1000` in the Peanut01 ROS
  parameter configuration.
- Set the bridge parameter fallback to `1000` ms.
- Set the supervisor fallback to `1_000_000_000` ns.
- Update contract and supervisor tests to enforce the unified value and timeout
  boundary.

The stopped-speed threshold remains `0.02 m/s`. Emergency-stop, lateral and
longitudinal approval, command freshness, feedback freshness, autonomous-mode,
F710 override, execution feedback checks, and fault latching remain unchanged.

## Behavior

When the expected MCU execution feedback does not match the requested state,
the supervisor waits up to 1000 ms. A matching response within that window
keeps actuation active. A mismatch beyond 1000 ms enters the existing latched
`FAULT` state and removes the real command publishers.

## Verification

- A contract test checks the YAML and bridge fallback are both 1000 ms.
- Supervisor tests prove a mismatch is tolerated before the 1000 ms boundary
  and faults after the boundary.
- The complete repository test suite and Python compilation must pass.
- After image deployment, the container must start with
  `enable_actuation=false`; runtime diagnostics must report `DISABLED` and the
  real control topics must have zero publishers.

No physical movement test or runtime actuation enable is part of this change.

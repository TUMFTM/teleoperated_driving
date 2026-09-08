# Peanut01 Teleoperation Speed Limit Design

**Date:** 2026-08-07

## Goal

Limit Peanut01 teleoperation speed to `0.8 m/s` in both forward and reverse,
with independent enforcement at the operator and vehicle sides.

## Current State

The operator-side `tod_command_creation` configuration permits up to
`10.0 m/s`. The vehicle-side safety gate only applies `warning_velocity` when
topic monitoring reports a warning, so it does not enforce a normal-state
speed ceiling. The F710 reference configuration uses `0.8 m/s` as the desired
common operating limit.

## Design

### Operator-side limit

Set `maxVelocity` to `0.8` in the deployed `tod_command_creation` parameter
file and its package-owned default configuration. Correct the deployed YAML
node selector so the parameter applies to
`/operator/direct_control/CommandCreator` rather than relying on the C++
fallback.

`CommandCreator` represents reverse motion through the gear command while the
primary velocity remains a non-negative magnitude. A single `0.8 m/s` limit
therefore applies equally to forward and reverse.

### Vehicle-side limit

Add a `max_velocity` parameter to `tod_safety_gate`, configured as `0.8 m/s`
for deployment. Every primary control command in teleoperation mode is capped
to this limit before publication, including commands received while topic
monitoring is healthy.

Existing monitoring behavior remains intact:

- `STATE_OK`: publish with the normal `max_velocity` cap.
- `STATE_WARN`: publish with the lower of `max_velocity` and
  `warning_velocity`.
- `STATE_NOT_RECEIVED` or `STATE_ERROR`: set velocity to zero.
- Non-teleoperation modes: do not publish control commands.

The speed field is a non-negative magnitude in this control path, so the
vehicle cap does not alter gear or steering behavior.

## Configuration

The deployed values are:

```yaml
maxVelocity: 0.8
max_velocity: 0.8
warning_velocity: 0.8
```

Acceleration, deceleration, steering limits, command timeouts, gear handling,
and emergency-stop behavior are outside this change.

## Testing

Add focused safety-gate tests for normal-state pass-through below the limit,
normal-state clamping above the limit, warning-state clamping, and error-state
stopping. Add deployment configuration tests that verify both operator and
vehicle limits are `0.8` and that the operator YAML selector matches the
launched node.

Run the focused tests first, followed by the repository test suite applicable
to deployment configuration and the affected ROS packages.

## Deployment Notes

Both operator and vehicle images must be rebuilt or updated together so the
two independent limits are active. Runtime verification should inspect the
effective ROS parameters on both nodes before any motion test. The first
vehicle test must retain the existing emergency-stop and lifted-wheel safety
procedure.

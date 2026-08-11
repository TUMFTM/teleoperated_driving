# Peanut01 Execution Threshold Alignment Design

**Date:** 2026-08-11

## Goal

Prevent false D/R execution faults when the G923 produces a small nonzero
velocity that the chassis interface still classifies as stopped.

## Confirmed Mismatch

The control supervisor currently treats any requested velocity with an absolute
value greater than `1e-9 m/s` as motion. The Peanut01 chassis interface uses
`stop_velocity_epsilon_mps = 0.02 m/s`; commands at or below that threshold
produce the safe neutral-stop CAN frame instead of a drive or reverse frame.

Therefore a request such as `0.01 m/s` makes the supervisor expect drive while
the chassis interface intentionally sends N, brake lock, and zero RPM. The
existing 500 ms execution window then faults even though both components are
behaving according to their own thresholds.

## Design

Change only `Supervisor._expected_execution()` so its motion decision uses the
existing `Parameters.stopped_velocity_mps` value:

```python
moving = abs(snapshot.requested_velocity_mps) > self.params.stopped_velocity_mps
```

With the deployed value `0.02`:

- `-0.02 <= requested_velocity_mps <= 0.02` is stopped;
- a D or R request inside that range expects disabled MCU plus brake lock;
- values above `0.02` in D expect strict forward execution feedback;
- values below `-0.02` in R expect strict reverse execution feedback.

The comparison is intentionally strict so the exact boundary remains stopped,
matching the chassis interface implementation.

## Unchanged Behavior

- The 500 ms execution confirmation window remains unchanged.
- D still requires MCU enabled, brake released, forward direction, and D
  feedback gear.
- R still requires MCU enabled, brake released, reverse direction, and R
  feedback gear.
- N still requires fresh software-neutral command evidence, disabled MCU, and
  brake lock.
- F710 priority, emergency behavior, arming duration, and fault latching remain
  unchanged.
- `enable_actuation` remains `false` during build and deployment verification.

## Testing

Supervisor tests will prove that:

- D requests at `0.01` and exactly `0.02 m/s` remain stopped and do not fault;
- R requests at `-0.01` and exactly `-0.02 m/s` remain stopped and do not fault;
- requests immediately outside the boundary still enter strict D/R execution
  confirmation;
- existing D/R mismatch, neutral, safety, and fault tests continue to pass.

Deployment will rebuild only the vehicle image, recreate only the vehicle
container, and verify `enable_actuation=false`. No physical motion test is part
of this change.

# Peanut01 MCU Direction Mapping Design

## Goal

Align the control supervisor's D/R execution checks with the direction codes
observed from the Peanut01 MCU so valid drive and reverse execution do not
produce false latched faults.

## Evidence

Synchronized G923, Autoware command, CAN command, and MCU feedback captures
showed the following execution states:

| ToD gear | Vehicle motion | CAN command | MCU enabled | MCU brake locked | MCU direction | MCU gear |
| --- | --- | --- | --- | --- | --- | --- |
| D (`3`) | Forward | enabled, gear `3`, nonzero RPM | `true` | `false` | `2` | `1` |
| R (`1`) | Reverse | enabled, gear `1`, nonzero RPM | `true` | `false` | `1` | `2` |

In both cases the MCU responded within approximately 15 ms. The supervisor
currently expects the opposite direction codes and therefore faults after the
500 ms execution confirmation window.

## Design

Change only `Supervisor._execution_matches`:

- D execution requires `mcu_direction == 2` and `mcu_gear == 1`.
- R execution requires `mcu_direction == 1` and `mcu_gear == 2`.

The existing requirements for MCU enable, brake release, command freshness,
safety approvals, emergency release, and autonomous mode remain unchanged.
Stopped behavior, the `0.02 m/s` stopped threshold, the 500 ms confirmation
timeout, and fault latching also remain unchanged.

## Testing

Add focused supervisor tests proving:

- D remains active when feedback is enabled, brake-released, direction `2`,
  and gear `1`.
- D faults after the timeout when the obsolete direction `1` is reported.
- R remains active when feedback is enabled, brake-released, direction `1`,
  and gear `2`.
- R faults after the timeout when the obsolete direction `2` is reported.

Run the focused supervisor suite, the full repository suite, Python compile
verification, and `git diff --check`.

## Deployment

Deploy only the updated supervisor module to the vehicle service. Rebuild and
recreate only `tod_vehicle`. The persistent configuration must remain
`enable_actuation: false`; no physical movement test is part of deployment.

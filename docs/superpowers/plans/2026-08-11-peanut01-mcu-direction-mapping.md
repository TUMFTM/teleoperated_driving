# Peanut01 MCU Direction Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align D/R execution confirmation with the direction codes measured from the Peanut01 MCU.

**Architecture:** Keep the existing supervisor state machine and change only the two direction comparisons in `Supervisor._execution_matches`. Preserve all enable, brake, gear, timeout, stopped-threshold, and fault-latching behavior.

**Tech Stack:** Python 3 dataclasses, pytest, ROS 2 vehicle deployment.

---

### Task 1: Correct D/R execution feedback matching

**Files:**
- Modify: `tests/test_peanut01_control_supervisor.py`
- Modify: `work/peanut01_control_supervisor.py:207`

- [ ] **Step 1: Write failing D/R direction tests**

Add tests that model the exact MCU states measured during the synchronized CAN captures:

```python
def test_drive_execution_accepts_measured_mcu_direction():
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=0.05, tod_gear=3)

    assert supervisor.step(requested).state is State.ACTIVE
    confirmed = fresh_late_snapshot(
        requested,
        elapsed_ns=400_000_000,
        mcu_enabled=True,
        mcu_brake_locked=False,
        mcu_direction=2,
        mcu_gear=1,
    )

    assert supervisor.step(confirmed).state is State.ACTIVE


def test_drive_execution_rejects_obsolete_direction_after_timeout():
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=0.05, tod_gear=3)
    supervisor.step(requested)

    decision = supervisor.step(
        fresh_late_snapshot(
            requested,
            mcu_enabled=True,
            mcu_brake_locked=False,
            mcu_direction=1,
            mcu_gear=1,
        )
    )

    assert decision.state is State.FAULT
    assert decision.publish_stop


def test_reverse_execution_accepts_measured_mcu_direction():
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=-0.05, tod_gear=1)

    assert supervisor.step(requested).state is State.ACTIVE
    confirmed = fresh_late_snapshot(
        requested,
        elapsed_ns=400_000_000,
        mcu_enabled=True,
        mcu_brake_locked=False,
        mcu_direction=1,
        mcu_gear=2,
    )

    assert supervisor.step(confirmed).state is State.ACTIVE


def test_reverse_execution_rejects_obsolete_direction_after_timeout():
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=-0.05, tod_gear=1)
    supervisor.step(requested)

    decision = supervisor.step(
        fresh_late_snapshot(
            requested,
            mcu_enabled=True,
            mcu_brake_locked=False,
            mcu_direction=2,
            mcu_gear=2,
        )
    )

    assert decision.state is State.FAULT
    assert decision.publish_stop
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run: `python -m pytest tests/test_peanut01_control_supervisor.py -q`

Expected: the measured-direction acceptance tests fail under the old mapping, and the obsolete-direction rejection tests fail because the old mapping accepts those states.

- [ ] **Step 3: Implement the measured direction mapping**

Update only the direction comparisons:

```python
if self._execution_expected == "drive":
    return (
        snapshot.mcu_enabled
        and not snapshot.mcu_brake_locked
        and snapshot.mcu_direction == 2
        and snapshot.mcu_gear == 1
    )
if self._execution_expected == "reverse":
    return (
        snapshot.mcu_enabled
        and not snapshot.mcu_brake_locked
        and snapshot.mcu_direction == 1
        and snapshot.mcu_gear == 2
    )
```

- [ ] **Step 4: Run focused and full verification**

Run: `python -m pytest tests/test_peanut01_control_supervisor.py -q`

Expected: all supervisor tests pass.

Run: `python -m pytest -q`

Expected: all repository tests pass with zero failures.

Run: `python -m compileall -q work`

Expected: exit code 0 with no output.

Run: `git diff --check`

Expected: exit code 0 with no output.

- [ ] **Step 5: Commit the implementation**

```bash
git add tests/test_peanut01_control_supervisor.py work/peanut01_control_supervisor.py
git commit -m "fix: align Peanut01 MCU direction feedback"
```

### Task 2: Deploy with actuation disabled

**Files:**
- Deploy: `work/peanut01_control_supervisor.py`
- Verify only: `config/config/package_config/tod_peanut01_interface/params.yaml`

- [ ] **Step 1: Verify the persistent gate remains disabled**

Run: `python -m pytest tests/test_peanut01_control_bridge.py::Peanut01ControlBridgeContractTest::test_shared_parameters_default_actuation_to_disabled -q`

Expected: PASS.

- [ ] **Step 2: Build and recreate only the vehicle service**

Confirm remote `enable_actuation: false`, synchronize the committed supervisor module, build `tod_vehicle`, inspect the candidate image, and recreate only `tod_vehicle`. Do not modify or restart the operator service.

- [ ] **Step 3: Perform read-only runtime verification**

Confirm the container is running with restart count 0, diagnostics report `DISABLED`, and all four real control topics have publisher count 0. Do not perform a physical movement test.

- [ ] **Step 4: Push the verified commits**

Run: `git push origin codex/peanut01-switchable-control-bridge`

Expected: the existing PR updates and the local branch matches its remote tracking branch.

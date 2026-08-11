# Peanut01 Execution Threshold Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the control supervisor and Peanut01 chassis interface use the same `0.02 m/s` stopped threshold so small G923 pedal inputs do not cause false D/R execution faults.

**Architecture:** Keep the existing `Parameters.stopped_velocity_mps` value as the single supervisor threshold. Change only execution classification; all CAN evidence, safety gates, feedback checks, timeouts, and fault behavior remain unchanged.

**Tech Stack:** Python 3 dataclasses, pytest, ROS 2 vehicle deployment.

---

### Task 1: Align execution motion classification

**Files:**
- Modify: `tests/test_peanut01_control_supervisor.py`
- Modify: `work/peanut01_control_supervisor.py:199`

- [ ] **Step 1: Write the failing stopped-boundary tests**

Add tests proving D/R requests at and inside `0.02 m/s` remain stopped beyond the existing 500 ms execution window, while requests immediately outside the boundary still require strict execution feedback:

```python
@pytest.mark.parametrize(
    ("velocity_mps", "gear"),
    ((0.01, 3), (0.02, 3), (-0.01, 1), (-0.02, 1)),
)
def test_execution_treats_velocity_at_or_below_stopped_threshold_as_stopped(
    velocity_mps, gear
):
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=velocity_mps, tod_gear=gear)

    assert supervisor.step(requested).state is State.ACTIVE
    decision = supervisor.step(fresh_late_snapshot(requested))

    assert decision.state is State.ACTIVE
    assert decision.publish_commands


@pytest.mark.parametrize(
    ("velocity_mps", "gear"),
    ((0.020001, 3), (-0.020001, 1)),
)
def test_execution_requires_drive_feedback_outside_stopped_threshold(
    velocity_mps, gear
):
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=velocity_mps, tod_gear=gear)
    supervisor.step(requested)

    decision = supervisor.step(fresh_late_snapshot(requested))

    assert decision.state is State.FAULT
    assert decision.publish_stop
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run: `python -m pytest tests/test_peanut01_control_supervisor.py -q`

Expected: the four at-or-inside cases fail because the current `1e-9` comparison classifies them as D/R motion; outside-boundary cases pass.

- [ ] **Step 3: Implement the minimal threshold change**

In `Supervisor._expected_execution`, replace the epsilon literal with the configured stopped threshold:

```python
moving = (
    abs(snapshot.requested_velocity_mps) > self.params.stopped_velocity_mps
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
git commit -m "fix: align Peanut01 stopped execution threshold"
```

### Task 2: Deploy with actuation disabled

**Files:**
- Deploy: `work/peanut01_control_supervisor.py`
- Verify only: `config/config/package_config/tod_peanut01_interface/params.yaml`

- [ ] **Step 1: Verify the persistent gate remains disabled**

Run: `python -m pytest tests/test_peanut01_control_bridge.py::Peanut01ControlBridgeContractTest::test_shared_parameters_default_actuation_to_disabled -q`

Expected: PASS.

- [ ] **Step 2: Build and recreate only the vehicle service**

Synchronize the committed supervisor module to the existing vehicle deployment repository, build `tod_vehicle`, inspect the candidate image, and recreate only `tod_vehicle`. Do not modify or restart the operator service.

- [ ] **Step 3: Perform read-only runtime verification**

Confirm the new vehicle container is running with restart count 0, `enable_actuation=false`, real control topic publisher counts are 0, and diagnostics report `DISABLED`. Do not perform a physical movement test.

- [ ] **Step 4: Push the verified commits**

Run: `git push origin codex/peanut01-switchable-control-bridge`

Expected: existing PR 11 updates successfully and the local branch matches its remote tracking branch.

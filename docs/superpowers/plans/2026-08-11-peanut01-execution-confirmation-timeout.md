# Peanut01 Execution Confirmation Timeout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Increase the Peanut01 execution-feedback confirmation timeout from 500 ms to 1000 ms without changing any other safety behavior.

**Architecture:** Keep the existing supervisor state machine and parameter flow. Unify the ROS configuration, bridge fallback, and supervisor fallback at 1000 ms, with tests that prove the pre-timeout and post-timeout boundaries.

**Tech Stack:** Python 3 dataclasses, pytest, ROS 2 parameters, Docker Compose vehicle deployment.

---

### Task 1: Lock the one-second timeout behavior with failing tests

**Files:**
- Modify: `tests/test_peanut01_control_bridge.py`
- Modify: `tests/test_peanut01_control_supervisor.py`

- [ ] **Step 1: Change the configuration contract expectation**

Update the shared-parameter assertion:

```python
self.assertEqual(1000, node["execution_confirmation_timeout_ms"])
```

- [ ] **Step 2: Add supervisor default and boundary tests**

Add:

```python
def test_default_execution_confirmation_timeout_is_one_second():
    assert Parameters().execution_confirmation_timeout_ns == 1_000_000_000


def test_execution_mismatch_is_tolerated_before_one_second_timeout():
    supervisor = make_active_supervisor()
    requested = active_snapshot(requested_velocity_mps=0.05, tod_gear=3)
    supervisor.step(requested)

    decision = supervisor.step(
        fresh_late_snapshot(requested, elapsed_ns=999_000_000)
    )

    assert decision.state is State.ACTIVE
    assert decision.publish_commands
```

Change the late-snapshot helper default so rejection tests exercise the new post-timeout boundary:

```python
def fresh_late_snapshot(snapshot, elapsed_ns=1_000_000_001, **changes):
```

- [ ] **Step 3: Run focused tests and confirm RED**

Run:

```powershell
python -m pytest tests/test_peanut01_control_bridge.py::Peanut01ControlBridgeContractTest::test_shared_parameters_default_actuation_to_disabled tests/test_peanut01_control_supervisor.py -q
```

Expected: failures show the configured and supervisor default values are still 500 ms, and the 999 ms mismatch faults under the old timeout.

### Task 2: Unify production defaults at 1000 ms

**Files:**
- Modify: `config/config/package_config/tod_peanut01_interface/params.yaml`
- Modify: `work/peanut01_control_bridge.py`
- Modify: `work/peanut01_control_supervisor.py`

- [ ] **Step 1: Update all three defaults**

Set the YAML value and bridge fallback to `1000`, and set the supervisor fallback to `1_000_000_000` ns:

```yaml
execution_confirmation_timeout_ms: 1000
```

```python
source_node.declare_parameter("execution_confirmation_timeout_ms", 1000).value
```

```python
execution_confirmation_timeout_ns: int = 1_000_000_000
```

- [ ] **Step 2: Run focused tests and confirm GREEN**

Run the focused command from Task 1. Expected: all selected tests pass.

- [ ] **Step 3: Run full local verification**

Run:

```powershell
python -m pytest -q
python -m compileall -q work
git diff --check
```

Expected: all tests pass and both checks exit with code 0.

- [ ] **Step 4: Commit the implementation**

```powershell
git add config/config/package_config/tod_peanut01_interface/params.yaml work/peanut01_control_bridge.py work/peanut01_control_supervisor.py tests/test_peanut01_control_bridge.py tests/test_peanut01_control_supervisor.py
git commit -m "fix: allow one second for Peanut01 execution feedback"
```

### Task 3: Deploy with real actuation disabled

**Files:**
- Deploy the committed configuration and Python modules to `/home/nvidia/teleoperated_driving_vehicle`.

- [ ] **Step 1: Disable the existing runtime latch before deployment**

Set the current runtime `enable_actuation` parameter to `false` and confirm the control bridge reports `DISABLED` before replacing the container.

- [ ] **Step 2: Synchronize, build, and recreate only the vehicle service**

Synchronize the committed files, build the `tod_vehicle` image, inspect the candidate image for the 1000 ms values, and recreate only `tod_vehicle`. Do not restart or modify the operator service.

- [ ] **Step 3: Perform read-only runtime verification**

Confirm the container is running with restart count 0, `enable_actuation` is false, diagnostics report `DISABLED`, and all four real control topics have publisher count 0. Do not enable actuation or perform a physical movement test.

- [ ] **Step 4: Push the verified commits**

```powershell
git push origin codex/peanut01-switchable-control-bridge
```

Expected: the existing PR contains the design, plan, tests, and implementation.

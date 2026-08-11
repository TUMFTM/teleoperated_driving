# Peanut01 Software Neutral Arming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let Peanut01 arm from a verified disabled, braked, zero-RPM `0x1A1` neutral command while preserving the MCU's retained D/R feedback as independent diagnostic truth.

**Architecture:** Extend `CanFeedbackTracker` with an independently timestamped decoded MCU command and a freshness-derived `software_neutral` result. Pass that result into the pure supervisor so ARMING and ACTIVE neutral checks use command evidence, while D/R execution checks continue to use strict MCU feedback. Expose both command and feedback values in bridge diagnostics without changing actuation defaults or CAN ownership.

**Tech Stack:** Python 3 dataclasses, ROS 2 `rclpy`, `diagnostic_msgs`, pytest, Docker Compose deployment.

---

### Task 1: Decode the MCU command and derive software neutral

**Files:**
- Modify: `tests/test_peanut01_can_feedback.py`
- Modify: `work/peanut01_can_feedback.py`

- [ ] **Step 1: Write failing command-decoding tests**

Add captured `0x1A1` data to `VALID_FRAMES` and tests that require a `command` object, an independent `mcu_command` age, and `software_neutral` to be true only for a fresh exact neutral-stop command:

```python
VALID_FRAMES = {
    0x1A1: "29000000000000D6",
    0x1A2: "891040050005011B",
    0x1A3: "01F4000A0064009C",
    0x1A4: "00000000000000FF",
    0x401: "20800004005500F1",
}

def test_decodes_captured_neutral_stop_command():
    tracker = populated_tracker()
    status = tracker.snapshot(NOW_NS)
    assert not status.command.enabled
    assert status.command.mode == 1
    assert status.command.gear == 2
    assert status.command.brake_mode == 1
    assert status.command.motor_rpm == 0
    assert status.software_neutral
    assert tracker.ages_ns(NOW_NS + 5)["mcu_command"] == 5

@pytest.mark.parametrize(
    "first_seven",
    (
        bytes.fromhex("A9000000000000"),
        bytes.fromhex("09000000000000"),
        bytes.fromhex("25000000000000"),
        bytes.fromhex("28000000000000"),
        bytes.fromhex("29000001000000"),
    ),
)
def test_software_neutral_rejects_enabled_wrong_mode_gear_brake_and_rpm(first_seven):
    payload = valid_payload(0x1A1, first_seven)
    status = populated_tracker({0x1A1: payload}).snapshot(NOW_NS)
    assert not status.software_neutral

def test_software_neutral_requires_a_fresh_valid_command():
    tracker = populated_tracker()
    assert not tracker.snapshot(NOW_NS + 300_000_001).software_neutral
    assert not tracker.update_json(raw_frame(0x1A1, "29000000000000D7"), NOW_NS + 1)
    assert tracker.snapshot(NOW_NS + 1).command.stamp_ns == NOW_NS
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run: `python -m pytest tests/test_peanut01_can_feedback.py -q`

Expected: FAIL because `0x1A1` is unrecognized and `FeedbackSnapshot` has no `command` or `software_neutral` fields.

- [ ] **Step 3: Implement the minimal command model and parser**

In `work/peanut01_can_feedback.py`, add `0x1A1` to recognized IDs, define the command model, keep it separately in the tracker, decode it, and derive freshness-based neutral semantics:

```python
@dataclasses.dataclass(frozen=True)
class McuCommand:
    enabled: bool = False
    mode: int = 0
    gear: int = 0
    brake_mode: int = 0
    motor_rpm: int = 0
    stamp_ns: int = 0

@dataclasses.dataclass(frozen=True)
class FeedbackSnapshot:
    mcu: McuStatus
    eps: EpsStatus
    command: McuCommand
    software_neutral: bool = False
    emergency_released: bool = False
    lateral_approved: bool = False
    longitudinal_approved: bool = False
    last_reject_reason: str = ""
```

Decode byte 0 as enable bit `0x80`, mode bits `(byte >> 5) & 0x03`, gear bits `(byte >> 2) & 0x03`, and electromagnetic-brake mode bits `byte & 0x03`; decode RPM from bytes 3-4 as big-endian. Accept software neutral only when the command timestamp is fresh and the decoded tuple equals `(False, 1, 2, 1, 0)`.

- [ ] **Step 4: Run parser tests and confirm GREEN**

Run: `python -m pytest tests/test_peanut01_can_feedback.py -q`

Expected: all parser tests pass, including checksum rejection and independent command freshness.

- [ ] **Step 5: Commit parser behavior**

```bash
git add tests/test_peanut01_can_feedback.py work/peanut01_can_feedback.py
git commit -m "feat: decode Peanut01 software neutral command"
```

### Task 2: Use software neutral in ARMING and ACTIVE neutral execution

**Files:**
- Modify: `tests/test_peanut01_control_supervisor.py`
- Modify: `work/peanut01_control_supervisor.py`

- [ ] **Step 1: Write failing supervisor tests**

Add `software_neutral=True` to `ready_snapshot()`. Replace the old “MCU not neutral” ARMING mutation with `software_neutral=False`, and explicitly verify retained D and R feedback can arm:

```python
@pytest.mark.parametrize("retained_gear", (1, 2))
def test_arming_accepts_software_neutral_with_retained_feedback_gear(retained_gear):
    supervisor = Supervisor(Parameters())
    supervisor.request_enable(True)
    first = dataclasses.replace(ready_snapshot(), mcu_gear=retained_gear)
    assert not supervisor.step(first).request_autonomous
    stable = dataclasses.replace(ready_snapshot(2_000_000_000), mcu_gear=retained_gear)
    assert supervisor.step(stable).request_autonomous

def test_arming_rejects_missing_software_neutral_even_with_feedback_neutral():
    supervisor = Supervisor(Parameters())
    supervisor.request_enable(True)
    snapshot = dataclasses.replace(ready_snapshot(), software_neutral=False)
    assert not supervisor.step(snapshot).request_autonomous
```

Update neutral ACTIVE tests to retain `mcu_gear=1` while requiring `software_neutral=True`, then cover stale/non-neutral evidence through the existing 500 ms mismatch window:

```python
def test_neutral_execution_accepts_software_neutral_with_retained_drive_feedback():
    supervisor = make_active_supervisor()
    neutral = active_snapshot(mcu_gear=1, software_neutral=True)
    assert supervisor.step(neutral).state is State.ACTIVE
    confirmed = fresh_late_snapshot(neutral, elapsed_ns=400_000_000)
    assert supervisor.step(confirmed).state is State.ACTIVE

def test_neutral_execution_missing_software_neutral_faults_after_timeout():
    supervisor = make_active_supervisor()
    neutral = active_snapshot(mcu_gear=1, software_neutral=False)
    assert supervisor.step(neutral).state is State.ACTIVE
    decision = supervisor.step(fresh_late_snapshot(neutral))
    assert decision.state is State.FAULT
    assert decision.publish_stop
```

- [ ] **Step 2: Run supervisor tests and confirm RED**

Run: `python -m pytest tests/test_peanut01_control_supervisor.py -q`

Expected: FAIL because `InputSnapshot` does not accept `software_neutral`, ARMING still requires `mcu_gear == 0`, and ACTIVE neutral still trusts feedback gear.

- [ ] **Step 3: Implement the supervisor rule changes**

Add `software_neutral: bool` to `InputSnapshot`. In `input_ready`, retain all stopped, approval, MCU power, disabled, and brake checks, but replace `snapshot.mcu_gear == 0` with `snapshot.software_neutral`. In `_execution_matches`, keep D/R branches unchanged and use:

```python
matches = not snapshot.mcu_enabled and snapshot.mcu_brake_locked
if self._execution_expected == "stopped_neutral":
    matches = matches and snapshot.software_neutral
return matches
```

- [ ] **Step 4: Run supervisor tests and confirm GREEN**

Run: `python -m pytest tests/test_peanut01_control_supervisor.py -q`

Expected: all supervisor tests pass; D/R still require enabled, brake-released, matching direction and feedback gear.

- [ ] **Step 5: Commit supervisor behavior**

```bash
git add tests/test_peanut01_control_supervisor.py work/peanut01_control_supervisor.py
git commit -m "feat: arm Peanut01 from software neutral evidence"
```

### Task 3: Wire software neutral into the bridge and diagnostics

**Files:**
- Modify: `tests/test_peanut01_control_bridge.py`
- Modify: `work/peanut01_control_bridge.py`

- [ ] **Step 1: Write the failing bridge contract test**

Require `_copy_inputs` to pass the derived result into `InputSnapshot`, and require diagnostics to separate command fields from feedback fields:

```python
def test_bridge_diagnostics_expose_command_and_feedback_gear_separately(self):
    text = BRIDGE.read_text(encoding="utf-8")
    for key in (
        "software_neutral",
        "command_enabled",
        "command_mode",
        "command_gear",
        "command_brake_mode",
        "command_motor_rpm",
        "mcu_command_age_ms",
        "mcu_gear",
    ):
        with self.subTest(key=key):
            self.assertIn(f'key="{key}"', text)
    self.assertIn('"software_neutral": feedback.software_neutral', text)
```

- [ ] **Step 2: Run the bridge test and confirm RED**

Run: `python -m pytest tests/test_peanut01_control_bridge.py -q`

Expected: FAIL because command/software-neutral diagnostic keys and snapshot wiring are absent.

- [ ] **Step 3: Implement bridge wiring and diagnostics**

Add `"software_neutral": feedback.software_neutral` to `snapshot_values`. Add diagnostic `KeyValue` entries for `software_neutral`, `command_enabled`, `command_mode`, `command_gear`, `command_brake_mode`, `command_motor_rpm`, and `mcu_command_age_ms`, using `feedback.command`, while retaining the existing `mcu_gear` entry from `feedback.mcu.gear`.

- [ ] **Step 4: Run bridge and full repository tests**

Run: `python -m pytest tests/test_peanut01_control_bridge.py -q`

Expected: bridge tests pass.

Run: `python -m pytest -q`

Expected: all tests pass with zero failures.

Run: `python -m compileall -q work`

Expected: exit code 0 with no output.

Run: `git diff --check`

Expected: exit code 0 with no output.

- [ ] **Step 5: Commit bridge wiring**

```bash
git add tests/test_peanut01_control_bridge.py work/peanut01_control_bridge.py
git commit -m "feat: expose Peanut01 software neutral diagnostics"
```

### Task 4: Build, deploy, and verify with actuation disabled

**Files:**
- Verify only: `docker/compose/vehicle-edge.yaml`
- Verify only: `docker/config/vehicle/peanut01-control-bridge.yaml`

- [ ] **Step 1: Verify the deployment input remains disabled**

Run locally: `python -m pytest tests/test_peanut01_control_bridge.py::ControlBridgeContractTests::test_shared_parameters_default_actuation_to_disabled -q`

Expected: PASS and `enable_actuation` remains `false`.

- [ ] **Step 2: Build the vehicle image on the vehicle host**

Copy only the committed bridge modules and tests through the existing repository deployment workflow, build the `tod_vehicle_edge` image, and require the build command to exit 0. Do not modify operator files because the implementation is vehicle-side only.

- [ ] **Step 3: Recreate only the vehicle container**

Inspect the compose-resolved service and candidate image first, then recreate only `tod_vehicle_edge`. Confirm restart count remains 0 and the configured `enable_actuation` value is `false` before any ROS inspection.

- [ ] **Step 4: Perform read-only live verification**

Inspect `/debug/tod_peanut01/bridge_diagnostics` and confirm the captured `0x1A1#29000000000000D6` produces:

```text
software_neutral=True
command_enabled=False
command_mode=1
command_gear=2
command_brake_mode=1
command_motor_rpm=0
mcu_gear=1 or 2 (retained feedback is shown independently)
```

Confirm the command age remains fresh, `enable_actuation=False`, and real control topics have publisher count 0. Do not change the parameter to true and do not perform steering, throttle, brake, or gear movement tests.

- [ ] **Step 5: Push the verified commits**

Run: `git status --short --branch`

Expected: clean worktree on `codex/peanut01-switchable-control-bridge`.

Run: `git push origin codex/peanut01-switchable-control-bridge`

Expected: existing PR 11 updates successfully.

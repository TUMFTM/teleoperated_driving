# Peanut01 Full Control CAN Feedback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the existing switchable TOD-to-Autoware bridge so a remote G923 can command Peanut01 steering and R/N/D motion at an operator-side maximum of 0.05 m/s, while CAN execution feedback and the higher-priority vehicle-local F710 enforce fail-closed arming and runtime supervision.

**Architecture:** A new pure Python CAN feedback module parses the read-only JSON stream on `/vehicle/can/raw`, validates the deployed MCU/EPS protocols, tracks independent receipt times, and derives lateral/longitudinal approvals. The pure supervisor consumes those approvals plus requested/observed execution state; the ROS wrapper only transports data, publishes `SafetyDriverStatus`, and dynamically creates real Autoware publishers after successful arming. Deployment remains non-actuating with `enable_actuation: false`.

**Tech Stack:** Python 3, ROS 2 Humble `rclpy`, `std_msgs/String`, `tod_vehicle_msgs/SafetyDriverStatus`, Autoware control/vehicle messages, pytest, YAML, Docker Compose

---

## Confirmed Protocol Input

The deployed vehicle script `/home/nvidia/peanut_can_raw_bridge/can_raw_string_bridge.py` was inspected read-only on 2026-08-11. Each `std_msgs/msg/String.data` value is compact JSON with this exact shape:

```json
{"stamp_ns":123,"interface":"can_mingnuo","id":418,"id_hex":"1A2","is_extended":false,"is_rtr":false,"is_error":false,"dlc":8,"data_hex":"891040050005011B"}
```

The implementation must use the local monotonic callback receipt time for freshness; `stamp_ns` is diagnostic metadata only. It must reject error/RTR/extended frames, inconsistent `id`/`id_hex`, non-eight-byte recognized frames, malformed hex, and invalid checksum/XOR without refreshing any status timestamp.

## File Structure

- `work/peanut01_can_feedback.py`: JSON frame validation, MCU/EPS decoding, independent timestamps, approval derivation, and diagnostic snapshot data.
- `work/peanut01_control_supervisor.py`: pure arming and active execution-confirmation state machine.
- `work/peanut01_control_bridge.py`: dual-domain ROS subscriptions/publications, safety status projection, diagnostics, and dynamic real publishers.
- `tests/test_peanut01_can_feedback.py`: protocol, corruption, freshness, and approval tests.
- `tests/test_peanut01_control_supervisor.py`: arming, D/R/N confirmation, timeout, EPS fault, and F710 override tests.
- `tests/test_peanut01_control_bridge.py`: topic, message, parameter, Docker, disabled-startup, and no-SocketCAN contracts.
- `tests/test_g923_gear_config.py`: operator R/N/D and 0.05 m/s limit contract.
- `config/config/package_config/tod_peanut01_interface/params.yaml`: read-only CAN topic and supervision timeouts, retaining disabled actuation.
- `config/config/package_config/tod_command_creation/params.yaml`: Peanut01 operator speed limit.
- `docker/dockerfile`: installs the feedback module with the existing bridge modules.
- `docs/superpowers/specs/2026-08-11-peanut01-full-control-can-feedback-design.md`: source design; no behavior changes are made here.

### Task 1: Implement Validated CAN Feedback Decoding

**Files:**
- Create: `tests/test_peanut01_can_feedback.py`
- Create: `work/peanut01_can_feedback.py`

- [ ] **Step 1: Write failing JSON and protocol tests**

Load the pure module with `runpy`. Use these valid frames; the first and fourth are captured from the vehicle and the middle two are deterministic protocol fixtures:

```python
VALID_FRAMES = {
    0x1A2: "891040050005011B",
    0x1A3: "01F4000A0064009C",
    0x1A4: "00000000000000FF",
    0x401: "20800004005500F1",
}


def raw_frame(can_id, data_hex, **changes):
    value = {
        "stamp_ns": 123,
        "interface": "can_mingnuo",
        "id": can_id,
        "id_hex": f"{can_id:X}",
        "is_extended": False,
        "is_rtr": False,
        "is_error": False,
        "dlc": 8,
        "data_hex": data_hex,
    }
    value.update(changes)
    return json.dumps(value, separators=(",", ":"))


def test_decodes_captured_mcu_and_eps_frames():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    for can_id, payload in VALID_FRAMES.items():
        assert tracker.update_json(raw_frame(can_id, payload), 1_000_000_000)
    status = tracker.snapshot(1_000_000_000)
    assert (status.mcu.power_up, status.mcu.enabled) == (True, False)
    assert (status.mcu.direction, status.mcu.gear, status.mcu.brake_locked) == (0, 2, True)
    assert (status.mcu.voltage_v, status.mcu.current_a, status.mcu.motor_rpm) == (50.0, 1.0, 100)
    assert status.mcu.error_codes == (0, 0, 0, 0, 0)
    assert not status.mcu.manual_override
    assert (status.eps.mode, status.eps.init_status) == (0x20, 0x55)
    assert (status.eps.error_1, status.eps.error_2) == (0, 0)
```

Also parameterize malformed JSON, missing/extra-typed fields, `dlc != 8`, odd/non-hex payloads, inconsistent IDs, RTR/error/extended flags, a changed MCU checksum byte, and a changed EPS XOR byte. Assert `update_json()` returns false and the relevant timestamp remains unchanged.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m pytest tests/test_peanut01_can_feedback.py -v
```

Expected: collection fails because `work/peanut01_can_feedback.py` does not exist.

- [ ] **Step 3: Add immutable protocol data types and strict envelope parsing**

Implement these public types and entry point:

```python
@dataclasses.dataclass(frozen=True)
class McuStatus:
    power_up: bool = False
    enabled: bool = False
    direction: int = 0
    gear: int = 0
    brake_locked: bool = True
    voltage_v: float = 0.0
    current_a: float = 0.0
    motor_rpm: int = 0
    error_codes: tuple = ()
    error_count: int = 0
    manual_override: bool = False
    stat1_stamp_ns: int = 0
    stat2_stamp_ns: int = 0
    error_stamp_ns: int = 0


@dataclasses.dataclass(frozen=True)
class EpsStatus:
    mode: int = 0
    init_status: int = 0
    error_1: int = 0
    error_2: int = 0
    angle_deg: float = 0.0
    status_stamp_ns: int = 0


@dataclasses.dataclass(frozen=True)
class FeedbackSnapshot:
    mcu: McuStatus
    eps: EpsStatus
    emergency_released: bool
    lateral_approved: bool
    longitudinal_approved: bool
    last_reject_reason: str = ""
```

`CanFeedbackTracker.update_json(payload, receipt_ns)` must call `json.loads`, require a JSON object, require exact types for `id`, `id_hex`, the three flags, `dlc`, and `data_hex`, normalize `id_hex` with `int(value, 16)`, and return false for unrecognized CAN IDs without mutating tracked data. Do not import `socket`, `can`, `python-can`, or any ROS package.

- [ ] **Step 4: Implement the exact deployed decoding and integrity checks**

Decode according to the inspected `peanut_autoware_vehicle_interface/chassis_feedback.cpp` behavior:

```python
def mcu_checksum(data):
    return (sum(data[:7]) & 0xFF) ^ 0xFF


def eps_xor(data):
    value = 0
    for byte in data[:7]:
        value ^= byte
    return value


# 0x1A2
power_up = bool(data[0] & 0x80)
enabled = bool(data[0] & 0x40)
direction = (data[0] >> 4) & 0x03
gear = (data[0] >> 2) & 0x03
brake_locked = bool(data[0] & 0x01)

# 0x1A3, big-endian unsigned values
voltage_v = int.from_bytes(data[0:2], "big") * 0.1
current_a = int.from_bytes(data[2:4], "big") * 0.1
motor_rpm = int.from_bytes(data[4:6], "big")

# 0x1A4
error_codes = tuple(data[0:5])
manual_override = data[5] == 1
error_count = data[6]

# 0x401
mode = data[0]
error_1 = data[2]
angle_deg = int.from_bytes(data[3:5], "big") * 0.1 - 1024.0
init_status = data[5]
error_2 = data[6]
```

Each valid recognized frame only replaces its own status group and receipt timestamp. Preserve the most recent rejection reason for diagnostics.

- [ ] **Step 5: Run protocol tests and verify GREEN**

Run:

```powershell
python -m pytest tests/test_peanut01_can_feedback.py -v
git diff --check
```

Expected: all decoder tests pass; no whitespace errors.

- [ ] **Step 6: Commit the decoder**

```powershell
git add work/peanut01_can_feedback.py tests/test_peanut01_can_feedback.py
git commit -m "feat: decode Peanut01 MCU and EPS feedback"
```

### Task 2: Derive Fresh Safety Approvals

**Files:**
- Modify: `tests/test_peanut01_can_feedback.py`
- Modify: `work/peanut01_can_feedback.py`

- [ ] **Step 1: Write failing freshness and approval tests**

Add tests proving:

```python
def test_approvals_require_all_independently_fresh_frames():
    tracker = CanFeedbackTracker(timeout_ns=300_000_000)
    for can_id, payload in VALID_FRAMES.items():
        tracker.update_json(raw_frame(can_id, payload), 1_000_000_000)
    approved = tracker.snapshot(1_300_000_000, emergency_fresh=True, emergency=False)
    assert approved.emergency_released
    assert approved.lateral_approved
    assert approved.longitudinal_approved

    stale = tracker.snapshot(1_300_000_001, emergency_fresh=True, emergency=False)
    assert not stale.lateral_approved
    assert not stale.longitudinal_approved


def test_mcu_enable_and_brake_are_not_prearm_approval_inputs():
    # 0x1A2 fixture reports enabled=False and brake_locked=True.
    status = populated_tracker().snapshot(NOW, emergency_fresh=True, emergency=False)
    assert status.longitudinal_approved
```

Add separate mutations for missing 0x1A2/1A3/1A4, MCU power down, each MCU error byte, MCU manual override, missing 0x401, EPS mode outside `{0x20, 0x23}`, init state outside `{0x55, 0xEE}`, either EPS error, stale emergency, and active emergency.

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
python -m pytest tests/test_peanut01_can_feedback.py -v
```

Expected: approval assertions fail because `snapshot()` does not yet derive freshness and approvals.

- [ ] **Step 3: Implement approval derivation**

Use inclusive freshness (`0 <= now_ns - stamp_ns <= timeout_ns`). Derive:

```python
emergency_released = emergency_fresh and not emergency
lateral_approved = (
    eps_fresh
    and eps.mode in (0x20, 0x23)
    and eps.init_status in (0x55, 0xEE)
    and eps.error_1 == 0
    and eps.error_2 == 0
)
longitudinal_approved = (
    mcu_stat1_fresh
    and mcu_stat2_fresh
    and mcu_error_fresh
    and mcu.power_up
    and not any(mcu.error_codes)
    and not mcu.manual_override
)
```

Expose per-group ages through a small `ages_ns(now_ns)` method for later diagnostics. Never use the unavailable VCU `0x18FA0121` frame in this phase.

- [ ] **Step 4: Run tests and commit**

```powershell
python -m pytest tests/test_peanut01_can_feedback.py -v
git add work/peanut01_can_feedback.py tests/test_peanut01_can_feedback.py
git commit -m "feat: derive Peanut01 safety approvals from CAN"
```

Expected: all CAN feedback tests pass.

### Task 3: Enforce Arming and Active Execution Confirmation

**Files:**
- Modify: `tests/test_peanut01_control_supervisor.py`
- Modify: `work/peanut01_control_supervisor.py`

- [ ] **Step 1: Extend the snapshot and write failing arming tests**

Extend `InputSnapshot` with:

```python
emergency_released: bool
lateral_approved: bool
longitudinal_approved: bool
mcu_power_up: bool
mcu_enabled: bool
mcu_direction: int
mcu_gear: int
mcu_brake_locked: bool
```

Extend `Parameters` with `execution_confirmation_timeout_ns=500_000_000`. Update `ready_snapshot()` to request TOD neutral (`tod_gear=2`) and report approved, powered MCU feedback with `mcu_enabled=False`, `mcu_gear=0`, and `mcu_brake_locked=True`. Add parameterized arming tests that independently reject non-N gear, nonzero requested speed, moving vehicle, missing approval, MCU enabled, MCU power down, unlocked brake, emergency, and F710 override. Verify any interruption restarts the full one-second arming timer.

- [ ] **Step 2: Run arming tests and verify RED**

```powershell
python -m pytest tests/test_peanut01_control_supervisor.py -v
```

Expected: constructor/signature failures and readiness assertions fail.

- [ ] **Step 3: Implement the stricter pre-arm predicate**

`input_ready()` must require all existing freshness/finite/TELEOPERATION/stopped checks plus:

```python
snapshot.tod_gear == 2
abs(snapshot.requested_velocity_mps) <= params.stopped_velocity_mps
snapshot.emergency_released
snapshot.lateral_approved
snapshot.longitudinal_approved
snapshot.mcu_power_up
not snapshot.mcu_enabled
snapshot.mcu_brake_locked
snapshot.mcu_gear == 0
not snapshot.local_override
```

Keep startup fail-closed even if YAML contains `enable_actuation: true`.

- [ ] **Step 4: Write failing D/R/N execution-window tests**

Add helpers that enter ACTIVE and then assert:

```python
# D request begins a confirmation window.
d_request = dataclasses.replace(active_snapshot(NOW), requested_velocity_mps=0.05, tod_gear=3)
assert supervisor.step(d_request).publish_commands

# Matching feedback before 500 ms remains active.
d_confirmed = dataclasses.replace(
    d_request, now_ns=NOW + 400_000_000,
    mcu_enabled=True, mcu_brake_locked=False, mcu_direction=1, mcu_gear=1,
)
assert supervisor.step(d_confirmed).state is State.ACTIVE

# Mismatch after 500 ms faults and requests a stop.
late = dataclasses.replace(d_request, now_ns=NOW + 500_000_001)
decision = supervisor.step(late)
assert decision.state is State.FAULT
assert decision.publish_stop
```

Mirror this for R (`tod_gear=1`, direction 2, MCU gear 2). Test N and zero-speed requirements: MCU disabled, brake locked, and, when requested gear is N, MCU gear 0. Independently test timeout failures for enable, brake, direction, and gear. Test that EPS/MCU approval loss and any F710 override fault immediately, and that releasing override never returns to ACTIVE without disable/re-arm.

- [ ] **Step 5: Run execution tests and verify RED**

```powershell
python -m pytest tests/test_peanut01_control_supervisor.py -v
```

Expected: D/R/N timeout tests fail because the supervisor does not track expected execution state.

- [ ] **Step 6: Implement execution-state tracking**

Represent the expected state as an immutable tuple/dataclass containing requested gear class and whether motion is requested. Reset `_execution_started_ns` whenever this expected state changes. In ACTIVE:

```python
if requested_motion and tod_gear == 3:
    matches = mcu_enabled and not brake_locked and direction == 1 and mcu_gear == 1
elif requested_motion and tod_gear == 1:
    matches = mcu_enabled and not brake_locked and direction == 2 and mcu_gear == 2
else:
    matches = not mcu_enabled and brake_locked
    if tod_gear == 2:
        matches = matches and mcu_gear == 0
```

Allow mismatch only while `now_ns - _execution_started_ns <= execution_confirmation_timeout_ns`. After that, latch `FAULT`, publish stop, and set a specific reason naming the mismatched execution state. Approval loss, emergency, invalid/stale input, mode exit, or F710 override remain immediate faults. Clear execution tracking on disable and successful re-arm.

- [ ] **Step 7: Run supervisor tests and commit**

```powershell
python -m pytest tests/test_peanut01_control_supervisor.py -v
git add work/peanut01_control_supervisor.py tests/test_peanut01_control_supervisor.py
git commit -m "feat: supervise Peanut01 CAN execution feedback"
```

Expected: all supervisor tests pass.

### Task 4: Integrate CAN Feedback and Safety Status Into the ROS Bridge

**Files:**
- Modify: `tests/test_peanut01_control_bridge.py`
- Modify: `work/peanut01_control_bridge.py`

- [ ] **Step 1: Write failing bridge contract tests**

Require these imports/topics and the absence of direct CAN access:

```python
for token in (
    "from peanut01_can_feedback import CanFeedbackTracker",
    "from std_msgs.msg import Bool, String",
    "SafetyDriverStatus",
    'CAN_FEEDBACK_TOPIC = "/vehicle/can/raw"',
    'SAFETY_STATUS_TOPIC = "/vehicle/interface/actuation/from_actuation/safety_driver_status"',
    "can_feedback_timeout_ms",
    "execution_confirmation_timeout_ms",
):
    assert token in bridge_text
for forbidden in ("socketcan", "python-can", "can_mingnuo"):
    assert forbidden not in bridge_text.lower()
```

Require that the safety publisher is created on the source/domain-7 node, while the raw CAN subscription is created on the target/domain-0 node. Require diagnostic keys for emergency/lateral/longitudinal approval, MCU power/enable/direction/gear/brake/errors/ages, EPS mode/init/errors/age, execution expectation/remaining time, and F710 override.

- [ ] **Step 2: Run contract tests and verify RED**

```powershell
python -m pytest tests/test_peanut01_control_bridge.py -v
```

Expected: missing module/topic/message/parameter assertions fail.

- [ ] **Step 3: Add shared CAN state and ROS wiring**

Import `String` and `SafetyDriverStatus`. Create `CanFeedbackTracker` under the existing shared lock. Subscribe on the target node:

```python
self.target_node.create_subscription(
    String, config["can_feedback_topic"], self.on_can_feedback, 500
)
```

`on_can_feedback()` calls `tracker.update_json(message.data, receipt_time_ns())`. On every timer iteration, snapshot CAN and emergency state under the lock, populate the extended `InputSnapshot`, and publish on the source node:

```python
message = SafetyDriverStatus()
message.vehicle_emergency_stop_released = feedback.emergency_released
message.vehicle_long_approved = feedback.longitudinal_approved
message.vehicle_lat_approved = feedback.lateral_approved
self.safety_status_publisher.publish(message)
```

Publishing safety status is independent of `enable_actuation`; stale/invalid input yields false fields.

- [ ] **Step 4: Extend parameters and diagnostics**

Declare:

```python
"can_feedback_topic": str(source_node.declare_parameter("can_feedback_topic", "/vehicle/can/raw").value),
"can_feedback_timeout_ms": int(source_node.declare_parameter("can_feedback_timeout_ms", 300).value),
"execution_confirmation_timeout_ms": int(source_node.declare_parameter("execution_confirmation_timeout_ms", 500).value),
```

Pass the timeout values to `CanFeedbackTracker` and `Parameters`. Add the specified status values to `DiagnosticStatus.values`; format missing ages as `unknown` and remaining confirmation time as milliseconds. Diagnostics describe state but never decide it.

- [ ] **Step 5: Preserve fail-closed publisher lifecycle**

Keep real publishers created only after an accepted AUTONOMOUS request. Add a
bridge `_fault_shutdown_pending` guard so the first `FAULT` decision starts the
existing three-cycle zero-velocity stop sequence exactly once; after the third
cycle call `_request_manual_and_destroy()`. Continue publishing diagnostics with
the latched supervisor reason, but do not recreate publishers or repeat the mode
request while faulted. Add static contract assertions for the guard and focused
pure tests proving the supervisor remains faulted after F710 B is released.

Pressing F710 B must therefore select F710 in the lower interface immediately,
fault the bridge, and prevent automatic G923 return. Only
`request_enable(False)` clears the latch; a later explicit true request must run
the complete arming sequence again.

- [ ] **Step 6: Run bridge and pure tests, then commit**

```powershell
python -m pytest tests/test_peanut01_can_feedback.py tests/test_peanut01_control_supervisor.py tests/test_peanut01_control_bridge.py -v
git diff --check
git add work/peanut01_control_bridge.py tests/test_peanut01_control_bridge.py
git commit -m "feat: publish Peanut01 CAN safety feedback"
```

Expected: all focused tests pass and no whitespace errors are reported.

### Task 5: Configure Operator Limit and Disabled Vehicle Deployment

**Files:**
- Modify: `tests/test_g923_gear_config.py`
- Modify: `tests/test_peanut01_control_bridge.py`
- Modify: `config/config/package_config/tod_command_creation/params.yaml`
- Modify: `config/config/package_config/tod_peanut01_interface/params.yaml`
- Modify: `docker/dockerfile`

- [ ] **Step 1: Write failing configuration tests**

Change the G923 assertion to:

```python
self.assertEqual(0.05, params["maxVelocity"])
self.assertEqual((1, 3, 2), (
    params["minGearPosition"],
    params["maxGearPosition"],
    params["defaultGearPosition"],
))
```

Extend bridge configuration tests to require:

```python
self.assertFalse(node["enable_actuation"])
self.assertEqual("/vehicle/can/raw", node["can_feedback_topic"])
self.assertEqual(300, node["can_feedback_timeout_ms"])
self.assertEqual(500, node["execution_confirmation_timeout_ms"])
self.assertNotIn("max_velocity", node)
```

Require `peanut01_can_feedback.py` in the Dockerfile module list.

- [ ] **Step 2: Run tests and verify RED**

```powershell
python -m pytest tests/test_g923_gear_config.py tests/test_peanut01_control_bridge.py -v
```

Expected: 0.05 limit, CAN parameters, and Docker module assertions fail.

- [ ] **Step 3: Apply the exact configuration**

Set the operator node to:

```yaml
maxVelocity: 0.05
minGearPosition: 1
maxGearPosition: 3
defaultGearPosition: 2
```

Add to the bridge node while retaining `enable_actuation: false`:

```yaml
can_feedback_topic: /vehicle/can/raw
can_feedback_timeout_ms: 300
execution_confirmation_timeout_ms: 500
```

Do not add a vehicle-side velocity clamp. Add `work/peanut01_can_feedback.py` to the existing `/opt/tod-tools/` Docker copy block.

- [ ] **Step 4: Run focused and full repository verification**

```powershell
python -m pytest tests/test_g923_gear_config.py tests/test_peanut01_can_feedback.py tests/test_peanut01_control_supervisor.py tests/test_peanut01_control_bridge.py -v
python -m pytest -q
git diff --check
```

Expected: focused and full suites pass; no whitespace errors.

- [ ] **Step 5: Commit configuration and packaging**

```powershell
git add tests/test_g923_gear_config.py tests/test_peanut01_control_bridge.py `
  config/config/package_config/tod_command_creation/params.yaml `
  config/config/package_config/tod_peanut01_interface/params.yaml `
  docker/dockerfile
git commit -m "config: gate Peanut01 full control with CAN feedback"
```

### Task 6: Build and Deploy Without Physical Actuation

**Hosts:** operator `user@192.168.188.16`; vehicle `nvidia@8.155.20.255:7027`

- [ ] **Step 1: Reconfirm deployment inputs before building**

On the local branch, run:

```powershell
git status --short
git log -5 --oneline
python -m pytest -q
```

Expected: only intentional plan/implementation state is present and all tests pass. On the vehicle, read `/home/nvidia/peanut_can_raw_bridge/can_raw_string_bridge.py` again and confirm the JSON keys still match the Confirmed Protocol Input section. Abort deployment if the schema changed.

- [ ] **Step 2: Build disposable operator and vehicle images**

On each host, first confirm the active container's
`com.docker.compose.project.config_files` label still names
`docker-compose.yaml,docker-compose.peanut01-video.yaml`. From the synchronized
repository checkout, build only the role hosted by that machine:

```bash
docker compose -f docker-compose.yaml -f docker-compose.peanut01-video.yaml build tod_operator
docker compose -f docker-compose.yaml -f docker-compose.peanut01-video.yaml build tod_vehicle
docker compose -f docker-compose.yaml -f docker-compose.peanut01-video.yaml images
```

Run only `tod_operator` on the operator host and only `tod_vehicle` on the
vehicle host. Expected: both builds complete successfully. Record repository,
tag, image ID, and creation time from `docker compose images`; verify the vehicle
image contains `/opt/tod-tools/peanut01_can_feedback.py` with a disposable
`docker compose run --rm --no-deps --entrypoint test tod_vehicle -f
/opt/tod-tools/peanut01_can_feedback.py` invocation.

- [ ] **Step 3: Run disposable non-actuating probes**

Start the vehicle image with its normal ROS networking but keep the YAML value false. Verify:

```bash
ros2 param get /vehicle/interface/peanut01/ControlBridge enable_actuation
ros2 topic info -v /control/command/control_cmd
ros2 topic info -v /control/command/gear_cmd
ros2 topic echo --once /vehicle/interface/actuation/from_actuation/safety_driver_status
ros2 topic echo --once /debug/tod_peanut01/bridge_diagnostics
```

Expected: `enable_actuation` is `False`; the control bridge is not a publisher on either real command topic; safety fields reflect live MCU/EPS/emergency feedback; diagnostics decode current `0x1A2/1A3/1A4/401` values and report `DISABLED`.

- [ ] **Step 4: Verify the operator command ceiling**

With the operator connected and G923 input active, select D and fully press the accelerator while observing the validated primary command:

```bash
ros2 topic echo /vehicle/safety/output/primary_control_cmd
```

Expected: requested velocity remains in `[0.0, 0.05]` m/s. Repeat in R and confirm TOD still publishes a non-negative magnitude no greater than 0.05 m/s. Do not set vehicle actuation true.

- [ ] **Step 5: Deploy the verified images with actuation disabled**

Replace only the established operator and vehicle services, preserving their current Compose names, mounts, ROS domain IDs, network mode, and restart policy. Immediately re-run the checks from Steps 3 and 4. If safety status, CAN diagnostics, or publisher absence differs, roll back to the recorded prior image IDs.

- [ ] **Step 6: Record the non-actuating verification**

Append deployment evidence to the pull request: commit SHA, image IDs, timestamps, parameter value, safety status, CAN diagnostic values, operator maximum observed velocity, and real command publisher counts. State explicitly: physical actuation was not enabled or tested.

Physical movement is outside this plan. It requires a separate explicit authorization, a lifted vehicle or controlled test area, an on-site emergency-stop operator, and a staged N/steering/D/R test procedure.

# Peanut01 Switchable Control Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the validated TOD command stream in ROS domain 7 into Peanut01 Autoware commands in domain 0, with physical actuation default-disabled and guarded by an explicit arming state machine.

**Architecture:** Pure Python mapping and supervision modules hold all safety-relevant decisions and are tested without ROS. A thin dual-context ROS wrapper follows the existing Peanut01 vehicle-state bridge pattern, publishes debug messages unconditionally, and creates real `/control/command/*` publishers only while the supervisor permits actuation. The first deployment keeps `enable_actuation: false` and performs no physical motion test.

**Tech Stack:** Python 3, ROS 2 Humble `rclpy`, Autoware vehicle/control messages, TOD messages, pytest/unittest, YAML, Docker Compose

---

## File Structure

- `work/peanut01_control_mapping.py`: pure TOD-to-Autoware value conversion.
- `work/peanut01_control_supervisor.py`: pure `DISABLED`/`ARMING`/`ACTIVE`/`FAULT` state machine.
- `work/peanut01_control_bridge.py`: ROS domain 7/domain 0 transport, parameters, publishers, subscribers, and control-mode service client.
- `tests/test_peanut01_control_mapping.py`: mapping tests, including reverse velocity and steering behavior.
- `tests/test_peanut01_control_supervisor.py`: arming, timeout, override, emergency, disable, and fault-reset tests.
- `tests/test_peanut01_control_bridge.py`: deployment and fail-closed static checks.
- `config/config/package_config/tod_peanut01_interface/params.yaml`: bridge parameters with actuation disabled.
- `src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py`: starts the bridge beside the existing dry-run interface.
- `docker/dockerfile`: installs the three bridge modules under `/opt/tod-tools`.

### Task 1: Implement Pure Command Mapping

**Files:**
- Create: `tests/test_peanut01_control_mapping.py`
- Create: `work/peanut01_control_mapping.py`

- [ ] **Step 1: Write failing mapping tests**

Create tests that load the module with `runpy` and exercise exact values:

```python
import math
import pathlib
import runpy
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAPPING = ROOT / "work/peanut01_control_mapping.py"


class Peanut01ControlMappingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mapping = runpy.run_path(str(MAPPING))

    def test_drive_and_reverse_use_signed_autoware_velocity(self):
        convert = self.mapping["convert_command"]
        self.assertEqual(0.8, convert(0.8, 1.6, 3, 0, 16.0).velocity_mps)
        self.assertEqual(-0.8, convert(0.8, -1.6, 1, 0, 16.0).velocity_mps)

    def test_reverse_steering_is_not_inverted_twice(self):
        command = self.mapping["convert_command"](0.5, -1.6, 1, 0, 16.0)
        self.assertEqual(-0.1, command.steering_tire_angle_rad)

    def test_park_neutral_and_unknown_gear_stop(self):
        convert = self.mapping["convert_command"]
        self.assertEqual(0.0, convert(0.8, 0.0, 0, 0, 16.0).velocity_mps)
        self.assertEqual(0.0, convert(0.8, 0.0, 2, 0, 16.0).velocity_mps)
        with self.assertRaises(ValueError):
            convert(0.8, 0.0, 99, 0, 16.0)

    def test_indicator_mapping_is_mutually_exclusive(self):
        convert = self.mapping["convert_command"]
        self.assertEqual((2, 1), (convert(0.0, 0.0, 0, 1, 16.0).turn, convert(0.0, 0.0, 0, 1, 16.0).hazard))
        self.assertEqual((3, 1), (convert(0.0, 0.0, 0, 2, 16.0).turn, convert(0.0, 0.0, 0, 2, 16.0).hazard))
        self.assertEqual((1, 2), (convert(0.0, 0.0, 0, 3, 16.0).turn, convert(0.0, 0.0, 0, 3, 16.0).hazard))

    def test_invalid_values_fail_closed(self):
        convert = self.mapping["convert_command"]
        with self.assertRaises(ValueError):
            convert(math.nan, 0.0, 3, 0, 16.0)
        with self.assertRaises(ValueError):
            convert(0.0, math.inf, 3, 0, 16.0)
        with self.assertRaises(ValueError):
            convert(0.0, 0.0, 3, 0, 0.0)
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```powershell
python -m pytest tests/test_peanut01_control_mapping.py -v
```

Expected: failure because `work/peanut01_control_mapping.py` does not exist.

- [ ] **Step 3: Implement the mapping module**

Implement immutable output data and exact mappings:

```python
import dataclasses
import math

TOD_PARK = 0
TOD_REVERSE = 1
TOD_NEUTRAL = 2
TOD_DRIVE = 3
TOD_SPORT = 4
TOD_HAUL = 5

AW_NEUTRAL = 1
AW_DRIVE = 2
AW_REVERSE = 20
AW_PARK = 22

AW_TURN_DISABLE = 1
AW_TURN_LEFT = 2
AW_TURN_RIGHT = 3
AW_HAZARD_DISABLE = 1
AW_HAZARD_ENABLE = 2


@dataclasses.dataclass(frozen=True)
class ConvertedCommand:
    velocity_mps: float
    steering_tire_angle_rad: float
    gear: int
    turn: int
    hazard: int


def convert_command(velocity, steering_wheel_angle, gear, indicator, steering_ratio):
    values = (velocity, steering_wheel_angle, steering_ratio)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("control values must be finite")
    if velocity < 0.0:
        raise ValueError("TOD velocity must be a non-negative magnitude")
    if steering_ratio <= 0.0:
        raise ValueError("steering_ratio must be positive")

    if gear == TOD_REVERSE:
        signed_velocity, aw_gear = -velocity, AW_REVERSE
    elif gear in (TOD_DRIVE, TOD_SPORT, TOD_HAUL):
        signed_velocity, aw_gear = velocity, AW_DRIVE
    elif gear == TOD_PARK:
        signed_velocity, aw_gear = 0.0, AW_PARK
    elif gear == TOD_NEUTRAL:
        signed_velocity, aw_gear = 0.0, AW_NEUTRAL
    else:
        raise ValueError(f"unsupported TOD gear: {gear}")

    if indicator == 0:
        turn, hazard = AW_TURN_DISABLE, AW_HAZARD_DISABLE
    elif indicator == 1:
        turn, hazard = AW_TURN_LEFT, AW_HAZARD_DISABLE
    elif indicator == 2:
        turn, hazard = AW_TURN_RIGHT, AW_HAZARD_DISABLE
    elif indicator == 3:
        turn, hazard = AW_TURN_DISABLE, AW_HAZARD_ENABLE
    else:
        raise ValueError(f"unsupported TOD indicator: {indicator}")

    return ConvertedCommand(
        velocity_mps=signed_velocity,
        steering_tire_angle_rad=steering_wheel_angle / steering_ratio,
        gear=aw_gear,
        turn=turn,
        hazard=hazard,
    )
```

- [ ] **Step 4: Run mapping tests and verify GREEN**

Run the focused test and expect all five tests to pass.

- [ ] **Step 5: Commit mapping behavior**

```powershell
git add work/peanut01_control_mapping.py tests/test_peanut01_control_mapping.py
git commit -m "feat: map TOD commands to Peanut01 Autoware commands"
```

### Task 2: Implement the Fail-Closed Supervisor

**Files:**
- Create: `tests/test_peanut01_control_supervisor.py`
- Create: `work/peanut01_control_supervisor.py`

- [ ] **Step 1: Write failing supervisor tests**

Use a pure data snapshot with nanosecond timestamps. Cover these transitions:

```python
def ready_snapshot(now_ns=1_000_000_000):
    return InputSnapshot(
        now_ns=now_ns,
        primary_stamp_ns=now_ns,
        secondary_stamp_ns=now_ns,
        status_stamp_ns=now_ns,
        feedback_stamp_ns=now_ns,
        tod_teleoperation=True,
        requested_velocity_mps=0.0,
        tod_gear=0,
        vehicle_velocity_mps=0.0,
        emergency=False,
        local_override=False,
        values_valid=True,
    )


def test_starts_disabled_even_when_configuration_requested_true():
    supervisor = Supervisor(Parameters())
    assert supervisor.state is State.DISABLED


def test_requires_one_second_of_continuous_readiness():
    supervisor = Supervisor(Parameters())
    supervisor.request_enable(True)
    assert supervisor.step(ready_snapshot()).state is State.ARMING
    later = ready_snapshot(2_000_000_000)
    decision = supervisor.step(later)
    assert decision.request_autonomous
    supervisor.on_mode_response(True)
    assert supervisor.state is State.ACTIVE


def test_stale_command_emergency_and_local_override_latch_fault():
    for mutation in ("stale", "emergency", "override"):
        supervisor = make_active_supervisor()
        snapshot = ready_snapshot(2_100_000_000)
        if mutation == "stale":
            snapshot.primary_stamp_ns -= 400_000_000
        elif mutation == "emergency":
            snapshot.emergency = True
        else:
            snapshot.local_override = True
        decision = supervisor.step(snapshot)
        assert decision.state is State.FAULT
        assert decision.publish_stop


def test_fault_requires_disable_before_rearming():
    supervisor = make_faulted_supervisor()
    supervisor.request_enable(True)
    assert supervisor.state is State.FAULT
    supervisor.request_enable(False)
    assert supervisor.state is State.DISABLED
```

- [ ] **Step 2: Run the tests and verify RED**

Expected: import failure because the supervisor module does not exist.

- [ ] **Step 3: Implement supervisor types and transitions**

Create `State`, `Parameters`, `InputSnapshot`, and `Decision` dataclasses plus
`Supervisor`. Implement these invariants:

```python
class State(enum.Enum):
    DISABLED = "DISABLED"
    ARMING = "ARMING"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"


def input_ready(snapshot, params):
    max_age = params.command_timeout_ns
    stamps = (
        snapshot.primary_stamp_ns,
        snapshot.secondary_stamp_ns,
        snapshot.status_stamp_ns,
        snapshot.feedback_stamp_ns,
    )
    return (
        snapshot.tod_teleoperation
        and all(0 <= snapshot.now_ns - stamp <= max_age for stamp in stamps)
        and abs(snapshot.vehicle_velocity_mps) <= params.stopped_velocity_mps
        and snapshot.tod_gear in (0, 2)
        and abs(snapshot.requested_velocity_mps) <= params.stopped_velocity_mps
        and not snapshot.emergency
        and not snapshot.local_override
        and snapshot.values_valid
    )
```

`request_enable(True)` returns a `Decision`, may only move `DISABLED` to
`ARMING`, and records no startup timestamp until the first ready snapshot.
`step()` resets the arming timer whenever readiness is lost. After the full
arming duration it emits one `request_autonomous=True` decision. A successful
mode response moves to `ACTIVE`; rejection moves to `FAULT`. Any active
violation emits `publish_stop=True` and latches `FAULT`.
`request_enable(False)` also returns a `Decision`: from `ACTIVE` or `FAULT` it
sets `publish_stop=True` and `request_manual=True`, then moves to `DISABLED`;
from `ARMING` it moves directly to `DISABLED` without creating real publishers.

- [ ] **Step 4: Run supervisor tests and verify GREEN**

Run:

```powershell
python -m pytest tests/test_peanut01_control_supervisor.py -v
```

Expected: all transition tests pass.

- [ ] **Step 5: Commit supervisor behavior**

```powershell
git add work/peanut01_control_supervisor.py tests/test_peanut01_control_supervisor.py
git commit -m "feat: add fail-closed Peanut01 bridge supervisor"
```

### Task 3: Implement the Dual-Domain ROS Bridge

**Files:**
- Create: `work/peanut01_control_bridge.py`
- Create: `tests/test_peanut01_control_bridge.py`

- [ ] **Step 1: Write bridge contract tests**

The tests inspect the script and require all source, feedback, debug, and real
topic names; they also require dynamic publisher destruction and prohibit
SocketCAN imports:

```python
class Peanut01ControlBridgeContractTest(unittest.TestCase):
    def test_bridge_uses_required_domains_and_topics(self):
        text = BRIDGE.read_text(encoding="utf-8")
        for topic in (
            "/vehicle/safety/output/primary_control_cmd",
            "/vehicle/safety/output/secondary_control_cmd",
            "/vehicle/statemachine/output/vehicle_status",
            "/vehicle/status/velocity_status",
            "/vehicle/status/steering_status",
            "/vehicle/status/control_mode",
            "/minguo/emergency_stop",
            "/minguo/teleop_override",
            "/debug/tod_peanut01/autoware_control_cmd",
            "/control/command/control_cmd",
            "/control/control_mode_request",
        ):
            self.assertIn(topic, text)
        self.assertIn("source_domain_id", text)
        self.assertIn("target_domain_id", text)

    def test_disabled_mode_can_destroy_real_publishers(self):
        text = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("create_real_publishers", text)
        self.assertIn("destroy_real_publishers", text)
        self.assertIn("destroy_publisher", text)
        self.assertNotIn("socketcan", text.lower())
        self.assertNotIn("python-can", text.lower())
```

- [ ] **Step 2: Run contract tests and verify RED**

Expected: failure because the bridge script does not exist.

- [ ] **Step 3: Implement the ROS wrapper**

Follow `work/peanut01_vehicle_state_bridge.py` for two explicit `rclpy.Context`
instances and a target-domain executor thread. The source node declares all
configuration parameters. Its internal requested state is always initialized
to `false`, regardless of the YAML value, so a restart cannot arm the bridge.
After initialization, an `add_on_set_parameters_callback` accepts an explicit
runtime `enable_actuation=true` change and passes it to
`Supervisor.request_enable()`; all other safety parameters are read-only while
the process is running.

The wrapper must:

- Store source and target callbacks under one `threading.Lock`.
- Subscribe to `SteeringReport` on `/vehicle/status/steering_status`, require it
  to be fresh during arming, and use its current tire angle as the arming target.
- Convert the latest primary/secondary pair through `convert_command`.
- Publish `Control`, `GearCommand`, `TurnIndicatorsCommand`, and
  `HazardLightsCommand` to debug topics every timer cycle.
- Leave acceleration, jerk, and steering rotation-rate optional flags false.
- Create real publishers only after a successful autonomous-mode response.
- Publish stop commands while faulted until `enable_actuation` becomes false.
- Destroy real publishers after disable and a MANUAL-mode request.
- Publish bridge state and rejection reason as diagnostics.
- Never publish `/minguo/teleop_override` and never access CAN.

Use generated constants such as `Status.TOD_STATUS_TELEOPERATION`,
`ControlModeCommand.Request.AUTONOMOUS`, and
`ControlModeCommand.Request.MANUAL`; do not hardcode their numeric values.

- [ ] **Step 4: Run pure and contract tests**

Run:

```powershell
python -m pytest tests/test_peanut01_control_mapping.py tests/test_peanut01_control_supervisor.py tests/test_peanut01_control_bridge.py -v
```

Expected: all tests pass without requiring ROS Python packages on Windows.

- [ ] **Step 5: Commit the ROS wrapper**

```powershell
git add work/peanut01_control_bridge.py tests/test_peanut01_control_bridge.py
git commit -m "feat: bridge TOD commands into Autoware domains"
```

### Task 4: Add Default-Disabled Deployment Configuration

**Files:**
- Modify: `config/config/package_config/tod_peanut01_interface/params.yaml`
- Modify: `src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py`
- Modify: `docker/dockerfile`
- Modify: `tests/test_peanut01_control_bridge.py`

- [ ] **Step 1: Extend deployment tests**

Parse the shared YAML file installed by `tod_launch` and assert this exact
selector and disabled value:

```python
node = "/vehicle/interface/peanut01/ControlBridge"
params = yaml.safe_load(DEPLOYED_PARAMS.read_text(encoding="utf-8"))
self.assertFalse(params[node]["ros__parameters"]["enable_actuation"])
```

Also require the vehicle launch to execute
`/opt/tod-tools/peanut01_control_bridge.py` with the shared parameter file and
require the Dockerfile to copy all three control-bridge modules.

- [ ] **Step 2: Run deployment tests and verify RED**

Expected: missing YAML selector, launcher, and Dockerfile copies.

- [ ] **Step 3: Add bridge parameters**

Preserve the dry-run node settings and add:

```yaml
/vehicle/interface/peanut01/ControlBridge:
  ros__parameters:
    enable_actuation: false
    source_domain_id: 7
    target_domain_id: 0
    command_timeout_ms: 300
    feedback_timeout_ms: 300
    arming_duration_ms: 1000
    stopped_velocity_mps: 0.02
    steering_ratio: 16.0
    publish_rate_hz: 20.0
```

- [ ] **Step 4: Launch and package the bridge**

Add an `ExecuteProcess` action to the Peanut01 video vehicle launcher:

```python
control_bridge_params = os.path.join(
    config_dir, "package_config", "tod_peanut01_interface", "params.yaml"
)
description.add_action(
    ExecuteProcess(
        cmd=[
            "python3",
            "/opt/tod-tools/peanut01_control_bridge.py",
            "--ros-args",
            "--params-file",
            control_bridge_params,
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
)
```

Add Dockerfile copies beside the existing Peanut01 helper scripts:

```dockerfile
COPY ./work/peanut01_control_mapping.py /opt/tod-tools/peanut01_control_mapping.py
COPY ./work/peanut01_control_supervisor.py /opt/tod-tools/peanut01_control_supervisor.py
COPY ./work/peanut01_control_bridge.py /opt/tod-tools/peanut01_control_bridge.py
```

- [ ] **Step 5: Run deployment and repository tests**

Run:

```powershell
python -m pytest tests/test_peanut01_control_bridge.py tests/test_peanut01_vehicle_state.py tests/test_peanut01_video.py -v
python -m pytest -q
git diff --check
```

Expected: all tests pass and no whitespace errors are reported.

- [ ] **Step 6: Commit deployment wiring**

```powershell
git add config/config/package_config/tod_peanut01_interface/params.yaml `
  src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py `
  docker/dockerfile tests/test_peanut01_control_bridge.py
git commit -m "config: deploy Peanut01 control bridge disabled"
```

### Task 5: Build and Verify on the Vehicle Without Actuation

**Host:** `nvidia@8.155.20.255:7027`

- [ ] **Step 1: Re-inspect the active vehicle source and Compose metadata**

Confirm the active service is still `tod_vehicle_edge`, record its ordered
Compose file list from Docker labels, and verify all target files are clean on
the remote host before synchronization. Preserve every unrelated remote change.

- [ ] **Step 2: Synchronize only reviewed bridge files**

Copy the three `work/` modules, the shared parameter file, vehicle video launcher,
Dockerfile, and focused tests. Do not replace whole package directories.

- [ ] **Step 3: Run remote pure tests before building**

Run:

```bash
python3 -m pytest \
  tests/test_peanut01_control_mapping.py \
  tests/test_peanut01_control_supervisor.py \
  tests/test_peanut01_control_bridge.py -v
```

Expected: all bridge tests pass.

- [ ] **Step 4: Build the candidate vehicle image**

Use the exact active Compose file order and build only `tod_vehicle`. Keep the
existing container running until the image build succeeds.

- [ ] **Step 5: Verify the candidate is fail-closed before recreation**

Inspect the candidate image and require the installed shared parameter file to
contain `enable_actuation: false`. Start a disposable no-network command if needed to
verify the script imports successfully; do not run the vehicle launch in the
disposable container.

- [ ] **Step 6: Recreate only the vehicle service**

Recreate `tod_vehicle` with the recorded Compose files and no other services.
Then confirm:

```bash
ros2 param get /vehicle/interface/peanut01/ControlBridge enable_actuation
```

Expected: `Boolean value is: False`.

- [ ] **Step 7: Verify topic-level isolation**

In domain 0, require bridge publishers on
`/debug/tod_peanut01/autoware_control_cmd` and no publisher from the bridge node
on `/control/command/control_cmd`. Confirm CAN traffic is not attributed to the
bridge process and the vehicle remains stationary.

- [ ] **Step 8: Verify conversion without physical motion**

Use the normal operator path while actuation remains disabled. Observe debug
topics for forward positive velocity, reverse negative velocity, steering
ratio conversion, gear mapping, and timeout stop. Do not set
`enable_actuation` to true.

- [ ] **Step 9: Record evidence and run final local verification**

Record container image ID, runtime parameter output, topic publisher evidence,
and focused test results. Run the complete local suite and `git diff --check`
again before reporting completion.

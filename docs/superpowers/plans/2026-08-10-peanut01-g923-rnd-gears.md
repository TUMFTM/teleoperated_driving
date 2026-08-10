# Peanut01 G923 R/N/D Gear Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Peanut01 G923 operator start in neutral and select only reverse, neutral, and drive with bounded paddle shifts.

**Architecture:** Add a small header-only gear selector so range validation and shift behavior can be tested without constructing a ROS node. `CommandCreator` loads legacy-compatible range parameters, while the Peanut01 deployment YAML explicitly selects `R/N/D` with default `N`. The operator image is rebuilt and verified with vehicle actuation disabled.

**Tech Stack:** C++14, ROS 2 Humble `rclcpp`, `ament_cmake_gtest`, TOD vehicle messages, YAML, Python/pytest, Docker Compose

---

## File Structure

- `src/tod_direct_control/tod_command_creation/include/tod_command_creation/gear_selector.hpp`: validates a configured TOD gear range and applies bounded paddle changes.
- `src/tod_direct_control/tod_command_creation/test/test_gear_selector.cpp`: pure C++ tests for default, boundaries, motion lockout, normalization, and invalid configuration.
- `src/tod_direct_control/tod_command_creation/CMakeLists.txt`: registers the gtest target.
- `src/tod_direct_control/tod_command_creation/package.xml`: declares the gtest dependency.
- `src/tod_direct_control/tod_command_creation/include/tod_command_creation/command_creator.hpp`: owns the configured selector.
- `src/tod_direct_control/tod_command_creation/src/command_creator.cpp`: loads parameters, initializes to the configured default, and delegates paddle selection.
- `src/tod_direct_control/tod_command_creation/config/package_config/tod_command_creation/params.yaml`: documents legacy-compatible defaults for generic launches.
- `config/config/package_config/tod_command_creation/params.yaml`: configures the deployed Peanut01 operator for `R/N/D`, default `N`, while preserving `maxVelocity: 0.8`.
- `tests/test_g923_gear_config.py`: checks the deployed selector, G923 paddle mapping, and command-creator wiring.

### Task 1: Add the Tested Gear Selector

**Files:**
- Create: `src/tod_direct_control/tod_command_creation/include/tod_command_creation/gear_selector.hpp`
- Create: `src/tod_direct_control/tod_command_creation/test/test_gear_selector.cpp`
- Modify: `src/tod_direct_control/tod_command_creation/CMakeLists.txt`
- Modify: `src/tod_direct_control/tod_command_creation/package.xml`

- [ ] **Step 1: Write the failing selector tests**

Create `test/test_gear_selector.cpp`:

```cpp
#include <gtest/gtest.h>

#include "tod_command_creation/gear_selector.hpp"
#include "tod_vehicle_msgs/VehicleEnums.h"

using tod_command_creation::GearSelector;

TEST(GearSelector, StartsFromConfiguredNeutral)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_NEUTRAL, selector.default_gear());
}

TEST(GearSelector, StopsAtReverseAndDriveBoundaries)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_DRIVE, selector.select(GEARPOSITION_DRIVE, true, false, 0.0F));
  EXPECT_EQ(GEARPOSITION_REVERSE, selector.select(GEARPOSITION_REVERSE, false, true, 0.0F));
}

TEST(GearSelector, StepsOnlyThroughReverseNeutralDrive)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_DRIVE, selector.select(GEARPOSITION_NEUTRAL, true, false, 0.0F));
  EXPECT_EQ(GEARPOSITION_REVERSE, selector.select(GEARPOSITION_NEUTRAL, false, true, 0.0F));
}

TEST(GearSelector, BlocksChangesWhileMoving)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(
    GEARPOSITION_NEUTRAL,
    selector.select(GEARPOSITION_NEUTRAL, true, false, 0.01F));
}

TEST(GearSelector, NormalizesAnOutOfRangeGearToDefault)
{
  const GearSelector selector(
    GEARPOSITION_REVERSE, GEARPOSITION_DRIVE, GEARPOSITION_NEUTRAL);
  EXPECT_EQ(GEARPOSITION_NEUTRAL, selector.select(GEARPOSITION_PARK, false, false, 0.0F));
  EXPECT_EQ(GEARPOSITION_NEUTRAL, selector.select(GEARPOSITION_SPORT, false, false, 0.0F));
}

TEST(GearSelector, RejectsInvalidConfiguration)
{
  EXPECT_THROW(GearSelector(3, 1, 2), std::invalid_argument);
  EXPECT_THROW(GearSelector(1, 3, 4), std::invalid_argument);
  EXPECT_THROW(GearSelector(-1, 3, 2), std::invalid_argument);
}
```

- [ ] **Step 2: Register the test and verify RED**

Add to `package.xml`:

```xml
<test_depend>ament_cmake_gtest</test_depend>
```

Add before `ament_package()` in the package `CMakeLists.txt`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gear_selector test/test_gear_selector.cpp)
  ament_target_dependencies(test_gear_selector tod_vehicle_msgs)
  target_include_directories(test_gear_selector PRIVATE include)
endif()
```

Synchronize only the new test plus the reviewed `CMakeLists.txt` and
`package.xml` into their matching paths under
`/home/user/teleoperated_driving_deploy`. This updates the inactive build
context only; it does not restart the running operator.

Build the operator builder image on the operator host:

```bash
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  build tod_operator_builder
```

Expected: compilation fails because `tod_command_creation/gear_selector.hpp` does not exist.

- [ ] **Step 3: Implement the minimal selector**

Create `gear_selector.hpp`:

```cpp
#pragma once

#include <stdexcept>

#include "tod_vehicle_msgs/VehicleEnums.h"

namespace tod_command_creation {

class GearSelector
{
public:
  GearSelector(const int min_gear, const int max_gear, const int default_gear)
  : min_gear_(min_gear), max_gear_(max_gear), default_gear_(default_gear)
  {
    const bool known_bounds =
      min_gear_ >= GEARPOSITION_PARK && max_gear_ <= GEARPOSITION_HAUL;
    if (!known_bounds || min_gear_ > max_gear_ ||
      default_gear_ < min_gear_ || default_gear_ > max_gear_)
    {
      throw std::invalid_argument("invalid gear selection range");
    }
  }

  int default_gear() const { return default_gear_; }

  int select(
    const int current_gear, const bool increase_edge, const bool decrease_edge,
    const float current_velocity) const
  {
    int selected =
      current_gear >= min_gear_ && current_gear <= max_gear_ ? current_gear : default_gear_;
    if (current_velocity >= 0.01F) {
      return selected;
    }
    if (increase_edge && selected < max_gear_) {
      ++selected;
    }
    if (decrease_edge && selected > min_gear_) {
      --selected;
    }
    return selected;
  }

private:
  int min_gear_;
  int max_gear_;
  int default_gear_;
};

}  // namespace tod_command_creation
```

- [ ] **Step 4: Build and run the focused test**

Synchronize `gear_selector.hpp` into its matching operator staging path, rebuild
`tod_operator_builder`, then run:

```bash
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  run --rm --no-deps tod_operator_builder \
  bash -lc '/home/tum/wsp/build/tod_command_creation/test_gear_selector'
```

Expected: six tests pass.

- [ ] **Step 5: Commit the selector**

```powershell
git add src/tod_direct_control/tod_command_creation/CMakeLists.txt `
  src/tod_direct_control/tod_command_creation/package.xml `
  src/tod_direct_control/tod_command_creation/include/tod_command_creation/gear_selector.hpp `
  src/tod_direct_control/tod_command_creation/test/test_gear_selector.cpp
git commit -m "test: cover configurable TOD gear selection"
```

### Task 2: Wire the Selector into CommandCreator

**Files:**
- Modify: `src/tod_direct_control/tod_command_creation/include/tod_command_creation/command_creator.hpp`
- Modify: `src/tod_direct_control/tod_command_creation/src/command_creator.cpp`
- Modify: `src/tod_direct_control/tod_command_creation/config/package_config/tod_command_creation/params.yaml`
- Modify: `config/config/package_config/tod_command_creation/params.yaml`
- Create: `tests/test_g923_gear_config.py`

- [ ] **Step 1: Write failing deployment and wiring tests**

Create `tests/test_g923_gear_config.py`:

```python
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEPLOYED_PARAMS = ROOT / "config/config/package_config/tod_command_creation/params.yaml"
G923_PARAMS = ROOT / "src/tod_operator_interface/tod_input_devices/config/logitechg923.yaml"
COMMAND_CREATOR = ROOT / "src/tod_direct_control/tod_command_creation/src/command_creator.cpp"


class G923GearConfigTest(unittest.TestCase):
    def test_peanut01_uses_rnd_with_neutral_default(self):
        config = yaml.safe_load(DEPLOYED_PARAMS.read_text(encoding="utf-8"))
        params = config["/operator/direct_control/CommandCreator"]["ros__parameters"]
        self.assertEqual(1, params["minGearPosition"])
        self.assertEqual(3, params["maxGearPosition"])
        self.assertEqual(2, params["defaultGearPosition"])
        self.assertEqual(0.8, params["maxVelocity"])

    def test_g923_paddles_keep_the_existing_button_mapping(self):
        config = yaml.safe_load(G923_PARAMS.read_text(encoding="utf-8"))
        buttons = config["/**"]["ros__parameters"]["button_config"]
        self.assertEqual(4, buttons["IncreaseGear"])
        self.assertEqual(5, buttons["DecreaseGear"])

    def test_command_creator_initializes_and_uses_the_selector(self):
        source = COMMAND_CREATOR.read_text(encoding="utf-8")
        for parameter in ("minGearPosition", "maxGearPosition", "defaultGearPosition"):
            self.assertIn(f'declare_parameter<int>("{parameter}"', source)
        self.assertIn("_gearSelector->default_gear()", source)
        self.assertIn("_gearSelector->select(", source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify RED**

Run locally:

```powershell
python -m pytest tests/test_g923_gear_config.py -v
```

Expected: the deployed YAML selector and three parameters are absent.

- [ ] **Step 3: Add selector ownership and load validated parameters**

In `command_creator.hpp`, include the selector and add its member:

```cpp
#include "tod_command_creation/gear_selector.hpp"

std::unique_ptr<GearSelector> _gearSelector;
```

In the constructor, after the existing numeric parameter declarations, declare
legacy-compatible defaults and construct the selector before the node can spin:

```cpp
const int min_gear = this->declare_parameter<int>(
  "minGearPosition", eGearPosition::GEARPOSITION_PARK);
const int max_gear = this->declare_parameter<int>(
  "maxGearPosition", eGearPosition::GEARPOSITION_SPORT);
const int default_gear = this->declare_parameter<int>(
  "defaultGearPosition", eGearPosition::GEARPOSITION_PARK);

try {
  _gearSelector = std::make_unique<GearSelector>(min_gear, max_gear, default_gear);
} catch (const std::invalid_argument & error) {
  RCLCPP_FATAL(this->get_logger(), "Invalid gear configuration: %s", error.what());
  throw;
}
init_control_messages();
```

Replace the hard-coded gear range and arithmetic in `set_gear()` with edge
detection followed by one selector call:

```cpp
const bool increase_edge =
  buttonState.at(joystick::ButtonPos::INCREASE_GEAR) == 1 &&
  _prevButtonState.at(joystick::ButtonPos::INCREASE_GEAR) == 0;
const bool decrease_edge =
  buttonState.at(joystick::ButtonPos::DECREASE_GEAR) == 1 &&
  _prevButtonState.at(joystick::ButtonPos::DECREASE_GEAR) == 0;

out.gear_position = _gearSelector->select(
  out.gear_position, increase_edge, decrease_edge, currentVelocity);
```

Keep the two existing previous-button-state assignments after the selector call.
Change `init_control_messages()` to:

```cpp
_secondaryControlMsg.gear_position = _gearSelector->default_gear();
```

- [ ] **Step 4: Configure legacy defaults and the Peanut01 deployment**

Add these values to the package's generic `/**` parameter file so generic
launches retain current behavior:

```yaml
minGearPosition:             0
maxGearPosition:             4
defaultGearPosition:         0
```

Change the deployed `config/config/package_config/tod_command_creation/params.yaml`
to the active node selector while preserving the current 0.8 m/s setting:

```yaml
/operator/direct_control/CommandCreator:
  ros__parameters:
    InvertSteeringInGearReverse:  true
    ConstraintSteeringRate:       false
    maxVelocity:                  0.8
    maxAcceleration:              4.0
    maxDeceleration:              9.0
    maxSteeringWheelAngleRate:    7.6
    minGearPosition:              1
    maxGearPosition:              3
    defaultGearPosition:          2
```

- [ ] **Step 5: Run focused tests and compile the operator**

Run locally:

```powershell
python -m pytest tests/test_g923_gear_config.py tests/test_deployment_config.py -v
git diff --check
```

Synchronize the focused files to the operator staging tree, build
`tod_operator_builder`, and run `test_gear_selector` again. Expected: Python
tests and all six C++ tests pass, and `OperatorCommandCreator` compiles.

- [ ] **Step 6: Commit command creation and configuration**

```powershell
git add src/tod_direct_control/tod_command_creation/include/tod_command_creation/command_creator.hpp `
  src/tod_direct_control/tod_command_creation/src/command_creator.cpp `
  src/tod_direct_control/tod_command_creation/config/package_config/tod_command_creation/params.yaml `
  config/config/package_config/tod_command_creation/params.yaml `
  tests/test_g923_gear_config.py
git commit -m "feat: limit Peanut01 G923 to RND gears"
```

### Task 3: Build, Deploy, and Verify the Operator Safely

**Host:** `user@192.168.188.16`

**Vehicle host used only for the safety-switch check:** `nvidia@8.155.20.255:7027`

- [ ] **Step 1: Re-check active deployment metadata and preserve remote state**

Confirm the operator service is `tod_operator`, the active container is
`tod_operator_edge`, and the ordered Compose files are still:

```text
docker-compose.yaml
docker-compose.override.yaml
docker-compose.peanut01-dry-run.yaml
docker-compose.peanut01-video.yaml
```

Compare every target file with the reviewed local version. Preserve unrelated
remote edits, especially the active `maxVelocity: 0.8` setting and G923 runtime
support files.

- [ ] **Step 2: Verify physical actuation is still disabled**

On the vehicle host, run in ROS domain 7:

```bash
docker exec tod_vehicle_edge bash -lc \
  'source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID=7 ros2 param get /vehicle/interface/peanut01/ControlBridge enable_actuation'
```

Expected: `Boolean value is: False`. Stop deployment if it is not false.

- [ ] **Step 3: Synchronize only the reviewed operator files**

Copy the selector header and test, command-creator header/source, package
metadata/CMake, both parameter files, and the focused Python test into matching
paths under `/home/user/teleoperated_driving_deploy`. Do not replace whole
directories or touch G923 steering configuration.

- [ ] **Step 4: Build and test the candidate without stopping the active operator**

From `/home/user/teleoperated_driving_deploy`, build the builder service and run
the focused test:

```bash
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  build tod_operator_builder

docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  run --rm --no-deps tod_operator_builder \
  bash -lc '/home/tum/wsp/build/tod_command_creation/test_gear_selector'
```

Expected: candidate build succeeds and six tests pass. The existing
`tod_operator_edge` container remains running throughout this step.

- [ ] **Step 5: Build and inspect the final operator image**

```bash
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  build tod_operator
```

Before recreation, run the candidate image with a non-launching shell and
confirm its installed command-creation parameter file contains:

```text
minGearPosition: 1
maxGearPosition: 3
defaultGearPosition: 2
maxVelocity: 0.8
```

- [ ] **Step 6: Recreate only the operator service**

```bash
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  up -d --no-deps tod_operator
```

Confirm `tod_operator_edge` is healthy and has not restarted unexpectedly.

- [ ] **Step 7: Verify runtime parameters and G923 mapping**

Inside `tod_operator_edge`, query:

```bash
ros2 param get /operator/direct_control/CommandCreator minGearPosition
ros2 param get /operator/direct_control/CommandCreator maxGearPosition
ros2 param get /operator/direct_control/CommandCreator defaultGearPosition
ros2 param get /operator/direct_control/CommandCreator maxVelocity
ros2 param get /operator/input_devices/InputDevice type
ros2 param get /operator/input_devices/InputDevice button_config.IncreaseGear
ros2 param get /operator/input_devices/InputDevice button_config.DecreaseGear
```

Expected values: `1`, `3`, `2`, `0.8`, `Usb`, `4`, and `5`.

- [ ] **Step 8: Verify the topic behavior with actuation disabled**

Enter teleoperation while the vehicle remains stationary. Observe
`/operator/network/data/to_vehicle/secondary_control_cmd` and exercise only the
G923 paddles:

```text
startup/reset -> gear_position: 2
right paddle  -> gear_position: 3
right again   -> gear_position: 3
left paddle   -> gear_position: 2
left paddle   -> gear_position: 1
left again    -> gear_position: 1
```

Confirm no sample contains `0` or `4`. Do not set `enable_actuation` to true.

- [ ] **Step 9: Run final repository verification and record evidence**

Run locally:

```powershell
python -m pytest -q
git diff --check
git status --short
```

Record the final operator image ID, builder test output, runtime parameters,
topic values, and vehicle `enable_actuation: false` result before reporting
completion.

# Peanut01 Teleoperation Speed Limit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enforce a `0.8 m/s` Peanut01 teleoperation speed limit independently on the operator and vehicle sides.

**Architecture:** The operator `CommandCreator` caps its non-negative speed magnitude at `0.8 m/s`. The vehicle `safety_gate` applies a second configurable cap to every teleoperation primary control command before forwarding it, while preserving the existing monitoring-state stop and warning behavior.

**Tech Stack:** ROS 2 Humble, C++17, `ament_cmake`, GoogleTest, Python `unittest`, YAML, Docker Compose

---

## File Structure

- `tests/test_deployment_config.py`: verifies effective node selectors and matching `0.8 m/s` deployment limits.
- `config/config/package_config/tod_command_creation/params.yaml`: deployed operator limit.
- `src/tod_direct_control/tod_command_creation/config/package_config/tod_command_creation/params.yaml`: package-owned operator default.
- `config/config/package_config/tod_safety_gate/params.yaml`: deployed vehicle limit.
- `src/tod_safety/tod_safety_gate/config/package_config/tod_safety_gate/params.yaml`: package-owned vehicle default.
- `src/tod_safety/tod_safety_gate/include/tod_safety_gate/primary_control_policy.hpp`: testable vehicle-side speed policy API.
- `src/tod_safety/tod_safety_gate/src/primary_control_policy.cpp`: monitoring-state and speed-limit policy.
- `src/tod_safety/tod_safety_gate/test/test_primary_control_policy.cpp`: focused policy tests.
- `src/tod_safety/tod_safety_gate/src/safety_gate.cpp`: ROS parameter declaration and policy integration.
- `src/tod_safety/tod_safety_gate/include/tod_safety_gate/safety_gate.hpp`: stores the normal speed limit.
- `src/tod_safety/tod_safety_gate/CMakeLists.txt`: builds and tests the policy library.
- `src/tod_safety/tod_safety_gate/package.xml`: declares the GoogleTest test dependency.
- `src/tod_safety/tod_safety_gate/README.md`: documents the normal and warning limits.

### Task 1: Lock Deployment Configuration Requirements

**Files:**
- Modify: `tests/test_deployment_config.py`

- [ ] **Step 1: Write the failing deployment tests**

Add `import yaml`, then add helpers and tests that parse the real YAML rather than matching text:

```python
def load_yaml(relative_path):
    path = REPO / relative_path
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class DeploymentConfigTests(unittest.TestCase):
    def test_operator_speed_limit_targets_running_command_creator(self):
        deployed = load_yaml(
            "config/config/package_config/tod_command_creation/params.yaml"
        )
        packaged = load_yaml(
            "src/tod_direct_control/tod_command_creation/config/"
            "package_config/tod_command_creation/params.yaml"
        )

        node = "/operator/direct_control/CommandCreator"
        self.assertEqual(0.8, deployed[node]["ros__parameters"]["maxVelocity"])
        self.assertEqual(0.8, packaged[node]["ros__parameters"]["maxVelocity"])

    def test_vehicle_speed_limit_targets_running_safety_gate(self):
        deployed = load_yaml(
            "config/config/package_config/tod_safety_gate/params.yaml"
        )
        packaged = load_yaml(
            "src/tod_safety/tod_safety_gate/config/"
            "package_config/tod_safety_gate/params.yaml"
        )

        node = "/vehicle/safety/safety_gate"
        for config in (deployed, packaged):
            params = config[node]["ros__parameters"]
            self.assertEqual(0.8, params["max_velocity"])
            self.assertEqual(0.8, params["warning_velocity"])
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```powershell
python -m pytest tests/test_deployment_config.py -v
```

Expected: the two new tests fail because the operator selector is `edgar`, `maxVelocity` is `10.0`, and `max_velocity` is absent.

- [ ] **Step 3: Commit the failing tests**

```powershell
git add tests/test_deployment_config.py
git commit -m "test: require dual-sided teleop speed limits"
```

### Task 2: Apply Operator and Vehicle Configuration

**Files:**
- Modify: `config/config/package_config/tod_command_creation/params.yaml`
- Modify: `src/tod_direct_control/tod_command_creation/config/package_config/tod_command_creation/params.yaml`
- Modify: `config/config/package_config/tod_safety_gate/params.yaml`
- Modify: `src/tod_safety/tod_safety_gate/config/package_config/tod_safety_gate/params.yaml`

- [ ] **Step 1: Replace both operator configurations**

Use the exact launched node name in both files:

```yaml
/operator/direct_control/CommandCreator:
  ros__parameters:
    InvertSteeringInGearReverse: true
    ConstraintSteeringRate: false
    maxVelocity: 0.8
    maxAcceleration: 4.0
    maxDeceleration: 9.0
    maxSteeringWheelAngleRate: 7.6
```

- [ ] **Step 2: Replace both vehicle safety-gate configurations**

Use the exact launched node name in both files:

```yaml
/vehicle/safety/safety_gate:
  ros__parameters:
    max_velocity: 0.8       # normal teleoperation limit in m/s
    warning_velocity: 0.8   # limit while topic monitoring warns, in m/s
    max_deceleration: 3.5   # retained existing parameter
    timeout: 500            # retained existing parameter in ms
```

- [ ] **Step 3: Run the deployment tests and verify GREEN**

Run:

```powershell
python -m pytest tests/test_deployment_config.py -v
```

Expected: all tests in `test_deployment_config.py` pass.

- [ ] **Step 4: Commit the configuration changes**

```powershell
git add config/config/package_config/tod_command_creation/params.yaml `
  src/tod_direct_control/tod_command_creation/config/package_config/tod_command_creation/params.yaml `
  config/config/package_config/tod_safety_gate/params.yaml `
  src/tod_safety/tod_safety_gate/config/package_config/tod_safety_gate/params.yaml
git commit -m "config: cap teleoperation speed at 0.8 mps"
```

### Task 3: Specify the Vehicle-side Policy

**Files:**
- Create: `src/tod_safety/tod_safety_gate/test/test_primary_control_policy.cpp`
- Modify: `src/tod_safety/tod_safety_gate/CMakeLists.txt`
- Modify: `src/tod_safety/tod_safety_gate/package.xml`

- [ ] **Step 1: Write the failing policy tests**

Create the test file:

```cpp
#include <gtest/gtest.h>

#include "tod_safety_gate/primary_control_policy.hpp"
#include "tod_status_msgs/msg/status.hpp"
#include "tod_topic_monitoring_msgs/msg/topic_state.hpp"

using tod_safety_gate::filter_primary_velocity;

TEST(PrimaryControlPolicy, DoesNotPublishOutsideTeleoperation) {
    const auto result = filter_primary_velocity(
        0.5F,
        tod_status_msgs::msg::Status::TOD_STATUS_IDLE,
        tod_topic_monitoring_msgs::msg::TopicState::STATE_OK,
        0.8F,
        0.8F);
    EXPECT_FALSE(result.has_value());
}

TEST(PrimaryControlPolicy, PassesNormalVelocityBelowLimit) {
    const auto result = filter_primary_velocity(
        0.5F,
        tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION,
        tod_topic_monitoring_msgs::msg::TopicState::STATE_OK,
        0.8F,
        0.8F);
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(0.5F, *result);
}

TEST(PrimaryControlPolicy, CapsNormalVelocityAtVehicleLimit) {
    const auto result = filter_primary_velocity(
        1.2F,
        tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION,
        tod_topic_monitoring_msgs::msg::TopicState::STATE_OK,
        0.8F,
        0.8F);
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(0.8F, *result);
}

TEST(PrimaryControlPolicy, UsesLowerWarningLimit) {
    const auto result = filter_primary_velocity(
        1.2F,
        tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION,
        tod_topic_monitoring_msgs::msg::TopicState::STATE_WARN,
        0.8F,
        0.6F);
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(0.6F, *result);
}

TEST(PrimaryControlPolicy, StopsForMissingOrErroredMonitoring) {
    for (const auto state : {
             tod_topic_monitoring_msgs::msg::TopicState::STATE_NOT_RECEIVED,
             tod_topic_monitoring_msgs::msg::TopicState::STATE_ERROR}) {
        const auto result = filter_primary_velocity(
            0.5F,
            tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION,
            state,
            0.8F,
            0.8F);
        ASSERT_TRUE(result.has_value());
        EXPECT_FLOAT_EQ(0.0F, *result);
    }
}
```

- [ ] **Step 2: Register the test target**

Add to `package.xml`:

```xml
<test_depend>ament_cmake_gtest</test_depend>
```

Add to `CMakeLists.txt` before `ament_package()`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_primary_control_policy
    test/test_primary_control_policy.cpp)
  target_link_libraries(test_primary_control_policy primary_control_policy)
  ament_target_dependencies(test_primary_control_policy
    tod_status_msgs tod_topic_monitoring_msgs)
endif()
```

- [ ] **Step 3: Build the test and verify RED**

Run in the ROS build environment:

```bash
colcon build --packages-select tod_safety_gate --cmake-args -DBUILD_TESTING=ON
```

Expected: build fails because `primary_control_policy.hpp` and the `primary_control_policy` target do not exist.

- [ ] **Step 4: Commit the failing tests**

```bash
git add src/tod_safety/tod_safety_gate/test/test_primary_control_policy.cpp \
  src/tod_safety/tod_safety_gate/CMakeLists.txt \
  src/tod_safety/tod_safety_gate/package.xml
git commit -m "test: specify vehicle teleop speed policy"
```

### Task 4: Implement and Integrate the Vehicle-side Policy

**Files:**
- Create: `src/tod_safety/tod_safety_gate/include/tod_safety_gate/primary_control_policy.hpp`
- Create: `src/tod_safety/tod_safety_gate/src/primary_control_policy.cpp`
- Modify: `src/tod_safety/tod_safety_gate/include/tod_safety_gate/safety_gate.hpp`
- Modify: `src/tod_safety/tod_safety_gate/src/safety_gate.cpp`
- Modify: `src/tod_safety/tod_safety_gate/CMakeLists.txt`

- [ ] **Step 1: Add the policy interface**

```cpp
#pragma once

#include <cstdint>
#include <optional>

namespace tod_safety_gate {

std::optional<float> filter_primary_velocity(
    float requested_velocity,
    uint8_t tod_status,
    uint8_t topic_state,
    float max_velocity,
    float warning_velocity);

}  // namespace tod_safety_gate
```

- [ ] **Step 2: Add the minimal policy implementation**

```cpp
#include "tod_safety_gate/primary_control_policy.hpp"

#include <algorithm>

#include "tod_status_msgs/msg/status.hpp"
#include "tod_topic_monitoring_msgs/msg/topic_state.hpp"

namespace tod_safety_gate {

std::optional<float> filter_primary_velocity(
    float requested_velocity,
    uint8_t tod_status,
    uint8_t topic_state,
    float max_velocity,
    float warning_velocity) {
    if (tod_status != tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
        return std::nullopt;
    }

    switch (topic_state) {
        case tod_topic_monitoring_msgs::msg::TopicState::STATE_NOT_RECEIVED:
        case tod_topic_monitoring_msgs::msg::TopicState::STATE_ERROR:
            return 0.0F;
        case tod_topic_monitoring_msgs::msg::TopicState::STATE_WARN:
            return std::min(requested_velocity,
                            std::min(max_velocity, warning_velocity));
        default:
            return std::min(requested_velocity, max_velocity);
    }
}

}  // namespace tod_safety_gate
```

- [ ] **Step 3: Build a reusable policy library**

Add before the executable in `CMakeLists.txt` and link it:

```cmake
add_library(primary_control_policy src/primary_control_policy.cpp)
target_include_directories(primary_control_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(primary_control_policy
  tod_status_msgs tod_topic_monitoring_msgs)

target_link_libraries(safety_gate primary_control_policy)
```

Include `primary_control_policy` in the existing `install(TARGETS ...)` block.
Install the public header with:

```cmake
install(DIRECTORY include/ DESTINATION include)
```

- [ ] **Step 4: Declare and store the vehicle limit**

Add to `SafetyGateNode`:

```cpp
float max_velocity_{0.8F};
```

Declare and load it in the constructor before subscriptions are created:

```cpp
this->declare_parameter<float>("max_velocity", 0.8F);
this->get_parameter("max_velocity", max_velocity_);
```

- [ ] **Step 5: Route primary commands through the policy**

Include the policy header and replace the callback's outer status check and
velocity assignments with:

```cpp
const auto limited_velocity = filter_primary_velocity(
    msg->velocity,
    tod_status_,
    topic_state_,
    max_velocity_,
    warning_velocity_);
if (!limited_velocity.has_value()) {
    return;
}

switch (topic_state_) {
    case tod_topic_monitoring_msgs::msg::TopicState::STATE_NOT_RECEIVED:
        RCLCPP_WARN_STREAM_ONCE(
            this->get_logger(),
            "not all monitored topics have been received, stopping vehicle");
        break;
    case tod_topic_monitoring_msgs::msg::TopicState::STATE_WARN:
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "warning state received from tod_topic_monitoring, limiting velocity");
        break;
    case tod_topic_monitoring_msgs::msg::TopicState::STATE_ERROR:
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "error state received from tod_topic_monitoring, stopping vehicle");
        break;
}

msg->velocity = *limited_velocity;
primary_control_cmd_publisher_->publish(*msg);
```

- [ ] **Step 6: Run the focused tests and verify GREEN**

```bash
colcon build --packages-select tod_safety_gate --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select tod_safety_gate --event-handlers console_direct+
colcon test-result --verbose
```

Expected: the package builds and all `tod_safety_gate` tests pass.

- [ ] **Step 7: Commit the implementation**

```bash
git add src/tod_safety/tod_safety_gate
git commit -m "feat: enforce vehicle teleop speed limit"
```

### Task 5: Document and Verify the Complete Change

**Files:**
- Modify: `src/tod_safety/tod_safety_gate/README.md`

- [ ] **Step 1: Document both vehicle parameters**

Update the parameter table to include:

```markdown
| max_velocity | double | 0.8 | maximum velocity for every teleoperation command |
| warning_velocity | double | 0.8 | maximum velocity while topic monitoring reports a warning |
```

- [ ] **Step 2: Run repository configuration tests**

```powershell
python -m pytest tests/test_deployment_config.py tests/test_peanut01_dry_run.py -v
```

Expected: all selected tests pass.

- [ ] **Step 3: Run whitespace and change-scope checks**

```powershell
git diff --check
git status --short
```

Expected: no whitespace errors; only the planned files and pre-existing user changes appear.

- [ ] **Step 4: Commit the documentation**

```powershell
git add src/tod_safety/tod_safety_gate/README.md
git commit -m "docs: describe teleop speed limits"
```

### Task 6: Roll Out and Verify Both Hosts

**Files:**
- Operator deployment: `/home/user/teleoperated_driving_deploy/config/config/package_config/tod_command_creation/params.yaml` on `user@192.168.188.16`
- Vehicle deployment: the corresponding source/config in the vehicle image used on `nvidia@8.155.20.255:7027`

- [ ] **Step 1: Confirm the updated vehicle image contains the new parameter**

Before restarting either side, inspect the candidate vehicle image:

```bash
ros2 run tod_safety_gate safety_gate --ros-args --help >/dev/null
```

Then start it in the normal Compose deployment and require the parameter dump
in Step 4 to contain `max_velocity`. Do not treat a YAML-only update as a
successful vehicle rollout if the node does not declare the parameter.

- [ ] **Step 2: Deploy the corrected operator configuration**

Update the mounted operator file from the reviewed repository version, then
recreate only `tod_operator` using its existing Compose profiles:

```bash
cd /home/user/teleoperated_driving_deploy
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  up -d --force-recreate tod_operator
```

- [ ] **Step 3: Deploy the updated vehicle image and configuration**

In `/home/nvidia/teleoperated_driving_vehicle`, update only the reviewed files
from this repository, then build and recreate the existing Peanut01 video
vehicle service with:

```bash
cd /home/nvidia/teleoperated_driving_vehicle
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.override.yaml \
  -f docker-compose.peanut01-dry-run.yaml \
  -f docker-compose.peanut01-video.yaml \
  up -d --build --force-recreate tod_vehicle
```

Before running the command, inspect the active container's Compose labels. If
its `com.docker.compose.project.config_files` value differs, use that exact
ordered file list instead of the list above so launch arguments are preserved.

- [ ] **Step 4: Verify effective ROS parameters**

On the operator:

```bash
ros2 param get /operator/direct_control/CommandCreator maxVelocity
```

Expected: `Double value is: 0.8`.

On the vehicle:

```bash
ros2 param get /vehicle/safety/safety_gate max_velocity
ros2 param get /vehicle/safety/safety_gate warning_velocity
```

Expected: both values are `0.8`.

- [ ] **Step 5: Perform a non-motion topic-level verification**

With the vehicle prevented from actuating, publish or replay a primary command
above `0.8 m/s` into the safety-gate input and verify its output is exactly
`0.8 m/s`. Repeat in forward and reverse gear, confirming the speed magnitude
is unchanged by gear selection. Do not perform an unrestricted ground motion
test as part of automated verification.

- [ ] **Step 6: Record rollout evidence**

Capture the effective parameter outputs, container image identifiers, and the
topic-level clamp result in the deployment log or handoff notes.

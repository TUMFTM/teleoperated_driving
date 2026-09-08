# Peanut01 TOD Projection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Display Peanut01 body-edge guide lines in `tod_visual` using measured front tire angle and a rear-axle kinematic bicycle model.

**Architecture:** Add a small, ROS-message-independent rear-axle projection library inside `tod_projection`, then let `OperatorLaneProjection` select either the existing legacy calculation or the new model from ROS parameters. Peanut01 deployment configuration selects tire-angle input and the rear-axle model; default package configuration preserves legacy behavior for other vehicles.

**Tech Stack:** C++17, ROS 2 Humble, `rclcpp`, `nav_msgs`, `tod_vehicle_msgs`, `ament_cmake_gtest`, Python `unittest`/`pytest`, YAML launch configuration.

---

## File Structure

- Create `src/tod_perception/tod_projection/include/tod_projection/rear_axle_projection.hpp`: pure geometry types, validation, and projection API.
- Create `src/tod_perception/tod_projection/src/rear_axle_projection.cpp`: rear-axle bicycle integration and body-corner transformation.
- Create `src/tod_perception/tod_projection/test/test_rear_axle_projection.cpp`: deterministic geometry unit tests.
- Modify `src/tod_perception/tod_projection/include/tod_projection/operator_lane_projection.hpp`: projection mode, steering source, and configuration state.
- Modify `src/tod_perception/tod_projection/src/operator_lane_projection.cpp`: parameter loading, validation, angle selection, and conversion to ROS paths.
- Modify `src/tod_perception/tod_projection/src/CMakeLists.txt`: build and link the geometry library.
- Modify `src/tod_perception/tod_projection/CMakeLists.txt`: register gtest and install configuration.
- Modify `src/tod_perception/tod_projection/package.xml`: add the gtest dependency.
- Create `src/tod_perception/tod_projection/config/package_config/tod_projection/params.yaml`: legacy-compatible package defaults.
- Modify `src/tod_perception/tod_projection/launch/tod_projection.launch.py`: load the projection parameter file.
- Create `config/config/package_config/tod_projection/params.yaml`: Peanut01 rear-axle/tire-angle deployment configuration.
- Modify `config/config/launch_setup_peanut01_video.yaml`: enable `tod_projection`.
- Create `tests/test_peanut01_projection.py`: deployment wiring and parameter regression tests.

### Task 1: Rear-Axle Projection Geometry

**Files:**
- Create: `src/tod_perception/tod_projection/include/tod_projection/rear_axle_projection.hpp`
- Create: `src/tod_perception/tod_projection/src/rear_axle_projection.cpp`
- Create: `src/tod_perception/tod_projection/test/test_rear_axle_projection.cpp`
- Modify: `src/tod_perception/tod_projection/src/CMakeLists.txt`
- Modify: `src/tod_perception/tod_projection/CMakeLists.txt`
- Modify: `src/tod_perception/tod_projection/package.xml`

- [ ] **Step 1: Write failing geometry tests**

Define a `Geometry` with wheelbase `0.8`, front overhang `0.295`, rear overhang `0.5778`, width `1.193`, and maximum tire angle `1.047`. Test that:

```cpp
TEST(RearAxleProjection, StraightPathStartsAtCurrentBodyCorners) {
  const auto paths = project(geometry(), 0.0, 1, 6.0, 40);
  ASSERT_EQ(paths.front_left.size(), 41U);
  EXPECT_NEAR(paths.front_left.front().x, 1.095, 1e-9);
  EXPECT_NEAR(paths.front_left.front().y, 0.5965, 1e-9);
  EXPECT_NEAR(paths.rear_right.front().x, -0.5778, 1e-9);
  EXPECT_NEAR(paths.rear_right.front().y, -0.5965, 1e-9);
  EXPECT_NEAR(paths.front_left.back().x, 7.095, 1e-9);
}

TEST(RearAxleProjection, SteeringSignsBendInOppositeDirections) {
  const auto left = project(geometry(), 0.2, 1, 6.0, 40);
  const auto right = project(geometry(), -0.2, 1, 6.0, 40);
  EXPECT_GT(left.front_left.back().y, left.front_left.front().y);
  EXPECT_LT(right.front_right.back().y, right.front_right.front().y);
}

TEST(RearAxleProjection, ReverseProgressesBehindRearAxle) {
  const auto paths = project(geometry(), 0.0, -1, 6.0, 40);
  EXPECT_LT(paths.rear_left.back().x, paths.rear_left.front().x);
}
```

Also test steering clamping and rejection of non-positive wheelbase, width, prediction length, and step count.

- [ ] **Step 2: Run the test to verify it fails**

Run inside a ROS 2 operator build environment:

```bash
colcon test --packages-select tod_projection --event-handlers console_direct+
```

Expected: build or test failure because `rear_axle_projection.hpp` and its implementation do not exist.

- [ ] **Step 3: Implement the pure projection library**

Expose focused standard-library types:

```cpp
struct Geometry {
  double wheelbase;
  double front_overhang;
  double rear_overhang;
  double width;
  double maximum_tire_angle;
};

struct Point { double x; double y; };
struct Paths {
  std::vector<Point> front_left;
  std::vector<Point> front_right;
  std::vector<Point> rear_left;
  std::vector<Point> rear_right;
};

Paths project(const Geometry &geometry, double tire_angle, int direction,
              double prediction_length, std::size_t prediction_steps);
```

Validate inputs with `std::invalid_argument`, clamp tire angle with
`std::clamp`, append the current footprint first, and then integrate exactly
`prediction_steps` equal-distance segments.

- [ ] **Step 4: Register and run the gtest**

Build a `rear_axle_projection` library, link it to `OperatorLaneProjection`,
and register `test_rear_axle_projection` under `BUILD_TESTING` using
`ament_add_gtest`.

Run:

```bash
colcon test --packages-select tod_projection --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all `tod_projection` geometry tests pass.

- [ ] **Step 5: Commit the geometry model**

```bash
git add src/tod_perception/tod_projection
git commit -m "feat: add rear axle lane projection model"
```

### Task 2: ROS Node Selection And Validation

**Files:**
- Modify: `src/tod_perception/tod_projection/include/tod_projection/operator_lane_projection.hpp`
- Modify: `src/tod_perception/tod_projection/src/operator_lane_projection.cpp`
- Create: `src/tod_perception/tod_projection/config/package_config/tod_projection/params.yaml`
- Modify: `src/tod_perception/tod_projection/launch/tod_projection.launch.py`
- Extend test: `src/tod_perception/tod_projection/test/test_rear_axle_projection.cpp`

- [ ] **Step 1: Add failing steering-selection tests**

Add a pure helper with explicit modes and test it independently:

```cpp
enum class SteeringAngleSource { SteeringWheelAngle, SteeringTireAngle };

EXPECT_DOUBLE_EQ(select_steering_angle(
    SteeringAngleSource::SteeringTireAngle, 0.7, 0.12, 6.1, 1.047), 0.12);
EXPECT_NEAR(select_steering_angle(
    SteeringAngleSource::SteeringWheelAngle, 0.7, 0.12, 6.1, 1.047),
    0.7 / 6.1 * 1.047, 1e-9);
EXPECT_THROW(parse_steering_angle_source("unknown"), std::invalid_argument);
```

- [ ] **Step 2: Run tests and observe the expected failure**

Run:

```bash
colcon test --packages-select tod_projection --event-handlers console_direct+
```

Expected: compile failure because the source-selection API does not exist.

- [ ] **Step 3: Integrate the new model into the node**

Declare and read:

```cpp
steering_angle_source = "steering_wheel_angle";
kinematic_reference = "legacy_center";
prediction_length_m = 6.0;
prediction_steps = 40;
```

For `rear_axle`, derive:

```cpp
wheelbase = distance_front_axle + distance_rear_axle;
front_overhang = distance_front_bumper - distance_front_axle;
rear_overhang = distance_rear_bumper - distance_rear_axle;
```

Select `msg.steering_tire_angle` for Peanut01, reject non-finite input, clamp
inside the geometry library, and convert the returned points to four stamped
`nav_msgs/msg/Path` messages in `base_footprint`. Keep the existing calculation
unchanged for `legacy_center`.

- [ ] **Step 4: Add package defaults and launch loading**

Create the default parameter file:

```yaml
/**:
  ros__parameters:
    steering_angle_source: steering_wheel_angle
    kinematic_reference: legacy_center
    prediction_length_m: 6.0
    prediction_steps: 40
```

Update `tod_projection.launch.py` to pass
`config_path/package_config/tod_projection/params.yaml` to the node.

- [ ] **Step 5: Run geometry tests and build the package**

```bash
colcon build --packages-up-to tod_projection --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select tod_projection --event-handlers console_direct+
colcon test-result --verbose
```

Expected: package builds and all projection tests pass.

- [ ] **Step 6: Commit node integration**

```bash
git add src/tod_perception/tod_projection
git commit -m "feat: support measured tire angle projection"
```

### Task 3: Peanut01 Deployment Wiring

**Files:**
- Create: `config/config/package_config/tod_projection/params.yaml`
- Modify: `config/config/launch_setup_peanut01_video.yaml`
- Create: `tests/test_peanut01_projection.py`

- [ ] **Step 1: Write failing deployment tests**

Test the exact deployed contract:

```python
def test_peanut01_enables_projection():
    profile = load_yaml(PROFILE)
    assert profile["packages_to_launch"]["operator"]["tod_projection"] is True

def test_peanut01_uses_rear_axle_and_tire_feedback():
    params = load_yaml(PARAMS)["/**"]["ros__parameters"]
    assert params["steering_angle_source"] == "steering_tire_angle"
    assert params["kinematic_reference"] == "rear_axle"
    assert params["prediction_length_m"] == 6.0
    assert params["prediction_steps"] == 40
```

Also assert all four existing projection-to-visual remappings remain present.

- [ ] **Step 2: Run the deployment test and verify it fails**

```bash
pytest -q tests/test_peanut01_projection.py
```

Expected: failure because projection is disabled and the deployment parameter
file does not exist.

- [ ] **Step 3: Add Peanut01 configuration**

Create:

```yaml
/**:
  ros__parameters:
    steering_angle_source: steering_tire_angle
    kinematic_reference: rear_axle
    prediction_length_m: 6.0
    prediction_steps: 40
```

Set `packages_to_launch.operator.tod_projection: True` in the Peanut01 video
profile. Do not enable trajectory guidance or pure pursuit.

- [ ] **Step 4: Run focused and full Python tests**

```bash
pytest -q tests/test_peanut01_projection.py
pytest -q
```

Expected: focused test passes and the full Python suite has no failures.

- [ ] **Step 5: Commit deployment wiring**

```bash
git add config/config/package_config/tod_projection/params.yaml \
  config/config/launch_setup_peanut01_video.yaml \
  tests/test_peanut01_projection.py
git commit -m "feat: enable Peanut01 lane projection"
```

### Task 4: Build And Runtime Verification

**Files:**
- No source changes expected.

- [ ] **Step 1: Build the operator target**

```bash
docker compose build tod_operator
```

Expected: the operator image, including `tod_projection`, builds successfully.

- [ ] **Step 2: Verify launch description without vehicle actuation**

Run the operator container with the Peanut01 video launch configuration and
confirm the ROS graph contains:

```text
/operator/projection/LaneProjection
/operator/projection/output/vehicle_lane_front_left
/operator/projection/output/vehicle_lane_front_right
/operator/projection/output/vehicle_lane_rear_left
/operator/projection/output/vehicle_lane_rear_right
```

Expected: the node and four path topics exist. This verification must not
start or modify vehicle-side actuation.

- [ ] **Step 3: Run final regression checks**

```bash
pytest -q
git diff --check
git status --short
```

Expected: all tests pass, no whitespace errors, and only intentional user-owned
pre-existing changes remain unstaged.

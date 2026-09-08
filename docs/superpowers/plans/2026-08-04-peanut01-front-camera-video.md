# Peanut01 Front Camera Video Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream the Peanut01 front camera through TOD RTSP at 960x540 and 1500 Kbit/s while preserving the ROS-domain-7 control path.

**Architecture:** Run only `VehicleRtspServer` in sensor ROS domain 0 and keep all other TOD nodes in domain 7. Use a separate Compose overlay and Peanut01 video launch profile so the existing control-only deployment remains a rollback option. Build the changed vehicle RTSP package as a small overlay on the Jetson instead of rebuilding the full vehicle image.

**Tech Stack:** ROS 2 Humble, Python launch, CycloneDDS, GStreamer RTSP/H.264, Docker Compose, YAML, Python unittest/pytest, C++17.

---

### Task 1: Add Deployment Contract Tests

**Files:**
- Create: `tests/test_peanut01_video.py`

- [ ] **Step 1: Write failing configuration tests**

Test that the video profile enables `tod_rtsp` but not `tod_lidar`, the camera topic resolves to `/sensing/camera/camera1/image_raw`, projection is disabled, stream settings are 1920x1080 input with `0p500` scaling and 1500 Kbit/s, and the Compose overlay selects the video launchers and sensor domain 0.

```python
def test_camera_topic_and_low_latency_profile(self):
    camera = yaml.safe_load(CAMERA.read_text(encoding="utf-8"))
    stream = yaml.safe_load(STREAM.read_text(encoding="utf-8"))["video_settings"]
    self.assertEqual("/sensing/camera/camera1/image_raw", camera["camera_topics_namespace"] + camera["camera0"]["name"] + camera["camera_image_name"])
    self.assertFalse(camera["camera0"]["project_on"])
    self.assertEqual((1920, 1080), (stream["width"], stream["height"]))
    self.assertEqual("0p500", stream["scaling_factor"])
    self.assertEqual(1500, stream["bitrate"])
```

- [ ] **Step 2: Write failing RTSP source-safety tests**

Assert that the vehicle launch file uses `TOD_RTSP_SENSOR_DOMAIN_ID`, both launch files support `TOD_RTSP_CONFIG_PATH`, the server creates one stream when no extra router IP exists, and `push_data()` copies exactly `latest_image_->data.size()` bytes into a GStreamer-owned buffer.

- [ ] **Step 3: Run tests and confirm failure**

Run: `python -m pytest -q tests/test_peanut01_video.py`

Expected: FAIL because the Peanut01 video files and RTSP safeguards do not exist yet.

### Task 2: Add Peanut01 Camera And Launch Configuration

**Files:**
- Create: `config/config/vehicle_config/peanut01/sensors-camera.yaml`
- Create: `config/config/vehicle_config/peanut01/visual-video.yaml`
- Create: `config/config/vehicle_config/peanut01/camera-calibration/camera1.yaml`
- Create: `config/config/vehicle_config/peanut01/stream_settings.yml`
- Create: `config/config/vehicle_config/peanut01/router_settings.yml`
- Create: `config/config/launch_setup_peanut01_video.yaml`
- Create: `src/tod_launch/launch/tod_vehicle_peanut01_video.launch.py`
- Create: `src/tod_launch/launch/tod_operator_peanut01_video.launch.py`
- Create: `docker-compose.peanut01-video.yaml`

- [ ] **Step 1: Add the single-camera sensor definition**

```yaml
camera0:
  name: /camera1
  is_front_facing: true
  is_fisheye: false
  project_on: false
  scalings: [0p500, 1p000]
  transition_bitrates: [500, 2500]

camera_topics_namespace: /sensing/camera
camera_image_name: /image_raw
```

- [ ] **Step 2: Add fixed low-latency stream settings**

Use source dimensions 1920x1080, zero crop, output dimensions 960x540, scaling `0p500`, H.264 zero-latency/ultrafast settings, and bitrate 1500.

- [ ] **Step 3: Add uncalibrated display metadata**

Create an explicitly uncalibrated camera file with 1920x1080 dimensions, identity rectification, and projection disabled in both sensor and visual configuration. Do not invent measured intrinsics.

- [ ] **Step 4: Add a video launch profile**

Copy the existing Peanut01 control package selection, change only `tod_rtsp` to `True`, and leave `tod_lidar`, projection, trajectory guidance, and transforms disabled.

- [ ] **Step 5: Add dedicated top-level launchers**

The vehicle and operator launchers must parse `launch_setup_peanut01_video.yaml`. The operator launcher retains `managerOnly=true`.

- [ ] **Step 6: Add a non-destructive Compose overlay**

The overlay must:

```yaml
services:
  tod_vehicle:
    environment:
      TOD_RTSP_SENSOR_DOMAIN_ID: ${TOD_RTSP_SENSOR_DOMAIN_ID:-0}
      TOD_RTSP_CONFIG_PATH: /home/tum/wsp/install/tod_launch/share/tod_launch/config/vehicle_config/peanut01
    command: ["bash", "-lc", "source /home/tum/peanut01_overlay/install/setup.bash && source /home/tum/peanut01_video_overlay/install/setup.bash && exec ros2 launch tod_launch tod_vehicle_peanut01_video.launch.py vehicleNetworkInterface:=${VEHICLE_NETWORK_INTERFACE:-eth1}"]
```

Mount the vehicle RTSP overlay read-only, mount the new top-level launchers into the installed launch directory, and do not replace the existing `/dev/input` or software-config mounts.

- [ ] **Step 7: Run configuration tests**

Run: `python -m pytest -q tests/test_peanut01_video.py tests/test_peanut01_dry_run.py`

Expected: RTSP source-safety tests still fail; configuration tests pass.

### Task 3: Make RTSP Safe And Sensor-Domain Aware

**Files:**
- Modify: `src/tod_network/tod_rtsp/launch/tod_rtsp_vehicle.launch.py`
- Modify: `src/tod_network/tod_rtsp/launch/tod_rtsp_operator.launch.py`
- Modify: `src/tod_network/tod_rtsp/src/vehicle/rtsp_server.cpp`
- Modify: `src/tod_network/tod_rtsp/src/vehicle/rtsp_stream.cpp`

- [ ] **Step 1: Isolate the vehicle video node in the sensor domain**

Set only `VehicleRtspServer`'s `ROS_DOMAIN_ID` through:

```python
additional_env={
    "ROS_DOMAIN_ID": EnvironmentVariable(
        "TOD_RTSP_SENSOR_DOMAIN_ID",
        default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
    )
}
```

Use `TOD_RTSP_CONFIG_PATH` as an optional override for Peanut01 stream/router settings on both vehicle and operator launches.

- [ ] **Step 2: Support a portable empty router list**

When `router_settings.yml` contains `ips: []`, create one server stream using `0.0.0.0` as the display address. The operator already creates its primary stream from the connected vehicle address.

- [ ] **Step 3: Apply initial scaling before dynamic reconfiguration**

In `gst_media_configure()`, derive 960x540 from the configured 1920x1080 and `0p500`, round dimensions to a multiple of 8, and set the `myscale` caps before streaming starts.

- [ ] **Step 4: Give GStreamer ownership of image bytes**

Replace the borrowed ROS vector pointer with an allocated GStreamer buffer:

```cpp
const auto data_size = latest_image_->data.size();
GstBuffer *buffer = gst_buffer_new_allocate(nullptr, data_size, nullptr);
gst_buffer_fill(buffer, 0, latest_image_->data.data(), data_size);
```

This removes the incorrect `width * step` length and prevents the ROS message from being freed while GStreamer still uses the data.

- [ ] **Step 5: Run all local tests**

Run: `python -m pytest -q tests/test_peanut01_video.py tests/test_peanut01_dry_run.py work/test_g923_persistence.py work/test_set_g923_autocenter.py`

Expected: all tests pass.

- [ ] **Step 6: Commit implementation**

```bash
git add docker-compose.peanut01-video.yaml config/config src/tod_launch/launch src/tod_network/tod_rtsp tests/test_peanut01_video.py
git commit -m "feat: add Peanut01 front camera streaming"
```

### Task 4: Build And Deploy The Vehicle RTSP Overlay

**Files:**
- Remote create: `/home/nvidia/teleoperated_driving_vehicle/.peanut01_video_overlay/`
- Remote update: `/home/nvidia/teleoperated_driving_vehicle/`
- Remote update: `/home/user/teleoperated_driving_deploy/`

- [ ] **Step 1: Run local static validation**

Run `git diff --check`, all related pytest suites, and `docker compose ... config` where Docker is available.

- [ ] **Step 2: Upload only required files to both endpoints**

Do not delete remote overrides or restart sensor/perception containers. Upload the new Compose overlay, Peanut01 config, launch wrappers, and RTSP source changes.

- [ ] **Step 3: Build only `tod_rtsp` on the vehicle**

Use the existing vehicle image and underlay:

```bash
colcon --log-base /overlay/log build --packages-select tod_rtsp \
  --install-base /overlay/install \
  --build-base /overlay/build \
  --cmake-args -DVEHICLE=ON -DOPERATOR=OFF -DCMAKE_BUILD_TYPE=Release
```

Expected: `tod_rtsp` completes successfully and `/overlay/install/setup.bash` exists.

- [ ] **Step 4: Validate both merged Compose configurations**

Use base, existing runtime override, control-only overlay, then video overlay in that order. Confirm only `VehicleRtspServer` receives sensor domain 0 and the main container still has ROS domain 7.

### Task 5: Restart TOD And Verify Video Plus Control

- [ ] **Step 1: Record pre-restart state**

Record running TOD, camera, perception, CenterPoint, and actuation containers. Confirm the camera topic still publishes and port 8554 is free.

- [ ] **Step 2: Recreate only the vehicle TOD service**

Start `tod_vehicle` with all four Compose files. Do not use `docker compose down`.

- [ ] **Step 3: Verify the vehicle stream**

Confirm `VehicleRtspServer` exists in ROS domain 0, subscribes to the camera topic, listens on port 8554, and exposes `/camera1`.

- [ ] **Step 4: Recreate only the operator TOD service**

Start the G923 helper and `tod_operator` with the same Compose stack. Confirm the helper exits 0 and operator startup remains non-blocking.

- [ ] **Step 5: Verify decoded video**

Confirm `/operator/network/video/camera1/image` publishes 960x540 images and video info reports nonzero frame rate. Confirm `tod_visual` remains running.

- [ ] **Step 6: Regression-check control**

Move the steering wheel and pedals while the vehicle wheels remain suspended. Confirm joystick messages at the operator and primary control commands at the vehicle; do not send autonomous motion commands.

- [ ] **Step 7: Verify unaffected containers**

Confirm camera, perception, CenterPoint, and actuation container start times did not change.

- [ ] **Step 8: Roll back on failure**

If video verification fails, recreate only TOD services without `docker-compose.peanut01-video.yaml`; retain logs and leave all sensor and control containers untouched.

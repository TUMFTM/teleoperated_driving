# Peanut01 Front Camera Video Design

## Goal

Add one low-latency front-camera stream to the existing Peanut01 teleoperation deployment without changing the LiDAR/radar stack or the existing vehicle-control path.

## Existing Environment

- Vehicle TOD container: ROS domain 7, host networking.
- Operator TOD container: ROS domain 7, host networking.
- Camera publisher: ROS domain 0.
- Source topic: `/sensing/camera/camera1/image_raw`.
- Source type: `sensor_msgs/msg/Image`.
- Source format: 1920x1080 `rgb8`, approximately 2.4 FPS during inspection.
- RTSP port 8554 is currently unused.

## Architecture

The vehicle `VehicleRtspServer` process runs in ROS domain 0 so it can subscribe directly to the camera topic. All other TOD processes remain in ROS domain 7. The RTSP server encodes the image as H.264 and exposes `/camera1` over the vehicle's host network.

The operator `OperatorRtspClients` process remains in ROS domain 7. It obtains the vehicle address from the existing TOD connection state, connects to `rtsp://192.168.188.70:8554/camera1`, decodes the stream, and publishes the image for `tod_visual`.

This avoids copying raw 6.2 MB images through a ROS domain bridge. The initial implementation uses fixed stream settings because the operator-side video configuration service cannot cross from domain 7 to the RTSP server in domain 0.

## Configuration

- Vehicle ID: `peanut01`.
- Camera name: `/camera1`.
- Camera namespace: `/sensing/camera`.
- Image suffix: `/image_raw`.
- Codec: H.264 with zero-latency tuning.
- Output resolution: 960x540.
- Target bitrate: 1500 Kbit/s.
- RTSP port: 8554.
- Camera projection: disabled for the first phase.
- Camera calibration: the current `/sensing/camera/camera1/camera_info` reports zero-valued matrices, so store an explicit uncalibrated placeholder and keep projection disabled. It must not be presented as a measured calibration.
- LiDAR/radar: disabled.

The sensor ROS domain is configured through `TOD_RTSP_SENSOR_DOMAIN_ID` and defaults to the container's existing ROS domain when unset. Only the vehicle RTSP process receives the domain override.

## Safety And Failure Handling

- Do not modify or restart camera, perception, CenterPoint, or vehicle actuation containers.
- Restart only the two TOD containers after configuration validation.
- Keep all control and safety nodes in ROS domain 7.
- A missing camera or failed RTSP server must not prevent TOD control nodes from starting.
- Preserve the current control-only configuration as the rollback path.

## Verification

1. Validate the Peanut01 camera and launch configuration before deployment.
2. Confirm `VehicleRtspServer` subscribes to `/sensing/camera/camera1/image_raw` in domain 0.
3. Confirm TCP port 8554 is listening on the vehicle.
4. Confirm the operator connects to `rtsp://192.168.188.70:8554/camera1`.
5. Confirm the operator publishes decoded camera images and reports nonzero video frame rate.
6. Confirm `tod_visual` displays the front-camera stream.
7. Confirm steering, accelerator, and brake commands still reach the vehicle after video startup.

## Out Of Scope

- LiDAR and millimeter-wave radar.
- Multiple cameras.
- Dynamic bitrate or crop control across ROS domains.
- Changes to sensor drivers or the perception stack.

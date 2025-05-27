# tod_visual

## Overview

The package `tod_visual` contains everything necessary to visualize videos, environmental data, vehicle state, management components, and more. It is primarily divided into three directories:

- **`tod_applications`**: Contains connection settings between the operator and the vehicle, allowing adjustments and selection of the control concept.
- **`tod_entities`**: Describes all entities that can be visualized within the virtual interface, including objects detected by vehicle automation or maps (e.g., Lanelet2 maps).
- **`tod_gl`**: Provides the graphics library responsible for all visualization tasks.

Entities and rendering are managed separately to facilitate rapid modifications and easy addition of new entities. Refer to each directory's README and Doxygen documentation for detailed explanations.

## Dependencies

- `rclcpp`
- `nav_msgs`
- `lanelet2_core`
- `lanelet2_io`
- `lanelet2_projection`
- `lanelet2`
- `pcl_ros`
- `tf2_geometry_msgs`
- `tod_helper`
- `tod_core`
- `tod_vehicle_msgs`
- `tod_automation_msgs`
- `tod_config_msgs`
- `tod_network_monitoring_msgs`
- `tod_trajectory_guidance_msgs`
- `tod_status_msgs`
- `tod_operator_msgs`
- `cv_bridge`

## Build Instructions

Build the package using:

```console
colcon build --packages-up-to tod_visual
```

## Launch Files

Launch visualization using:

```console
ros2 launch tod_visual tod_visual.launch.py
```

## Configuration Files

Example parameters (`config/package_config/tod_visual/params.yaml`):

```yaml
enable_driving_lane: true              # Shows lane visualization
enable_lanelet_map: true               # Shows Lanelet map
enable_object_list: true               # Shows detected object bounding boxes in 3D visualization
enable_point_cloud: true               # Shows transmitted and compressed point cloud in the 3D environment
enable_trajectory: true                # Shows trajectory for trajectory guidance
map: "bakerlap_v1.1.1.osm"             # Map file located in 'tod_visual/resources/maps'; leave empty if no map is used
map_origin: [48.13912660925, 11.55603317499, 0.0]  # Latitude, longitude, altitude of map origin; select a point included in the map
```

For further details, refer to the individual READMEs within each directory.

<details>
<summary>Old Documentation</summary>

# tod_visual

The package provides a function-rich and flexible HMI for the operator to perform the teleoperation.
It displays received data from the Perception and Bridge packages in various ways.
A 3D world, comparable to the [rviz package](http://wiki.ros.org/rviz), is constructed using the OpenGL API. Inspired by the open
source [game engine Hazel](https://github.com/TheCherno/Hazel), this package uses the Entity Component System (ECS) design pattern through the
[entt library](https://skypjack.github.io/entt/), based on the composition over inheritance principle. There is also support for an HMD.
Snapshots of the HMI, exhibiting a view of the vehicle model and giving an impression of the projection of video streams
on either rectangles or a spherical canvas, can be seen in the examples below.

## Usage

It is possible to interact with the tod_visual and to change the camera perspective. The following key presses are used:
  * Arrow Up/Down/Left/Right: Circle around target point
  * Page Up/Down: Move target point up and down

## Dependencies
  * ROS Packages: see `package.xml`
  * Third Party:
    * assimp
    * GLM
    * glfw3 3.2
    * Freetype
    * yaml-cpp
    * OpenGL
    * OpenVR
    ```bash
    sudo apt-get install libassimp-dev -y
    sudo apt-get install libglm-dev -y
    sudo apt-get install libglfw3-dev -y
    sudo apt-get install libfreetype6-dev -y
    sudo apt-get install libyaml-cpp-dev -y
    # OpenGL
    sudo apt-get install build-essential libxmu-dev libxi-dev libgl-dev libosmesa-dev -y
    sudo apt-get install libglew-dev -y
    # OpenVR
    sudo add-apt-repository multiverse -y
    sudo apt-get install steam steam-devices libvulkan1 -y
    cd /tmp && git clone https://github.com/ValveSoftware/openvr.git openvr
    cd openvr && git checkout tags/v1.14.15
    mkdir build && cd build && cmake .. && make -j32
    sudo make install && sudo ldconfig
    cd /tmp && sudo rm -r openvr && cd
    ```

## Documentation

The HMI is constructed through a number of entities, listed in the following table.
| Entity Class | Description | Sample Usage (and Subscriptions) |
|---|---|---|
| Base Footprint | To render the position of vehicle. | Track the states of vehicle. (`/Operator/VehicleBridge/odometry`) |
| Camera | To render the camera perspective. | Creates an openGL camera that can be assigned to a framebuffer. |
| Coordinate System | To render a relative position of (data) objects in the scene. | Describe relative positions of data, e.g., a laser scan relative to the sensor. Used to transform positions in coordinate system of Base Footprint to coherently render the complete scene. |
| Display | To render alpha-numeric content (text). | Display various vehicle data such as desired/actual velocity, gear position, etc. (`/Operator/VehicleBridge/vehicle_data`, `/Operator/CommandCreation/primary_control_cmd`, `/Operator/CommandCreation/secondary_control_cmd`)|
| Floor | To render a virtual ground of the 3D world.| Introduce a virtual floor to improve interaction capabilities. |
| Grid | To render a grid of the 3D world. | Display a squared grid to improve spatial perception. |
| Laser Scan | To render 2D lidar scans. | Display point cloud data. (`/Operator/Lidar/*/scan`) |
| Object List | To render objects in the scene. | Can be used to visualize a topic of type tod_automation_msgs::ObjectList. |
| Path | To render paths. | Display projected vehicle motion (`/Operator/Projection/vehicle_lane_front_*`) |
| Top View | To render rectangular display of scene, captured by another scene camera. | Display bird's eye view of scene. Usually useful for narrow corridor or parking maneuvers. |
| Video | To render videos on video canvas (Supported projection modes: `RECTANGULAR=0`, `SPHERE=1`, `HALF_SPHERE_WITH_GROUND_PLANE=2`, `GROUND_PLANE=3`, `ROBINSON=4` [unwrap sphere to flat surface]). | Display video streams. (`/Operator/Video/*/image_raw`) |

The HMI also detects key presses and mouse clicks, published under the following topics.
  * `/Operator/Visual/KeyPress` [[tod_operator_msgs/KeyPress](TBD)]
  * `/Operator/Visual/MousePositionClick` [[geometry_msgs/Point](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Point.html)]

When launching the HMI with a given `vehicleID`, the data of some entities is partially serialized in `yaml/<vehicleID>.yaml`. This file can be modified before launching the HMI again.

## Demo

A rosbag for a playback demo has been recorded in the [SVL Driving Simulator](https://www.svlsimulator.com/). Execute the following steps to run the demo.
  * Clone the [container repository](https://github.com/TUMFTM/teleoperated_driving) and follow the steps under "Getting started" to build the workspace.
    For brevity, you can build `tod_visual`, only. See above, for the commands to install its third party dependencies.
    ```bash
    catkin build tod_visual
    ```
  * Download the rosbag from [here](https://mediatum.ub.tum.de/1636609?v=1) and place it under the following file path:
    `/home/$USER/Documents/ToD_Playback/lgsvl-for-visual.bag`.
  * From the sourced workspace, launch the demo.
    ```bash
    source devel/setup.bash # or `setup.zsh`, depending on your shell
    roslaunch tod_visual demo.launch
    ```
  * The `tod_visual` should open and playback the demo as shown below.
    <img src="doc/playback.gif" width="700" />

## Video Display Examples

Examples of the HMI with videos being displayed on a spherical video canvas or on rectangles.
![Alt](doc/visual_sphere.png "display of videos on sphere")
![Alt](doc/visual_rectangle.png "display of videos on rectangles")

<details>
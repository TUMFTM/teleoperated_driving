# TUM FTM Teleoperated Driving Software

This is the container repository of the TUM FTM Teleoperated Driving Software Stack. The stack is ROS-based and tested on Ubuntu 18.04 and 20.04 with ROS Melodic/Noetic, only. We are currently working on `ROS2` support.

Within this repository, the ROS packages are grouped thematically in the following submodules:
  * wsp/src/tod_common
  * wsp/src/tod_vehicle_interface
  * wsp/src/tod_perception
  * wsp/src/tod_operator_interface
  * wsp/src/tod_control

A video, showcasing the software on three different vehicle systems is available on [YouTube](https://www.youtube.com/watch?v=bQZLCOpOAQc).
[![Video Demonstration](https://img.youtube.com/vi/bQZLCOpOAQc/0.jpg)](https://www.youtube.com/watch?v=bQZLCOpOAQc)

## System Architecture

The system architecture is depicted in the following graphic. The color of the packages corresponds to the grouping of the packages in the respective submodules.

![Alt](doc/architecture_tof.png "system architecture")

## Getting Started

Use the following commands to clone, build and run the TUM FTM Teleoperated Driving Software Stack. **A more extensive documentation for the usage of the software can be found under `doc/`**.

* Clone this repository.
* Initialize all submodules.

  ```bash
  git submodule update --recursive --init
  ```

* Install the dependencies of each respective package. A list of dependencies with install commands is found in each package's README. Alternatively, use the docker images from the folder `docker`.
* Create the config of your vehicle with the corresponding `vehicleID`, following the template in the `tod_vehicle_interface/tod_vehicle_config` package.
* Several nodes are waiting for the `vehicleID` to be available on the parameter server. See package `tod_core` for more details.
* Build and source the workspace.

  ```bash
  cd wsp
  catkin config --extend /opt/ros/melodic
  catkin init --workspace .
  rosdep install --from-paths src --ignore-src -y --rosdistro=melodic # or noetic etc., depending on your distro
  catkin build
  source devel/setup.bash # or `setup.zsh`, depending on your shell
  ```

* On the vehicle side:
  * Set your `vehicleID` in the `tod_common/tod_launch/launch/vehicle.launch` and launch the vehicle nodes.

    ```bash
    roslaunch tod_launch vehicle.launch
    ```

* On the operator side:
  * Set your `vehicleID` in the `tod_common/tod_launch/launch/operator.launch` and launch the operator nodes.

    ```bash
    roslaunch tod_launch operator.launch
    ```

  * In the Manager GUI, enter the IP addresses for the vehicle and the operator side, hit connect, select the control mode and hit start to start with the teleoperation.

## Playback Demo

A rosbag for a playback demo has been recorded in the [SVL Driving Simulator](https://www.svlsimulator.com/). Execute the following steps to run the demo.
* Download the rosbag from [here](https://mediatum.ub.tum.de/1636609?v=1) and place it under the following file path:
  `/home/$USER/Documents/ToD_Playback/lgsvl.bag`.
* In `both.launch`, set `vehicleID:=lgsvl`
* In `vehicle.launch`, set  `mode:=playbackAll` (or `mode:=playbackSim`, see `doc/` for a detailed description of the launch modes).
* After having built and sourced the workspace, run `roslaunch tod_launch both.launch`.
* In the Manager GUI, enter the localhost (`127.0.0.1`) as both, the broker and operator IP address.
* Hit connect.

The data from the rosbag (cameras, laser scanner, vehicle state data) will be played back and visualized the the `tod_visual` window.

## Publication

The paper is available on [IEEE](https://ieeexplore.ieee.org/document/9742859).

```
@INPROCEEDINGS{9742859,
  author={Schimpe, Andreas and Feiler, Johannes and Hoffmann, Simon and Majstorović, Domagoj and Diermeyer, Frank},
  booktitle={2022 International Conference on Connected Vehicle and Expo (ICCVE)}, 
  title={Open Source Software for Teleoperated Driving}, 
  year={2022},
  pages={1-6},
  doi={10.1109/ICCVE52871.2022.9742859}}
```

# TUM FTM Teleoperated Driving Software

This is the container repository of the TUM FTM Teleoperated Driving Software Stack. The stack is ROS-based and tested on Ubuntu 18.04 with ROS Melodic, only.

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
    ```
    git submodule update --recursive --init
    ```
  * Install the dependencies of each respective package. A list of dependencies with install commands is found in each package's README.
  * Create the config of your vehicle with the corresponding `vehicleID`, following the template in the `tod_vehicle_interface/tod_vehicle_config` package.
  * Initialize, build and source the workspace.
    ```
    cd wsp
    catkin init --workspace .
    cd wsp
    catkin build
    source devel/setup.bash # or `setup.zsh`, depending on your shell
    ```
  * On the vehicle side: 
    * Set your `vehicleID` in the `tod_common/tod_launch/launch/vehicle.launch` and launch the vehicle nodes.
      ```
      roslaunch tod_launch vehicle.launch
      ```
  * On the operator side: 
    * Set your `vehicleID` in the `tod_common/tod_launch/launch/operator.launch` and launch the operator nodes.
      ```
      roslaunch tod_launch operator.launch
      ```
    * In the Manager GUI, enter the IP addresses for the vehicle and the operator side, hit connect, select the control mode and hit start to start with the teleoperation.


## Playback Demo
*coming soon*


## Publication
*coming soon*

# The **tod_msgs** Repositoy

This Repository contains packages that hold the custom ROS2 message definitions used throughout the teleoperation software stack. 

The repository holds different packages that represent a set of messages that are assicoated with different interfaces in the tod architecture. For each element in the architecture, a package is defined:

| Package                 | Purpose                                                             | Example |
| --------                | -------                                                             | ------- |
| `tod_automation_msgs`   | Messages related to interaction with the vehicle's automation       | Perception data, Trajectories|
| `tod_config_msgs`       | Services related to configuration of provided data streams          | Video config, Encoder config | 
| `tod_monitoring_msgs`   | Messages related to monitoring of certain system components         | Network metrics | 
| `tod_network_msgs`      | Messages and services related to network and package status         | Package info with latency, size etc.| 
| `tod_operator_msgs`     | Messages that are originating on the operator side                  | Mesh, Colored polygons for visualization | 
| `tod_safety_msgs`       | Messages related to safety mechanisms                               | Safety Issues identified by a watchdog | 
| `tod_status_msgs`       | Messages related to the tod_state_machine                           | Status of the state machine | 
| `tod_vehicle_msgs`      | Messages related to the vehicle's interface (sensors and actuators) | Vehicle kinematic state, Pointcloud Streams | 
| `tod_trajectory_guidance_msgs`      | Messages related to the trajectory guidance workflow and visualizations | Path, Trajectory and Trajectory Guidance State | 



`include` folders inside the packages contain helper headers with message constants 

## Should you add your Custom Messages here?
If you plan to create a message that only your control concept uses: **No** 
<br /> → Do this in your own Repository, e.g. a package `tod_trajectory_guidance_msgs` in the `tod_trajectory_guidance` repository

If you plan to extend existing interfaces or add a new interface definition for the general tod code: **Yes**

## Additional Remarks
Building the packages containing messages and services is done in a typical ROS2 manner (via `CMakeLists.txt` and `package.xml`).

## Doxygen
\version 1.0
\author TUMFTM
\defgroup tod_msgs ToD Messages
\ingroup tod
\brief message definitions used in the TUM teleoperation software-stack
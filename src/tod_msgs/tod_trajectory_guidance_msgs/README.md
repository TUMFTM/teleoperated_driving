# The **tod_trajectory_guidance_msgs** Package

This package contains message definitions specific for trajectory guidance.

| Message                       | Purpose                                                                                   |
| --------                      | -------                                                                                   |
| `ControlPoint`                | Target position (x,y,z) and metadata to track the processing state of the point           |
| `ControlPoints`               | List of ControlPoint, results from input of the Operator                                  |
| `PathPoint`                   | Point on a path via pose and point metadata                                               |
| `Path`                        | List of PathPoint for the reference path as well as left and right boundary               |
| `PpLog`                       | Logging information of PathPoint                                                          |
| `Trajectory`                  | Target trajectory via list of TrajectoryPoint                                             |
| `TrajectoryGuidanceControl`   | Contains continous execution information for the trajectory, including target velocity    |
| `TrajectoryGuidanceState`     | Trajectory Guidance internal state information                                            |
| `TrajectoryPoint`             | Similar to PathPoint, but relevant for execution via the controller                       |


## When to use service or message?
ROS2 services can be used to configure setting when only one node controls the parameters and requests the change with another node.
Differently, the ROS2 message is best applied when mulitple nodes make decisions on the parameter an must be synchronized and know the current parameter status. 

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_trajectory_guidance_msgs```.
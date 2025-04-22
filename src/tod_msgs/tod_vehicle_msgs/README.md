# The **tod_vehicle_msgs** Package

This package contains message definitions relevant for the vehicle interface.

| Message                 | Purpose                                                                                             |
| --------                | -------                                                                                             |
| `CompressedPointCloud`  | Contains compressed point cloud data                                                                |
| `PrimaryControlCmd`     | Primary Control Command to be executed by the vehicle, includes target velocity and steering angle  |
| `PrimaryVehicleState`   | Current Primary Vehicle State from the vehicle, fields similar to the PrimaryControlCmd             |
| `SafetyDriverStatus`    | Current status of the safety driver interface inside the vehicle                                    |
| `SecondaryControlCmd`   | Secondary Control Command to be executed by the vehicle, includes gear, indicators and much more    |
| `SecondaryVehicleState` | Secondary Vehicle State, fields similar to the SecondaryControlCmd                                  |
| `VideoInfo`             | Information about the camera stream provided by the vehicle                                         |


## When to use service or message?
ROS2 services can be used to configure setting when only one node controls the parameters and requests the change with another node.
Differently, the ROS2 message is best applied when mulitple nodes make decisions on the parameter an must be synchronized and know the current parameter status. 

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_vehicle_msgs```.
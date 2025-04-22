# The **tod_status_msgs** Package

This package contains message definitions to publish the current state of the system.

| Message                 | Purpose                                                                         |
| --------                | -------                                                                         |
| `ManagerButtonStatus`   | Encodes user interactions in the tod_visual manager gui to trigger state transitions or value changes in the tod state machine |
| `Status`                | Current system state in the subsystem (vehicle, operator), subscribed by the majority of nodes |


## When to use service or message?
ROS2 services can be used to configure setting when only one node controls the parameters and requests the change with another node.
Differently, the ROS2 message is best applied when mulitple nodes make decisions on the parameter an must be synchronized and know the current parameter status. 

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_status_msgs```.
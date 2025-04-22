# The **tod_safety_msgs** Package

This package contains message definitions to publish safety states after evaluating metrics or encountering errors inside a node. 
The messages are subscribed in the safety layer to counteract accordingly.

| Message                 | Purpose                                                                         |
| --------                | -------                                                                         |
| `GateState`             | Can be used to publish a degradation level after evaluating a metric            |
| `SafetyIssue`           | Describes a Safety Issue that includes a string identifier and a description as well as the respective node where it occured                                 |
| `SafetyState`           | Summarizes multiple SafetyIssues for one node                                  |

## When to use service or message?
ROS2 services can be used to configure setting when only one node controls the parameters and requests the change with another node.
Differently, the ROS2 message is best applied when mulitple nodes make decisions on the parameter an must be synchronized and know the current parameter status. 

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_safety_msgs```.
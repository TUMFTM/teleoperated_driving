# The **tod_operator_msgs** Package

This package contains message definitions to publish metadata of data packages transmitted over the network.

| Message                 | Purpose                                                                         |
| --------                | -------                                                                         |
| `Color`                 | RGB Color with float fields for the three chanels                               |
| `ColoredPoint`          | geometry_msgs::msg::Point with an asigned color                                 |
| `ColoredPolygon`        | List of ColoredPoints that creat a Polygone                                     |
| `KeyPress`              | Id of the pressed key and the respective timestamp                              |
| `Mesh`                  | Defines triangle mesh with colored vertices                                     |

| Service                   | Purpose                                                                       |
| --------                  | --------                                                                      |
| 'InputDevice'             | Used to change the input device via string identifier, returns success        |

## When to use service or message?
ROS2 services can be used to configure setting when only one node controls the parameters and requests the change with another node.
Differently, the ROS2 message is best applied when mulitple nodes make decisions on the parameter an must be synchronized and know the current parameter status. 

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_operator_msgs```.
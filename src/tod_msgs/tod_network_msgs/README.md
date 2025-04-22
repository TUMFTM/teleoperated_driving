# The **tod_network_msgs** Package

This package contains message definitions to publish metadata of data packages transmitted over the network.

| Message                 | Purpose                                                                         |
| --------                | -------                                                                         |
| `PaketInfo`             | Network Metadata about the received data, includes the latency and packet size  |


## When to use service or message?
ROS2 services can be used to configure setting when only one node controls the parameters and requests the change with another node.
Differently, the ROS2 message is best applied when mulitple nodes make decisions on the parameter an must be synchronized and know the current parameter status. 

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_network_msgs```.
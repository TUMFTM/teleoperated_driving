# The **tod_config_msgs** Package

This package contains message definitions relevant for configuring and synchronizing tod system specific node parameters via multiple different decision making nodes.

The package holds different messages and services to carry information from tod configurations:

| Message                 | Purpose                                                             |
| --------                | -------                                                             |
| `VideoConfig`           | Configuration Parameters related to the Video Stream                |


| Service                 | Purpose                                                             
| --------                | -------                                                             |
| `VideoConfig`           | Configuration Parameters related to the Video Stream                |
| `EncoderConfig.srv`     | Configuration for the Video Streaming Encoding                      |


## When to use service or message?
ROS2 services can be used to configure setting when only one node controls the parameters and requests the change with another node.
Differently, the ROS2 message is best applied when mulitple nodes make decisions on the parameter an must be synchronized and know the current parameter status. 

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_config_msgs```.
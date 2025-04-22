# tod_transform
Contains tf-tree broadcasters for odometry (dynamic transforms) and static vehicle tree. For the information on dependencies please refer to the `package.xml`.

## Nodes
The package consists of the following set of nodes for both, the vehicle and operator side.

### CommonOdomTransformPublisher
**Subscription**: `/Vehicle/Interface/Sensing/Odometry/odom` or `/Operator/Network/Data/FromVehicle/odom` ([nav_msgs/msg/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)) for vehicle and operator side respectively

**Publication**: `/tf` [(tf2_msgs/msg/TFMessage)](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html)

**Explanation**: This node publishes a transform from a map frame to a odom frame which is the frame on the moving vehicle which the odometry data is defined in. As the vehicle moves, the transform is changing over time

### CommonTransformTreePublisher
**Publication**: `/tf_static` [(tf2_msgs/msg/TFMessage)](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html)

**Explanation**: This node publishes a set of static transforms, normally used for definition of sensor placements on the vehicle relative to a reference frame on the vehicle
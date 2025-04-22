# tod_command_forwarder
This package holds a node that republishes the primary_control_command that are created in the tod_command_creation package if Direct Control mode is selected.

# Dependencies
> **Please note**  
> This Package was developed for ROS2 Humble.

for `tod_*` package dependencies and external ROS2 Packages: See `package.xml`

## Nodes

This package holds one Node: 

## VehicleForwardPrimaryCtrlCmd
This node republishes the primary_control_command if Direct Control mode is selected.

**Subscriptions:**
 * `/Vehicle/Network/Data/FromOperator/primary_control_cmd` ([tod_vehicle_msgs/msg/PrimaryControlCmd](TODO))

**Published Topics:**
 * `/Vehicle/DirectControl/primary_control_cmd` ([tod_vehicle_msgs/msg/PrimaryControlCmd](TODO))

## Build the Package and launching the Node

* Build and source the workspace.
  ```bash
  colcon build --packages-select tod_direct_control && source install/setup.bash # or `setup.zsh`, depending on your shell
  ```

* Launch the nodes on the operator side
  ```bash
  ros2 launch tod_direct_control tod_direct_control.py
  ```

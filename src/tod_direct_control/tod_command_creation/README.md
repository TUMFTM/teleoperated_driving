# tod_command_creation
This package provides nodes to calculate the primary and secondary control commands from the joystick inputs given by ''tod_input_devices'' and send them to the vehicle.
It launches the `OperatorCmdCreator` node and transmits the primary and secondary commands to the vehicle using respective senders.


# Dependencies

> **Please note**  
> This Package was developed for ROS2 Humble.

for `tod_*` package dependencies and external ROS2 Packages: See `package.xml`

## Nodes

- [CommandCreator](#commandcreator)

### CommandCreator
If connected to vehicle, calculates primary and secondary control commands from joystick message.


**Subscriptions:**
 * `/Operator/InputDevices/joystick` ([sensor_msgs/msg/Joy](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html)) Contains the state of buttons and axes of the current input device.
 * `/Operator/StateMachine/operator_status` ([tod_status_msgs/msg/Status](TODO)) Contains the connection Status to the Vehicle.

**Published Topics:**
 * `/Operator/Network/Data/ToVehicle/primary_control_cmd` ([tod_vehicle_msgs/msg/PrimaryControlCmd](TODO))
 * `/Operator/Network/Data/ToVehicle/secondary_control_cmd` ([tod_vehicle_msgs/msg/SecondaryControlCmd](TODO))

 **Parameters:**

 The following parameters can be set at startup or during runtime
  * `vehicleID`: Identificator for your vehicle as specified in the `tod_vehicle_interface`
  * `InvertSteeringInGearReverse`: True to invert the steering angle when in reverse gear
  * `ConstraintSteeringRate`: True to constraint the steering rate through the `maxSteeringWheelAngleRate` param (see next point)
  * `maxSteeringWheelAngleRate`: Maximum steering rate given to the vehicle's actor in $rad \, s^{-1}$
  * `maxVelocity`: Maximum longitudinal velocity that can be given to the vehicle's actor in $m \, s^{-1}$
  * `maxAcceleration`: Maximum longitudinal acceleration given to the vehicle's actor in $m \, s^{-2}$
  * `maxDeceleration`: Maximum longitudinal deceleration given to the vehicle's actor in $m \, s^{-2}$

 ## Build the Package and launch the Nodes
 * Build and source the workspace.
    ```bash
    colcon build --packages-select tod_command_creation && source install/setup.bash # or `setup.zsh`, depending on your shell
    ```
  * Launch the nodes on the operator side
    ```bash
    ros2 launch tod_command_creation tod_command_creation_operator_launch.py
    ```

  * Launch the nodes on the vehicle side
    ```bash
    ros2 launch tod_command_creation tod_command_creation_vehicle_launch.py
    ```
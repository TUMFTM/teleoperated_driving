# tod_core

## Usage
see [parameter_test_node.cpp](/test/parameter_test_node.cpp)

```cpp
colcon build --packages-select tod_core && . install/setup.zsh
ros2 run tod_core test_node
```
Set vehicleID using

```cpp
ros2 param set test_node vehicleID 'tum-q7'
```
or
```cpp
ros2 service call /test_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: "vehicleID", value: {type: 4, string_value: "tum-q7"}}]}"
```

## Run tests
```cpp

colcon build --packages-select tod_core
. install/setup.zsh
colcon test --packages-select tod_core
colcon test-result --verbose
```
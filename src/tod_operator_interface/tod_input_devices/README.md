# tod_input_devices  

This package allovw to handle multiple input devices for the TUM Teleoperation Software Stack.
Amongst physical input devices such as USB devices, it also provides a 'virtual' input device that lets you steer the vehicle using a Joystick GUI application.



## Supported input devices  
* **USB Input Devices:** Works with various USB input devices like Joysticks, XBOX-Controllers, Steering Wheels, etc.
* **Virtual Input Devices:** Opens a virtual joystick on the screen.
* **Senso Wheel:** Implementation to interact with SensoDrive SensoWheel. 
(CAN-Communication)
  > **Please note**  
  > Building and using the sensowheel code needs PCAN drivers to be installed and loaded. For this, either use the provided tod docker images or install it locally.
  > Also, make sure to turn the option `SENSO ON` in the `src/CMakeLists.txt` file (Line 13)
* **Custom Input Device:** Please adhere to [Adding and Configuring new Input Devices](#adding-and-configuring-new-input-devices). 

## Dependencies  
  * C++17 
  * ROS Packages: see `package.xml`
  * Other Librariesgit 
    * [Qt](https://www.qt.io/) >= 5.9 (Widgets QuickWidgets): 
      ```
      sudo apt-get install qt5-default qml-module-qtquick2 qtpositioning5-dev
      ```
    * PCAN Library if the Sensowheel shall be used

## Nodes

### InputDevice
Reads axis values and button states from hardware or virtual input device.

**Published Topics:**
  * `/operator/input_devices/joystick` ([sensor_msgs/msg/Joy](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/Joy.html)) Contains the state of buttons and axes of the current input device.

**Advertised Services:**
  * `/operator/input_devices/service/change_input_device` ([tod_operator_msgs/srv/InputDevice](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/srv/InputDevice.srv)). Service to switch between different input devices on runtime. This service can be called from the `tod_manager`. The service call requires the path to the desired `.yaml` file (see [Configuration](#configuration)).

**Parameters:**
  * `debug`: True to print debug messages to the console
  * `ConfigMode`: True to print axis and value info logs to the console if new input is recieved
  * Parameters regarding the setup of every input device can be taken from the .yaml files in the `config` folder


## Build & Launch
Build the package using 
  ```
  colcon build --packages-select tod_input_devices
  ```

Launch the Nodes (by default only [InputDevice](#inputdevice)) using
  ```
  ros2 launch tod_input_devices tod_input_devices_launch.py
  ```

If you want to print debug information or the axes values or want to change the default input device, please specify the parameters at startup using the CLI or specify the parameters in the .py launch file and rebuild the package. Parameters can also be changed at runtime using the CLI.

## Adding and Configuring new Input Devices
The folder `/config` contains multiple predefined configurations for input devices. The configuration for the device can be set in `/launch/tod_input_devices_launch.py` with the parameter `config_file_name` or at Runtime using a Service Call as described [here](#inputdevice). 

To create a new configuration for a USB input device proceed with the following steps:

1. Copy the `template.yaml` under `config/` to `<desired_name>.yaml` in the same folder.
2. In `/launch/tod_input_devices_launch.py` set `ConfigMode` to `true` and `config_file_name` to your `<desired_name>.yaml`.  
3. Start the Launch File: `ros2 launch tod_input_devices tod_input_devices_launch.py`  
4. Press a Button -> Assign the number showing up in the Terminal to the desired action in `<desired_name>.yaml`  
5. Move a Axis -> Assign the number showing up in the Terminal to the desired action in `<desired_name>.yaml`  
6. Save `<desired_name>.yaml` and set `ConfigMode` to `false`
7. **Rebuild the package** and use this configuration by setting `ConfigFile` to your `<desired_name>.yaml` in `/launch/tod_input_devices.launch`.

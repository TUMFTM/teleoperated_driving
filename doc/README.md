# Detailed Instructions to Build, Run and Develop this Software

## Build the Software
After cloning this repository, initialization of the submodules, and installation of dependencies, the software can be built.
  * Enter the workspace.  
    `cd tof/wsp`
  * Build the workspace.  
    `catkin build`
  * To speed up compile time or skip nodes (targets) which miss dependencies, there is a list of compile options through which certain targets can be enabled/disabled. 
    `catkin build -DCOMPILE_OPTION=ON/OFF`
      * `OPERATOR`: Build or skip operator side nodes. 
      * `VEHICLE`: Build or skip vehicle side nodes. 
  * In case packages still lack dependencies, and they cannot be skipped using the compile options above, they can be ignored by creating a `CATKIN_IGNORE` file in the respective package. To do this via the command line, use the following command, exemplarily for the `tod_video` package.  
    `touch src/tod_perception/tod_video/CATKIN_IGNORE`

## Launch the Software
Once everything built, the software can be launched. 
  * Source the workspace.  
    `source devel/setup.zsh # or setup.bash - depending on your shell`
  * Launch the nodes. 
    * Operator side.  
    `roslaunch tod_launch operator.launch`
    * Vehicle side.  
      `roslaunch tod_launch vehicle.launch`
    * Both sides.  
      `roslaunch tod_launch both.launch`

## Configure the Launch of the Software
The launch files mentioned above can be configured as required.
  * `vehicleID`: Identifier (string) of the vehicle that is being teleoperated. Each vehicle requires the provision of a configuration in the `tod_vehicle_interface/tod_vehicle_config` package and a bridge package under `tod_vehicle_interface` (for vehicle side only).  
  * `launchPackageName`: Flag (boolean) to enable/disable launch of a package. Useful if certain packages are not needed, or could not be built due missing dependencies. 
  * `mode`: Mode (string) how the software should be launched (relevant for vehicle side only).
    * `vehicle`: To teleoperate an actual vehicle. Launches all drivers and vehicle hardware interfaces as specified in the bridge package of the respective `vehicleID`. 
    * `playbackAll`: Plays back sensor (e.g., camera, lidar, and odometry) and vehicle data signals, coming from a rosbag file.  
    * `playbackSim`: Plays back some sensor data (e.g., camera and lidar) streams. In addition, a vehicle simulation node is launched, consuming control commands and generating odometry and vehicle data streams. 
    * `onlySim`: Launches only the vehicle simulation node. 

## Usage with minimal Requirements
The software can be used and tested on one computer without additional hardware or a rosbag for playback. 
  * Set the `mode` to `onlySim`. 
  * Launch both sides. 
  * In the manager GUI: 
    * Enter the IP address of the localhost (127.0.0.1) for both, the broker and the operator.
    * Hit Connect. 
    * Select the Direct Control mode (should be selected by default).
    * If not already opened up, select the virtual input device. This is a separate window on screen that can be controlled using the keyboard and the mouse. 
    * Hit Start. 
  * In the virtual input device window: 
    * Shift gear up/down using the keys `T` and `G`. (Works only when desired velocity is zero)
    * Increase/decrease desired velocity using the keys `W` and `S`. (Works only the gear position is not in park)
  * In the visual window, see how the desired command values of the display change. If desired velocity is greater zero, the vehicle also starts moving. 
  * In the virtual input device window: 
    * To steer the vehicle, click, hold and drag the grey button left and right.
    * Increase/decrease the desired velocity dragging the grey button up and down. 

## Useful ROS Debugging Tools
  * `rqt_multiplot`: Plots message values in real time. 
    * Launch of a default configuration (with values of, e.g., the desired/actual steering angle, velocity or gear position) can be enabled/disabled in the `operator.launch`.
    * Reset plots with  backspace button in the top right (grey background: all plots). 
    * Configurations can be adapted, stored and loaded with the buttons in the top right as well. 
  * `rviz`: Visualizes certain ROS messages, e.g., paths, markers, point clouds, or laser scans. 
  * `rqt_graph`: Visualizes the connections between ROS nodes of the software. 
  * `rqt_topic`: Monitors message content of topics at runtime. Source the built workspace before starting this tool. Otherwise, some message formats are unknown. 
  * `rqt_image_view`: Visualizes image topics. 
  * In the command line: 
    * `rostopic hz /Topic/Name`: Prints current publish rate under the given topic. 
    * `rostopic echo /Topic/Name`: Prints content of messages being published under the given topic.
    * `rosnode list /Name/Space`: Prints list of currently active nodes within the (optionally) given namespace. Work similar with `rostopic`, `rosservice` or `rosparam`.

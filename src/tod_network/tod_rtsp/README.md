# tod_rtsp

## Overview

This ROS package provides a low-latency video streaming framework. It handles the transmission of multiple videos from the teleoperated vehicle to the operator. Configuration logic other then the specific bandwith settings for each stream are excluded from the package and located in other repositories. Currently used for streaming is the h264 codec.The package supports the use of multiple streaming sources for one image stream. this way, the stability of the stream can be ensured by having redundant streams to choose the latest image from. To allow multiple streams, all static ips of the other routers have to be configured during launch. 

## Nodes

OperatorRtspClients node:
  * Subscribed Topics: 
    * /operator/statemachine/output/operator_status (tod_status_msgs/msg/Status)
  * Published Topics: 
    * /Network/Video/FromVehicle/frontcenter/image (sensor_msgs/msg/Image)
    * /Network/Video/FromVehicle/frontcenter/video_info (tod_vehicle_msgs/msg/VideoInfo)
    * /Network/Video/FromVehicle/frontcenter/paket_info (tod_network_msgs/msg/PaketInfo)
  * Services: None
  * Parameters:
    * router_config_path: string to the router configuration file (should be locally stored in the config folder, might be overwritten for global configuration)
    * stream_config_path: string to the camera configuration folder (should be locally stored in the config folder, might be overwritten for global configuration)
    * image_output_format: string stating the image format. currently supported: RGB, UYVY, BGR, BGRA, GRAY8)
    * rtsp_port: port of the expected rtsp stream

## Dependencies
  * C++17
  * ROS Packages: see `package.xml`
  * Third Party:
    * [GStreamer](https://gstreamer.freedesktop.org/) (gstreamer-1.0, gstreamer-rtsp-server-1.0, gstreamer-rtp-1.0, gstreamer-app-1.0)
      ```bash
      sudo apt-get install libgstreamer1.0-0 -y
      sudo apt-get install gstreamer1.0-plugins-base -y
      sudo apt-get install gstreamer1.0-plugins-good -y
      sudo apt-get install gstreamer1.0-plugins-bad -y
      sudo apt-get install gstreamer1.0-plugins-ugly -y
      sudo apt-get install gstreamer1.0-libav -y
      sudo apt-get install gstreamer1.0-doc -y
      sudo apt-get install gstreamer1.0-tools -y
      sudo apt-get install gstreamer1.0-x -y
      sudo apt-get install gstreamer1.0-alsa -y
      sudo apt-get install gstreamer1.0-gl -y
      sudo apt-get install gstreamer1.0-gtk3 -y
      sudo apt-get install gstreamer1.0-qt5 -y
      sudo apt-get install gstreamer1.0-pulseaudio -y
      sudo apt-get install libgstreamer1.0-dev -y
      sudo apt-get install libgstreamer-plugins-base1.0-dev -y
      sudo apt-get install libgstrtspserver-1.0-dev -y
      ```

## TOD Package dependencies
the following tod packages are required:
  * tod_network_msgs
  * tod_status_msgs
  * tod_vehicle_msgs
  * tod_core

## Build
* install gstreamer dependencies
* modify the camera configuration of your platform or add a new folder for your platform.
* modify the ip address configuration if your platform has additional routers for streams in addition to the default ip set during connection
* build
```console
colcon build --packages-up-to tod_rtsp
```

## RUN
* OperatorRtspClients on the operator side: 
  * run with custom parameters:
    ```bash
    ros2 run tod_rtsp OperatorRtspClients --ros-args -p router_config_path:=<yourpath/to/router/config> -p stream_config_path:=<yourpath/to/camera/config> -p image_output_format:=<your format> -p rtsp_port:<yourport>
    ```

## Launch Files

* launch with launch file and default parameters:
    ```bash
    ros2 launch tod_rtsp launch_client.launch.py
    ```

## Configuration Files

Configuration files used are:
  * router_settings.yml: contains a list of ips as strings. Add additional ips for all additional routers that you plan to use on the vehicle to stream data. 
  * sensors-camera.yaml

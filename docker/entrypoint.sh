#!/bin/bash

###--- Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS2 ${ROS_DISTRO}"

###--- Source workspace
if [ -f /home/${DOCKER_USERNAME}/wsp/install/setup.bash ]
then
  source /home/${DOCKER_USERNAME}/wsp/install/setup.bash
  echo "Sourced ToD Workspace"
fi

###--- Configure DDS paths based on architecture
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    # ARM64
    echo "/opt/ros/humble/lib/aarch64-linux-gnu/" | sudo tee /etc/ld.so.conf.d/libddsc.conf > /dev/null
elif [ "$ARCH" = "x86_64" ]; then
    # x86-64
    echo "/opt/ros/humble/lib/x86_64-linux-gnu/" | sudo tee /etc/ld.so.conf.d/libddsc.conf > /dev/null
else
    echo "Could not determine path to DDS lib on architecture: $ARCH"
    exit 1
fi

###--- Configure librclcpp and tod_msgs paths based on architecture
echo "/opt/ros/humble/lib/" | sudo tee /etc/ld.so.conf.d/librclcpp.conf > /dev/null
echo "/home/tum/wsp/install/tod_msgs/lib/" | sudo tee /etc/ld.so.conf.d/libtod_msgs.conf > /dev/null

###--- Load can drivers   
if [ "$LOAD_PCAN_DRIVERS" = "true" ]; then
  sudo modprobe pcan
fi

###--- Update dynamic linker bindings
sudo ldconfig

###--- Update capabilities necessary for packet capturing
if [ -f /home/${DOCKER_USERNAME}/wsp/install/tod_network_monitoring/lib/tod_network_monitoring/PacketLogger ]; then
    sudo setcap "cap_net_raw,cap_dac_override+ep" install/tod_network_monitoring/lib/tod_network_monitoring/PacketLogger
else
    echo "Could not set capabilities to PacketLogger! Try building first and then source again"
fi

###--- Execute passed command
exec "$@"
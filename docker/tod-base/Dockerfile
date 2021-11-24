# syntax = docker/dockerfile:1.0-experimental
FROM nvidia/cudagl:10.0-devel-ubuntu18.04
LABEL Name=tod-base Version=0.0.1

# Prevent user interaction while installing packages
ENV DEBIAN_FRONTEND="noninteractive" \
    TERM="xterm"
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections


#############
## General ##
#############
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    apt-utils \
    build-essential \
    cmake \
    make \
    git \
    pkg-config \
    unzip \
    usbutils \
    vim \
    nano \
    wget \
    zsh \
    curl \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


#############
#### ROS ####
#############

RUN sh -c "echo 'deb http://packages.ros.org/ros/ubuntu bionic main' > /etc/apt/sources.list.d/ros-latest.list" && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y "ros-melodic-desktop" && \
    apt install python-rosdep && \
    rosdep init && \
    rosdep update \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

###########
## GCC 8 ##
###########
RUN apt-get -y update && apt-get install -y gcc-8 g++-8 && rm -rf /var/lib/apt/lists/* \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7

###############
## Libraries ##
###############

RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    libcurlpp-dev \
    libassimp-dev \
    alsa-utils \
    avahi-utils \
    pulseaudio \
    pulseaudio-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

###############
## ROS Stuff ##
###############
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    ros-melodic-rqt-multiplot \
    rviz \
    ros-melodic-tf2 \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-velodyne \
    ros-melodic-velodyne-msgs \
    ros-melodic-grid-map\
    ros-melodic-grid-map-core \
    ros-melodic-grid-map-ros \
    ros-melodic-pcl-ros \
    ros-melodic-pcl-conversions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*



### catkin tools
RUN sh -c "echo 'deb http://packages.ros.org/ros/ubuntu bionic main' > /etc/apt/sources.list.d/ros-latest.list" && \
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add - && \
    apt-get update && \
    apt-get install -y python-catkin-tools


#################
###### Paho client for MQTT #####
#################
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    fakeroot \
    devscripts \
    dh-make \
    lsb-release \
    libssl-dev \
    doxygen \
    graphviz \
    libcppunit-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN cd /tmp/ && \
    git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c/ && \
    git checkout v1.3.1 && \
    cmake -Bbuild -H. -DPAHO_WITH_SSL=ON -DPAHO_ENABLE_TESTING=OFF && \
    cmake --build build/ --target install && \
    ldconfig && \
    cd /tmp && \
    rm -rf paho.mqtt.c

RUN cd /tmp/ && \
    git clone https://github.com/eclipse/paho.mqtt.cpp.git && \
    cd paho.mqtt.cpp && \
    git checkout v1.1 && \
    cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE && \
    cmake --build build/ --target install && \
    ldconfig && \
    cd /tmp && \
    rm -r paho.mqtt.cpp


#############################
## GSTREAMER for tod_video ##
#############################
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*



########################
## GLM, GLFW and GLEW ##
########################
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    libglm-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

#######################
### nvida settings ####
#######################
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

######################
## Additional Setup ##
######################
# change root password to root
RUN echo "root:root" | chpasswd

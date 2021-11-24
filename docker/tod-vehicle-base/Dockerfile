# syntax = docker/dockerfile:1.0-experimental
FROM tod-base
LABEL Name=tod-vehicle-base Version=0.0.1


##################
## ROS Packages ##
##################
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    ros-melodic-image-exposure-msgs \
    ros-melodic-wfov-camera-msgs \
    ros-melodic-statistics-msgs \
    ros-melodic-velodyne-driver \
    ros-melodic-robot-localization \
    ros-melodic-oxford-gps-eth \
    ros-melodic-autoware-msgs \
    ros-melodic-ackermann-msgs \
    ros-melodic-lgsvl-msgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


###############################
## Mosquitto Broker for MQTT ##
###############################
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    mosquitto \
    mosquitto-clients \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN sudo /etc/init.d/mosquitto stop \
    rm /etc/mosquitto/mosquitto.conf
COPY ./tod-vehicle-base/mosquitto.conf /etc/mosquitto/


#############################
## GSTREAMER for tod_video ##
#############################
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    libgstrtspserver-1.0-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


#######################
## Docker Entrypoint ##
#######################
COPY ./tod-vehicle-base/scripts/* /usr/bin/
RUN ["chmod", "+x", "/usr/bin/docker-entrypoint.sh"]
ENTRYPOINT ["/usr/bin/docker-entrypoint.sh"]

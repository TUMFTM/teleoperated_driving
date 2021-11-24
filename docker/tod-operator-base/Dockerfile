# syntax = docker/dockerfile:1.0-experimental
FROM tod-base
LABEL Name=tod-operator-base Version=0.0.1


###############
## Qt Stuff ##
###############
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    qt5-default \
    qt5-qmake \
    qtbase5-dev-tools \
    qt5-doc \
    qml-module-qtquick2 \
    qtpositioning5-dev \
    qtlocation5-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


########################
## GLM, GLFW and GLEW ##
########################
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    libglfw3-dev \
    libglew-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


##############
## POLYLINE ##
##############
RUN cd /tmp/ && \
    git clone https://github.com/CrushedPixel/Polyline2D.git polyline && \
    cd polyline && \
    mkdir build && \
    cd build && \
    cmake .. && \
    sudo make install -j32 && \
    sudo ldconfig && \
    cd /tmp && \
    rm -r polyline


##############
## FREETYPE ##
##############
RUN cd /tmp/ &&\
    git clone https://github.com/aseprite/freetype2.git && \
    cd freetype2 && \
    cmake -E make_directory build && \
    cmake -E chdir build cmake -D BUILD_SHARED_LIBS:BOOL=true .. && \
    cmake --build build && \
    sudo cmake --build build --target install && \
    sudo ldconfig && \
    cd /tmp && \
    rm -r freetype2


############
## OPENVR ##
############
RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    software-properties-common \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository multiverse -y

RUN apt-get -y update && apt-get install -q -y --no-install-recommends \
    steam-devices \
    libvulkan1 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    git clone https://github.com/ValveSoftware/openvr.git openvr && \
    cd openvr && \
    git checkout tags/v1.14.15 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j32 && \
    sudo make install && \
    sudo ldconfig && \
    cd /tmp && \
    rm -r openvr

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


######################
## Additional Setup ##
######################
COPY ./tod-operator-base/scripts/* /usr/bin/
RUN ["chmod", "+x", "/usr/bin/docker-entrypoint.sh"]
ENTRYPOINT ["/usr/bin/docker-entrypoint.sh"]

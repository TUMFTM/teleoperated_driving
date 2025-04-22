# ToD Logger {#tod_logger_docs}
 
## Overview
 
This package depicts the Logging component of the ToD-Architecture. The purpose of this package is to bundle all available debug and logging infos within the entire ToD-System and provide it to the user or operator. Depending on the user's preference both the operator's and vehicle's loggings are considered or either the operator or the vehicle part is logged. 
 
 
## Nodes
 
Node TopicLogger: Subscribes all topics with a given namespace (/Operator/Logging, /Operator/Debug, /Vehicle/Logging, /Vehicle/Debug)

    Subscribed Topics: All topics within the namespace /Operator/Logging, /Operator/Debug, /Vehicle/Logging, /Vehicle/Debug
    Published Topics: -
    Services: -
    Parameters: logfileSuffix (string, "_tod_log", suffix for the logfile name), log_path (string, "./", path of the log file), logger_namespace (string, "both", logging mode: both, operator, vehicle)

 
## Prerequisites / Dependencies
 
This Package was developed for ROS2 Humble.
 
## ToD Package dependencies
- diagnostic_msgs
- yaml-cpp
 
## Build
 
```console
colcon build --packages-up-to tod_logger
```

## Running the Package
 
```console 
ros2 run tod_logger TopicLogger
```

## Launch Files
 
```console
ros2 launch tod_logger tod_logger.launch.py
```
 
## Configuration Files

find the configuration file in ```config/config/package_config/tod_logger```


## Doxygen Information
\version 1.0
\author TUMFTM
\defgroup tod_logger ToD Logger
\ingroup tod_logging
\brief Implementation of the Logger
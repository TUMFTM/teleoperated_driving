# tod_topic_monitoring {#tod_topic_monitoring_docs}

## Overview

The tod_topic_monitoring package provides tools to detect timeouts of pre-defined topics.


## Nodes

### topic_monitor

This node initialized a topic watchdog for all the topics defined in the ```config/package_config/tod_topic_monitoring/params.yaml```. A tod_topic_monitoring_msgs/TopicState is published.

#### Subscribed Topic

| topic name | message type |
|---|---|
| input/status | tod_status_msgs/Status |

#### Published Topics

| topic name | message type |
|---|---|
| output/topic_monitoring_status | tod_topic_monitoring_msgs/TopicState |

#### Parameters

| parameter | type | default value | description |
|---|---|---|---|
| warning_factor | float | 0.5 | multiply this with timeout to get a warning status for the topics |
| topic_names | std::vector<std::string> | {"input/status"} | topic names to be monitored |
| topics.topic_name.type | std::string | - | type of the topic e.g. tod_status_msgs/Status |
| topics.topic_name.timeout | int | - | timeout of the topic in ms |
| topics.topic_name.control_modes | std::vector<int64_t> | - | control modes the topic should be monitored in |

## Prerequisites / Dependencies

This Package was developed for Ros2 Humble.

## TOD Package dependencies

- tod_status_msgs
- tod_topic_monitoring_msgs

## Build

```console
colcon build --packages-up-to tod_topic_monitoring
```

## Running the Package

```console
ros2 run tod_topic_monitoring topic_monitor
```

## Launch Files

```console
ros2 launch tod_topic_monitoring tod_topic_monitoring.launch.py
```

## Configuration Files

find the configuration file in ```config/package_config/tod_topic_monitoring```

## Doxygen stuff
\version 1.0
\author Florian Pfab
\defgroup tod_topic_monitoring ToD Network Monitoring
\defgroup tod_topic_watchdog ROS2 Topic Watchdog
\brief tools to monitor topic timeouts
\ingroup tod_monitoring
# tod_network_monitoring_msgs
 
## Overview
 
This package contains the message & service definitions needed to interact with the tod_network_monitoring package.
 
## Messages
### NetworkMetrics
| Type              | Name              | Unit                  |
| ---               | ---               | ---                   |
| std_msgs/Header   | header            | -                     |
| float32           | rx_bitrate_mbps   | megabits per second   |
| float32           | tx_bitrate_mbps   | megabits per second   |
| float32           | rx_packets_s      | packets per second    |
| float32           | tx_packets_s      | packets per second    |
| float32           | latency           | milliseconds          |
| float32           | link_quality      | success rate [0,1]    |

## Services
### BandwidthService
| Type              | Name                  | Unit/Description                                              |
| ---               | ---                   | ---                                                           |
| string            | hostname              | IP-address                                                    |
| bool              | test_vehicle_upload   | if test should be executed in a reverse direction (iperf3 -R) |
|                   |                       |                                                               |
| int64             | bitrate_mbps          | megabits per second                                           |
| int64             | transferred_bytes     | bytes                                                         |

### LatencyService
| Type              | Name                  | Unit/Description                                              |
| ---               | ---                   | ---                                                           |
| string            | hostname              | IP-address                                                    |
|                   |                       |                                                               |
| float32           | latency               | seconds                                                       |

### NetworkMonitorService
| Type              | Name                  | Unit/Description                                              |
| ---               | ---                   | ---                                                           |
| bool              | set_monitor_mode      | 0 = inactive, 1 = active                                      |
|                   |                       |                                                               |
| bool              | is_active             | 0 = inactive, 1 = active                                      |

### PacketCaptureService
| Type              | Name                  | Unit/Description                                              |
| ---               | ---                   | ---                                                           |
| bool              | set_capture_mode      | 0 = inactive, 1 = active                                      |
|                   |                       |                                                               |
| bool              | is_active             | 0 = inactive, 1 = active                                      |

## Prerequisites / Dependencies

This Package was developed for Ros2 Humble.
### Dependencies
- ament_cmake
- std_msgs
- rosidl_default_generators

## Build

```console
colcon build --packages-up-to tod_network_monitoring_msgs

```

## Doxygen
\version 1.0
\author TUMFTM
\defgroup tod_network_monitoring_msgs ToD Monitoring Messages
\ingroup tod_msgs
\brief message definitions used to monitor the network
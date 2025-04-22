# tod_network_monitoring {#tod_network_monitoring_docs}

This package contains a suite of standalone network monitoring tools, wrapped in their corresponding ROS2 nodes.

Every active measurement is being triggered via Service Calls from the Operator over the `tod_operator_interface`, while passive measurements are implicitly published to a dedicated topic called `NetworkMetrics`, being available on both Operator and Vehicle side.

## Nodes

### Network Monitor

The Network Monitor module passively measures various network metrics such as latency, link quality, and throughput. It publishes these metrics to a ROS2 topic named `NetworkMetrics`, which is available on both vehicle and operator sides with their respective suffixes.

#### Key Features:

- **Metrics**: Latency, link quality, and throughput measurements.
- **Link Estimation**: Under the hood, this module runs the **Rate-Adaptive-Link-Quality-Estimator** (RALQE), which reacts to lost packets and provides an estimation of the link quality. Reactiveness to packet loss is set to an empirically chosen value but might be adapted if needed (currently defined in `include/tod_network_monitoring/network_monitor.hpp` as RALQ_DECAY=0.75).

#### ROS2 Topics:

- `/operator/monitoring/output/network_metrics`
- `/vehicle/monitoring/output/network_metrics`
- See `tod_network_monitoring_msgs/msgs/NetworkMetrics` for the message type.

#### ROS2 Service Calls:

- **set_monitoring_status**:
    - Request:
        - (string) vehicle_ip_address: the IP address or Hostname of the vehicle. In the case of Vehicle -> Operator, this corresponds to the Operator IP address or Hostname.
        - (bool) set_monitor_mode: **true**, if monitoring should be started and **false** if it should be stopped.
    - Response:
        - (bool) is_active: the current monitoring status.

#### ROS2 Launch Parameters:

- **network_interface**: The target network interface for the throughput monitoring. This parameter can be passed as a launch argument for deployment containers in `docker-compose.yaml` (see Configuration).
- **update_timeout**: (seconds) The timeout periode to configure the behaviour of awaiting responses like ICMP_ECHO_RESPONSE.
- **update_time_interval**: (seconds) The time interval to take periodic measurements (e.g. from a network interface), the value should be set to a fraction of UPDATE_TIMEOUT typically half or less.

#### Configuration

In `include/config.h`:
- **UPDATE_TIME_INTERVAL**: the interval between sending ICMP_ECHO_REQUESTS.
- **UPDATE_TIMEOUT**: the timout interval for expecting an ICMP_ECHO_RESPONSE.

In `~/docker-compose.yaml` for ros2 launch command:
- **vehicleNetworkInterface:=**
    - default: `eth0`
    - *EDGAR*: `eth_opt_0` (verify the correct port at the switch)
    - *RC-Car*: `tun0`
- **operatorNetworkInterface:=**
    - Operator: `tun0` (when using VPN)

### Packet Logger

The Packet Logger module utilizes `libpcap` to capture packets from the specified network interface and store them in a **.pcap** file. Using the `OperatorManagerWidget`, we can choose between not capturing, capturing only on the operator side, or capturing both on the operator and vehicle.

#### Key Features:

- **Interface Configuration**: Traffic can be captured on different network interfaces. It is crucial to select the correct interface, see Configuration.
- **Log Storage**: The storage location for the resulting trace can be modified, see Configuration. In the case of docker, ensure that the target volume **is mounted in docker** as well.
- **Custom Packet Filters**: The packet filter applied to the capture session can be chosen from a variety of pre-defined filters, or custom filters can be added and compiled.

#### ROS2 Service Calls:

- **set_capture_status**:
    - Request:
        - (bool) set_capture_mode: **true**, if packet capture should be started and **false** if it should be stopped. Make sure to call this service again and stop it, as otherwise the trace can not be written to storage.
    - Response:
        - (bool) is_active: the current capturing status.

#### ROS2 Launch Parameters:
- **network_interface**: The target network interface, where packets should be captured. This parameter can be passed as a launch argument for deployment containers in `docker-compose.yaml` (see Configuration).
- **logging_directory**: The target directory to store the log files.

#### Configuration

In `include/config.h`:
- **LOG_PATH**: the absolute path, where the resulting trace will be stored as a `.pcap` file.

In `~/docker-compose.yaml` for ros2 launch command:
- **vehicleNetworkInterface:=**
    - default: `eth0`
    - *EDGAR*: `eth_opt_0` (verify the correct port at the switch)
    - *RC-Car*: `tun0`
- **operatorNetworkInterface:=**
    - Operator: `tun0` (when using VPN)

In `include/filter_definitions.h`:
- Here we can choose from pre-defined filters or just define a custom one. The filter syntax is analogue to the one used in `Wireshark`. By making the filter more specific, we can save storage!

#### Capabilities for Capturing

Due to certain restrictions, we can't do packet capture with capabilities on `tod_dev` because Docker mounts `/install` on a 'nosuid' filesystem.

##### Workarounds for tod_dev:

1. Run `ros2 launch` with `sudo`:
    - (+) Allows packet capture.
    - (-) Runs the whole stack as root.

2. Do not mount `/install` to `/home/tum/wsp/install`:
    - (+) Keeps `/install` on an 'suid' filesystem.
    - (+) Capability management is taken care of automatically by `entrypoint.sh`
    - (-) Requires rebuilding with colcon every time container is restarted.

##### Out-of-the-Box for tod_vehicle and tod_operator:
`entrypoint.sh` will:

1. Add capabilities to the packet logger executable:
    ```sh
    sudo setcap "cap_net_raw,cap_dac_override+ep" install/tod_network_monitoring/lib/tod_network_monitoring/PacketLogger
    ```

2. Dynamically link the necessary libraries.

**NOTE:** If there is a bug in the future regarding runtime linker bindings, identify the missing library and add it to `/etc/ld.so.conf.d/` by adapting the `entrypoint.sh` file.

### Network Tester

The Network Tester module is responsible for launching performance tests against the network. **BE RESPONSIBLE** when launching bandwidth tests, as it will flood the network with traffic.

#### Key Features:

- **Latency Test**: Send multiple ICMP_ECHO_REQUEST, measure the response time and average the results for an estimation of the expected RTT.
- **Bandwidth Test**: Execute Uplink and Downlink throughput measurements from the perspective of the vehicle.

#### ROS2 Service Calls:

- **latency_service**:
    - Request:
        - (string) hostname: the IP address or Hostname of the target host.
    - Response:
        - (float32) latency: the measured latency in milliseconds.
- **bandwidth_service**:
    - Request:
        - (string) hostname: the IP address or Hostname of the target host where the `iperf3` server is running.
        - (bool) test_vehicle_upload: **true** if reverse measurement (download) and **false** if normal measurement (upload).
    - Response:
        - (int64) bitrate_mbps: the measured maximum of achievable bitrate in megabits per second.
        - (int64) transferred_bytes: the total amount of transferred bytes during the measurement.

#### Configuration

Requires an `iperf3` server running **on the vehicle** to execute bandwidth tests. For *EDGAR* as well as for the *RC-Car*, `systemd` is configured to automatically launch `iperf3.service` on startup, and will restart it on failure.

In `include/config.h`:
- **IPERF3_PORT**: the TCP port on which `iperf3` expects incoming connections.
- **BANDWIDTH_MEASUREMENT_TIME**: the duration in seconds for how long we want to execute a bandwidth test.

#### ROS2 Launch Parameters:
- **update_time_interval**: (seconds) The time interval to take periodic measurements (e.g. from a network interface), the value should be set to a fraction of UPDATE_TIMEOUT typically half or less.
- **ping_timeout**: (seconds) The time until until break if no response is observed.
- **iperf3_port**: The Port wo which the IPERF3 server opens a TCP port and the client connects to.
- **bandwidth_measurement_time**: (seconds) The duration for the bandwidth test.

## Dependencies

All necessary dependencies are accounted for in the `dockerfile`, but for completeness, here is a compiled list of dependencies that this module requires:
```bash
libpcap-dev     # to capture packets from a network interface and write them to a .pcap file
iputils-ping    # to execute latency measurements
iperf3          # to launch bandwidth tests
```

## TOD Package dependencies

- tod_network_monitoring_msgs

## Build
 
```console
colcon build --packages-up-to tod_network_monitoring
```
 
## Running the Package
 
```bash
ros2 run tod_network_monitoring network_monitor --ros-args -p network_interface:=eth0 -p update_timeout:=0.3 -p update_time_interval:=0.1
ros2 run tod_network_monitoring packet_logger --ros-args -p network_interface:=eth0 -p logging_directory:="/var/log/tod_network_monitoring"
ros2 run tod_network_monitoring network_tester --ros-args -p update_time_interval:=0.1 -p ping_timeout:=4 -p iperf3_port:=5201 -p bandwidth_measurement_time:=5
```
 
## Launch Files
  
```console
ros2 launch tod_network_monitoring tod_network_monitoring_operator.launch.py
ros2 launch tod_network_monitoring tod_network_monitoring_vehicle.launch.py
```
 
## Configuration Files
 
- If using the provided library for network monitoring, the params in ```include/config.h``` are used.
- If running the node using ```ros2 run ...```, the params in ```include/config.h``` are used.
- If running the nodes using ```ros2 launch ...```, the params in ```config/params.yaml```are used.

## Doxygen stuff
\version 1.0
\author Florian Pfab
\defgroup tod_network_monitoring ToD Network Monitoring 
\brief tools to monitor the network performance
\ingroup tod_monitoring
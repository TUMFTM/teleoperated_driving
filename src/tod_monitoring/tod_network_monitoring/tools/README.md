
# TOD Tools

## Overview

This folder contains a collection of nice utilities to work with ROS2 topics, network interface configurations and *more*.

- **Extract Scripts:** The `extract_*` scripts provide functionality to extract data from a topic that is currently published to or a pre-captured `ROS2 bag` and store the captured messages in a CSV file.
- **Merge Scripts:** The `merge` scripts combine multiple CSV files and store the merged data in a single file.
- **Plot Scripts:** The `plot_*` scripts provide functionality to visualize the given input data using `matplotlib`.
- **Protocol Analysis:** It is possible to analyze a given `.pcap` trace and compute protocol usage, as well as plot it in a nice way.
- **Traffic Control:** This shell script provides a nice interactive wrapper around the `tc qdisc` utility. Messing with your network interface just became fun!

## Usage

To ensure you have all the necessary dependencies installed, run the following command:

`pip install -r requirements.txt`

### Data Extractor for GNSS

This script subscribes to a ROS2 topic to extract GNSS data and saves it to a CSV file. Per default, the script extracts `latitude`, `longitude`, and `height`.

#### Steps to Use:

1. **Ensure message defintions:** Ensure `novatel_oem7_msgs` is available in your `$PYTHONPATH`.
   - **Important:** **DO NOT** use the package from the 'apt' repo, but rather use the one provided by the `EDGAR` stack.
2. **Publish the topic:**
   - Run `ros2 bag play` with your ROS bag file containing the GNSS data **OR** use a topic that is currently available.
   - Change `TOPIC_NAME` in the script to your desired topic if needed.
3. **Execute the script:**
   - Run `python3 extract_bestgnsspos.py`.
4. **Stop the script:**
   - Let the bag replay finish, then hit `CTRL+C` in the terminal where the Python script is running.

### Data Extractor for NetworkMetrics

This script can be used for arbitrary topics. Just make sure to `import` the message definitions at the top of the script.

#### Steps to Use:

Follow the same steps as for [Data Extractor for GNSS](#data-extractor-for-gnss).

### Merging CSV files

To correlate GNSS data with the correlating latency/link_quality at the given timestamp, you can use the `merge_gnss_and_metrics.py` script. This will merge both input CSV files at a timestamp, with a given `TOLERANCE` value. Feel free to adjust to your needs. It may be straightforward with other topics!

### Plotting GNSS and NetworkMetrics

These scripts generate an interactive map to visualize latency and link quality data with given latitude and longitude points from a CSV file (e.g. the resulting file from merging above).

- **Functionality:**
  - Reads GNSS and NetworkMetrics data from a CSV file.
  - Applies smoothing to latitude and longitude values.
  - Normalizes latency / link quality for color mapping.
  - Saves the map to an HTML file which you can open in your browser.

#### Steps to Use:

You can adjust the `INPUT_FILEPATH` parameter to the path where your merged data CSV file is stored. Also, the colormap can be changed as well as the actual map tiles.

Matplotlib colormaps: [here](https://matplotlib.org/stable/users/explain/colors/colormaps.html)

Folium map tiles: [here](https://python-visualization.github.io/folium/latest/getting_started.html#Choosing-a-tileset)


### Data Extractor for .pcap Files

This skeleton can be used to extract meaningful data out of `.pcap` files. These files can be created using the `PacketLogger` module from `tod_network_monitoring` or from tools like **Wireshark** and **tcpdump**.

#### Steps to Use:

Adapt `compute_protocol_usage.py` to your needs. Per default, it filters out traffic between two hosts (e.g. operator <--> vehicle) and computes the usage between UDP, TCP and ICMP.

Just run `python3 compute_protocol_usage.py`. It will save the results in a CSV file that can be used for visualization. Feel free to manipulate the filters in the script to match your specific use-case!

### Plotting Protocol Usage

The corresponding plot script for the extracted CSV data from the stage before produces a simple bar graph that represents the protocol usage with some nice coloring.

### Traffic Control

If you want to simulate different network conditions in your Docker container or on your host machine, the shell script `traffic_control.sh` provides a nice interactive menu to do so! You can choose to add delay, limit bandwidth and burst, and also simulate packet loss.

#### Steps to Use:

Executing `./traffic_control.sh` puts you into a nice menu where you have various options to manipulate your network interface. Make sure that this interface is the correct one! Also, *be careful* when using this in Docker, as you might lose connection when packet loss is too high!

*NOTE for Docker*: Depending on the network driver of your container (e.g. Bridge vs. Host), you might want to be cautious when using this script. In host-mode, you share the same network stack with the host (hence the name). Configurations to the network interface in your Docker container **will also apply to your host machine**!

#### Changing Docker Network Drivers

In your `docker-compose.yaml`, add or modify the `network_mode` parameter.

- **network_mode: host**
    - the container will share the network stack with the host machine. therefore, there is no network isolation.
    - applications running in docker can be directly accessed via the network without the need for port-forwarding.
    - default behaviour for TOD deployment containers.
- **network_mode: bridge**
    - the container has its own network stack (e.g. own IP-address).
    - applications running in docker have to be mapped to a port on the host machine.
    - useful when you want to isolate the container from the host system. regular traffic from the host system is transparent to the container.
    - default behaviour, if *network_mode* is not specified at all.

More infos about Docker network drivers: [here](https://docs.docker.com/network/drivers/)
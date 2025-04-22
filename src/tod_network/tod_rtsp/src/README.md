# tod_video nodes

The package consists of the following nodes.

**Nodes:**

- [VehicleRtspServer](#vehiclertspserver)
- [VehicleBandwidthManager](#vehiclebandwidthmanager)
- [VehiclePredQoSClient](#vehiclepredqosclient)
- [VehicleConfigsReceive](#vehicleconfigsreceive)
- [OperatorRtspClients](#operatorrtspclients)
- [OperatorSceneManager](#operatorscenemanager)
- [OperatorVideoConfigSend](#operatorvideoconfigsend)
- [OperatorBitrateConfigSend](#operatorbitrateconfigsend)
- [OperatorBitrateIntegrator](#operatorbitrateintegrator)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

# VehicleRtspServer

Hosts a GStreamer RTSP Video Server for multiple videos, published as raw image topics, as specified in vehicle's `sensors-camera.yaml` file in the vehicle config.
Videos are compressed using the H.264 codec. Videos stream settings can be reconfigured at runtime.

**Advertised Services:**

- `/Vehicle/Video/RtspServer/set_parameters`
  ([tod_video/VideoConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Video.cfg))
  Dynamic reconfigure of video streams: bitrate, scaling, cropping.

**Subscribed Topics:**

- `/Vehicle/Video/*/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
  Raw images of each camera '\*'. Image topic pattern (/Vehicle/Video) and name (image_raw) can be set to something different in sensors-camera.yaml file.
- `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to operator.

# VehicleBandwidthManager

Performs bitrate allocation and scaling selection for all video streams in video rate control modes 'Automatic' and 'Collective'. Parameters are selected based on rate-quality-curves, as specified in vehicle's `sensors-camera.yaml` file in the vehicle config.
Reconfigure requests ([tod_video/VideoConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Video.cfg))
are sent to VehicleRtspServer.

**Advertised Services:**

- `/Vehicle/Video/BandwidthManager/set_parameters`
  ([tod_video/BitrateConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Bitrate.cfg))
  Dynamic reconfigure of target bitrate sum for all videos.

**Subscribed Topics:**

- `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to operator and current video rate control mode.

# VehiclePredQoSClient

Gets predictions of available bitrate at current gps position of vehicle from bandwidth map, loaded
from the `bandwith_map.yaml` in the yaml directory of the `tod_video` package. Look-up is implemented using a [KDTree](https://github.com/crvs/KDTree) and [std::map](https://en.cppreference.com/w/cpp/container/map).
On change in bitrate prediction and when in video rate control mode 'Automatic', sends reconfigure
requests ([tod_video/BitrateConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Bitrate.cfg))
to VehicleBandwidthManager.

**Subscribed Topics:**

- `/Vehicle/VehicleBridge/gps/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html))
  Current gps position of vehicle.
- `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to operator and current video rate control mode.

**Advertised Topics:**

- `/Vehicle/Video/bitrate_prediction_on_gps` ([geometry_msgs::PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html))
  Current bitrate prediction, stored in z, at current gps position, stored in x and y. Published in video rate control mode 'Automatic'.

# VehicleConfigsReceive

Receives reconfigure requests from operator via [MQTT](https://mqtt.org/).

- In video rate control mode 'Single', node forwards received video config
  ([tod_video/VideoConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Video.cfg))
  to VehicleRtspServer.
- In video rate control mode 'Collective', node forwards received bitrate config
  ([tod_video/BitrateConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Bitrate.cfg))
  to VehicleBandwidthManager.

**Subscribed Topics:**

- `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to operator and current video rate control mode.

# OperatorRtspClients

Creates pipeline for rtsp video clients, as specified in vehicle's `sensors-camera.yaml` file in the vehicle config,
connecting to rtsp server hosted by VehicleRtspServer. Videos can be paused and resumed at runtime.

**Advertised Topics:**

- `/Operator/Video/*/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
  Decoded raw images for each camera '\*'. Output format of image can bet set via launch file.
- `/Operator/Video/*/paket_info` ([tod_msgs/PaketInfo](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PaketInfo.msg))
  Info for each RTP paket received (paket size, sequence number).
- `/Operator/Video/*/video_info` ([tod_msgs/VideoInfo](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VideoInfo.msg))
  Info for each video stream '\*' (current framerate, image width/height, bitrate).

**Advertised Services:**

- `/Operator/Video/RtspClients/set_parameters`
  ([tod_video/VideoConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Video.cfg))
  Pause and resume video streams.

**Subscribed Topics:**

- `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to vehicle.

# OperatorSceneManager

GUI for operator to reconfigure video scene. Options depend on current video rate control mode.

- In video rate control mode 'Single', individual video streams can be reconfigured (play, pause, crop, bitrate, scaling). Video configs
  ([tod_video/VideoConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Video.cfg))
  are requested to OperatorVideoConfigSend for transmission to vehicle.
- In video rate control mode 'Collective', total target bitrate of all videos can be reconfigured. Bitrate configs
  ([tod_video/BitrateConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Bitrate.cfg))
  are requested to OperatorBitrateConfigSend for transmission to vehicle.
- In video rate control mode 'Automatic', nothing can be reconfigured.

**Advertised Topics:**

- `/Operator/Video/bitrate_desired_on_gps` ([geometry_msgs::PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html))
  Current total bitrate desired by operator, stored in z, at current gps position, stored in x and y.
  Published in video rate control mode 'Single' and 'Collective'.

**Subscribed Topics:**

- `/Operator/VehicleBridge/gps/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html))
  Current gps position of vehicle.
- `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to vehicle and current video rate control mode.

# OperatorVideoConfigSend

Forwards video reconfigure requests from operator to vehicle via [MQTT](https://mqtt.org/).

**Advertised Services:**

- `/Operator/Video/VideoConfigSend/set_parameters`
  ([tod_video/VideoConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Video.cfg))
  Dynamic reconfigure of video streams: bitrate, scaling, cropping. Transmitted to vehicle in video rate control mode 'Single'.

**Subscribed Topics:**

- `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to operator and current video rate control mode.

# OperatorBitrateConfigSend

Forwards bitrate reconfigure requests from operator to vehicle via [MQTT](https://mqtt.org/).

**Advertised Services:**

- `/Operator/Video/BitrateConfigSend/set_parameters`
  ([tod_video/VideoConfig](https://github.com/TUMFTM/tod_perception/blob/master/tod_video/cfg/Video.cfg))
  Dynamic reconfigure of total target bitrate of video streams. Transmitted to vehicle in video rate control mode 'Collective'.

**Subscribed Topics:**

- `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to operator and current video rate control mode.

# OperatorBitrateIntegrator

Sums current bitrate over all videos.

**Advertised Topics:**

- `/Operator/Video/totalBitrateOnGps` ([geometry_msgs::PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html))
  Current bitrate summed over all videos, stored in z, at current gps position, stored in x and y.

**Subscribed Topics:**

- `/Operator/VehicleBridge/gps/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html))
  Current gps position of vehicle.
- `/Operator/Video/*/video_info` ([tod_msgs/VideoInfo](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VideoInfo.msg))
  Info for each video '\*' (current framerate, image width/height, bitrate).

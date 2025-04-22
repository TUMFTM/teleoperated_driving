# TOD Lidar {#tod_lidar_docs}
====

## Overview

The TOD Lidar package provides comprehensive LiDAR processing functionalities for the TOD stack, including filtering, compression, and decompression of point cloud data. The package is organized into two main components:

- **Vehicle Components** for encoding and compressing LiDAR data before transmission.
- **Operator Components** for decoding and transforming the received compressed point clouds.

These components leverage [PCL](https://pointclouds.org/) and [Draco](https://github.com/google/draco) libraries to efficiently process large point clouds for real-time applications. The package is based on [PointCloudTransport](https://github.com/ros-perception/point_cloud_transport) and full credit for the Draco Integration goes to them!

Content:
- **TOD Lidar Compression** \ref tod_lidar_compression  
  - **Vehicle Components**
    - *PointCloudEncoderNode*: Processes raw LiDAR data by applying filtering, downsampling, optional smoothing, and then compresses it using Draco.
    - *PointCloudEncoder*: Implements the core encoding algorithms.
  - **Operator Components**
    - *PointCloudDecoderNode*: Receives, decodes, and transforms compressed LiDAR data.
    - *PointCloudDecoder*: Implements the decoding routines and coordinate frame transformations.

## Executables

- **Vehicle:**
  - `PointCloudEncoderNode`
- **Operator:**
  - `PointCloudDecoderNode`

## Subscribed Topics

- **Vehicle (Encoding Node):**
  - LiDAR sensor topic (configured via `./config/vehicle_config/{vehicleID}/sensors-lidar.yaml`)
    - e.g. `/sensing/lidar/concatenated/pointcloud`
  - (Optionally) `/pointcloud_compressed` for internal processing feedback
- **Operator (Decoding Node):**
  - `input/pointcloud_compressed` – Compressed point cloud data published by the vehicle node

## Published Topics

- **Vehicle:**
  - `output/pointcloud_compressed` – Encoded point cloud ready for network transmission
- **Operator:**
  - `output/pointcloud_decompressed` – Decompressed and transformed LiDAR data for further processing

## Parameters

### Common Parameters
- `vehicleID` (string, default: "edgar")  
  Identifier used to load vehicle-specific sensor configurations.
- `config_path` (string, default: "")  
  Path to the configuration directory containing sensor parameters.

### PointCloudEncoderNode Specific Parameters
- `enable_logging` (bool, default: false)  
  Enable detailed logging for processing steps.
- `target_points` (int, default: 100001)  
  Target number of points to retain after processing.
- `smooth_pointclouds` (bool, default: false)  
  Enable exponential decay smoothing for noise reduction.
- `history_size` (int, default: 3)  
  Number of past frames stored for smoothing.
- `exponential_decay_rate` (double, default: 0.9)  
  Decay rate used during smoothing.
- `voxel_leaf_size` (double, default: 0.45)  
  Leaf size for the approximate voxel grid filter.
- `grid_size` (double, default: 0.05)  
  Grid resolution for uniform sampling and smoothing.
- **Crop Box Parameters:**
  - `crop_box_min_x`, `crop_box_min_y`, `crop_box_min_z` (doubles) – Minimum cropping bounds.
  - `crop_box_max_x`, `crop_box_max_y`, `crop_box_max_z` (doubles) – Maximum cropping bounds.
- **Draco Encoding Parameters:**
  - `draco_encode_method` (int, default: 2)
  - `draco_encode_speed` (int, default: 1)
  - `draco_decode_speed` (int, default: 1)
  - `draco_quatization_position` (int, default: 7)

### PointCloudDecoderNode Specific Parameters
- `target_frame` (string, default: "base_link")  
  The target coordinate frame to which the decoded point cloud will be transformed.

## Build

```bash
colcon build --packages-up-to tod_lidar
```

- **Relevant CMake Arguments:**
  - Build Type: Release / Debug
  - Generation of compile commands (e.g., via `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`)

## Launch

To start the TOD Lidar nodes, use the provided launch files. For example:

```bash
ros2 launch tod_lidar tod_lidar.launch.py
```

This launch file initializes both the encoding (vehicle) and decoding (operator) nodes as required.

## Doxygen Documentation
\version 1.0  
\author Niklas Krauss  
\defgroup tod_lidar TOD Lidar  
\brief Logical Grouping for LiDAR Processing in the TOD Stack  
\ingroup tod_perception  

---

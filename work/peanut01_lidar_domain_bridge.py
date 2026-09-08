#!/usr/bin/env python3

import os
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tod_vehicle_msgs.msg import CompressedPointCloud


SOURCE_TOPIC = "/peanut01/lidar/compressed/domain0"
DESTINATION_TOPIC = "/vehicle/network/data/to_operator/pointcloud_compressed"


def main():
    sensor_domain_id = int(os.getenv("TOD_LIDAR_SENSOR_DOMAIN_ID", "0"))
    tod_domain_id = int(os.getenv("ROS_DOMAIN_ID", "7"))
    sensor_context = Context()
    tod_context = Context()
    rclpy.init(context=sensor_context, domain_id=sensor_domain_id)
    rclpy.init(context=tod_context, domain_id=tod_domain_id)

    sensor_node = Node("peanut01_lidar_bridge_source", context=sensor_context)
    tod_node = Node("peanut01_lidar_bridge_destination", context=tod_context)
    publisher = tod_node.create_publisher(
        CompressedPointCloud, DESTINATION_TOPIC, 1
    )
    sensor_node.create_subscription(
        CompressedPointCloud, SOURCE_TOPIC, publisher.publish, 1
    )
    sensor_executor = SingleThreadedExecutor(context=sensor_context)
    tod_executor = SingleThreadedExecutor(context=tod_context)
    sensor_executor.add_node(sensor_node)
    tod_executor.add_node(tod_node)
    tod_thread = threading.Thread(target=tod_executor.spin, daemon=True)
    tod_thread.start()

    sensor_node.get_logger().info(
        f"Bridging compressed lidar from domain {sensor_domain_id} "
        f"to domain {tod_domain_id}"
    )
    try:
        sensor_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_executor.shutdown()
        tod_executor.shutdown()
        sensor_node.destroy_node()
        tod_node.destroy_node()
        rclpy.shutdown(context=sensor_context)
        rclpy.shutdown(context=tod_context)


if __name__ == "__main__":
    main()

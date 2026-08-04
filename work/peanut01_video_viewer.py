#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


TOPIC = "/operator/network/video/camera1/image"
WINDOW_NAME = "Peanut01 Front Camera"


class Peanut01VideoViewer(Node):
    def __init__(self):
        super().__init__("peanut01_video_viewer")
        self._unsupported_encoding = None
        self._subscription = self.create_subscription(
            Image,
            TOPIC,
            self._on_image,
            qos_profile_sensor_data,
        )
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Waiting for video on {TOPIC}")

    def _on_image(self, message):
        if message.encoding == "rgb8":
            rgb = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.width, 3
            )
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif message.encoding == "bgr8":
            frame = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.width, 3
            )
        elif message.encoding == "mono8":
            frame = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.width
            )
        else:
            if self._unsupported_encoding != message.encoding:
                self.get_logger().error(
                    f"Unsupported image encoding: {message.encoding}"
                )
                self._unsupported_encoding = message.encoding
            return

        cv2.imshow(WINDOW_NAME, frame)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Peanut01VideoViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

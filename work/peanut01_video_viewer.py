#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tod_config_msgs.srv import VideoConfig


CAMERAS = (
    ("LEFT", "/frontleft", "/operator/network/video/frontleft/image"),
    ("CENTER", "/frontcenter", "/operator/network/video/frontcenter/image"),
    ("RIGHT", "/frontright", "/operator/network/video/frontright/image"),
)
WINDOW_NAME = "Peanut01 Cameras"
TILE_SIZE = (426, 240)
VIDEO_CONFIG_SERVICE = "/operator/network/config/to_vehicle/set_video_config"
ROTATIONS = {
    "LEFT": cv2.ROTATE_90_CLOCKWISE,
    "RIGHT": cv2.ROTATE_90_CLOCKWISE,
}


def fit_frame_to_tile(frame):
    target_width, target_height = TILE_SIZE
    scale = min(target_width / frame.shape[1], target_height / frame.shape[0])
    width = max(1, int(round(frame.shape[1] * scale)))
    height = max(1, int(round(frame.shape[0] * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (width, height), interpolation=interpolation)

    tile = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    left = (target_width - width) // 2
    top = (target_height - height) // 2
    tile[top : top + height, left : left + width] = resized
    return tile


class Peanut01VideoViewer(Node):
    def __init__(self):
        super().__init__("peanut01_video_viewer")
        self._frames = {}
        self._unsupported_encodings = {}
        self._activation_queue = [stream for _, stream, _ in CAMERAS]
        self._activation_future = None
        self._activation_started = 0.0
        self._video_config_client = self.create_client(
            VideoConfig, VIDEO_CONFIG_SERVICE
        )
        self._activation_timer = self.create_timer(2.0, self._activate_next_stream)
        self._subscriptions = [
            self.create_subscription(
                Image,
                topic,
                lambda message, name=name: self._on_image(name, message),
                qos_profile_sensor_data,
            )
            for name, _, topic in CAMERAS
        ]
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, TILE_SIZE[0] * len(CAMERAS), TILE_SIZE[1])
        self.get_logger().info(
            "Waiting for front camera streams: "
            + ", ".join(topic for _, _, topic in CAMERAS)
        )

    def _activate_next_stream(self):
        if not self._activation_queue:
            self._activation_timer.cancel()
            return

        if self._activation_future is not None:
            if time.monotonic() - self._activation_started <= 8.0:
                return
            self._activation_future.cancel()
            self._activation_future = None
            self.get_logger().warning("Video activation timed out; retrying")

        if not self._video_config_client.service_is_ready():
            return

        stream = self._activation_queue[0]
        request = VideoConfig.Request()
        request.camera_name = stream
        request.paused = False
        request.actual_width = 960
        request.actual_height = 540
        request.scaling_factor = "0p500"
        request.width = 1920
        request.height = 1080
        request.offset_width = 0
        request.offset_height = 0
        request.bitrate = 1500

        self._activation_future = self._video_config_client.call_async(request)
        self._activation_started = time.monotonic()
        self._activation_future.add_done_callback(
            lambda future, stream=stream: self._on_activation_response(
                stream, future
            )
        )

    def _on_activation_response(self, stream, future):
        try:
            response = future.result()
            if response is None or response.empty != 1:
                self.get_logger().warning(f"Could not activate {stream}; retrying")
                return
            if self._activation_queue and self._activation_queue[0] == stream:
                self._activation_queue.pop(0)
            self.get_logger().info(f"Activated video stream {stream}")
        except Exception as error:
            self.get_logger().warning(f"Could not activate {stream}: {error}")
        finally:
            self._activation_future = None

    def _on_image(self, name, message):
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
            mono = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.width
            )
            frame = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        else:
            if self._unsupported_encodings.get(name) != message.encoding:
                self.get_logger().error(
                    f"Unsupported {name} image encoding: {message.encoding}"
                )
                self._unsupported_encodings[name] = message.encoding
            return

        if name in ROTATIONS:
            frame = cv2.rotate(frame, ROTATIONS[name])
        self._frames[name] = fit_frame_to_tile(frame)
        self._render()

    def _render(self):
        tiles = []
        for name, _, _ in CAMERAS:
            tile = self._frames.get(name)
            if tile is None:
                tile = np.zeros((TILE_SIZE[1], TILE_SIZE[0], 3), dtype=np.uint8)
                cv2.putText(
                    tile,
                    "NO SIGNAL",
                    (TILE_SIZE[0] // 2 - 65, TILE_SIZE[1] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (180, 180, 180),
                    2,
                    cv2.LINE_AA,
                )
            else:
                tile = tile.copy()
            cv2.putText(
                tile,
                name,
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            tiles.append(tile)

        cv2.imshow(WINDOW_NAME, np.hstack(tiles))
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

#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewer(Node):
    """Show the Gen0 front camera in a small, resizable OpenCV window."""

    def __init__(self):
        super().__init__('gen0_camera_viewer')
        self.declare_parameter('image_topic', '/gen0_model/front_camera')
        self.declare_parameter('window_title', 'Gen0 Sweeper - Front Camera')
        self.declare_parameter('max_width', 960)
        self.declare_parameter('max_height', 720)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.window_title = str(self.get_parameter('window_title').value)
        self.max_width = max(160, int(self.get_parameter('max_width').value))
        self.max_height = max(120, int(self.get_parameter('max_height').value))
        self.bridge = CvBridge()
        self.last_frame_time = None
        self.smoothed_fps = 0.0

        try:
            cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
            placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                placeholder,
                f'Waiting for {self.image_topic}',
                (28, 185),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.window_title, placeholder)
            cv2.waitKey(1)
        except cv2.error as exc:
            raise RuntimeError(
                'Unable to create the camera window. Ensure WSLg/X11 display '
                'forwarding is available, or launch with camera_view:=false.'
            ) from exc

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Camera window ready; waiting for images on {self.image_topic}'
        )

    def on_image(self, message):
        try:
            # Decode common 8-bit camera streams directly. This also avoids a
            # cv_bridge/NumPy compatibility issue in some ROS Humble images.
            if message.encoding in ('bgr8', 'rgb8'):
                frame = np.frombuffer(message.data, dtype=np.uint8).reshape(
                    message.height, message.width, 3
                )
                if message.encoding == 'rgb8':
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:
                    frame = frame.copy()
            else:
                frame = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding='bgr8'
                )
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to decode camera frame: {exc}')
            return

        now = time.monotonic()
        if self.last_frame_time is not None:
            interval = now - self.last_frame_time
            if interval > 0.0:
                instantaneous_fps = 1.0 / interval
                self.smoothed_fps = (
                    instantaneous_fps
                    if self.smoothed_fps == 0.0
                    else 0.9 * self.smoothed_fps + 0.1 * instantaneous_fps
                )
        self.last_frame_time = now

        height, width = frame.shape[:2]
        scale = min(self.max_width / width, self.max_height / height, 1.0)
        if scale < 1.0:
            frame = cv2.resize(
                frame,
                (int(width * scale), int(height * scale)),
                interpolation=cv2.INTER_AREA,
            )

        label = f'{self.image_topic}  {width}x{height}  {self.smoothed_fps:.1f} FPS'
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (0, 0, 0), -1)
        cv2.putText(
            frame,
            label,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (80, 255, 80),
            1,
            cv2.LINE_AA,
        )
        cv2.imshow(self.window_title, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            self.get_logger().info('Camera window closed by user')
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Publish a low-bandwidth JPEG stream for the browser driving console."""

from io import BytesIO
import time

from PIL import Image as PilImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraCompressor(Node):
    """Drop stale frames and encode only the newest frame at a bounded rate."""

    def __init__(self):
        super().__init__('camera_compressor')
        self.declare_parameter('input_topic', '/gen0_model/driver_camera')
        self.declare_parameter(
            'output_topic', '/gen0_model/driver_camera/compressed')
        self.declare_parameter('max_width', 640)
        self.declare_parameter('max_height', 480)
        self.declare_parameter('max_fps', 8.0)
        self.declare_parameter('jpeg_quality', 58)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.max_width = max(160, int(self.get_parameter('max_width').value))
        self.max_height = max(120, int(self.get_parameter('max_height').value))
        self.max_fps = max(0.5, float(self.get_parameter('max_fps').value))
        self.jpeg_quality = min(
            90, max(25, int(self.get_parameter('jpeg_quality').value)))
        self.min_period = 1.0 / self.max_fps
        self.last_encode_at = 0.0

        self.publisher = self.create_publisher(
            CompressedImage, self.output_topic, 1)
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f'Camera compressor: {self.input_topic} -> {self.output_topic}, '
            f'{self.max_width}x{self.max_height} @ {self.max_fps:.1f} FPS, '
            f'JPEG {self.jpeg_quality}')

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_encode_at < self.min_period:
            return
        self.last_encode_at = now
        try:
            image = self.decode_image(message)
            resampling = getattr(PilImage, 'Resampling', PilImage)
            image.thumbnail((self.max_width, self.max_height), resampling.LANCZOS)
            buffer = BytesIO()
            image.save(
                buffer,
                format='JPEG',
                quality=self.jpeg_quality,
                optimize=False,
            )
            output = CompressedImage()
            output.header = message.header
            output.format = 'jpeg'
            output.data = buffer.getvalue()
            self.publisher.publish(output)
        except Exception as error:  # Keep the stream alive after a malformed frame.
            self.get_logger().error(
                f'Unable to compress camera frame: {error}',
                throttle_duration_sec=5.0,
            )

    @staticmethod
    def decode_image(message):
        encoding = str(message.encoding or 'rgb8').lower()
        decoders = {
            'rgb8': ('RGB', 'RGB'),
            'bgr8': ('RGB', 'BGR'),
            'rgba8': ('RGBA', 'RGBA'),
            'bgra8': ('RGBA', 'BGRA'),
            'mono8': ('L', 'L'),
            '8uc1': ('L', 'L'),
            '8uc3': ('RGB', 'BGR'),
        }
        if encoding not in decoders:
            raise ValueError(f'unsupported encoding {encoding!r}')
        mode, raw_mode = decoders[encoding]
        image = PilImage.frombytes(
            mode,
            (int(message.width), int(message.height)),
            bytes(message.data),
            'raw',
            raw_mode,
            int(message.step),
            1,
        )
        return image.convert('RGB') if image.mode != 'RGB' else image


def main(args=None):
    rclpy.init(args=args)
    node = CameraCompressor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

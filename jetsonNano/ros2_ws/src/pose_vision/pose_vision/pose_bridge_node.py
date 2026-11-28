#!/usr/bin/env python3
"""ROS 2 bridge node forwarding camera frames to the host TensorRT YOLO-Pose server."""

import json
import socket
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
]


class PoseBridgeNode(Node):
    """Subscribes to camera images, queries TensorRT over TCP, and publishes pose outputs."""

    def __init__(self):
        super().__init__('pose_bridge_node')

        self.declare_parameter('trt_host', '127.0.0.1')
        self.declare_parameter('trt_port', 5555)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('pose_image_topic', '/vision/image_pose')
        self.declare_parameter('keypoints_topic', '/vision/pose_keypoints')
        self.declare_parameter('reconnect_delay', 2.0)
        self.declare_parameter('inference_width', 320)
        self.declare_parameter('inference_height', 240)

        self.trt_host = str(self.get_parameter('trt_host').value)
        self.trt_port = int(self.get_parameter('trt_port').value)
        image_topic = str(self.get_parameter('image_topic').value)
        pose_image_topic = str(self.get_parameter('pose_image_topic').value)
        keypoints_topic = str(self.get_parameter('keypoints_topic').value)
        self.reconnect_delay = float(self.get_parameter('reconnect_delay').value)
        self.infer_w = int(self.get_parameter('inference_width').value)
        self.infer_h = int(self.get_parameter('inference_height').value)

        self.sock = None
        self.last_connect_attempt = 0.0
        self.last_warn_time = 0.0

        self.sub = self.create_subscription(
            Image, image_topic, self.cb_image, qos_profile_sensor_data
        )
        self.pub_img = self.create_publisher(Image, pose_image_topic, qos_profile_sensor_data)
        self.pub_kpts = self.create_publisher(String, keypoints_topic, 10)

        self.count = 0
        self.t_start = time.monotonic()

        self._ensure_connected()
        self.get_logger().info(
            f'PoseBridgeNode ready ({image_topic} -> {keypoints_topic}, {pose_image_topic} | '
            f'TRT={self.trt_host}:{self.trt_port}).'
        )

    def _close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def _ensure_connected(self):
        if self.sock is not None:
            return True

        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_delay:
            return False
        self.last_connect_attempt = now

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.connect((self.trt_host, self.trt_port))
            sock.settimeout(5.0)
            self.sock = sock
            self.get_logger().info(
                f'Connecte au serveur TensorRT ({self.trt_host}:{self.trt_port}).'
            )
            return True
        except OSError as exc:
            sock.close()
            if now - self.last_warn_time >= 5.0:
                self.last_warn_time = now
                self.get_logger().warn(
                    f'Serveur TensorRT indisponible ({self.trt_host}:{self.trt_port}): {exc}'
                )
            return False

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('Connexion TensorRT fermee')
            buf.extend(chunk)
        return bytes(buf)

    @staticmethod
    def _decode_image(msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(arr.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return arr.reshape(msg.height, msg.width, 3)
        if msg.encoding in ('yuyv', 'yuv422_yuy2'):
            return cv2.cvtColor(arr.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_YUYV)
        return None

    def _draw_overlay(self, frame, detections):
        vis = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = map(int, det['box'])
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f"{det['score']:.2f}",
                (x1, max(12, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
            )
            kpts = det['kpts']
            for i, j in SKELETON:
                if kpts[i][2] > 0.5 and kpts[j][2] > 0.5:
                    cv2.line(
                        vis,
                        (int(kpts[i][0]), int(kpts[i][1])),
                        (int(kpts[j][0]), int(kpts[j][1])),
                        (255, 180, 0),
                        2,
                    )
            for kx, ky, kc in kpts:
                if kc > 0.5:
                    cv2.circle(vis, (int(kx), int(ky)), 2, (0, 0, 255), -1)
        return vis

    def cb_image(self, msg: Image):
        if not self._ensure_connected():
            return

        frame = self._decode_image(msg)
        if frame is None:
            return

        if frame.shape[1] != self.infer_w or frame.shape[0] != self.infer_h:
            frame = cv2.resize(frame, (self.infer_w, self.infer_h), interpolation=cv2.INTER_LINEAR)
        elif not frame.flags['C_CONTIGUOUS']:
            frame = np.ascontiguousarray(frame)

        h, w = frame.shape[:2]
        raw_bytes = frame.tobytes()

        try:
            self.sock.sendall(struct.pack('!III', h, w, len(raw_bytes)) + raw_bytes)
            resp_len = struct.unpack('!I', self._recv_exact(4))[0]
            raw_json = self._recv_exact(resp_len).decode('utf-8')
        except (OSError, ConnectionError, struct.error, UnicodeDecodeError) as exc:
            self.get_logger().warn(f'Erreur communication TensorRT ({exc}); reconnexion...')
            self._close_socket()
            return

        kpts_msg = String()
        kpts_msg.data = raw_json
        self.pub_kpts.publish(kpts_msg)

        if self.pub_img.get_subscription_count() > 0:
            detections = json.loads(raw_json)
            vis = self._draw_overlay(frame, detections)
            out = Image()
            out.header = msg.header
            out.height, out.width = h, w
            out.encoding = 'bgr8'
            out.step = w * 3
            out.data = vis.tobytes()
            self.pub_img.publish(out)

        self.count += 1
        now = time.monotonic()
        if now - self.t_start >= 5.0:
            fps = self.count / (now - self.t_start)
            self.get_logger().debug(
                f'Inference rate: {fps:.1f} FPS (video subscribers: {self.pub_img.get_subscription_count()})'
            )
            self.count = 0
            self.t_start = now


def main(args=None):
    rclpy.init(args=args)
    node = PoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_socket()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

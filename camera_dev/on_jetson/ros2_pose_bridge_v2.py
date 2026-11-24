#!/usr/bin/env python3
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
    (11, 13), (13, 15), (12, 14), (14, 16)
]


class Ros2PoseBridge(Node):
    def __init__(self):
        super().__init__("ros2_pose_bridge")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect(("192.168.1.10", 5555))

        self.sub = self.create_subscription(Image, "/image_raw", self.cb_image, qos_profile_sensor_data)
        self.pub_img = self.create_publisher(Image, "/image_pose", 1)
        self.pub_kpts = self.create_publisher(String, "/pose_keypoints", 1)

        self.count = 0
        self.t_start = time.time()
        self.get_logger().info("Connecté au serveur TensorRT (mode BGR brut sans JPEG).")

    def recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Connexion TensorRT fermée")
            buf.extend(chunk)
        return bytes(buf)

    def cb_image(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(arr.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        elif msg.encoding == "bgr8":
            frame = arr.reshape(msg.height, msg.width, 3)
        elif msg.encoding in ("yuyv", "yuv422_yuy2"):
            frame = cv2.cvtColor(arr.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_YUYV)
        else:
            return

        # Ramène à 320x240 si usb_cam tourne en 640x480
        if frame.shape[1] > 320:
            frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)
        elif not frame.flags["C_CONTIGUOUS"]:
            frame = np.ascontiguousarray(frame)

        h, w = frame.shape[:2]
        raw_bytes = frame.tobytes()
        self.sock.sendall(struct.pack("!III", h, w, len(raw_bytes)) + raw_bytes)

        resp_len = struct.unpack("!I", self.recv_exact(4))[0]
        raw_json = self.recv_exact(resp_len).decode("utf-8")

        # Publication légère des keypoints pour le jeu MemoMove
        kpts_msg = String()
        kpts_msg.data = raw_json
        self.pub_kpts.publish(kpts_msg)

        # Dessin et publication vidéo uniquement si un client écoute /image_pose
        if self.pub_img.get_subscription_count() > 0:
            detections = json.loads(raw_json)
            vis = frame.copy()
            for det in detections:
                x1, y1, x2, y2 = map(int, det["box"])
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f"{det['score']:.2f}", (x1, max(12, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                kpts = det["kpts"]
                for i, j in SKELETON:
                    if kpts[i][2] > 0.5 and kpts[j][2] > 0.5:
                        cv2.line(vis, (int(kpts[i][0]), int(kpts[i][1])),
                                 (int(kpts[j][0]), int(kpts[j][1])), (255, 180, 0), 2)
                for kx, ky, kc in kpts:
                    if kc > 0.5:
                        cv2.circle(vis, (int(kx), int(ky)), 2, (0, 0, 255), -1)

            out = Image()
            out.header = msg.header
            out.height, out.width = h, w
            out.encoding = "bgr8"
            out.step = w * 3
            out.data = vis.tobytes()
            self.pub_img.publish(out)

        self.count += 1
        now = time.time()
        if now - self.t_start >= 2.0:
            fps = self.count / (now - self.t_start)
            self.get_logger().info(f"Cadence réelle inférence : {fps:.1f} FPS (abonnés vidéo: {self.pub_img.get_subscription_count()})")
            self.count = 0
            self.t_start = now


def main():
    rclpy.init()
    node = Ros2PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
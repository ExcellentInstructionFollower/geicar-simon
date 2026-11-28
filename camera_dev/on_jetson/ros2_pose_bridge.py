#!/usr/bin/env python3
import json
import socket
import struct
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),          # Tête
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), # Bras
    (5, 11), (6, 12), (11, 12),              # Torse
    (11, 13), (13, 15), (12, 14), (14, 16)   # Jambes
]


class Ros2PoseBridge(Node):
    def __init__(self):
        super().__init__("ros2_pose_bridge")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(("127.0.0.1", 5555))

        self.sub = self.create_subscription(Image, "/image_raw", self.cb_image, 1)
        self.pub = self.create_publisher(Image, "/image_pose", 1)
        self.get_logger().info("Connecté au serveur TensorRT. Publication sur /image_pose.")

    def recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Connexion TensorRT fermée")
            buf.extend(chunk)
        return bytes(buf)

    def cb_image(self, msg):
        # Conversion directe ROS Image -> OpenCV BGR sans cv_bridge
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(arr.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        elif msg.encoding == "bgr8":
            frame = arr.reshape(msg.height, msg.width, 3).copy()
        elif msg.encoding in ("yuyv", "yuv422_yuy2"):
            frame = cv2.cvtColor(arr.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_YUYV)
        else:
            return

        # Envoi au serveur TensorRT sur l'hôte
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ok:
            return
        data = jpg.tobytes()
        self.sock.sendall(struct.pack("!I", len(data)) + data)

        resp_len = struct.unpack("!I", self.recv_exact(4))[0]
        detections = json.loads(self.recv_exact(resp_len).decode("utf-8"))

        # Dessin des boîtes et squelettes sur l'image
        for det in detections:
            x1, y1, x2, y2 = map(int, det["box"])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Person {det['score']:.2f}", (x1, max(15, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            kpts = det["kpts"]
            for i, j in SKELETON:
                if kpts[i][2] > 0.5 and kpts[j][2] > 0.5:
                    pt1 = (int(kpts[i][0]), int(kpts[i][1]))
                    pt2 = (int(kpts[j][0]), int(kpts[j][1]))
                    cv2.line(frame, pt1, pt2, (255, 180, 0), 2)
            for kx, ky, kc in kpts:
                if kc > 0.5:
                    cv2.circle(frame, (int(kx), int(ky)), 3, (0, 0, 255), -1)

        # Publication sur /image_pose
        out = Image()
        out.header = msg.header
        out.height, out.width = frame.shape[:2]
        out.encoding = "bgr8"
        out.step = out.width * 3
        out.data = frame.tobytes()
        self.pub.publish(out)


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
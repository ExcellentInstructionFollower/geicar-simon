#!/usr/bin/env python3
import argparse
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

KEYPOINT_NAMES = [
    "00 - Nez",
    "01 - Oeil gauche",
    "02 - Oeil droit",
    "03 - Oreille gauche",
    "04 - Oreille droite",
    "05 - Epaule gauche",
    "06 - Epaule droite",
    "07 - Coude gauche",
    "08 - Coude droit",
    "09 - Poignet gauche",
    "10 - Poignet droit",
    "11 - Hanche gauche",
    "12 - Hanche droite",
    "13 - Genou gauche",
    "14 - Genou droit",
    "15 - Cheville gauche",
    "16 - Cheville droite",
]


class KeypointsPrinter(Node):
    def __init__(self, min_conf: float):
        super().__init__("keypoints_printer")
        self.min_conf = min_conf
        self.frame_id = 0
        self.sub = self.create_subscription(
            String, "/pose_keypoints", self.cb_keypoints, 10
        )
        self.get_logger().info(
            f"Ecoute de /pose_keypoints (seuil confiance >= {self.min_conf:.2f})..."
        )

    def cb_keypoints(self, msg: String):
        self.frame_id += 1
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON invalide recu : {e}")
            return

        lines = [f"\n================ Frame #{self.frame_id} ================"]

        if not detections:
            lines.append("Aucune personne detectee.")
            print("\n".join(lines))
            return

        for idx, det in enumerate(detections, start=1):
            box = det.get("box", [0, 0, 0, 0])
            score = det.get("score", 0.0)
            kpts = det.get("kpts", [])

            lines.append(
                f"Personne #{idx} | Confiance globale : {score:.3f} | "
                f"Box [x1={box[0]:.1f}, y1={box[1]:.1f}, x2={box[2]:.1f}, y2={box[3]:.1f}]"
            )
            lines.append("-" * 62)
            lines.append(f"{'Point cle':<24} | {'X (px)':>8} | {'Y (px)':>8} | {'Confiance':>9}")
            lines.append("-" * 62)

            for i, kp in enumerate(kpts):
                x, y, conf = kp[0], kp[1], kp[2]
                if conf < self.min_conf:
                    continue
                name = KEYPOINT_NAMES[i] if i < len(KEYPOINT_NAMES) else f"Point {i:02d}"
                lines.append(f"{name:<24} | {x:>8.1f} | {y:>8.1f} | {conf:>9.3f}")

        print("\n".join(lines), flush=True)


def main():
    parser = argparse.ArgumentParser(description="Affiche en texte les keypoints ROS 2")
    parser.add_argument(
        "--min-conf",
        type=float,
        default=0.0,
        help="Confiance minimale pour afficher un point cle (defaut: 0.0 = tout afficher)",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = KeypointsPrinter(min_conf=args.min_conf)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
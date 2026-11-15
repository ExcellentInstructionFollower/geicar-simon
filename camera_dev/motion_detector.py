#!/usr/bin/env python3
from collections import deque
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MIN_CONF = 0.5


def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def angle_deg(a, b, c):
    """Angle en degrés au sommet B formé par les segments BA et BC."""
    bax, bay = a[0] - b[0], a[1] - b[1]
    bcx, bcy = c[0] - b[0], c[1] - b[1]
    norm = math.hypot(bax, bay) * math.hypot(bcx, bcy)
    if norm < 1e-6:
        return 0.0
    cos_val = max(-1.0, min(1.0, (bax * bcx + bay * bcy) / norm))
    return math.degrees(math.acos(cos_val))


class MotionDetector(Node):
    def __init__(self):
        super().__init__("motion_detector")
        self.sub = self.create_subscription(String, "/pose_keypoints", self.cb_keypoints, 10)
        self.pub = self.create_publisher(String, "/detected_motion", 10)

        # Mémoire temporelle pour lissage et détection dynamique
        self.smooth_kpts = None
        self.alpha = 0.6  # Facteur de lissage EMA (0.6 = réactif, 0.3 = très lissé)
        self.wrist_history = deque(maxlen=15)  # Historique ~1.5-2s pour détecter un salut
        self.center_x_history = deque(maxlen=10)
        self.last_valid_time = 0.0

        # Validation par maintien (anti-rebond)
        self.candidate_pose = "NEUTRE"
        self.hold_count = 0
        self.confirmed_pose = "NEUTRE"

        self.get_logger().info("Détecteur de mouvement actif sur /pose_keypoints -> /detected_motion")

    def valid(self, kpts, *indices):
        return all(kpts[i][2] >= MIN_CONF for i in indices)

    def cb_keypoints(self, msg: String):
        detections = json.loads(msg.data)
        if not detections:
            self.smooth_kpts = None
            self.wrist_history.clear()
            return

        # Sélectionne la personne avec la meilleure confiance
        det = max(detections, key=lambda d: d["score"])
        raw_kpts = det["kpts"]

        # 1. Lissage temporel EMA des coordonnées (x, y)
        if self.smooth_kpts is None:
            self.smooth_kpts = [list(kp) for kp in raw_kpts]
        else:
            for i in range(17):
                if raw_kpts[i][2] >= MIN_CONF:
                    self.smooth_kpts[i][0] = self.alpha * raw_kpts[i][0] + (1 - self.alpha) * self.smooth_kpts[i][0]
                    self.smooth_kpts[i][1] = self.alpha * raw_kpts[i][1] + (1 - self.alpha) * self.smooth_kpts[i][1]
                self.smooth_kpts[i][2] = raw_kpts[i][2]

        k = self.smooth_kpts

        # 2. Référentiel corporel (épaules gauche=5, droite=6)
        if not self.valid(k, 5, 6):
            return
        shoulder_w = max(dist(k[5], k[6]), 10.0)
        mid_shoulder_x = 0.5 * (k[5][0] + k[6][0])
        self.center_x_history.append(mid_shoulder_x)

        # 3. Analyse des postures statiques (Bras)
        left_arm_up = self.valid(k, 5, 7, 9) and (k[9][1] < k[5][1] - 0.2 * shoulder_w)
        right_arm_up = self.valid(k, 6, 8, 10) and (k[10][1] < k[6][1] - 0.2 * shoulder_w)

        left_arm_side = (
            self.valid(k, 5, 7, 9)
            and abs(k[9][1] - k[5][1]) < 0.45 * shoulder_w
            and (k[9][0] - k[5][0]) > 0.9 * shoulder_w
            and angle_deg(k[5], k[7], k[9]) > 140.0
        )
        right_arm_side = (
            self.valid(k, 6, 8, 10)
            and abs(k[10][1] - k[6][1]) < 0.45 * shoulder_w
            and (k[6][0] - k[10][0]) > 0.9 * shoulder_w
            and angle_deg(k[6], k[8], k[10]) > 140.0
        )

        # 4. Analyse dynamique (Salut de la main & déplacement latéral)
        dynamic_event = None
        if right_arm_up:
            self.wrist_history.append(k[10][0])
            if len(self.wrist_history) >= 8:
                diffs = [self.wrist_history[i] - self.wrist_history[i - 1] for i in range(1, len(self.wrist_history))]
                significant = [d for d in diffs if abs(d) > 0.08 * shoulder_w]
                sign_changes = sum(
                    1 for i in range(1, len(significant)) if significant[i] * significant[i - 1] < 0
                )
                if sign_changes >= 2:
                    dynamic_event = "SALUT_MAIN_DROITE"
        else:
            self.wrist_history.clear()

        if len(self.center_x_history) >= 5:
            dx_body = (self.center_x_history[-1] - self.center_x_history[0]) / shoulder_w
            if dx_body > 0.45:
                dynamic_event = "DEPLACEMENT_GAUCHE_IMAGE"
            elif dx_body < -0.45:
                dynamic_event = "DEPLACEMENT_DROITE_IMAGE"

        # 5. Classification de la posture courante
        if left_arm_up and right_arm_up:
            pose = "DEUX_BRAS_LEVES"
        elif left_arm_side and right_arm_side:
            pose = "BRAS_EN_CROIX"
        elif left_arm_up:
            pose = "BRAS_GAUCHE_LEVE"
        elif right_arm_up:
            pose = "BRAS_DROIT_LEVE"
        else:
            pose = "NEUTRE"

        # 6. Validation temporelle (maintien sur 4 frames)
        if pose == self.candidate_pose:
            self.hold_count += 1
        else:
            self.candidate_pose = pose
            self.hold_count = 1

        if self.hold_count == 4 and pose != self.confirmed_pose:
            self.confirmed_pose = pose
            self.get_logger().info(f"Nouvelle posture validée : {self.confirmed_pose}")

        if dynamic_event and (time.time() - self.last_valid_time > 1.0):
            self.last_valid_time = time.time()
            self.get_logger().info(f"Mouvement dynamique détecté : {dynamic_event}")

        # Publication de l'état complet
        out = String()
        out.data = json.dumps({
            "posture": self.confirmed_pose,
            "mouvement": dynamic_event,
            "largeur_epaules_px": round(shoulder_w, 1),
        })
        self.pub.publish(out)


def main():
    rclpy.init()
    node = MotionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
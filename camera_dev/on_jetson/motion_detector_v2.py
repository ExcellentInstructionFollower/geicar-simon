#!/usr/bin/env python3
from collections import deque
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MIN_CONF = 0.5
MIN_PERSON_CONF = 0.55

# Seuils réglables
WALK_SCALE_THRESHOLD = 0.12    # +12% / -12% de variation d'échelle pour marche avant/arrière
ROTATION_THRESHOLD = 0.22      # Variation minimale du ratio d'épaules pour détecter une rotation
AMP_67_THRESHOLD = 0.22        # Écart vertical minimal entre les 2 mains (en % largeur d'épaules)
MAX_67_INTERVAL = 1.2          # Délai max (s) entre deux alternances haut/bas du mouvement 67


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
        self.sub_reset = self.create_subscription(String, "/reset_67", self.cb_reset_67, 10)
        self.pub = self.create_publisher(String, "/detected_motion", 10)

        # Suivi de la personne cible (proximité + confiance)
        self.last_target_center = None

        # Lissage temporel EMA
        self.smooth_kpts = None
        self.alpha = 0.6

        # Historiques temporels
        self.wrist_history = deque(maxlen=15)
        self.center_x_history = deque(maxlen=10)
        self.scale_history = deque(maxlen=14)
        self.yaw_history = deque(maxlen=8)

        # Orientation (DE_FACE / DE_DOS) avec anti-rebond
        self.candidate_orientation = "DE_FACE"
        self.orientation_hold_count = 0
        self.confirmed_orientation = "DE_FACE"

        # Posture statique
        self.candidate_pose = "NEUTRE"
        self.hold_count = 0
        self.confirmed_pose = "NEUTRE"

        # Marche avant / arrière
        self.candidate_walk = "IMMOBILE"
        self.walk_hold_count = 0
        self.confirmed_walk = "IMMOBILE"

        # Rotation gauche / droite
        self.candidate_rot = "AUCUNE"
        self.rot_hold_count = 0
        self.confirmed_rot = "AUCUNE"

        self.last_dynamic_time = 0.0

        # Machine à états pour le mouvement "67" et son compteur
        self.compteur_67 = 0
        self.last_67_side = 0
        self.last_67_switch_time = 0.0
        self.half_cycles_67 = 0
        self.last_67_valid_time = 0.0

        self.get_logger().info(
            "Détecteur actif (/pose_keypoints -> /detected_motion) | "
            "Cible prioritaire, De dos, Rotations, Marche & Mouvement 67 activés."
        )

    def cb_reset_67(self, _msg: String):
        self.compteur_67 = 0
        self.half_cycles_67 = 0
        self.last_67_side = 0
        self.get_logger().info("Compteur 67 remis à zéro (0).")

    def valid(self, kpts, *indices):
        return all(kpts[i][2] >= MIN_CONF for i in indices)

    def select_primary_person(self, detections):
        """Sélectionne uniquement la personne la plus proche et la plus fiable."""
        candidates = [d for d in detections if d.get("score", 0.0) >= MIN_PERSON_CONF]
        if not candidates:
            return None

        best_det = None
        best_priority = -1.0

        for d in candidates:
            x1, y1, x2, y2 = d["box"]
            area = max(1.0, (x2 - x1) * (y2 - y1))
            score = d["score"]

            # Priorité combinant proximité (aire) et confiance
            priority = area * (score ** 2)

            # Légère priorité de continuité si la personne était déjà suivie
            if self.last_target_center is not None:
                cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
                if math.hypot(cx - self.last_target_center[0], cy - self.last_target_center[1]) < 60.0:
                    priority *= 1.15

            if priority > best_priority:
                best_priority = priority
                best_det = d

        if best_det is not None:
            bx1, by1, bx2, by2 = best_det["box"]
            self.last_target_center = (0.5 * (bx1 + bx2), 0.5 * (by1 + by2))

        return best_det

    def cb_keypoints(self, msg: String):
        now = time.time()
        detections = json.loads(msg.data)
        det = self.select_primary_person(detections)

        if det is None:
            self.smooth_kpts = None
            self.last_target_center = None
            self.wrist_history.clear()
            self.scale_history.clear()
            self.yaw_history.clear()
            self.last_67_side = 0
            self.half_cycles_67 = 0
            return

        raw_kpts = det["kpts"]

        # 1. Lissage EMA des coordonnées
        if self.smooth_kpts is None:
            self.smooth_kpts = [list(kp) for kp in raw_kpts]
        else:
            for i in range(17):
                if raw_kpts[i][2] >= MIN_CONF:
                    self.smooth_kpts[i][0] = self.alpha * raw_kpts[i][0] + (1 - self.alpha) * self.smooth_kpts[i][0]
                    self.smooth_kpts[i][1] = self.alpha * raw_kpts[i][1] + (1 - self.alpha) * self.smooth_kpts[i][1]
                self.smooth_kpts[i][2] = raw_kpts[i][2]

        k = self.smooth_kpts

        # 2. Référentiel corporel (épaules 5=gauche, 6=droite et hanches 11=gauche, 12=droite)
        if not self.valid(k, 5, 6):
            return
        shoulder_w = max(dist(k[5], k[6]), 10.0)
        mid_shoulder_x = 0.5 * (k[5][0] + k[6][0])
        mid_shoulder_y = 0.5 * (k[5][1] + k[6][1])
        self.center_x_history.append(mid_shoulder_x)

        hips_valid = self.valid(k, 11, 12)
        if hips_valid:
            mid_hip_y = 0.5 * (k[11][1] + k[12][1])
            torso_h = max(abs(mid_hip_y - mid_shoulder_y), 12.0)
            body_scale = 0.5 * shoulder_w + 0.5 * torso_h
        else:
            mid_hip_y = mid_shoulder_y + 1.5 * shoulder_w
            torso_h = max(1.5 * shoulder_w, 12.0)
            body_scale = shoulder_w

        # 3. Détection DE_FACE vs DE_DOS
        # De face : épaule gauche (5) est à droite sur l'image (k[5][0] > k[6][0]) + visage visible
        # De dos  : épaule gauche (5) est à gauche sur l'image (k[5][0] < k[6][0]) + nez/yeux masqués
        signed_yaw = (k[5][0] - k[6][0]) / torso_h
        face_visible = (k[0][2] >= 0.5) or (k[1][2] >= 0.5 and k[2][2] >= 0.5)
        hips_inverted = hips_valid and (k[11][0] < k[12][0])

        if (signed_yaw < -0.15 and not face_visible) or (signed_yaw < -0.22 and hips_inverted):
            orient_candidate = "DE_DOS"
        elif signed_yaw > 0.15 or face_visible:
            orient_candidate = "DE_FACE"
        else:
            orient_candidate = self.confirmed_orientation

        if orient_candidate == self.candidate_orientation:
            self.orientation_hold_count += 1
        else:
            self.candidate_orientation = orient_candidate
            self.orientation_hold_count = 1

        if self.orientation_hold_count >= 3 and orient_candidate != self.confirmed_orientation:
            self.confirmed_orientation = orient_candidate
            self.get_logger().info(f"Orientation changée : {self.confirmed_orientation}")

        is_back = self.confirmed_orientation == "DE_DOS"

        # 4. Détection des rotations du corps (TOURNE_GAUCHE / TOURNE_DROITE)
        # Estimation du côté tourné vers la caméra (sin_proxy > 0 : côté droit proche, < 0 : côté gauche proche)
        if k[0][2] >= 0.5:
            sin_proxy = (k[0][0] - mid_shoulder_x) / shoulder_w
        elif k[4][2] >= 0.45 and k[4][2] > k[3][2] + 0.1:
            sin_proxy = 0.5
        elif k[3][2] >= 0.45 and k[3][2] > k[4][2] + 0.1:
            sin_proxy = -0.5
        else:
            sin_proxy = -0.3 if is_back else 0.3

        self.yaw_history.append((now, signed_yaw, sin_proxy))
        rot_candidate = "AUCUNE"

        if len(self.yaw_history) >= 4:
            dt_yaw = self.yaw_history[-1][0] - self.yaw_history[0][0]
            if 0.15 <= dt_yaw <= 1.2:
                dyaw = self.yaw_history[-1][1] - self.yaw_history[0][1]
                mean_sin = sum(item[2] for item in self.yaw_history) / len(self.yaw_history)
                turn_delta = dyaw if mean_sin < 0 else -dyaw

                if turn_delta > ROTATION_THRESHOLD:
                    rot_candidate = "TOURNE_GAUCHE"
                elif turn_delta < -ROTATION_THRESHOLD:
                    rot_candidate = "TOURNE_DROITE"

        if rot_candidate == self.candidate_rot:
            self.rot_hold_count += 1
        else:
            self.candidate_rot = rot_candidate
            self.rot_hold_count = 1

        if self.rot_hold_count >= 2 and rot_candidate != self.confirmed_rot:
            self.confirmed_rot = rot_candidate
            if self.confirmed_rot != "AUCUNE":
                self.get_logger().info(f"Rotation détectée ({self.confirmed_orientation}) : {self.confirmed_rot}")

        # 5. Détection de la marche avant / arrière (adaptée à DE_FACE et DE_DOS)
        self.scale_history.append((now, body_scale))
        walk_candidate = "IMMOBILE"
        if len(self.scale_history) >= 6:
            dt_span = self.scale_history[-1][0] - self.scale_history[0][0]
            if 0.4 <= dt_span <= 2.5:
                old_samples = [s for _, s in list(self.scale_history)[:3]]
                new_samples = [s for _, s in list(self.scale_history)[-3:]]
                old_scale = sum(old_samples) / len(old_samples)
                new_scale = sum(new_samples) / len(new_samples)
                rel_change = (new_scale - old_scale) / old_scale

                # De face : grossir = avancer. De dos : rétrécir = avancer (s'éloigner dans le sens du regard)
                egocentric_change = -rel_change if is_back else rel_change

                if egocentric_change > WALK_SCALE_THRESHOLD:
                    walk_candidate = "MARCHE_AVANT"
                elif egocentric_change < -WALK_SCALE_THRESHOLD:
                    walk_candidate = "MARCHE_ARRIERE"

        if walk_candidate == self.candidate_walk:
            self.walk_hold_count += 1
        else:
            self.candidate_walk = walk_candidate
            self.walk_hold_count = 1

        if self.walk_hold_count >= 3 and walk_candidate != self.confirmed_walk:
            self.confirmed_walk = walk_candidate
            if self.confirmed_walk != "IMMOBILE":
                self.get_logger().info(f"Marche détectée ({self.confirmed_orientation}) : {self.confirmed_walk}")

        # 6. Détection du mouvement "67" (fonctionne de face et de dos)
        arms_ready_67 = False
        if self.valid(k, 5, 6, 9, 10):
            min_y_67 = mid_shoulder_y - 0.35 * shoulder_w
            max_y_67 = mid_hip_y + 0.35 * shoulder_w
            wrists_in_zone = (min_y_67 <= k[9][1] <= max_y_67) and (min_y_67 <= k[10][1] <= max_y_67)

            if self.valid(k, 7, 8):
                left_elbow_angle = angle_deg(k[5], k[7], k[9])
                right_elbow_angle = angle_deg(k[6], k[8], k[10])
                elbows_ok = (left_elbow_angle < 155.0) or (right_elbow_angle < 155.0)
            else:
                # De dos, les coudes peuvent être partiellement masqués par le buste
                elbows_ok = is_back

            arms_ready_67 = wrists_in_zone and elbows_ok

        if arms_ready_67:
            diff_y = (k[9][1] - k[10][1]) / shoulder_w
            if diff_y < -AMP_67_THRESHOLD:
                current_side = -1
            elif diff_y > AMP_67_THRESHOLD:
                current_side = 1
            else:
                current_side = 0

            if current_side != 0 and current_side != self.last_67_side:
                if self.last_67_side != 0 and (now - self.last_67_switch_time) <= MAX_67_INTERVAL:
                    self.half_cycles_67 += 1
                    if self.half_cycles_67 % 2 == 0:
                        self.compteur_67 += 1
                        self.last_67_valid_time = now
                        self.get_logger().info(
                            f"Mouvement 67 détecté ({self.confirmed_orientation}) ! Compteur : {self.compteur_67}"
                        )
                else:
                    self.half_cycles_67 = 0

                self.last_67_side = current_side
                self.last_67_switch_time = now

        if (now - self.last_67_switch_time) > (MAX_67_INTERVAL + 0.3):
            self.last_67_side = 0
            self.half_cycles_67 = 0

        mouvement_67_en_cours = (now - self.last_67_valid_time) < 0.8

        # 7. Postures statiques des bras (adaptées à l'inversion gauche/droite de dos)
        side_sign = -1.0 if is_back else 1.0

        left_arm_up = self.valid(k, 5, 7, 9) and (k[9][1] < k[5][1] - 0.2 * shoulder_w)
        right_arm_up = self.valid(k, 6, 8, 10) and (k[10][1] < k[6][1] - 0.2 * shoulder_w)

        left_arm_side = (
            self.valid(k, 5, 7, 9)
            and abs(k[9][1] - k[5][1]) < 0.45 * shoulder_w
            and side_sign * (k[9][0] - k[5][0]) > 0.9 * shoulder_w
            and angle_deg(k[5], k[7], k[9]) > 140.0
        )
        right_arm_side = (
            self.valid(k, 6, 8, 10)
            and abs(k[10][1] - k[6][1]) < 0.45 * shoulder_w
            and side_sign * (k[6][0] - k[10][0]) > 0.9 * shoulder_w
            and angle_deg(k[6], k[8], k[10]) > 140.0
        )

        # 8. Salut de la main & pas latéral
        dynamic_event = None
        if mouvement_67_en_cours:
            dynamic_event = "MOUVEMENT_67"
        elif self.confirmed_rot != "AUCUNE":
            dynamic_event = self.confirmed_rot
        elif right_arm_up:
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

        if dynamic_event is None and len(self.center_x_history) >= 5:
            dx_body = (self.center_x_history[-1] - self.center_x_history[0]) / shoulder_w
            if dx_body > 0.45:
                dynamic_event = "PAS_GAUCHE" if not is_back else "PAS_DROIT"
            elif dx_body < -0.45:
                dynamic_event = "PAS_DROIT" if not is_back else "PAS_GAUCHE"

        # 9. Classification posture statique
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

        if pose == self.candidate_pose:
            self.hold_count += 1
        else:
            self.candidate_pose = pose
            self.hold_count = 1

        if self.hold_count == 4 and pose != self.confirmed_pose:
            self.confirmed_pose = pose
            self.get_logger().info(f"Posture validée ({self.confirmed_orientation}) : {self.confirmed_pose}")

        if dynamic_event and dynamic_event not in ("MOUVEMENT_67", "TOURNE_GAUCHE", "TOURNE_DROITE") and (now - self.last_dynamic_time > 1.0):
            self.last_dynamic_time = now
            self.get_logger().info(f"Mouvement détecté ({self.confirmed_orientation}) : {dynamic_event}")

        # 10. Publication JSON complète sur /detected_motion
        out = String()
        out.data = json.dumps({
            "orientation": self.confirmed_orientation,
            "posture": self.confirmed_pose,
            "marche": self.confirmed_walk,
            "rotation": self.confirmed_rot,
            "mouvement": dynamic_event,
            "mouvement_67_actif": mouvement_67_en_cours,
            "compteur_67": self.compteur_67,
            "confiance_cible": round(det["score"], 3),
            "echelle_corps_px": round(body_scale, 1),
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
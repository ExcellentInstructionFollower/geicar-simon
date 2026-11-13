#!/usr/bin/env python3
"""Publish a stop request when the lidar sees an obstacle ahead."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleStopNode(Node):
    def __init__(self):
        # Enregistre ce programme comme nœud ROS 2 nommé obstacle_stop_node.
        super().__init__('obstacle_stop_node')

        # Paramètres modifiables au lancement, sans changer le code.
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stop_topic', '/obstacle_stop')
        self.declare_parameter('stop_distance', 0.6)
        self.declare_parameter('release_distance', 0.75)
        self.declare_parameter('front_center_angle_deg', 0.0)
        self.declare_parameter('front_half_angle_deg', 35.0)
        self.declare_parameter('scan_timeout', 0.5)

        # Lit les paramètres. Les calculs d'angles se font en radians,
        # même si les valeurs sont configurées en degrés.
        self._stop_distance = float(self.get_parameter('stop_distance').value)
        self._release_distance = float(self.get_parameter('release_distance').value)
        self._front_center_angle = math.radians(
            float(self.get_parameter('front_center_angle_deg').value)
        )
        self._front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value)
        )
        self._scan_timeout = float(self.get_parameter('scan_timeout').value)

        # Vérifie les paramètres pour éviter une configuration incohérente.
        if self._stop_distance <= 0.0:
            raise ValueError('stop_distance must be greater than zero')
        if self._release_distance <= self._stop_distance:
            raise ValueError('release_distance must be greater than stop_distance')
        if not 0.0 < self._front_half_angle <= math.pi:
            raise ValueError('front_half_angle_deg must be in (0, 180]')
        if self._scan_timeout <= 0.0:
            raise ValueError('scan_timeout must be greater than zero')

        scan_topic = str(self.get_parameter('scan_topic').value)
        stop_topic = str(self.get_parameter('stop_topic').value)

        # L'éditeur envoie l'état d'arrêt; l'abonné reçoit les scans du lidar.
        self._stop_publisher = self.create_publisher(Bool, stop_topic, 10)
        self._scan_subscription = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10
        )

        # Au démarrage, on demande l'arrêt tant qu'aucun scan récent n'a été reçu.
        self._last_scan_time = None
        self._stop_requested = True

        # Publie régulièrement l'état, y compris si aucun scan n'arrive.
        self._status_timer = self.create_timer(0.1, self._publish_status)

        self.get_logger().info(
            f'Watching {scan_topic}; stopping below {self._stop_distance:.2f} m '
            f'and releasing above {self._release_distance:.2f} m '
            f'within +/-{math.degrees(self._front_half_angle):.1f} degrees of '
            f'{math.degrees(self._front_center_angle):.1f} degrees. '
            f'Publishing stop state on {stop_topic}.'
        )

    def _scan_callback(self, scan: LaserScan):
        # Mémorise l'heure de réception pour détecter ensuite un scan périmé.
        self._last_scan_time = time.monotonic()

        # Sans mesures exploitables, on adopte un comportement prudent et on arrête.
        if not scan.ranges or scan.angle_increment == 0.0:
            self._set_stop(True, 'invalid or empty lidar scan')
            return

        points_in_front = 0
        valid_distances = []
        for index, distance in enumerate(scan.ranges):
            # Chaque élément de ranges correspond à un angle :
            # angle_min + index * angle_increment.
            angle = scan.angle_min + index * scan.angle_increment

            # Ramène l'angle dans [-pi, pi], puis mesure son écart au centre
            # du secteur surveillé à l'avant du véhicule.
            angle = math.atan2(math.sin(angle), math.cos(angle))
            angle_from_front = math.atan2(
                math.sin(angle - self._front_center_angle),
                math.cos(angle - self._front_center_angle),
            )
            if abs(angle_from_front) > self._front_half_angle:
                continue

            points_in_front += 1

            # Ignore les valeurs infinies/NaN (pas de retour lidar) et les
            # distances hors de la portée maximale annoncée par le scan.
            if math.isfinite(distance) and 0.0 <= distance <= scan.range_max:
                valid_distances.append(distance)

        # Si le scan ne couvre pas le secteur demandé, on ne peut pas garantir
        # que la voie est libre.
        if points_in_front == 0:
            self._set_stop(True, 'scan contains no measurements in the front sector')
            return

        # Si toutes les mesures valides sont absentes (par exemple inf),
        # aucune proximité d'obstacle n'est signalée.
        closest = min(valid_distances, default=math.inf)

        # Hystérésis : on arrête sous stop_distance, mais on ne repart qu'une
        # fois la distance supérieure à release_distance pour éviter les à-coups.
        should_stop = (
            closest <= self._release_distance
            if self._stop_requested
            else closest <= self._stop_distance
        )
        if should_stop:
            self._set_stop(True, f'obstacle at {closest:.2f} m')
        else:
            self._set_stop(False, 'front sector clear')

    def _set_stop(self, stop: bool, reason: str):
        # Ne journalise que les changements d'état pour éviter un message
        # console à chaque tour du lidar.
        if stop != self._stop_requested:
            self._stop_requested = stop
            if stop:
                self.get_logger().warning(f'STOP requested: {reason}')
            else:
                self.get_logger().info('Obstacle stop released: front sector clear')

    def _publish_status(self):
        # Si le lidar n'a jamais répondu ou que son dernier scan est trop ancien,
        # garder l'arrêt actif. Le message périodique permet aussi à car_control
        # de détecter si ce nœud cesse lui-même de fonctionner.
        scan_is_stale = (
            self._last_scan_time is None
            or time.monotonic() - self._last_scan_time > self._scan_timeout
        )
        if scan_is_stale:
            self._set_stop(True, 'lidar scan is missing or stale')

        # Bool.data vaut True pour demander l'arrêt, False pour le relâcher.
        msg = Bool()
        msg.data = self._stop_requested
        self._stop_publisher.publish(msg)


def main(args=None):
    # Initialise ROS 2, exécute le nœud, puis libère proprement les ressources.
    rclpy.init(args=args)
    node = ObstacleStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

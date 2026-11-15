#!/usr/bin/env python3
"""Creates sequences of moves for the car."""

import math
import time

import rclpy
from rclpy.node import Node
from interfaces.msg import AutonomousCommand
from std_msgs.msg import Bool


class ObstacleStopNode(Node):
    def __init__(self):
        # Enregistre ce programme comme nœud ROS 2 nommé obstacle_stop_node.
        super().__init__('obstacle_stop_node')

        # Paramètres modifiables au lancement, sans changer le code.
        self.declare_parameter('command_topic', '/autonomous_command')
        self.declare_parameter('feedback_topic', '/autonomous_feedback')

        us_topic = str(self.get_parameter('us_topic').value)
        stop_topic = str(self.get_parameter('stop_topic').value)

        # L'éditeur envoie l'état d'arrêt; l'abonné reçoit les scans du lidar.
        self._command_publisher = self.create_publisher(AutonomousCommand, autonomous_command, 10)
        self._feedback_subscription = self.create_subscription(
            Bool, autonomous_feedback, self._feedback_callback, 10
        )

        # Publie régulièrement l'état, y compris si aucun scan n'arrive.
        self._status_timer = self.create_timer(0.1, self._publish_status)

        """
        self.get_logger().info(
            f'Watching {us_topic}; stopping below {self._stop_distance:.2f} m '
            f'and releasing above {self._release_distance:.2f} m '
            f'Publishing stop state on {stop_topic}.'
        )"""

    def _us_callback(self, us_data: Ultrasonic):
        # Mémorise l'heure de réception pour détecter ensuite un scan périmé.
        self._last_scan_time = time.monotonic()

        valid_distances = [us_data.front_left, 
            us_data.front_center, 
            us_data.front_right,
            us_data.rear_left, 
            us_data.rear_center, 
            us_data.rear_right]

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
            self._set_stop(True, 'ultrasound scan is missing or stale')

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
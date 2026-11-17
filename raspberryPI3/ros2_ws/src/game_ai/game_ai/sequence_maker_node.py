#!/usr/bin/env python3
"""ROS 2 node scaffold for generating step sequences."""

import rclpy
from rclpy.node import Node
from interfaces.msg import AutonomousCommand
import random

class SequenceGeneratorNode(Node):
    """Node scaffold; sequence generation will be implemented later."""

    def __init__(self):
        super().__init__('sequence_generator_node')

        self.command_publisher = self.create_publisher(
            AutonomousCommand,
            '/autonomous_command',
            10,
        )

        self._generated_steps = 0
        self._sequence_timer = self.create_timer(12.0, self.generate_sequence)

    def generate_sequence(self):
        if self._generated_steps >= 20:
            self._sequence_timer.cancel()
            return

        move = random.randint(1, 4)

        command = AutonomousCommand()
        command.move = move
        self.command_publisher.publish(command)
        self.get_logger().info(f'[SEQ-GEN-NODE] Generated step : {move}')
        self._generated_steps += 1

        if self._generated_steps >= 20:
            self._sequence_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SequenceGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Game session manager."""

""""Terminal 1 - initialize game"""
""""ros2 run game_ai game_manager_node"""

""""Terminal 2 - observe sequence"""
""""ros2 topic echo /game/sequence"""

""""Terminal 3 - play game"""
#Start and restart
""""ros2 service call /game/start std_srvs/srv/Trigger "{}" """
#Send sequence
""""ros2 topic pub --once /game/player_input std_msgs/msg/Int32MultiArray "{data: [2]}" """
""""ros2 topic pub --once /game/player_input std_msgs/msg/Int32MultiArray "{data: [2, 4]}" """
"""" ... """
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
from std_srvs.srv import Trigger
from interfaces.msg import AutonomousCommand



class GameManagerNode(Node):
    MAX_STEPS = 20

    def __init__(self):
        super().__init__('game_manager_node')

        self.sequence = []
        self.game_active = False

        # Publish current sequence: [1], [1, 4], [1, 4, 2], ...
        self.sequence_publisher = self.create_publisher(
            Int32MultiArray, '/game/sequence', 10
        )

        # Evaluate player answer.
        self.result_publisher = self.create_publisher(
            Bool, '/game/correct', 10
        )

        # Receive player list
        self.player_subscription = self.create_subscription(
            Int32MultiArray,
            '/game/player_input',
            self.verify_player_sequence,
            10,
        )

        # Starts or restart game.
        self.start_service = self.create_service(
            Trigger, '/game/start', self.start_game
        )

        self.command_publisher = self.create_publisher(
            AutonomousCommand,
            '/autonomous_command',
            10
        )

        self.ready_publisher = self.create_publisher(
            Bool,
            '/game/ready_for_input',
            10
        )

        self.get_logger().info('Game manager ready. Call /game/start.')

    def start_game(self, request, response):
        self.sequence = []
        self.game_active = True
        self.add_move()  # First round generate one move

        response.success = True
        response.message = 'Game started. First sequence published.'
        return response

    def add_move(self):
        """Add one movement to the list."""
        move = random.randint(1, 4)
        self.sequence.append(move)

        message = Int32MultiArray()
        message.data = self.sequence.copy()
        self.sequence_publisher.publish(message)

        self.get_logger().info(
            f'Round {len(self.sequence)} - sequence: {self.sequence}'
        )
        self.begin_playback()

    def begin_playback(self):
        self.waiting_for_input = False #CHANGE
        self.playing_sequence = True
        self.playback_index = 0
        self.ready_publisher.publish(Bool(data=False))

        # Publish Moves in Car command
        self.publish_next_move()

    def publish_next_move(self):
        move = self.sequence[self.playback_index]
        command = AutonomousCommand()
        #command.start = True
        command.move = int(move)
        self.command_publisher.publish(command)
        self.playback_index += 1
        self.get_logger().info(
            f'Playing move {self.playback_index}/{len(self.sequence)}: {move}'
        )

    def verify_player_sequence(self, message):
        """Generate next movement after review"""
        if not self.game_active:
            self.get_logger().warn('No active game. Call /game/start first.')
            return

        player_input = list(message.data)
        correct = player_input == self.sequence
        self.result_publisher.publish(Bool(data=correct))

        if not correct:
            self.get_logger().warn(
                f'Incorrect sequence: {player_input}. '
                f'Expected {len(self.sequence)} moves. Try again.'
            )
            return  # For now - after will restart the game.

        self.get_logger().info('Correct sequence!')

        if len(self.sequence) >= self.MAX_STEPS:
            self.game_active = False
            self.get_logger().info('Game finished: 20 rounds completed!')
            return

        self.add_move()  # only after right answer.


def main(args=None):
    rclpy.init(args=args)
    node = GameManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

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
#Send move to perform to autonomous node
""""ros2 topic pub --once /autonomous_command  interfaces/msg/AutonomousCommand "{movetype: 1, movement_done: false}" """
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
from std_srvs.srv import Trigger
from interfaces.msg import AutonomousCommand



class GameManagerNode(Node):
    MAX_STEPS = 4

    def __init__(self):
        super().__init__('game_manager_node')

        self.sequence = []
        self.game_running = False
        self.autonomous_ack = False
        self.waiting_for_input = False
        self.playing_sequence = False
        self.playback_index = 0
        self._playback_timer = None

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

        self.ack_autonomous = self.create_subscription(
            AutonomousCommand,
            '/movement_ack',
            self.recv_autonomous_ack,
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
        self.game_running = True
        self.waiting_for_input = False
        self.run_game()
        response.success = True
        response.message = 'Game started. First sequence published.'
        return response

    def add_move(self):
        """Append a random movement and play the complete sequence."""
        self.sequence.append(self.get_step())

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
        if self.playback_index >= len(self.sequence):
            return

        move = self.sequence[self.playback_index]
        self.publish_move_to_perform(move)
        self.autonomous_ack = False
        self.playback_index += 1
        self.get_logger().info(
            f'Playing move {self.playback_index}/{len(self.sequence)}: {move}'
        )

    # recv acknowledgement from autonomous node

    def recv_autonomous_ack(self, message):
        if self.playing_sequence and message.movement_done:
            self.autonomous_ack = True
            self.get_logger().info('Ack received from autonomous')



    def verify_player_sequence(self, message):
        """Check the player's answer after the full sequence was played."""
        if not self.game_running:
            self.get_logger().warn('No active game. Call /game/start first.')
            return
        if not self.waiting_for_input:
            self.get_logger().warn('Sequence is not ready for player input yet.')
            return

        self.waiting_for_input = False
        player_input = list(message.data)
        correct = player_input == self.sequence
        self.publish_step_correctness(correct)

        if not correct:
            self.get_logger().warn(
                f'Incorrect sequence: {player_input}. '
                f'Expected {len(self.sequence)} moves. Replaying the sequence.'
            )
            self.begin_playback()
            return

        self.get_logger().info('Correct sequence!')

        if len(self.sequence) >= self.MAX_STEPS:
            self.game_running = False
            self.get_logger().info('Game finished: 3 rounds completed!')
            return

        self.add_move()  # only after right answer.

    # publish towards Screen node if step was correctly done or not

    def publish_step_correctness(self, correct):

        self.result_publisher.publish(Bool(data=correct))

    # should be call when game launch topic sent from Movement detect Node is set to true
    # generate the sequence to perform
    # send one by one to autonomous node and screen node
    # wait for acknowledgment
    # once all the step is performed by the car and displayed by the screen, publishes the detected movement from AI
    # compare expected step with detected one and publish one the screen whether the player has correctly performed the step or not
    # reapeat all of the above until the last sequence is done 

    def run_game(self):
        """Start the first round and service acknowledgements asynchronously."""
        self.add_move()
        if hasattr(self, '_playback_timer') and self._playback_timer is not None:
            self._playback_timer.cancel()
        self._playback_timer = self.create_timer(0.05, self.advance_playback)

    def advance_playback(self):
        # Advance to the next move when the autonomous node acknowledges one.
        if not self.playing_sequence or not self.autonomous_ack:
            return

        if self.playback_index < len(self.sequence):
            # publish_next_move consumes the ack and increments playback_index.
            self.publish_next_move()
            return

        # The last published move has now been acknowledged.
        self.autonomous_ack = False
        self.playing_sequence = False
        self.waiting_for_input = True
        self.ready_publisher.publish(Bool(data=True))
        self.get_logger().info('Sequence playback complete. Ready for player input.')
    

    # publish on the movement_to_perform topic the move to perform, would be received by Auto and Screen Node
    # move : move to publish

    def publish_move_to_perform(self, move):

        command = AutonomousCommand()
        command.movetype = move
        command.movement_done = False
        self.command_publisher.publish(command)

    # return a new step to append in self.sequence

    def get_step(self):

        return random.randint(1, 4)
    


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

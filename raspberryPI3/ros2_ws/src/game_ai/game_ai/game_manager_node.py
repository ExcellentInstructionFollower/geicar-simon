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
        self.game_running = False
        self.expected_step
        self.autonomous_ack  = False

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
        self.game_running = True
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
        ## Sending the initial sequence is working, 
        ## it does not advance to next movement, due to not yet implemented feedback logic in 
        if self.autonomous_ack :
            move = self.sequence[self.playback_index]
            command = AutonomousCommand()
            #command.start = True
            command.move = int(move)
            self.command_publisher.publish(command)
            self.playback_index += 1
            self.get_logger().info(
                f'Playing move {self.playback_index}/{len(self.sequence)}: {move}'
            )

    def recv_ack(self, message):
        self.ack = message.movement_done


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

    # publish towards Screen node if step was correctly done or not

    def publish_step_correctness(self, correct):

        
        pass

    # the callback function for Game launch subscription, store in game_active, communicate with AI node
    # create self.start : bool
    
    def check_start(self, message):

        

        pass


    # callback that store in Node attribute the AI detected movement and, later, in another function, compare to expected movement.
    # create self.ai_move_detected : direction

    def get_performed_move(self, message):        

        pass

    # should be call when game launch topic sent from Movement detect Node is set to true
    # generate the sequence to perform
    # send one by one to autonomous node and screen node
    # wait for acknowledgment
    # once all the step is performed by the car and displayed by the screen, publishes the detected movement from AI
    # compare expected step with detected one and publish one the screen whether the player has correctly performed the step or not
    # reapeat all of the above until the last sequence is done 

    def run_game(self):

        pass
    

    # publish on the movement_to_perform topic the move to perform, would be received by Auto and Screen Node
    # move : move to publish

    def publish_move_to_perform(self, move):


        pass

    # return a new step to append in self.sequence

    def get_step(self):

        pass

    # generate the whole step sequence, call several time get_step fct
    # n : number of step in the sequence

    def generate_sequence(self, n):

        pass

    # called when autonomous node sent his acknowledgement, then check if move_done is true 


    def check_car_mouvement_performed(self, message):

        pass
    


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

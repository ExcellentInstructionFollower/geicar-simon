#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
import threading
import time
import zmq

from interfaces.msg import AutonomousCommand


ZMQ_PORT = 5555
UPDATES_PER_SECOND = 2.0
RAISE_LEFT_ARM = 5
RAISE_RIGHT_ARM = 6
RAISE_BOTH_ARMS = 7
MOVE_TIME_NS = 3e9


class ZmqSenderNode(Node):
    def __init__(self):
        super().__init__('zmq_sender_node')

        # 1. Initialize ZeroMQ (Pubisher)
        self.zmq_ctx = zmq.Context()
        self.socket = self.zmq_ctx.socket(zmq.PUB)

        # "tcp://*:5555" == listen on all network interfaces on port 5555
        try:
            self.socket.bind(f"tcp://*:{ZMQ_PORT}")
            self.get_logger().info(f"ZeroMQ Publisher bound to port {ZMQ_PORT}")
        except zmq.ZMQError as e:
            self.get_logger().error(f"Failed to bind ZMQ port: {e}")
            raise e

        # 2. Shared Data
        self.lock = threading.Lock()
        self.latest_data = {
            "state": 0,
            "left_arm_state": 0,
            "right_arm_state": 0,
            "game_state": -1
        }

        # 3. ROS 2 Subscriptions and Publishers
        self.create_subscription(Bool, '/stop_topic', self.stop_callback, 10)
        self.create_subscription(AutonomousCommand, '/autonomous_command', self.command_callback, 10)
        self.create_subscription(Bool, '/game/correct', self.game_callback, 10)

        self.ack_publisher = self.create_publisher(AutonomousCommand, "/autonomous_ack", 10)

        # 4. Transmission Timer
        self.timer = self.create_timer(1.0/UPDATES_PER_SECOND, self.publish_to_gui)

        # 5. Arm movement handling logic
        self.move_in_play = False
        self.move_type = 0
        self.time_since_move_start = time.monotonic_ns()

    def stop_callback(self, data):
        with self.lock:
            if data.data: 
                #STOP WAS REQUESTED
                self.latest_data['state'] = 0
            else:
                #CAR CAN MOVE
                self.latest_data['state'] = 1

    def command_callback(self, data):
        with self.lock:
            self.latest_data['left_arm_state'] = 0
            self.latest_data['right_arm_state'] = 0
            self.time_since_move_start = time.monotonic_ns()
            self.move_type = data.movetype
            self.move_in_play = True
            
            if data.movetype == RAISE_BOTH_ARMS:
                self.latest_data['left_arm_state'] = 1
                self.latest_data['right_arm_state'] = 1
            elif data.movetype == RAISE_LEFT_ARM:
                self.latest_data['left_arm_state'] = 1
            elif data.movetype == RAISE_RIGHT_ARM:
                self.latest_data['right_arm_state'] = 1
            else:
                self.move_in_play = False

                

    def game_callback(self, data):
        with self.lock:
            if data.data:
                self.latest_data['game_state'] = 3
            else:
                self.latest_data['game_state'] = 4

    # --- Sender Loop ---
    def publish_to_gui(self):
        try:
            must_send_ack = False
            movetype_performed = 0

            with self.lock:
                # Thread safety
                if self.move_in_play and (time.monotonic_ns - self.time_since_move_start >= MOVE_TIME_NS):
                    self.move_in_play = False
                    must_send_ack = True
                    movetype_performed = self.move_type
                    
                data_to_send = self.latest_data.copy()

            if must_send_ack:
                msg = AutonomousCommand()
                msg.movement_done = True
                msg.movetype = movetype_performed
                self.ack_publisher.publish(msg)

            #self.get_logger().info(f"Envoi ZMQ : {data_to_send}")
            self.socket.send_pyobj(data_to_send, flags=zmq.NOBLOCK)
        except zmq.Again: # buffer is full
            pass
        except Exception as e:
            self.get_logger().warn(f"ZMQ Send Error: {e}")

    def destroy_node(self):
        self.socket.close()
        self.zmq_ctx.term()
        super().destroy_node()

def main():
    rclpy.init()
    node = ZmqSenderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


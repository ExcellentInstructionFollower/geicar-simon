#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
import threading
import time
import zmq

from interfaces.msg import State, ArduinoData, AI, FollowingStateEvent

ZMQ_PORT = 5555
UPDATES_PER_SECOND = 2.0

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
            "temperature": 0,
            "humidity": 0,
            "gesture": "none",
            "following_state": -1
        }

        # 3. ROS 2 Subscriptions
        self.create_subscription(State, '/state', self.state_callback, 10)
        self.create_subscription(ArduinoData, '/arduino_data', self.arduino_callback, 10)
        self.create_subscription(AI, '/ai_perception_data', self.gesture_callback, 10)
        self.create_subscription(FollowingStateEvent, '/following_state_event', self.following_state_callback, 10)

        # 4. Transmission Timer
        self.timer = self.create_timer(1.0/UPDATES_PER_SECOND, self.publish_to_gui)

    def state_callback(self, data):
        with self.lock:
            self.latest_data['state'] = data.state

    def arduino_callback(self, data):
        with self.lock:
            self.latest_data['temperature'] = data.temp
            self.latest_data['humidity'] = data.humidity

    def gesture_callback(self, data):
        # print(f"gui sender recved",data.gesture)
        with self.lock:
            self.latest_data['gesture'] = data.gesture.class_name

    def following_state_callback(self, data):
        with self.lock:
            self.latest_data['following_state'] = data.state

    # --- Sender Loop ---
    def publish_to_gui(self):
        try:
            with self.lock:
                # Thread safety
                data_to_send = self.latest_data.copy()
            
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


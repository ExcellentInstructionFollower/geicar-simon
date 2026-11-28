import sys
import time
import zmq
import signal
import json
import requests
import websockets
import asyncio
from html import unescape
from threading import Thread

from PyQt6.QtWidgets import QApplication
from PyQt6.QtQml import QQmlApplicationEngine
from PyQt6.QtCore import QObject, pyqtSignal, pyqtProperty, QThread, QTimer

import os

class NetworkWorker(QThread):
    signal = pyqtSignal(dict)

    def run(self):
        ROS_IP = '192.168.1.1' # 'localhost'
        ROS_PORT = 5555

        print(f"[ZMQ] Thread started. Initializing context...")

        try:
            context = zmq.Context()
            socket = context.socket(zmq.SUB)
            
            print(f"[ZMQ] Connecting to tcp://{ROS_IP}:{ROS_PORT}...")
            socket.connect(f"tcp://{ROS_IP}:{ROS_PORT}")
            
            socket.subscribe("") # sub to all
            print("[ZMQ] Connection established & Subscribed to all topics.")
            print("[ZMQ] Waiting for first packet...")

            while True:
                try:
                    data = socket.recv_pyobj()
                    if data.get("following_state",-1) != -1:
                        print(f"[ZMQ] Received: {data}")
                    self.signal.emit(data)

                except zmq.ZMQError as e:
                    print(f"[ZMQ] Socket Error: {e}")
                    time.sleep(1)
                    
                except KeyboardInterrupt:
                    print("[ZMQ] User interrupted.")
                    break

        except Exception as e:
            print(f"[ZMQ] Critical Initialization Error: {e}")


class DashboardBackend(QObject):
    def __init__(self):
        super().__init__()
        self._state = 0
        self._left_arm = 0
        self._right_arm = 0
        self._gameState = "Deactivated"

        self.buffered_game_state = 0

        self.follow_timer = QTimer()
        self.follow_timer.setInterval(3000)
        self.follow_timer.setSingleShot(True)
        self.follow_timer.timeout.connect(self.follow_timeout)
        # ---------------------------


    stateChanged = pyqtSignal()
    leftArmChanged = pyqtSignal()
    rightArmChanged = pyqtSignal()
    gameStateChanged = pyqtSignal()

    @pyqtProperty(int, notify=stateChanged)
    def state(self):
        return self._state

    @pyqtProperty(int, notify=leftArmChanged)
    def left_arm(self):
        return self._left_arm

    @pyqtProperty(int, notify=rightArmChanged)
    def right_arm(self):
        return self._right_arm
   
    @pyqtProperty(str, notify=gameStateChanged)
    def gameState(self):
        return self._gameState

    def follow_timeout(self):
        print("And of follow timer, using buffered state")
        self._state = self.gstate_to_str(self.buffered_game_state)
        self.followinStateChanged.emit()

    def gstate_to_str(self,state):
        if state == 1:
            return "Follow the car's movement's !"
        elif state == 2:
            return "Now do the same moves as the car"
        elif state == 3:
            return "Good job ! Can you do it again with one more move ?"
        elif state == 4:
            return "Oops, it looks like you didn't do the right moves... "
        else:
            print(f"[WARN] unknown following state received: {state}")
            return "Unknown state"

    def update_follow(self,fstate):
        # Buffered events
        if fstate == -1:
            return # no new fstate
        else: # always buffer latest fstate
            self.buffered_game_state = fstate

        if self._state == 0: # mode STOP:
            self._gameState = "Deactivated"
        else:
            # if self.follow_timer.isActive():
            #     return # do not update while timer is running
            # if fstate == 1: # Master lost
            #     self.follow_timer.start()
            self._gameState = self.gstate_to_str(fstate)
        
        self.gameStateChanged.emit()

    def update_data(self, data: dict):
        # Mise à jour des données standard
        self._state = data.get("state", 0)
        self._left_arm = data.get("left_arm_state", 0)
        self._right_arm = data.get("right_arm_state", 0)

        # --- FOLLOWING_STATE ---
        self.update_follow(data.get("game_state",-1))
        
        # Emission des autres signaux
        self.stateChanged.emit()
        self.leftArmChanged.emit()
        self.rightArmChanged.emit()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL) # Should allow closing with ctrl+c

    app = QApplication(sys.argv)
    
    engine = QQmlApplicationEngine()
    backend = DashboardBackend()
    engine.rootContext().setContextProperty("backend", backend)
    
    if os.path.exists("dashboard_game.qml"):
        engine.load("dashboard_game.qml")
    else:
        engine.load("/home/pi/code/dashboard_game.qml")

    if not engine.rootObjects():
        print("No QML file found!")
        sys.exit(-1)

    worker = NetworkWorker()
    worker.signal.connect(backend.update_data)
    worker.start()

    sys.exit(app.exec())

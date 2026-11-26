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

async def websocket_loop(callback):
    num_errors = 0
    while num_errors<10:
        try:
            async with websockets.connect("wss://magictintin.fr/ws") as websocket:
                # await websocket.send("luthen/main:ping")
                await websocket.send("micasend:ping")
                while True:
                    response = await websocket.recv()
                    # print(f"Response: {response}")
                    callback(response)
        except Exception as e:
            num_errors += 1
            print("Error:",e)

class DashboardBackend(QObject):
    def __init__(self):
        super().__init__()
        self._state = 0
        self._temp = 0
        self._humid = 0
        self._gesture = "none"
        self._followingState = "Deactivated"
        self._motd = ""

        self.buffered_following_state = 0

        # --- AJOUT LOGIQUE TIMER ---
        # self.gesture_timer = QTimer()
        # self.gesture_timer.setInterval(300)  # 5000 ms = 5 secondes
        # self.gesture_timer.setSingleShot(True) # Le timer ne s'exécute qu'une fois
        # self.gest
        # ure_timer.timeout.connect(self.reset_gesture) # Quand le temps est écoulé, on appelle cette fonction

        self.follow_timer = QTimer()
        self.follow_timer.setInterval(3000)
        self.follow_timer.setSingleShot(True)
        self.follow_timer.timeout.connect(self.follow_timeout)
        # ---------------------------

        self.socket_thread = Thread(target=lambda:asyncio.run(websocket_loop(self.update_motd)))
        self.socket_thread.daemon = True
        self.socket_thread.start()

    stateChanged = pyqtSignal()
    tempChanged = pyqtSignal()
    humidChanged = pyqtSignal()
    gestureChanged = pyqtSignal()
    followinStateChanged = pyqtSignal()
    motdChanged = pyqtSignal()

    @pyqtProperty(int, notify=stateChanged)
    def state(self):
        return self._state

    @pyqtProperty(int, notify=tempChanged)
    def temperature(self):
        return self._temp

    @pyqtProperty(int, notify=humidChanged)
    def humidity(self):
        return self._humid

    @pyqtProperty(str, notify=gestureChanged)
    def gesture(self):
        return self._gesture
    
    @pyqtProperty(str, notify=followinStateChanged)
    def followingState(self):
        return self._followingState
    
    @pyqtProperty(str, notify=motdChanged)
    def motd(self):
        return self._motd

    # --- AJOUT : Fonction appelée quand le timer est fini ---
    def reset_gesture(self):
        print("Fin du délai : remise à zéro du geste.")
        self._gesture = "none"
        self.gestureChanged.emit()
    # ------------------------------------------------------

    def follow_timeout(self):
        print("And of follow timer, using buffered state")
        self._state = self.fstate_to_str(self.buffered_following_state)
        self.followinStateChanged.emit()

    def fstate_to_str(self,state):
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
            self.buffered_following_state = fstate

        if self._state == 0: # mode STOP:
            self._followingState = "Deactivated"
        else:
            # if self.follow_timer.isActive():
            #     return # do not update while timer is running
            # if fstate == 1: # Master lost
            #     self.follow_timer.start()
            self._followingState = self.fstate_to_str(fstate)
        
        self.followinStateChanged.emit()

    def update_data(self, data: dict):
        # Mise à jour des données standard
        self._state = data.get("state", 0)
        self._temp = data.get("temperature", 0)
        self._humid = data.get("humidity", 0)
        
        # --- LOGIQUE INTELLIGENTE GESTE ---
        incoming_gesture: str = data.get("gesture", "none").replace('_',' ')
        
        self._gesture = incoming_gesture
        self.gestureChanged.emit()
        # if incoming_gesture != "none":
        #     # Cas 1 : On reçoit un VRAI geste
        #     # On met à jour immédiatement et on (re)lance le timer
        #     if self._gesture != incoming_gesture:
        #         self._gesture = incoming_gesture
        #         self.gestureChanged.emit()
        #     
        #     # On redémarre le timer (ça remet le compteur à 5s même s'il tournait déjà)
        #     self.gesture_timer.start()

        # else:
        #     # Cas 2 : On reçoit "none"
        #     # Si le timer tourne, on IGNORE ce "none" (on est en période de maintien)
        #     if self.gesture_timer.isActive():
        #         pass 
        #     else:
        #         # Si le timer ne tourne pas, on peut afficher "none"
        #         if self._gesture != "none":
        #             self._gesture = "none"
        #             self.gestureChanged.emit()
        # ----------------------------------

        # --- FOLLOWING_STATE ---
        self.update_follow(data.get("following_state",-1))
        
        # Emission des autres signaux
        self.stateChanged.emit()
        self.tempChanged.emit()
        self.humidChanged.emit()
        # Note: gestureChanged est géré dans la logique ci-dessus pour éviter les clignotements
    
    def update_motd(self,_):
        rep = requests.get("https://micasend.magictintin.fr/msg?getmsg=json")
        js = json.loads(unescape(rep.text))[-1]
        sender:str = js.get("sender","???")
        msg:str = js.get("content","")
        msg = msg.replace('§',' ') # because baptiste is an injenir
        self._motd = f"{sender}: {msg}"
        self.motdChanged.emit()

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

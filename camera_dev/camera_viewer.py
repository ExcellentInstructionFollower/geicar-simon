#!/usr/bin/env python3
"""
Affiche le flux caméra de la Jetson (via web_video_server) sur le PC.

Principe : tunnel SSH PC -> Raspberry -> Jetson:8080, puis lecture du flux MJPEG.

Prérequis côté voiture (dans le conteneur ros-humble de la Jetson) :
  - connection ssh : ssh -J pi@10.105.1.168 jetson@192.168.1.10
  - les nœuds caméra lancés  : ros2 launch geicar_start_jetson geicar.jetson.launch.py
  - le serveur vidéo lancé   : ros2 run web_video_server web_video_server

Usage :
  python3 camera_viewer.py --pi 10.105.1.XX
  python3 camera_viewer.py --pi 10.105.1.XX --browser      # ouvre le navigateur à la place
Dépendances (optionnel) : pip install opencv-python
"""
import argparse
import socket
import subprocess
import sys
import time
import webbrowser


def port_open(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pi", required=True, help="IP de la Raspberry, ex: 10.105.1.XX")
    p.add_argument("--pi-user", default="pi")
    p.add_argument("--jetson", default="192.168.1.10", help="IP de la Jetson (réseau interne)")
    p.add_argument("--port", type=int, default=8080, help="port web_video_server et port local")
    p.add_argument("--topic", default="/image_raw")
    p.add_argument("--quality", type=int, default=50, help="qualité JPEG (1-100)")
    p.add_argument("--browser", action="store_true", help="ouvrir dans le navigateur au lieu d'OpenCV")
    args = p.parse_args()

    if port_open(args.port):
        sys.exit(f"Le port local {args.port} est déjà utilisé. Fermez l'ancien tunnel ou changez --port.")

    url = (f"http://localhost:{args.port}/stream?topic={args.topic}"
           f"&type=mjpeg&quality={args.quality}")

    # Tunnel SSH (le mot de passe de la Raspberry sera demandé dans ce terminal)
    cmd = ["ssh", "-N", "-o", "ServerAliveInterval=30",
           "-L", f"{args.port}:{args.jetson}:{args.port}",
           f"{args.pi_user}@{args.pi}"]
    print("Ouverture du tunnel SSH... (mot de passe Raspberry si demandé)")
    tunnel = subprocess.Popen(cmd)

    try:
        # Attendre que le tunnel soit prêt
        for _ in range(120):
            if tunnel.poll() is not None:
                sys.exit("Le tunnel SSH s'est arrêté. Vérifiez l'IP et le mot de passe.")
            if port_open(args.port):
                break
            time.sleep(0.5)
        else:
            sys.exit("Tunnel SSH non établi après 60 s.")

        print(f"Tunnel OK. Flux : {url}")

        if args.browser:
            webbrowser.open(url)
            print("Ctrl+C pour fermer le tunnel.")
            tunnel.wait()
            return

        try:
            import cv2
        except ImportError:
            print("OpenCV absent (pip install opencv-python) : ouverture dans le navigateur.")
            webbrowser.open(url)
            print("Ctrl+C pour fermer le tunnel.")
            tunnel.wait()
            return

        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            sys.exit("Impossible d'ouvrir le flux. web_video_server est-il lancé sur la Jetson ?")

        print("Appuyez sur 'q' dans la fenêtre pour quitter.")
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Flux interrompu.")
                break
            cv2.imshow("Camera Jetson", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        pass
    finally:
        tunnel.terminate()


if __name__ == "__main__":
    main()
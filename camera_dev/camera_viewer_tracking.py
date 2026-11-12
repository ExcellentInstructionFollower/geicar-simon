#!/usr/bin/env python3
"""
Affiche le flux caméra de la Jetson (via web_video_server) sur le PC.
Intègre le Tracking de Pose en temps réel via YOLO (Ultralytics).

Prérequis côté voiture (dans le conteneur ros-humble de la Jetson) :
activation caméra :
terminal 1: 
  ssh -J pi@10.105.1.168 jetson@192.168.1.10
  sudo docker start -ai ros-humble (à faire une seule fois pour la voiture)
  ros2 launch geicar_start_jetson geicar.jetson.launch.py
   ou
  ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=40 -p framerate:=30.0 -p pixel_format:=yuyv


terminal 2: 
serveur vidéo (autre terminal) :
  ssh -J pi@10.105.1.168 jetson@192.168.1.10
  sudo docker exec -it ros-humble bash
  source /opt/ros/humble/setup.bash
  ros2 run web_video_server web_video_server

sur la jetson : 
python3 ~/camera_dev/trt_pose_server.py

terminal 4 (docker):
source /opt/ros/humble/setup.bash
python3 /root/camera_dev/ros2_pose_bridge.py
 
Usage (avec tracking) :
  python3 camera_viewer.py --pi 10.105.1.168 --weights weights.pt

envoi code
# Envoyer un fichier unique
scp -J pi@10.105.1.168 /chemin/local/mon_script.py jetson@192.168.1.10:/home/camera_dev

# Envoyer un dossier complet
scp -J pi@10.105.1.168 -r /chemin/local/mon_projet jetson@192.168.1.10:/home/camera_dev

rsync -avz -e "ssh -J pi@10.105.1.168" ~/Memomove/geicar-simon/camera_dev jetson@192.168.1.10:~/


/usr/src/tensorrt/bin/trtexec \
  --onnx=~/camera_dev/yolov8n-pose.onnx \
  --saveEngine=/camera_dev/yolov8n-pose.engine \
  --fp16 \
  --workspace=1024

"""

import argparse
import os
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
    p.add_argument("--quality", type=int, default=None,
                   help="qualité JPEG (1-100). Défaut : 50 pour l'affichage, 95 si enregistrement")
    p.add_argument("--width", type=int, default=None, help="largeur du flux (redimensionné par le serveur)")
    p.add_argument("--height", type=int, default=None, help="hauteur du flux (redimensionné par le serveur)")
    p.add_argument("--fps", type=float, default=0, help="cadence max d'affichage/enregistrement (0 = pas de limite)")
    p.add_argument("--browser", action="store_true", help="ouvrir dans le navigateur au lieu d'OpenCV")
    p.add_argument("--record", metavar="FICHIER.mp4", help="enregistrer la vidéo dans ce fichier")
    p.add_argument("--frames-dir", metavar="DOSSIER", help="sauvegarder des images JPEG dans ce dossier")
    p.add_argument("--every", type=int, default=1, help="avec --frames-dir : une image sur N (défaut 1)")
    
    # Nouveaux arguments pour YOLO
    p.add_argument("--weights", default="weights.pt", help="Chemin vers le fichier de poids YOLO")
    p.add_argument("--no-track", action="store_true", help="Désactiver le tracking YOLO")
    args = p.parse_args()

    if (args.width is None) != (args.height is None):
        sys.exit("Indiquez --width et --height ensemble.")

    if port_open(args.port):
        sys.exit(f"Le port local {args.port} est déjà utilisé. Fermez l'ancien tunnel ou changez --port.")

    recording = bool(args.record or args.frames_dir)
    if args.quality is None:
        args.quality = 95 if recording else 50

    url = (f"http://localhost:{args.port}/stream?topic={args.topic}"
           f"&type=mjpeg&quality={args.quality}")
    if args.width is not None:
        url += f"&width={args.width}&height={args.height}"

    # Tunnel SSH
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
            sys.exit("OpenCV absent. Installez-le avec : pip install opencv-python")

        # Chargement du modèle YOLO
        model = None
        if not args.no_track:
            try:
                from ultralytics import YOLO
                print(f"Chargement du modèle YOLO avec les poids : {args.weights}...")
                model = YOLO(args.weights)
            except ImportError:
                sys.exit("Le module 'ultralytics' est absent. Installez-le avec : pip install ultralytics")
            except Exception as e:
                sys.exit(f"Erreur lors du chargement du modèle YOLO : {e}")

        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            sys.exit("Impossible d'ouvrir le flux. web_video_server est-il lancé sur la Jetson ?")

        writer = None
        if args.frames_dir:
            os.makedirs(args.frames_dir, exist_ok=True)
        n = 0
        saved = 0
        min_dt = 1.0 / args.fps if args.fps > 0 else 0.0
        last_t = 0.0

        print("Appuyez sur 'q' dans la fenêtre pour quitter.")
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Flux interrompu.")
                break

            # Limitation de cadence
            now = time.time()
            if now - last_t < min_dt:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue
            last_t = now

            # ---------------------------------------------------------
            # APPLICATION DU TRACKING YOLO (POSE)
            # ---------------------------------------------------------
            if model is not None:
                # persist=True permet d'activer le tracker pour garder le même ID d'une frame à l'autre
                results = model.track(frame, persist=True, verbose=False)
                
                # plot() dessine directement les squelettes (pose), les bounding boxes et les IDs
                frame = results[0].plot()
            # ---------------------------------------------------------

            if args.record:
                if writer is None:
                    h, w = frame.shape[:2]
                    rec_fps = args.fps if args.fps > 0 else 10.0
                    writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"),
                                             rec_fps, (w, h))
                writer.write(frame)

            if args.frames_dir and n % max(1, args.every) == 0:
                name = f"{time.strftime('%Y%m%d_%H%M%S')}_{n:06d}.jpg"
                cv2.imwrite(os.path.join(args.frames_dir, name), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved += 1
            n += 1

            cv2.imshow("Camera Jetson (YOLO Tracking)" + (" [REC]" if recording else ""), frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        if recording:
            print(f"{n} images traitées, {saved} images JPEG sauvegardées.")

    except KeyboardInterrupt:
        pass
    finally:
        tunnel.terminate()

if __name__ == "__main__":
    main()
    
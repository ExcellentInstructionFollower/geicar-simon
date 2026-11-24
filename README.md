# MemoMove — Pipeline Vision, Pose Estimation & Détection de Mouvements (GEICAR)

Ce module gère l'acquisition caméra, l'inférence temps réel **YOLOv8n-pose** accélérée par **TensorRT** sur NVIDIA Jetson Nano, le pont ROS 2 Humble et la reconnaissance de mouvements (*motion detection*) pour l'interaction avec le véhicule autonome.

# Pour lancer la détection : 

A faire une fois par PC: 

  ssh-keygen -t ed25519

  ssh-copy-id pi@10.105.1.168

  ssh-copy-id -o ProxyJump=pi@10.105.1.168 jetson@192.168.1.10

Pour lancer la détection :
  camera_dev/launch_detection.sh

Pour afficher un topic (exemple topic /detected_motion)
  ssh -J pi@10.105.1.168 jetson@192.168.1.10
  sudo docker exec -it ros-humble bash
  source /opt/ros/humble/setup.bash
  ros2 topic echo /detected_motion


---

## 1. Architecture du système

Le système est réparti entre l'hôte **Jetson Nano** (pour l'accès direct au GPU/TensorRT sans surcharge Docker) et le conteneur **`ros-humble`** (pour l'écosystème ROS 2) :

```text
[Caméra USB /dev/video0]
        │
        ▼ (Docker: usb_cam ou geicar.jetson.launch.py)
  /image_raw  ──────────────────────────────────┐
        │                                       │
        ▼ (Docker: ros2_pose_bridge.py)         ▼ (Docker: web_video_server)
   [Socket / IPC]                       Flux HTTP :8080 (Debug PC)
        │
        ▼ (Hôte Jetson: trt_pose_server_V2.py + yolov8n-pose_320.engine)
  Keypoints squelette (COCO 17 points)
        │
        ▼ (Docker: ros2_pose_bridge.py)
  Topics Pose / Image annotée (/image_pose, keypoints)
        │
        ▼ (Docker: motion_detector_v2.py)
  Événements de mouvement (/detected_motion, marche avant/arrière, rotations, gestes)



# GeiCar Project

The GeiCar project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous car in order to carry out different missions. Several projects are exposed on the [official website] (https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiCar. The present code as well as the documentation is the result of internship carried out by [Alexis24](https://github.com/Alexix24) (Alexis Pierre Dit Lambert)

The platform is developped and maintained by :

* DI MERCURIO Sébastien
* LOMBARD Emmanuel
* MARTIN José

The projects are (or were) surpervised by:

* CHANTHERY Elodie
* DELAUTIER Sébastien
* MONTEIL Thierry
* LE BOTLAN Didier
* AURIOL Guillaume
* DI MERCURIO Sébastien

## Quick User Guide
###Turn the car on and off
* To turn on the car:
  * Toggle the red button to bring the power.
  * Press the START push button (hold it down for a short while).
  * Wait until Raspberry boot up and connect to it using its IP address (written on the board): `ssh pi@10.105.1.xx`
  * When connected start ROS subsystem using :`ros2 launch geicar_start geicar.launch.py`
  * Then, you will get a report of subsystems and be able to control the car using XBOX controler 

* To turn off the car:
	* Use the red button as a switch to turn off the power.

###Use of this repository
* First of all, [fork this repository](https://docs.github.com/en/get-started/quickstart/fork-a-repo) to get yours.
* Then, clone your fresh and new repository on your computer: `git clone https://github.com/<your id>/<your wonderful repo>`
* Have a look in "general" directory for how to connect and work with your car
* For more 'in depth' documentation on a particular subsystem, have a look in following directories:
    * raspberryPI3: Everything about raspberry setup
    * nucleoL476: Stuff about GPS-RTK and IMU
    * nucleoF103: informations on motors control, main power and ultrasonics sensors
    * jetsonNano: Directory containing info on IA, Camera and Lidar
    * simulation: if you want to setup a carla simulation environment

__warning__
You normally do not need to change firmware running in F103 and F476 boards. You main work is on the raspberry and jetson side.




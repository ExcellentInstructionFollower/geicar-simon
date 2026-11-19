#!/bin/bash

PI_HOST="pi@10.105.1.168"
JETSON_HOST="jetson@192.168.1.10"
SESSION="memomove"

# Choix du mode caméra via argument (défaut: usb_cam, option: geicar)
MODE=${1:-"usb_cam"}

echo "🚀 Connexion à la Jetson et démarrage de la session tmux '$SESSION' (mode: $MODE)..."

ssh -t -J "$PI_HOST" "$JETSON_HOST" "
# 1. Nettoyer une éventuelle session précédente et démarrer le conteneur Docker
tmux kill-session -t $SESSION 2>/dev/null
sudo docker start ros-humble
sleep 2

# 2. Fenêtre 1 (Docker) : Caméra ou stack complète GEICAR
tmux new-session -d -s $SESSION -n 'cam_geicar'
if [ '$MODE' = 'geicar' ]; then
    tmux send-keys -t $SESSION:cam_geicar 'sudo docker exec -it ros-humble bash -c \"source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash 2>/dev/null; ros2 launch geicar_start_jetson geicar.jetson.launch.py\"' C-m
else
    tmux send-keys -t $SESSION:cam_geicar 'sudo docker exec -it ros-humble bash -c \"source /opt/ros/humble/setup.bash && ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=320 -p image_height:=240 -p framerate:=10.0 -p pixel_format:=yuyv\"' C-m
fi

# 3. Fenêtre 2 (Docker) : Web Video Server (optionnel)
tmux new-window -t $SESSION -n 'web_video'
tmux send-keys -t $SESSION:web_video 'sudo docker exec -it ros-humble bash -c \"source /opt/ros/humble/setup.bash && ros2 run web_video_server web_video_server\"' C-m

# 4. Fenêtre 3 (Host Jetson) : Serveur d'inférence TensorRT YOLO-Pose
tmux new-window -t $SESSION -n 'trt_pose'
tmux send-keys -t $SESSION:trt_pose 'python3 /home/jetson/camera_dev/trt_pose_server_V2.py --weights /home/jetson/camera_dev/yolov8n-pose_320.engine' C-m

# Pause pour laisser le serveur TensorRT charger l'engine FP16 avant le bridge ROS 2
sleep 3

# 5. Fenêtre 4 (Docker) : Bridge ROS 2 Pose
tmux new-window -t $SESSION -n 'pose_bridge'
tmux send-keys -t $SESSION:pose_bridge 'sudo docker exec -it ros-humble bash -c \"source /opt/ros/humble/setup.bash && python3 /root/camera_dev/ros2_pose_bridge.py\"' C-m

# 6. Fenêtre 5 (Docker) : Détecteur de mouvements
tmux new-window -t $SESSION -n 'motion_det'
tmux send-keys -t $SESSION:motion_det 'sudo docker exec -it ros-humble bash -c \"source /opt/ros/humble/setup.bash && python3 /root/camera_dev/motion_detector_v2.py\"' C-m

# Attacher le terminal sur la fenêtre du détecteur de mouvement
tmux select-window -t $SESSION:motion_det
tmux attach-session -t $SESSION
"
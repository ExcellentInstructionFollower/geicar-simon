import cv2
from ultralytics import YOLO

# 1. Chargement des poids (remplace 'best.pt' par le chemin de tes poids)
# Astuce Jetson : Pour des FPS maximaux, utilise un export TensorRT ('best.engine')
model_path = "weights.pt"
model = YOLO(model_path)

# 2. Source vidéo
# - 0 pour une webcam USB standard (V4L2)
# - Un chemin de fichier vidéo (ex: "test.mp4")
# - Ou un pipeline GStreamer pour une caméra CSI (voir snippet ci-dessous)
cap = cv2.VideoCapture(0)

# Pipeline GStreamer (à décommenter si tu utilises une caméra CSI type IMX219 / PiCam sur Jetson) :
"""
gstreamer_pipeline = (
    "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
    "nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
)
cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
"""

if not cap.isOpened():
    print("Erreur : Impossible d'accéder au flux vidéo.")
    exit()

print("Démarrage de l'inférence YOLO Pose sur la Jetson...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. Inférence GPU (device=0 force l'utilisation du GPU CUDA de la Jetson)
    results = model.predict(source=frame, device=0, verbose=False, conf=0.5)

    # 4. Annotation visuelle automatique des personnes et squelettes
    annotated_frame = results[0].plot()

    # 5. Extraction manuelle des keypoints (pour logique métier / traitement de mouvement)
    if results[0].keypoints is not None:
        # Array NumPy de forme (N_personnes, 17_joints, 2) avec coordonnées (x, y)
        keypoints = results[0].keypoints.xy.cpu().numpy()
        confidences = results[0].keypoints.conf.cpu().numpy()

        for idx, person_kpts in enumerate(keypoints):
            # Exemple : extraction du nez (joint 0) ou des poignets (joints 9 et 10)
            # nose_x, nose_y = person_kpts[0]
            pass

    # 6. Affichage
    cv2.imshow("Jetson YOLO Pose Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
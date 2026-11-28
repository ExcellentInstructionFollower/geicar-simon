# Message publie par motion_detector_node sur /vision/detected_motion
# Decrit la presence, la posture et les mouvements detectes d'un joueur.

# Constantes pour game_move (alignees sur AutonomousCommand / autonomous_driving)
int8 MOVE_NONE=0
int8 MOVE_FORWARD=1
int8 MOVE_BACKWARD=2
int8 MOVE_LEFT=3
int8 MOVE_RIGHT=4

# Horodatage et qualite de detection
std_msgs/Header header
bool person_detected false      # true si une personne valide est detectee
float32 confidence 0.0          # Score de confiance YOLO [0.0 ; 1.0]
float32 body_scale_px 0.0       # Echelle apparente du corps en pixels

# Interpretation haut niveau pour le jeu MemoMove (game_ai)
int8 game_move 0                # MOVE_NONE=0, MOVE_FORWARD=1, MOVE_BACKWARD=2, MOVE_LEFT=3, MOVE_RIGHT=4
bool start_gesture false        # true lorsque le geste de demarrage/special est actif (mouvement 67)

# Etats detailles de la detection
string orientation              # "DE_FACE" | "DE_DOS" | ""
string posture                  # "NEUTRE" | "BRAS_GAUCHE_LEVE" | "BRAS_DROIT_LEVE" | "DEUX_BRAS_LEVES" | "BRAS_EN_CROIX" | ""
string walk                     # "IMMOBILE" | "MARCHE_AVANT" | "MARCHE_ARRIERE" | ""
string rotation                 # "AUCUNE" | "TOURNE_GAUCHE" | "TOURNE_DROITE" | ""
string dynamic_event            # "MOUVEMENT_67" | "TOURNE_GAUCHE" | "TOURNE_DROITE" | "SALUT_MAIN_DROITE" | "PAS_GAUCHE" | "PAS_DROIT" | ""

# Suivi du geste "67"
bool is_67_active false
uint32 count_67 0

# Resultat de reconnaissance du visage suivi (publie par episode / throttle).
# Equivalent ROS2 de la dataclass RecognitionResult (cf. msgs.py). Pas de type
# std_msgs/vision_msgs adequat -> message custom.

std_msgs/Header header    # stamp = instant de la reconnaissance

string  status            # known / unknown / idle
int32   id_pred           # id personne, -1 si inconnu
string  name              # nom (ou "unknown")
float32 score             # cosinus LISSE (EMA), sert au badge
float32 raw_score         # cosinus instantane non lisse
float32 stability         # taux de frames "known" sur l'episode (0..1)
string  id_lot            # lot de l'episode (acquisition)
uint32  lock_id           # identifiant d'episode
string  mode              # off / recognition / acquisition
# NB : la vignette (thumb 112x112) n'est PAS transportee ici -> topic image dedie
# si un HUD distant en a besoin.

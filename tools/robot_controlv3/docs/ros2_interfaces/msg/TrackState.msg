# Etat du suivi : partie specifique NON couverte par vision_msgs/Detection2DArray.
# Publie conjointement a la Detection2DArray sur /tracking/result.
# Remplace le champ `tstate` (dict) de TrackingResult (cf. msgs.py).

string  mode          # none / mil / vit / auto
bool    locked        # verrou stable en cours
string  src           # detector / tracker / predicted
float32 score         # score de suivi courant

# Prediction de trajectoire (modes anticip / coast) -- cf. touche P.
string  pred_phase    # off / anticip / coast
float32 pred_nx       # cible anticipee, normalisee [-1..+1]
float32 pred_ny

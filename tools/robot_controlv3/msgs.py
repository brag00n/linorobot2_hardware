r"""msgs - types de messages echanges entre nodes (futurs .msg ROS2).

Chaque dataclasse represente la donnee unitaire d'un topic (file profondeur 1).
Le nommage des champs est volontairement proche de ce que produiraient de vrais
messages ROS2, pour que la migration soit mecanique.

Graphe des topics (voir RobotMain) :
  /camera/image      ImageMsg        Camera  -> Tracking + Core (affichage)
  /tracking/config   TrackingConfig  Core    -> Tracking      (touches F/M/T/P, MCP)
  /tracking/result   TrackingResult  Tracking-> Core          (detection + etat suivi)
  /tracking/metrics  TrackingMetrics Tracking-> Core          (perf/HUD)
  /servo/cmd         ServoCmd        Tracking + Core -> Servo  (cible ou pilotage manuel)
  /servo/state       ServoState      Servo   -> Core          (angles + fluidite)
  /board/telemetry   BoardTelemetry  Board   -> Core          (batterie, yaw, liaison)
  /grovepi/telemetry GrovePiTelemetry GrovePi -> Core          (ultrasons + IMU, liaison)
"""
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple, List


@dataclass
class ImageMsg:
    """Image capturee (deja flippee/tournee par le capteur). frame = ndarray BGR."""
    seq: int
    stamp: float
    frame: Any


@dataclass
class TrackingConfig:
    """Configuration du suivi (issue des touches F/M/T/P et du MCP control.json)."""
    active: bool = False                 # suivi arme (touche F)
    detector: Optional[str] = None       # haar / dnn / yunet
    track_mode: Optional[str] = None     # none / mil / vit / auto
    predict_mode: Optional[str] = None   # off / anticip / coast


@dataclass
class TrackingResult:
    """Resultat d'une detection (pleine res) + etat de suivi, publie par NOUVELLE detection."""
    seq: int
    faces: List[Tuple[int, int, int, int]]
    main: Optional[Tuple[int, int, int, int]]
    nx: Optional[float]
    ny: Optional[float]
    area_pct: float
    det_fps: float
    tstate: dict = field(default_factory=dict)   # trackState() : mode/locked/src/score/pred_*


@dataclass
class RecognitionConfig:
    """Configuration/commande du noeud de reconnaissance (touches r/l, MCP).

    mode : off / recognition / acquisition (etat continu du noeud).
    command : commande ponctuelle a executer une fois (train / acquire_file /
    recognize_file), None sinon. `seq` rend la commande discrete (le noeud
    n'execute qu'a chaque seq neuf, file profondeur 1). path/id_lot : arguments
    des commandes fichier manuelles.
    """
    mode: Optional[str] = None
    command: Optional[str] = None
    seq: int = 0
    path: Optional[str] = None
    id_lot: Optional[str] = None


@dataclass
class RecognitionResult:
    """Resultat de reconnaissance du visage suivi (publie par episode/throttle).

    status : known (id_pred+name) / unknown (aucune personne >= seuil) / idle
    (rien a reconnaitre : pas de verrou stable ou mode off). score = cosinus LISSE
    (EMA, sert au badge) ; raw_score = cosinus instantane non lisse ; stability =
    taux de frames « known » sur l'episode courant (0..1, indice anti-flicker).
    id_lot = lot de l'episode courant (acquisition). lock_id = episode.
    """
    seq: int
    status: str = "idle"
    id_pred: Optional[int] = None
    name: str = "unknown"
    score: float = 0.0
    raw_score: float = 0.0
    stability: float = 0.0
    id_lot: Optional[str] = None
    lock_id: int = 0
    mode: str = "off"


@dataclass
class TrainState:
    """Etat du node dedie a l'apprentissage (FaceTrainNode) -> Core (HUD) + FaceRecogNode.

    running : un batch d'enrolement tourne dans le worker (UN SEUL a la fois).
    lines   : log FENETRE des dernieres etapes (ring buffer) pour le panneau HUD.
    summary : synthese du dernier batch termine (dict) ou None si jamais lance / en cours.
    done_seq: incremente a CHAQUE fin de batch -> signal de rechargement de galerie
    pour le FaceRecogNode (qui ecoute ce topic). `seq` = version d'etat (publie au
    changement) pour eviter le spam sur le bus profondeur 1.
    """
    seq: int = 0
    running: bool = False
    lines: List[str] = field(default_factory=list)
    summary: Optional[dict] = None
    done_seq: int = 0


@dataclass
class TrackingMetrics:
    """Metriques de perf/etat pour le HUD (publiees a chaque tour)."""
    det_fps: float = 0.0
    detector: str = ""
    track_mode: str = "none"
    predict_mode: str = "off"
    active: bool = False
    n_faces: int = 0


@dataclass
class ServoCmd:
    """Commande servo. `kind` distingue asservissement suivi et pilotage manuel.

    kind="track"      : viser (nx, ny) (cible mesuree/anticipee/coast du suivi) ;
    kind="home"       : retour doux au repos (coast expire) ;
    kind="center"     : recentrage immediat (touche C) ;
    kind="nudge_pan"  : pas relatif pan  (fleches G/D)   -> delta en deg ;
    kind="nudge_tilt" : pas relatif tilt (fleches H/B)   -> delta en deg.
    `seq` rend la commande discrete : le ServoNode n'applique qu'a chaque seq neuf
    (evite de re-appliquer un nudge relatif reste sur le bus profondeur 1).
    """
    kind: str
    seq: int = 0
    nx: Optional[float] = None
    ny: Optional[float] = None
    delta: float = 0.0
    phase: str = "off"           # phase de prediction au moment de l'ordre (info)


@dataclass
class ServoState:
    """Etat des servos pan/tilt : angles reels, zone morte, fluidite (motionStats)."""
    angleH: float = 0.0
    angleV: float = 0.0
    deadzone: float = 0.14
    motion: dict = field(default_factory=dict)   # motionStats() : step_max, vel_*, ...


@dataclass
class BoardTelemetry:
    """Snapshot carte STM32 (liaison serie) : batterie, IMU (roll/pitch/yaw), vitesses, liaison."""
    battery: Optional[float] = None
    yaw: Optional[float] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    encoders: Optional[list] = None
    ok: int = 0
    bad: int = 0


@dataclass
class GrovePiTelemetry:
    """Snapshot carte capteurs GrovePi+ : ultrasons + IMU + liaison.

    `connected` = port serie ouvert (la carte peut etre absente : l'app tourne
    quand meme). Les champs `*_age` (s depuis la derniere trame de la famille)
    permettent a l'HMI de distinguer « connecte mais silencieux » de « frais ».
    ultra : 4 distances mm (None = pas d'echo / canal muet).
    """
    connected: bool = False
    version: Optional[str] = None
    # IMU (fusion bord : roll/pitch en deg ; accel/gyro bruts int16)
    roll: Optional[float] = None
    pitch: Optional[float] = None
    accel: Optional[Tuple[int, int, int]] = None
    gyro: Optional[Tuple[int, int, int]] = None
    imu_age: Optional[float] = None
    # Ultrasons HC-SR04 (mm, None = pas d'echo)
    ultra: List[Optional[int]] = field(default_factory=lambda: [None, None, None, None])
    ultra_age: Optional[float] = None
    # Telemetre IR Sharp
    ir_dist: Optional[int] = None
    ir_adc: Optional[int] = None
    ir_age: Optional[float] = None
    ok: int = 0
    bad: int = 0

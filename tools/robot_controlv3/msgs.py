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
    """Snapshot carte STM32 (liaison serie) : batterie, cap, vitesses, liaison."""
    battery: Optional[float] = None
    yaw: Optional[float] = None
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    encoders: Optional[list] = None
    ok: int = 0
    bad: int = 0

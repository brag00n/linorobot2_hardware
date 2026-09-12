r"""CameraNode - node capteur : publie le flux camera sur /camera/image.

Enveloppe la couche device.sensor.RobotSensorWebCam de robot_control (reutilisee
PAR IMPORT, zero algorithme reecrit : ouverture/scan auto, flip, rotate). Le node
n'ajoute que l'interface ROS (un Port de sortie + une sequence d'image).

Sous ROS2 : deviendrait un rclpy.Node avec un publisher sensor_msgs/Image et un
timer a la cadence camera ; le corps de process() serait quasi identique.
"""
import time

from robot_control.device.sensor.RobotSensorWebCam import RobotSensorWebCam

from ..roslite import Node
from ..msgs import ImageMsg


class CameraNode(Node):
    """Capture une image par tour et la publie sur /camera/image (ImageMsg)."""

    def __init__(self, args, telemetry=None):
        super().__init__("camera")
        self._args = args
        self._tel = telemetry
        self.width, self.height = _parse_size(args.size)
        self.cam = RobotSensorWebCam(
            index=args.index, backend=args.backend, width=self.width, height=self.height,
            fps=args.fps, flip=args.flip, rotate=args.rotate, preferName=args.camera_name,
            skipNames=args.skip_name.split(","), telemetry=telemetry)
        self._seq = 0
        self.ok = False                    # succes de la DERNIERE lecture (garde Core)
        self.read_fail = 0                 # lectures consecutives echouees (garde Core)
        self.available = False             # camera ouverte (affichee « presente » par l'HMI)
        self._retry_t = 0.0                # date du dernier essai de (re)ouverture
        self.source = "ext"                # camera courante : ext (USB pan-tilt) / int (webcam PC)
        # cibles de nommage par source (scan auto guide par nom) : externe = camera USB,
        # interne = webcam integree du PC (on ecarte la camera IR).
        self._prefer = {"ext": args.camera_name, "int": "Integrated"}
        self._skip = {"ext": args.skip_name.split(","), "int": ["IR Camera"]}
        self._out = self.create_output("/camera/image", ImageMsg)

    def on_start(self):
        """Tente d'ouvrir la camera ; ABSENCE NON FATALE (l'app doit demarrer sans).

        Contrairement a l'ancien comportement (RuntimeError -> arret), une camera
        absente laisse le node en mode « indisponible » : process() reessaie
        periodiquement de l'ouvrir (auto-reconnexion, « affiche si dispo »).
        """
        self._try_open()

    def _try_open(self):
        """Essaie d'ouvrir la camera. Retourne True si ouverte. Journalise l'etat."""
        self._retry_t = time.time()
        try:
            opened = self.cam.open()
        except Exception:
            opened = False
        self.available = bool(opened)
        if opened:
            print(f"Camera index {self.cam.index} : {self.cam.width}x{self.cam.height} "
                  f"(backend {self.cam.backend}, flip {self._args.flip}, "
                  f"rotate {self._args.rotate})")
        else:
            print("[camera] aucune camera exploitable (scan index 0..5) -> "
                  "demarrage sans camera, nouvel essai en tache de fond.")
        return opened

    def process(self):
        """Lit une image (flip/rotate dans read()) et la publie ; reessaie si absente."""
        if not self.available:
            if time.time() - self._retry_t >= 2.0:   # reconnexion camera ~0,5 Hz
                self._try_open()
            self.ok = False
            return
        ok, frame = self.cam.read()
        if not ok or frame is None:
            self.ok = False
            self.read_fail += 1
            if self.read_fail > 30:            # camera perdue : reprend le scan auto
                self.available = False
                try:
                    self.cam.release()
                except Exception:
                    pass
            return                         # pas de publication : le bus garde la precedente
        self.ok = True
        self.read_fail = 0
        self._seq += 1
        self._out.set(ImageMsg(seq=self._seq, stamp=time.time(), frame=frame))

    def switch_camera(self):
        """Bascule interne <-> externe : libere la camera courante et rouvre l'autre
        source (scan auto par nom). Non fatal : si la nouvelle source echoue, le node
        repasse « indisponible » et reessaie en tache de fond (comme a l'ouverture)."""
        self.source = "int" if self.source == "ext" else "ext"
        try:
            self.cam.release()
        except Exception:
            pass
        self.available = False
        self.cam.index = "auto"                        # force le scan par nom
        self.cam.preferName = self._prefer[self.source]
        self.cam.skipNames = self._skip[self.source]
        print(f"[camera] bascule source -> {self.source} "
              f"(prefere '{self.cam.preferName}')")
        self._try_open()

    def on_stop(self):
        self.cam.release()


def _parse_size(size):
    """'1280x720' -> (1280, 720). Sortie explicite si invalide (comme l'ancien main)."""
    try:
        w, h = (int(v) for v in size.lower().split("x"))
        return w, h
    except ValueError:
        import sys
        sys.exit(f"--size invalide : {size!r}")

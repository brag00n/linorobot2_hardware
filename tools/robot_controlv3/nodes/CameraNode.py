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
        self._out = self.create_output("/camera/image", ImageMsg)

    def on_start(self):
        """Ouvre la camera (scan auto guide par les noms) ; sortie fatale si aucune."""
        if not self.cam.open():
            raise RuntimeError(
                "Aucune camera exploitable trouvee (scan index 0..5, backends "
                "msmf/dshow/any). Verifier le branchement.")
        print(f"Camera index {self.cam.index} : {self.cam.width}x{self.cam.height} "
              f"(backend {self.cam.backend}, flip {self._args.flip}, rotate {self._args.rotate})")

    def process(self):
        """Lit une image (flip/rotate appliques dans read()) et la publie si ok."""
        ok, frame = self.cam.read()
        if not ok or frame is None:
            self.ok = False
            self.read_fail += 1
            return                         # pas de publication : le bus garde la precedente
        self.ok = True
        self.read_fail = 0
        self._seq += 1
        self._out.set(ImageMsg(seq=self._seq, stamp=time.time(), frame=frame))

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

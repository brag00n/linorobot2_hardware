r"""GrovePiNode - node carte capteurs GrovePi+ : publie /grovepi/telemetry.

Enveloppe la couche communication.GroveComSerial de robot_control (reutilisee PAR
IMPORT : protocole de trames GrovePi+ v0.3, thread lecteur + reconnexion auto). Le
node expose le snapshot capteurs (ultrasons HC-SR04, IMU MPU6050, IR Sharp, liaison)
sur un topic, exactement comme BoardNode le fait pour la STM32.

La carte peut etre ABSENTE au demarrage : GroveComSerial ouvre le port en tache de
fond et le rouvre tant qu'il manque. Le node publie donc toujours un message ; son
champ `connected` dit si la carte est la (l'HMI l'affiche des qu'elle apparait).

Sous ROS2 : publisher d'un message capteurs a cadence fixe (comme BoardNode).
"""
from robot_control.communication.GroveComSerial import GroveComSerial

from ..roslite import Node
from ..msgs import GrovePiTelemetry


class GrovePiNode(Node):
    """Publie l'etat de la carte capteurs GrovePi+ sur /grovepi/telemetry."""

    def __init__(self, port="COM6", baud=115200, telemetry=None):
        super().__init__("grovepi")
        self.link = GroveComSerial(port, baud, telemetry=telemetry)
        self._out = self.create_output("/grovepi/telemetry", GrovePiTelemetry)

    def process(self):
        """Publie une copie coherente du dernier snapshot capteurs."""
        s = self.link.snapshot()
        self._out.set(GrovePiTelemetry(
            connected=s["connected"], version=s["version"],
            roll=s["roll"], pitch=s["pitch"], accel=s["accel"], gyro=s["gyro"],
            imu_age=s["imu_age"], ultra=s["ultra"], ultra_age=s["ultra_age"],
            ir_dist=s["ir_dist"], ir_adc=s["ir_adc"], ir_age=s["ir_age"],
            ok=s["ok"], bad=s["bad"]))

    def on_stop(self):
        self.link.close()

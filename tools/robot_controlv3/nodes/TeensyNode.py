r"""TeensyNode - node 3e carte de controle (Teensy) : publie /teensy/telemetry.

Enveloppe la couche communication.TeensyComSerial de robot_control (sous-classe de
RobotComSerial : MEME protocole de trames binaires que la STM32/ESP32, funcodes
0x0A/0x0C/0x0D identiques ; seuls changent le VID:PID PJRC 16C0:0483 et le cpr=48).
La carte peut etre ABSENTE au demarrage : TeensyComSerial ouvre le port en tache de
fond et le rouvre tant qu'il manque.

Calque de WSEsp32Node : par defaut le node POSSEDE son lien (le construit dans
__init__, le ferme dans on_stop()). Cas « Teensy carte principale » : le Core lui
INJECTE un lien deja construit (`link=`), partage avec les moteurs/gateway comme
BoardNode ; le node ne le ferme alors PAS (proprietaire = Core). Le calcul rpm
(tics/s -> tr/min via cpr) reprend BoardNode -- ici 4 encodeurs REELS.

/!\ Pas de magneto (MPU6050) -> TeensyTelemetry n'a ni mag ni heading (contrairement
    a Esp32Telemetry) ; le yaw est relatif/derivant (integration gyro-z firmware).

Sous ROS2 : publisher d'un message de telemetrie a cadence fixe.
"""
from robot_control.communication.TeensyComSerial import TeensyComSerial

from ..roslite import Node
from ..msgs import TeensyTelemetry


class TeensyNode(Node):
    """Publie l'etat de la 3e carte de controle Teensy sur /teensy/telemetry."""

    def __init__(self, port="COM9", baud=115200, telemetry=None, cpr=None, link=None,
                 rx_prefix=""):
        super().__init__("teensy")
        if link is not None:
            # lien injecte par le Core (Teensy = carte de controle principale) :
            # partage avec les moteurs/gateway ; le node n'en est PAS proprietaire.
            self.link = link
            self._owns_link = False
        else:
            # carte secondaire : rx_prefix="teensy_" pour ne pas ecraser les cles
            # rx:speed/imu/encoder de la carte primaire dans le state.json/log.
            if cpr is None:
                self.link = TeensyComSerial(port, baud, telemetry=telemetry,
                                            rx_prefix=rx_prefix)
            else:
                self.link = TeensyComSerial(port, baud, telemetry=telemetry, cpr=cpr,
                                            rx_prefix=rx_prefix)
            self._owns_link = True
        self._out = self.create_output("/teensy/telemetry", TeensyTelemetry)

    def process(self):
        """Publie une copie coherente des dernieres metriques carte Teensy."""
        snap = self.link.snapshot()
        # rpm par moteur = tics/s (fenetre glissante) / cpr * 60 (cf. BoardNode).
        sp = self.link.encSpeed()
        cpr = self.link.cpr or 48.0
        rpm = None if sp is None else [round(s / cpr * 60.0, 1) for s in sp]
        self._out.set(TeensyTelemetry(
            connected=self.link.connected,
            battery=snap.get("battery"), yaw=snap.get("yaw"),
            roll=snap.get("roll"), pitch=snap.get("pitch"),
            vx=snap.get("vx", 0.0), vy=snap.get("vy", 0.0), vz=snap.get("vz", 0.0),
            rpm=rpm,
            ts_speed=snap.get("ts_speed"), ts_imu=snap.get("ts_imu"),
            ts_enc=snap.get("ts_enc"),
            speed_age=snap.get("speed_age"), imu_age=snap.get("imu_age"),
            enc_age=snap.get("enc_age"),
            ok=snap.get("ok", 0), bad=snap.get("bad", 0)))

    def on_stop(self):
        # ne ferme que le lien que le node possede (lien injecte = ferme par le Core).
        if self._owns_link:
            self.link.close()

r"""WSEsp32Node - node carte de controle ESP32 WaveShare : publie /wsesp32/telemetry.

Enveloppe la couche communication.Esp32ComSerial de robot_control (sous-classe de
RobotComSerial : MEME protocole de trames binaires que la STM32, funcodes
0x0A/0x0C/0x0D identiques ; seuls changent le VID:PID CP210x, le baud 921600 et la
verification du port par sniff). La carte peut etre ABSENTE au demarrage :
Esp32ComSerial ouvre le port en tache de fond et le rouvre tant qu'il manque.

Calque de GrovePiNode : par defaut le node POSSEDE son lien (le construit dans
__init__, le ferme dans on_stop()). Cas « ESP32 carte principale » : le Core lui
INJECTE un lien deja construit (`link=`), partage avec les moteurs/gateway comme
BoardNode ; le node ne le ferme alors PAS (proprietaire = Core). Le calcul rpm
(tics/s -> tr/min via cpr) reprend BoardNode.

Sous ROS2 : publisher d'un message de telemetrie a cadence fixe.
"""
from robot_control.communication.Esp32ComSerial import Esp32ComSerial

from ..roslite import Node
from ..msgs import Esp32Telemetry


class WSEsp32Node(Node):
    """Publie l'etat de la carte ESP32 WaveShare sur /wsesp32/telemetry."""

    def __init__(self, port="COM8", baud=921600, telemetry=None, cpr=None, link=None,
                 rx_prefix="", protocol="yahboom", on_port_resolved=None):
        super().__init__("wsesp32")
        if link is not None:
            # lien injecte par le Core (ESP32 = carte de controle principale) :
            # partage avec les moteurs/gateway ; le node n'en est PAS proprietaire.
            self.link = link
            self._owns_link = False
        else:
            # carte secondaire : rx_prefix="esp32_" pour ne pas ecraser les cles
            # rx:speed/imu/encoder/mag de la carte primaire dans le state.json/log.
            if cpr is None:
                self.link = Esp32ComSerial(port, baud, telemetry=telemetry,
                                           rx_prefix=rx_prefix, protocol=protocol,
                                           on_port_resolved=on_port_resolved)
            else:
                self.link = Esp32ComSerial(port, baud, telemetry=telemetry, cpr=cpr,
                                           rx_prefix=rx_prefix, protocol=protocol,
                                           on_port_resolved=on_port_resolved)
            self._owns_link = True
        self._out = self.create_output("/wsesp32/telemetry", Esp32Telemetry)

    def process(self):
        """Publie une copie coherente des dernieres metriques carte ESP32."""
        snap = self.link.snapshot()
        # rpm par moteur = tics/s (fenetre glissante) / cpr * 60 (cf. BoardNode).
        sp = self.link.encSpeed()
        cpr = self.link.cpr or 2114.0
        rpm = None if sp is None else [round(s / cpr * 60.0, 1) for s in sp]
        self._out.set(Esp32Telemetry(
            connected=self.link.connected,
            battery=snap.get("battery"), yaw=snap.get("yaw"),
            roll=snap.get("roll"), pitch=snap.get("pitch"),
            vx=snap.get("vx", 0.0), vy=snap.get("vy", 0.0), vz=snap.get("vz", 0.0),
            rpm=rpm,
            mag=snap.get("mag"), heading=snap.get("heading"),
            ts_speed=snap.get("ts_speed"), ts_imu=snap.get("ts_imu"),
            ts_enc=snap.get("ts_enc"), ts_mag=snap.get("ts_mag"),
            speed_age=snap.get("speed_age"), imu_age=snap.get("imu_age"),
            enc_age=snap.get("enc_age"), mag_age=snap.get("mag_age"),
            ok=snap.get("ok", 0), bad=snap.get("bad", 0)))

    def on_stop(self):
        # ne ferme que le lien que le node possede (lien injecte = ferme par le Core).
        if self._owns_link:
            self.link.close()

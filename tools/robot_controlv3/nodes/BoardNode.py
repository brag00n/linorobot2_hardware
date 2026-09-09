r"""BoardNode - node liaison serie STM32 : publie /board/telemetry.

Enveloppe la couche communication.RobotComSerial de robot_control (reutilisee PAR
IMPORT : protocole Yahboom v4, thread lecteur interne). Le node expose le snapshot
carte (batterie, cap, vitesses, liaison) sur un topic.

Le meme objet `link` (RobotComSerial) sert aussi de porte materielle a ServoNode
(sendServo) et au drive clavier du Core (sendMotor/stop) : il est construit par le
Core et injecte ici et dans ServoNode (comme un handle de peripherique partage).

Sous ROS2 : publisher d'un message de telemetrie a cadence fixe.
"""
from ..roslite import Node
from ..msgs import BoardTelemetry


class BoardNode(Node):
    """Publie l'etat de la carte STM32 sur /board/telemetry a chaque tour."""

    def __init__(self, link):
        super().__init__("board")
        self.link = link                   # RobotComSerial (construit et ouvert par le Core)
        self._out = self.create_output("/board/telemetry", BoardTelemetry)

    def process(self):
        """Publie une copie coherente des dernieres metriques carte."""
        snap = self.link.snapshot()
        self._out.set(BoardTelemetry(
            battery=snap.get("battery"), yaw=snap.get("yaw"),
            vx=snap.get("vx", 0.0), vy=snap.get("vy", 0.0), vz=snap.get("vz", 0.0),
            encoders=snap.get("encoders"), ok=snap.get("ok", 0), bad=snap.get("bad", 0)))

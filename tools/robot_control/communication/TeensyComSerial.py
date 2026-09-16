r"""TeensyComSerial - Liaison serie avec la 3e carte de controle : un Teensy.

Carte Teensy (PJRC, USB CDC natif) du robot bamboo4WD_V4_Teensy, faisant tourner
firmware/teensy_bamboo compile avec -D ENABLE_CONNECTOR_SERIAL_FRAME (branche
SerialFrame, EXCLUSIVE de micro-ROS). Cette branche emet/recoit EXACTEMENT le meme
protocole de trames binaires que la STM32 et l'ESP32 (memes funcodes 0x0A/0x0C/0x0D
/0x12/0x13/0x14/0x50/0x51) -> on reutilise integralement RobotComSerial : decodeurs,
FrameParser, snapshot(), sendCmdVel().

Cette sous-classe n'ajoute que ce qui differe cote Teensy :
  (a) VID:PID PJRC (16C0:0483) pour prioriser le bon port au scan auto ;
  (b) ses propres defauts (cpr=48 = COUNTS_PER_REV firmware, baud nominal).

/!\ Contrairement a l'ESP32 WaveShare (Esp32ComSerial), le Teensy est un USB CDC
    natif : (1) il n'expose qu'UN port a ce VID:PID -> pas besoin de sniff pour
    departager comme l'ESP32 (deux CP2102) ; (2) son CDC ne fait PAS d'auto-reset
    sur DTR/RTS -> AUCUNE suppression DTR/RTS necessaire. On ne surcharge donc NI
    _probe NI _open : le comportement de base (confiance au port prefere, repli
    scan auto priorise VID:PID) convient tel quel. Le baud est ignore par le
    transfert USB CDC ; on garde 115200 pour coherence.

/!\ cpr=48 (COUNTS_PER_REV firmware), WHEEL_DIAMETER/LR_WHEELS_DISTANCE sont des
    PLACEHOLDERS tant que la geometrie roue Teensy n'est pas mesuree : les
    rpm/vitesses seront faux jusqu'a calibration (cf. plan, sous-tache separee).
"""
try:
    from .RobotComSerial import RobotComSerial     # importe en tant que package
except ImportError:                                # importe a plat (dossier sur sys.path)
    from RobotComSerial import RobotComSerial

# Defauts specifiques a la carte Teensy
TEENSY_DEFAULT_BAUD = 115200          # ignore par l'USB CDC natif, garde pour coherence
TEENSY_DEFAULT_CPR = 48.0             # COUNTS_PER_REV firmware (placeholder, cf. firmware)
TEENSY_VID_PID = ("16C0:0483",)       # PJRC (Teensy USB Serial)


class TeensyComSerial(RobotComSerial):
    """Liaison serie avec la carte Teensy (protocole trames binaires).

    Identique a RobotComSerial hormis les defauts (baud/cpr/VID:PID). Aucune
    surcharge de _probe/_open : le Teensy CDC n'a ni double-port ni auto-reset
    DTR/RTS, donc le comportement d'ouverture de base convient.
    """

    def __init__(self, port="COM9", baud=TEENSY_DEFAULT_BAUD, telemetry=None,
                 cpr=TEENSY_DEFAULT_CPR, vid_pid=TEENSY_VID_PID, rx_prefix="",
                 protocol="yahboom", on_port_resolved=None, **codec_kwargs):
        super().__init__(port=port, baud=baud, telemetry=telemetry, cpr=cpr,
                         vid_pid=vid_pid, rx_prefix=rx_prefix, protocol=protocol,
                         on_port_resolved=on_port_resolved, **codec_kwargs)

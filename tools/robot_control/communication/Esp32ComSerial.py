r"""Esp32ComSerial - Liaison serie avec la carte de controle ESP32 WaveShare.

Carte "General Driver for Robots" Rev1.2 (ESP32-WROOM-32) du robot
bamboo4WD_V4_WSEsp32, faisant tourner firmware/esp32_bamboo avec le 4e connecteur
ConnectorSerialFrame (env platformio bamboov3-wirshare_bamboo_serial). Ce
connecteur emet/recoit EXACTEMENT le meme protocole de trames binaires que la
STM32 (memes funcodes 0x0A/0x0C/0x0D/0x12/0x13/0x14/0x50/0x51) -> on reutilise
integralement RobotComSerial : decodeurs, FrameParser, snapshot(), sendCmdVel().

Cette sous-classe n'ajoute que ce qui differe cote carte ESP32 :
  (a) VID:PID CP210x (10C4:EA60) pour prioriser le bon port au scan auto ;
  (b) ses propres defauts (UART @921600, cpr geometrie Waveshare) ;
  (c) verification du port PREFERE par sniff protocole (contrairement a la STM32
      ou l'ouverture du port prefere suffit) : la carte WaveShare expose DEUX
      CP2102 au meme VID:PID (comm ESP32 + donnees LIDAR), et l'ESP32 emet des
      trames 0xFB indistinguables de la STM32. Seul le sniff (>=2 trames valides)
      departage le port de comm du port LIDAR. Meme logique que GroveComSerial.

/!\ cpr=2114 (COUNTS_PER_REV1 firmware) est un PLACEHOLDER tant que la geometrie
    roue Waveshare n'est pas mesuree : les rpm/vitesses seront faux jusqu'a
    calibration (cf. plan, sous-tache separee).
"""
import time

import serial

try:
    from .RobotComSerial import RobotComSerial, FrameParser   # importe en tant que package
except ImportError:                              # importe a plat (dossier sur sys.path)
    from RobotComSerial import RobotComSerial, FrameParser

# Defauts specifiques a la carte ESP32 WaveShare
ESP32_DEFAULT_BAUD = 921600           # UART du firmware esp32_bamboo (monitor_speed)
ESP32_DEFAULT_CPR = 2114.0            # COUNTS_PER_REV1 (placeholder Waveshare, cf. firmware)
ESP32_VID_PID = ("10C4:EA60",)        # CP210x (Silicon Labs) des deux ports de la carte


class Esp32ComSerial(RobotComSerial):
    """Liaison serie avec la carte ESP32 WaveShare (protocole trames binaires).

    Identique a RobotComSerial hormis les defauts (baud/cpr/VID:PID) et la
    verification du port prefere par sniff.
    """

    def __init__(self, port="COM8", baud=ESP32_DEFAULT_BAUD, telemetry=None,
                 cpr=ESP32_DEFAULT_CPR, vid_pid=ESP32_VID_PID, rx_prefix=""):
        super().__init__(port=port, baud=baud, telemetry=telemetry, cpr=cpr,
                         vid_pid=vid_pid, rx_prefix=rx_prefix)

    def _probe(self, port, timeout=2.5):
        """Ouvre un port candidat DTR/RTS DESASSERTES et compte les trames 0xFB.

        /!\ Difference critique avec la STM32 (CH340) : sur les CP210x de la carte
        WaveShare, DTR et RTS sont cables aux broches EN (reset) et GPIO0 (boot) de
        l'ESP32. La forme `serial.Serial(port, baud)` de la classe de base asserte
        ces lignes -> l'ESP32 part en reset/download et ne debite que des octets
        parasites (0x80/0x00), jamais de trame valide. On construit donc le port
        SANS l'ouvrir, on force dtr=rts=False, puis .open() : l'ESP32 tourne (ou
        redemarre proprement une seule fois). Timeout allonge (2.5 s) pour couvrir
        un eventuel reboot (~1.5 s) + quelques trames avant de valider.
        """
        try:
            s = serial.Serial()
            s.port = port
            s.baudrate = self.baud
            s.timeout = 0.1
            s.dtr = False
            s.rts = False
            s.open()
        except Exception:
            return 0, None
        parser = FrameParser()
        ok = 0
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                chunk = s.read(256)
                if not chunk:
                    continue
                for _func, _data, good, _raw in parser.feed(chunk):
                    if good:
                        ok += 1
                if ok >= 2:
                    break
        except Exception:
            pass
        if ok >= 2:
            return ok, s
        try:
            s.close()
        except Exception:
            pass
        return ok, None

    def _open(self):
        """Ouvre la liaison en VERIFIANT meme le port prefere par sniff.

        Contrairement a la STM32 (port prefere = confiance a l'ouverture), l'ESP32
        WaveShare a deux CP2102 (10C4:EA60) : comm + LIDAR. On sniffe donc le port
        prefere ; s'il ne parle pas le protocole (>=2 trames), on bascule sur le
        scan auto priorise VID:PID (herite de _candidates).
        """
        # 1) port prefere : ouvert PUIS verifie par sniff (pas de confiance aveugle).
        ok, s = self._probe(self.port)
        if s is not None:
            self.ser = s
            self.last_err = None
            if self.tel:
                self.tel.log("event", msg="com_open", port=self.port,
                             mode="prefere", frames=ok)
            return
        # 2) scan auto : candidats CP210x priorises, departages par sniff protocole.
        for cand in self._candidates():
            ok, s = self._probe(cand)
            if s is not None:
                self.port = cand
                self.ser = s
                self.last_err = None
                if self.tel:
                    self.tel.log("event", msg="com_open", port=cand,
                                 mode="auto", frames=ok)
                return
        if self.tel:
            self.tel.log("event", msg="com_fail", err=self.last_err)

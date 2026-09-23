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
    from .RobotComSerial import RobotComSerial      # importe en tant que package
    from .wirecodec import make_codec
except ImportError:                              # importe a plat (dossier sur sys.path)
    from RobotComSerial import RobotComSerial
    from wirecodec import make_codec

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
                 cpr=ESP32_DEFAULT_CPR, vid_pid=ESP32_VID_PID, rx_prefix="",
                 protocol="yahboom", on_port_resolved=None, **codec_kwargs):
        super().__init__(port=port, baud=baud, telemetry=telemetry, cpr=cpr,
                         vid_pid=vid_pid, rx_prefix=rx_prefix, protocol=protocol,
                         on_port_resolved=on_port_resolved, **codec_kwargs)

    def _probe(self, port, timeout=2.5):
        """Ouvre un port candidat SANS TOUCHER A DTR/RTS et compte les trames du protocole.

        /!\ Ne JAMAIS manipuler DTR/RTS sur les CP210x de la carte WaveShare : ces deux
        lignes vont au circuit d'auto-reset, cable sur EN (reset) et GPIO0 (boot) de
        l'ESP32. Ce circuit est inerte quand les deux lignes sont dans le MEME etat, et
        c'est le cas a l'ouverture (l'OS les asserte toutes les deux) : l'application
        tourne et debite. Mais pyserial ecrit les lignes L'UNE APRES L'AUTRE, donc tout
        reglage -- meme vers un etat symetrique comme dtr=rts=False -- fait passer la
        carte par un etat asymetrique : elle redemarre EN MODE DOWNLOAD et devient
        DEFINITIVEMENT muette jusqu'au prochain reset. Mesure a l'appui : port ouvert
        intact = 73 trames/s ; apres un seul reglage des lignes = 0 octet, et le retour
        a l'etat initial ne la ramene pas.

        (Une version anterieure forcait dtr=rts=False en croyant eviter un reset ; le
        "bruit 0x80/0x00" qui avait motive ce choix etait en fait le flux applicatif a
        921600 lu a 115200. Le sniff comptait alors 2 trames -- celles deja tamponnees
        avant le reset -- puis plus rien, ce qui faisait passer une carte saine pour
        morte. Si un reset est un jour necessaire, utiliser resetToApp().)
        """
        try:
            s = serial.Serial(port, self.baud, timeout=0.1)
        except Exception:
            return 0, None
        codec = make_codec(self.protocol, **self._codec_kwargs)
        ok = 0
        sysid = None
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                chunk = s.read(256)
                if not chunk:
                    continue
                events, n_ok, _n_bad = codec.feed(chunk)
                ok += n_ok
                for kind, payload in events:
                    if kind == "heartbeat":
                        sysid = payload.get("sysid")
                if ok >= 2:
                    break
        except Exception:
            pass
        if ok >= 2:
            if sysid is not None:
                self.rx_sysid = sysid
            return ok, s
        try:
            s.close()
        except Exception:
            pass
        return ok, None

    def resetToApp(self):
        """Redemarre la carte SUR L'APPLICATION par le circuit d'auto-reset (USB seul).

        Cablage mesure sur cette carte : RTS -> EN (reset), DTR -> GPIO0 (boot), assertion
        pyserial = niveau BAS sur la broche. Demarrer l'application exige donc GPIO0 HAUT
        au moment ou EN est relache : dtr=False, rts=True (reset tenu), pause, rts=False.
        Toute autre sequence relache EN avec GPIO0 bas -> bootloader, donc silence. C'est
        la SEULE facon de recuperer une carte laissee en mode download, et la ROM le
        confirme par "boot:0x13 (SPI_FAST_FLASH_BOOT)".
        """
        if self.ser is None:
            return False
        try:
            self.ser.dtr = False           # GPIO0 haut = demarrage normal
            self.ser.rts = True            # EN bas = carte en reset
            time.sleep(0.15)
            self.ser.rts = False           # EN haut = l'application demarre
            time.sleep(1.8)                # duree de boot avant les premieres trames
            return True
        except Exception as e:
            self.last_err = str(e)
            return False

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
                self._notify_port_resolved(cand)   # COM change -> persiste le profil
                return
        if self.tel:
            self.tel.log("event", msg="com_fail", err=self.last_err)

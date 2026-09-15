r"""GroveComSerial - Liaison serie avec la carte capteurs GrovePi+ Bambou.

Pendant de RobotComSerial (STM32) pour la carte capteurs GrovePi+ v0.3 (ATmega328P
detourne, cf. firmware/grovepi_bamboo/). MEME structure de trame que la STM32
([0xFF][ID][LEN][FUNC][donnees...][CHK], little-endian) mais protocole DISTINCT :
ID 0xFB carte->hote (0xFC hote->carte), FUNC report 0x60/0x61/0x62 prefixes d'un
timestamp uint32 ms (horloge monotone bord). La carte AUTO-EMET a ~20 Hz : ce
module est un pur ECOUTEUR (aucune requete), il decode les trames report et expose
un snapshot thread-safe (IMU roll/pitch + bruts, 4 ultrasons mm, IR).

Modele identique a RobotComSerial : un thread lecteur, reconnexion automatique
(re-scan des ports COM), ecritures inexistantes ici (lecture seule). La carte peut
etre absente au demarrage : le thread reessaie l'ouverture toutes les 1,5 s et
`connected` reflete l'etat courant -> l'app demarre sans la carte et l'affiche des
qu'elle apparait (cf. RobotControlCore, exigence « demarrer sans grovepi »).

Protocole report (cf. firmware/grovepi_bamboo/tools/check.py, source unique) :
  0x51 VERSION      [major][minor][patch]
  0x60 REPORT_IMU   [ts u32][roll i16*100][pitch i16*100][ax ay az gx gy gz i16]
  0x61 REPORT_ULTRA [ts u32][4x u16 mm]           (0xFFFF = pas d'echo)
  0x62 REPORT_IR    [ts u32][dist u16][adc u16]
"""
import threading
import time

import serial
from serial.tools import list_ports

BAUD = 115200
PTO_HEAD = 0xFF
PTO_ID_TX = 0xFB          # carte -> hote (sens ecoute)
PTO_ID_RX = 0xFC          # hote -> carte

FUNC_VERSION      = 0x51
FUNC_REPORT_IMU   = 0x60
FUNC_REPORT_ULTRA = 0x61
FUNC_REPORT_IR    = 0x62

DIST_NONE = 0xFFFF        # ultrason : pas d'echo (hors portee)


def _s16(lo, hi):
    v = lo | (hi << 8)
    return v - 0x10000 if v & 0x8000 else v


def _u16(lo, hi):
    return lo | (hi << 8)


def _u32(d, off=0):
    return d[off] | (d[off + 1] << 8) | (d[off + 2] << 16) | (d[off + 3] << 24)


class GroveFrameParser:
    """Machine a etats : accepte les octets recus, rend les trames carte->hote.

    Ne reconnait que les trames d'ID 0xFB (sens carte->hote) : ainsi, branche par
    erreur sur le port de la STM32 (qui emet en 0xFC), aucune trame n'est validee
    -> le scan auto n'adopte pas ce port.
    """

    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk):
        self.buf.extend(chunk)
        out = []
        while True:
            start = self._find_header()
            if start is None:
                break
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 3:
                break
            length = self.buf[2]
            total = length + 2
            if length < 2 or total > 64:
                del self.buf[:2]
                continue
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            func, data, chk = raw[3], raw[4:total - 1], raw[total - 1]
            out.append((func, data, (sum(raw[2:total - 1]) & 0xFF) == chk))
        return out

    def _find_header(self):
        b = self.buf
        for i in range(len(b) - 1):
            if b[i] == PTO_HEAD and b[i + 1] == PTO_ID_TX:
                return i
        return len(b) - 1 if b and b[-1] == PTO_HEAD else None


class GroveComSerial:
    """Ecouteur serie non bloquant de la carte capteurs. Reconnexion automatique."""

    def __init__(self, port="COM6", baud=BAUD, telemetry=None, vid_pid=None):
        self.port = port
        self.baud = baud
        self.tel = telemetry
        # VID:PID cibles ("VVVV:PPPP") pour PRIORISER les bons ports au scan auto
        # (None = pas de filtrage, comportement historique). Cf. RobotComSerial.
        self.vid_pid = tuple(vid_pid) if vid_pid else None
        self.ser = None
        self.lock = threading.RLock()
        self.last_err = None

        # Derniers etats decodes (sous self.lock)
        self.version = None                       # "maj.min.patch"
        self.roll = self.pitch = None             # deg
        self.accel = None                         # (ax, ay, az) bruts
        self.gyro = None                          # (gx, gy, gz) bruts
        self.ultra = [None, None, None, None]     # mm, None = pas d'echo
        self.ir_dist = None                       # mm
        self.ir_adc = None                        # adc brut
        self.imu_t = 0.0                          # date de la derniere trame IMU
        self.ultra_t = 0.0                        # date de la derniere trame ultra
        self.ir_t = 0.0
        self.ts_imu = self.ts_ultra = self.ts_ir = None   # timestamps bord (ms)
        self.ok = 0
        self.bad = 0

        self._stop = False
        self._reader = threading.Thread(target=self._readLoop, daemon=True)
        self._reader.start()

    # --- connexion / thread lecteur ----------------------------------------
    def _matches_vid_pid(self, p):
        """True si le port <p> (ListPortInfo) matche l'un des VID:PID cibles.

        Compare sur .vid/.pid (int) et, en repli, sur .hwid. Insensible a la casse.
        Si aucun VID:PID cible : toujours False (comportement historique).
        """
        if not self.vid_pid:
            return False
        want = {vp.upper().replace("VID:PID=", "").strip() for vp in self.vid_pid}
        vid = getattr(p, "vid", None)
        pid = getattr(p, "pid", None)
        if vid is not None and pid is not None:
            if f"{vid:04X}:{pid:04X}" in want:
                return True
        hwid = (getattr(p, "hwid", "") or "").upper()
        return any(vp in hwid for vp in want)

    def _candidates(self):
        """Ports COM disponibles hors port prefere (deja tente en premier).

        Si des VID:PID cibles sont definis, les ports qui matchent sont places
        EN TETE (priorises avant le sniff protocole) ; sinon ordre systeme inchange.
        """
        try:
            infos = [p for p in list_ports.comports() if p.device != self.port]
        except Exception:
            return []
        if self.vid_pid:
            preferred = [p.device for p in infos if self._matches_vid_pid(p)]
            others = [p.device for p in infos if not self._matches_vid_pid(p)]
            return preferred + others
        return [p.device for p in infos]

    def _probe(self, port, timeout=0.8):
        """Ouvre un port et compte les trames carte (0xFB) valides.

        La carte auto-emet a ~20 Hz : >=2 trames 0xFB decodees en <timeout> => c'est
        bien la GrovePi (et pas la STM32, qui emet en 0xFC). Retourne
        (nb_ok, handle_ouvert|None) ; handle garde ouvert seulement si identifie.
        """
        try:
            s = serial.Serial(port, self.baud, timeout=0.1)
        except Exception:
            return 0, None
        parser = GroveFrameParser()
        ok = 0
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                chunk = s.read(128)
                if not chunk:
                    continue
                for _func, _data, good in parser.feed(chunk):
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
        """Ouvre la carte : port prefere (verifie qu'il parle 0xFB), sinon scan auto.

        Contrairement a RobotComSerial (qui fait confiance au port prefere des qu'il
        s'ouvre), on VERIFIE meme le port prefere : deux cartes partagent le meme
        format de trame, on ne veut pas capter par erreur le port de la STM32.
        """
        ok, s = self._probe(self.port)
        if s is not None:
            self.ser = s
            self.last_err = None
            if self.tel:
                self.tel.log("event", msg="grove_open", port=self.port,
                             mode="prefere", frames=ok)
            return
        for cand in self._candidates():
            ok, s = self._probe(cand)
            if s is not None:
                self.port = cand
                self.ser = s
                self.last_err = None
                if self.tel:
                    self.tel.log("event", msg="grove_open", port=cand,
                                 mode="auto", frames=ok)
                return
        if self.tel:
            self.tel.log("event", msg="grove_fail", err=self.last_err)

    def _readLoop(self):
        parser = GroveFrameParser()
        while not self._stop:
            if self.ser is None:
                self._open()
                if self.ser is None:
                    time.sleep(1.5)                # reessai reconnexion
                    continue
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.last_err = str(e)
                self._closeSer()
                continue
            if not chunk:
                continue
            for func, data, good in parser.feed(chunk):
                with self.lock:
                    if not good:
                        self.bad += 1
                        continue
                    self.ok += 1
                    self._apply(func, data)

    def _apply(self, func, data):
        """Met a jour l'etat a partir d'une trame decodee (sous verrou)."""
        now = time.time()
        if func == FUNC_REPORT_IMU and len(data) >= 20:
            self.ts_imu = _u32(data, 0)
            self.roll = _s16(data[4], data[5]) / 100.0
            self.pitch = _s16(data[6], data[7]) / 100.0
            self.accel = (_s16(data[8], data[9]), _s16(data[10], data[11]),
                          _s16(data[12], data[13]))
            self.gyro = (_s16(data[14], data[15]), _s16(data[16], data[17]),
                         _s16(data[18], data[19]))
            self.imu_t = now
            if self.tel:
                self.tel.log_rx("grove_imu", roll=self.roll, pitch=self.pitch)
        elif func == FUNC_REPORT_ULTRA and len(data) >= 12:
            self.ts_ultra = _u32(data, 0)
            vals = []
            for i in range(4):
                v = _u16(data[4 + 2 * i], data[5 + 2 * i])
                vals.append(None if v == DIST_NONE else v)
            self.ultra = vals
            self.ultra_t = now
            if self.tel:
                self.tel.log_rx("grove_ultra", mm=list(self.ultra))
        elif func == FUNC_REPORT_IR and len(data) >= 8:
            self.ts_ir = _u32(data, 0)
            d = _u16(data[4], data[5])
            self.ir_dist = None if d == DIST_NONE else d
            self.ir_adc = _u16(data[6], data[7])
            self.ir_t = now
        elif func == FUNC_VERSION and len(data) >= 3:
            self.version = "%d.%d.%d" % (data[0], data[1], data[2])

    def _closeSer(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    # --- proprietes ---------------------------------------------------------
    @property
    def connected(self):
        return self.ser is not None

    def snapshot(self):
        """Copie coherente des derniers etats (sous verrou), avec anciennete."""
        now = time.time()
        with self.lock:
            return {
                "connected": self.ser is not None,
                "version": self.version,
                "roll": self.roll, "pitch": self.pitch,
                "accel": self.accel, "gyro": self.gyro,
                "ultra": list(self.ultra),
                "ir_dist": self.ir_dist, "ir_adc": self.ir_adc,
                "imu_age": (now - self.imu_t) if self.imu_t else None,
                "ultra_age": (now - self.ultra_t) if self.ultra_t else None,
                "ir_age": (now - self.ir_t) if self.ir_t else None,
                "ok": self.ok, "bad": self.bad,
            }

    def close(self):
        self._stop = True
        self._closeSer()
        if self.tel:
            self.tel.log("event", msg="grove_close", port=self.port)

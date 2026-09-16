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

PROTOCOLE DE FIL ENFICHABLE (parametre `protocol`, comme RobotComSerial) :
  - "yahboom" (DEFAUT) : trames report maison, decodees par GroveYahboomCodec (ci-dessous).
      0x51 VERSION      [major][minor][patch]
      0x60 REPORT_IMU   [ts u32][roll i16*100][pitch i16*100][ax ay az gx gy gz i16]
      0x61 REPORT_ULTRA [ts u32][4x u16 mm]           (0xFFFF = pas d'echo)
      0x62 REPORT_IR    [ts u32][dist u16][adc u16]
  - "mavlink" : MAVLink v2, dialecte `bamboo` (sysid 4), via le MavlinkCodec partage
      (cf. wirecodec) : ATTITUDE + SCALED_IMU (IMU), DISTANCE_SENSOR x5 (ultra id 0..3,
      IR id 4). Note : en MAVLink l'accel est en mg et le gyro en mrad/s (SCALED_IMU
      standard), la ou le yahboom expose des bruts LSB ; l'ADC IR n'a pas d'equivalent
      standard -> ir_adc reste None en mode mavlink.

Les deux codecs produisent les MEMES evenements normalises consommes par _applyEvent :
  ("version",{version})  ("imu",{roll,pitch,ts})  ("imu_raw",{accel,gyro,ts})
  ("ultra",{id,mm,ts})   ("ir",{id,mm,adc?,ts})   -> snapshot inchange (seam stable).
"""
import threading
import time

import serial
from serial.tools import list_ports

from .wirecodec import make_codec

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


class GroveYahboomCodec:
    """Codec du protocole report maison de la GrovePi -> evenements normalises.

    Enveloppe fine autour de GroveFrameParser : meme interface que MavlinkCodec
    (feed -> (events, n_ok, n_bad)) pour que GroveComSerial route les deux protocoles
    par le meme chemin (_applyEvent). Les evenements sont identiques a ceux du
    MavlinkCodec pour la carte capteurs -> l'etat expose (snapshot) ne depend pas du fil.
    """

    def __init__(self):
        self._parser = GroveFrameParser()

    def feed(self, chunk):
        events, n_ok, n_bad = [], 0, 0
        for func, data, good in self._parser.feed(chunk):
            if not good:
                n_bad += 1
                continue
            n_ok += 1
            events.extend(self._decode(func, data))
        return events, n_ok, n_bad

    @staticmethod
    def _decode(func, data):
        if func == FUNC_REPORT_IMU and len(data) >= 20:
            ts = _u32(data, 0)
            # ATTITUDE (roll/pitch en deg) + bruts accel/gyro (LSB) separes, comme
            # en MAVLink (ATTITUDE + SCALED_IMU) : deux evenements distincts.
            return [
                ("imu", {"roll": _s16(data[4], data[5]) / 100.0,
                         "pitch": _s16(data[6], data[7]) / 100.0, "ts": ts}),
                ("imu_raw", {"accel": (_s16(data[8], data[9]), _s16(data[10], data[11]),
                                       _s16(data[12], data[13])),
                             "gyro": (_s16(data[14], data[15]), _s16(data[16], data[17]),
                                      _s16(data[18], data[19])), "ts": ts}),
            ]
        if func == FUNC_REPORT_ULTRA and len(data) >= 12:
            ts = _u32(data, 0)
            out = []
            for i in range(4):
                v = _u16(data[4 + 2 * i], data[5 + 2 * i])
                out.append(("ultra", {"id": i, "mm": None if v == DIST_NONE else v, "ts": ts}))
            return out
        if func == FUNC_REPORT_IR and len(data) >= 8:
            ts = _u32(data, 0)
            d = _u16(data[4], data[5])
            return [("ir", {"id": 4, "mm": None if d == DIST_NONE else d,
                            "adc": _u16(data[6], data[7]), "ts": ts})]
        if func == FUNC_VERSION and len(data) >= 3:
            return [("version", {"version": "%d.%d.%d" % (data[0], data[1], data[2])})]
        return []


class GroveComSerial:
    """Ecouteur serie non bloquant de la carte capteurs. Reconnexion automatique."""

    def __init__(self, port="COM6", baud=BAUD, telemetry=None, vid_pid=None,
                 protocol="yahboom", expected_sysid=4, **codec_kwargs):
        self.port = port
        self.baud = baud
        self.tel = telemetry
        # VID:PID cibles ("VVVV:PPPP") pour PRIORISER les bons ports au scan auto
        # (None = pas de filtrage, comportement historique). Cf. RobotComSerial.
        self.vid_pid = tuple(vid_pid) if vid_pid else None
        # Protocole de fil ("yahboom" par defaut, ou "mavlink") : selectionne le codec.
        self.protocol = (protocol or "yahboom").lower()
        self._codec_kwargs = codec_kwargs
        self._codec = self._make_codec()
        # En MAVLink, plusieurs cartes parlent le meme fil : on n'adopte un port que
        # si son HEARTBEAT porte le sysid attendu (GrovePi = 4) -> pas de capture
        # accidentelle d'une carte de controle (sysid 1/2/3). Ignore en yahboom (le
        # format report 0xFB est deja exclusif a cette carte).
        self.expected_sysid = expected_sysid
        self.rx_sysid = None
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
    def _make_codec(self):
        """Fabrique le codec du protocole actif : GroveYahboomCodec (report maison
        0xFB) en yahboom, MavlinkCodec partage (dialecte bamboo) en mavlink. Le
        codec yahboom des cartes de controle (wirecodec) NE convient PAS ici : la
        GrovePi a son propre format report, distinct de la STM32."""
        if self.protocol == "mavlink":
            return make_codec("mavlink", **self._codec_kwargs)
        return GroveYahboomCodec()

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
        """Ouvre un port et compte les trames valides du protocole actif.

        La carte auto-emet a ~20 Hz : >=2 trames decodees en <timeout> => c'est bien
        une carte capteurs. En yahboom, le format report 0xFB est deja exclusif (la
        STM32 emet en 0xFC -> aucune trame validee sur son port). En mavlink, on exige
        EN PLUS un HEARTBEAT de sysid attendu (GrovePi=4) pour ne pas capter une carte
        de controle (sysid 1/2/3) qui parle le meme fil ; le timeout est alors rallonge
        (HEARTBEAT ~1 Hz). Retourne (nb_ok, handle|None) ; handle garde si identifie.
        """
        need_sysid = self.protocol == "mavlink" and self.expected_sysid is not None
        if need_sysid and timeout < 1.6:
            timeout = 1.6                          # laisser passer 1-2 HEARTBEAT
        try:
            s = serial.Serial(port, self.baud, timeout=0.1)
        except Exception:
            return 0, None
        codec = self._make_codec()
        ok = 0
        sysid_ok = not need_sysid
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                chunk = s.read(128)
                if not chunk:
                    continue
                events, n_ok, _n_bad = codec.feed(chunk)
                ok += n_ok
                for kind, payload in events:
                    if kind == "heartbeat" and payload.get("sysid") == self.expected_sysid:
                        sysid_ok = True
                        self.rx_sysid = payload.get("sysid")
                if ok >= 2 and sysid_ok:
                    break
        except Exception:
            pass
        if ok >= 2 and sysid_ok:
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
            events, n_ok, n_bad = self._codec.feed(chunk)
            with self.lock:
                self.ok += n_ok
                self.bad += n_bad
                for kind, payload in events:
                    self._applyEvent(kind, payload)

    def _applyEvent(self, kind, p):
        """Met a jour l'etat a partir d'un evenement normalise (sous verrou).

        Evenements produits par le codec du protocole actif (yahboom ou mavlink) ->
        l'etat expose (snapshot) est identique quel que soit le fil. Rappel unites :
        en mavlink accel=mg / gyro=mrad/s (SCALED_IMU), en yahboom bruts LSB ;
        ir_adc n'existe qu'en yahboom (None en mavlink).
        """
        now = time.time()
        if kind == "imu":
            self.roll = p["roll"]
            self.pitch = p["pitch"]
            self.ts_imu = p.get("ts", self.ts_imu)
            self.imu_t = now
            if self.tel:
                self.tel.log_rx("grove_imu", roll=self.roll, pitch=self.pitch)
        elif kind == "imu_raw":
            self.accel = p["accel"]
            self.gyro = p["gyro"]
            self.ts_imu = p.get("ts", self.ts_imu)
            self.imu_t = now
        elif kind == "ultra":
            i = p["id"]
            if 0 <= i < 4:
                self.ultra[i] = p["mm"]
            self.ts_ultra = p.get("ts", self.ts_ultra)
            self.ultra_t = now
            if self.tel:
                self.tel.log_rx("grove_ultra", mm=list(self.ultra))
        elif kind == "ir":
            self.ir_dist = p["mm"]
            self.ir_adc = p.get("adc")             # None en mavlink (pas d'ADC standard)
            self.ts_ir = p.get("ts", self.ts_ir)
            self.ir_t = now
        elif kind == "version":
            self.version = p["version"]
        elif kind == "heartbeat":
            self.rx_sysid = p.get("sysid")

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

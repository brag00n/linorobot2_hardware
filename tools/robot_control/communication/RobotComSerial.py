r"""RobotComSerial - Liaison serie avec la carte de controle du Bamboo v4.

Portage de l'ancien board_link.BoardLink vers le style Bambou4WD_python (classe
RobotXxx, methodes camelCase). Ouvre le port (@115200 par defaut), lit en continu
la telemetrie auto-emise (thread lecteur) et expose l'envoi des commandes
(moteurs, servos, cmd_vel, parametres).

Le PROTOCOLE DE FIL est enfichable via le parametre `protocol` (cf. wirecodec) :
  - "yahboom" (DEFAUT) : trames binaires maison, decodeurs reutilises depuis
    firmware/stm32_bamboo/tools/ros_monitor.py (l'ancien COM reste toujours
    disponible : choix de configuration, pas un remplacement) ;
  - "mavlink" : MAVLink v2, dialecte `bamboo`, protocole cible commun aux 3 cartes.
Cette classe ne connait plus le format du fil : elle delegue au codec le decodage
(feed -> evenements normalises) et la construction des trames (builders -> bytes),
et ne manipule que des evenements/etats independants du protocole.

Modele : les ECRITURES se font dans le thread appelant, les LECTURES dans le
thread lecteur -> pas de conflit read/write sur le handle.

/!\ Un seul programme peut tenir le port a la fois : couper le serveur MCP avant.
"""
import math
import threading
import time
from collections import deque

import serial
from serial.tools import list_ports

from .wirecodec import make_codec, CAR_TYPE_CPR

DEFAULT_CPR = 1320.0       # tics/tour par defaut (Mecanum/4-roues 330RPM ; cf CAR_TYPE_CPR)


class RobotComSerial:
    """Liaison serie non bloquante avec la carte. Reconnexion automatique."""

    def __init__(self, port="COM4", baud=115200, telemetry=None, cpr=DEFAULT_CPR,
                 vid_pid=None, rx_prefix="", protocol="yahboom", **codec_kwargs):
        self.port = port
        self.baud = baud
        self.tel = telemetry
        # Protocole de fil ("yahboom" par defaut, ou "mavlink") : selectionne le codec.
        self.protocol = (protocol or "yahboom").lower()
        self._codec_kwargs = codec_kwargs
        self._codec = make_codec(self.protocol, **codec_kwargs)
        # Prefixe des sous-cles rx:<...> loggees (state.json/jsonl). "" = carte
        # PRINCIPALE (rx:speed/imu/encoder, lues telles quelles par le MCP status) ;
        # une carte SECONDAIRE du banc recoit "teensy_"/"esp32_" pour ne PAS ecraser
        # les cles de la primaire (sinon plusieurs cartes ecrivent rx:imu en concurrence).
        self._rx_prefix = rx_prefix
        self.cpr = float(cpr)          # tics/tour (conversion tics/s -> tr/min)
        # VID:PID cibles ("VVVV:PPPP" hex) pour PRIORISER les bons ports au scan auto
        # (None = pas de filtrage : decouverte par sniff protocole seul, comportement
        # historique STM32/CH340). Cf. Esp32ComSerial (CP210x) et les profils robot.
        self.vid_pid = tuple(vid_pid) if vid_pid else None
        self.ser = None
        self.lock = threading.RLock()
        self.last_err = None

        # Derniers etats decodes (thread-safe via self.lock)
        self.battery = None            # V
        self.vx = self.vy = self.vz = None
        self.yaw = None                # deg
        self.roll = self.pitch = None  # deg
        self.encoders = None           # [M1..M4]
        # Champ magnetique (extension carte ESP32 WaveShare / BAMBOO_MAG ; la STM32
        # ne l'emet pas -> reste None). mag = (mx, my, mz) en uT ; heading = cap
        # boussole en degres [0..360[ derive de (mx, my).
        self.mag = None
        self.heading = None
        self.ok = 0
        self.bad = 0
        # sysid MAVLink de la carte, capte au HEARTBEAT (None en yahboom / avant sync).
        self.rx_sysid = None

        # Timestamps horloge interne carte (ms) prefixes des trames de metriques
        # + heure de reception hote (pour le calcul d'age).
        self.ts_speed = None           # ms carte (vitesse+batterie)
        self.ts_imu = None             # ms carte (attitude)
        self.ts_enc = None             # ms carte (encodeurs)
        self.ts_mag = None             # ms carte (magneto)
        self.speed_t = 0.0             # time.time() de la derniere trame vitesse
        self.imu_t = 0.0               # time.time() de la derniere trame attitude
        self.enc_t = 0.0               # time.time() de la derniere trame encodeurs
        self.mag_t = 0.0               # time.time() de la derniere trame magneto

        # Rapports requete/reponse (getPid/getWheelGeom/getCarType) + horodatage :
        # une lecture ecrit la requete puis attend un horodatage plus recent.
        self.car_type = None           # octet type de chassis (0x01..0x06)
        self.car_type_t = 0.0
        self.wheel_geom = None         # dict (shape decode_wheel_geom)
        self.wheel_geom_t = 0.0
        self.pid = {}                  # index (1..5) -> (dict {index,kp,ki,kd}, t)

        # Historique encodeur (vitesse instantanee) + baseline de calibration
        self.enc_hist = deque(maxlen=600)   # (t, [M1..M4])
        self.baseline = None                # [M1..M4] de reference (calibrate)

        self._stop = False
        self._reader = threading.Thread(target=self._readLoop, daemon=True)
        self._reader.start()

    # --- connexion / thread lecteur ----------------------------------------
    def _matches_vid_pid(self, p):
        """True si le port <p> (ListPortInfo) matche l'un des VID:PID cibles.

        Compare sur .vid/.pid (int) et, en repli, sur .hwid (ex. 'USB VID:PID=10C4:EA60
        SER=...'). Insensible a la casse. Si aucun VID:PID cible : toujours False
        (pas de priorisation, comportement historique).
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

    def _preferred_port_ok(self):
        """La carte presente sur le port PREFERE porte-t-elle le bon VID:PID ?

        Controle d'identite INSTANTANE (lecture de l'enumeration, sans ouvrir le
        port) : « le nom de la carte associee au COM configure ». Sert a n'accorder
        la connexion instantanee qu'a la bonne carte ; sinon on bascule en scan.
        - pas de VID:PID cible -> True (rien a controler, comportement historique) ;
        - port configure absent de l'enumeration -> False (ex. COM perime) ;
        - un autre device branche sur ce COM -> False (mauvaise carte).
        NB : deux adaptateurs de meme VID:PID sont indiscernables ici -> c'est le
        probe+sysid du scan qui tranche (cf. WaveShare 2x CP-2102).
        """
        if not self.vid_pid:
            return True
        try:
            for p in list_ports.comports():
                if p.device == self.port:
                    return self._matches_vid_pid(p)
        except Exception:
            return False
        return False   # port prefere absent de l'enumeration

    def _candidates(self):
        """Autres ports COM disponibles (le port prefere exclu, deja tente).

        Si des VID:PID cibles sont definis, les ports qui matchent sont places
        EN TETE (priorises avant le sniff protocole) ; l'ordre relatif du reste
        est conserve. Sans VID:PID cible : ordre systeme inchange (historique).
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
        """Ouvre un port candidat et compte les trames valides du protocole actif.

        Le port de la carte auto-emet en continu : >=2 trames decodees en
        <timeout> => c'est bien la carte. Utilise un codec NEUF (etat de parsing
        isole du thread lecteur) et, en MAVLink, capte le sysid du HEARTBEAT.
        Retourne (nb_ok, handle_ouvert|None) ; le handle n'est garde ouvert que
        si le port est identifie (deja synchronise, pas de reouverture).
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
                chunk = s.read(128)
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

    def _open(self):
        # 1) port prefere : connexion INSTANTANEE, mais seulement si la carte
        #    presente sur ce COM porte le bon VID:PID (controle d'identite). Un COM
        #    perime ou un autre device branche dessus -> on ne s'y accroche pas, on
        #    passe directement au scan (sinon on ouvrirait la mauvaise carte).
        if self._preferred_port_ok():
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.last_err = None
                if self.tel:
                    self.tel.log("event", msg="com_open", port=self.port, mode="prefere")
                return
            except Exception as e:                 # port occupe / absent / errone
                self.last_err = str(e)
                self.ser = None
        else:
            self.last_err = f"carte attendue absente du port prefere {self.port}"
        # 2) scan auto : on cherche le port qui PARLE le protocole actif (probe+sysid).
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

        Les evenements sont produits par le codec du protocole actif (independants
        du format du fil) ; les sous-cles log_rx restent identiques a l'historique.
        """
        if kind == "speed":
            self.vx = p["vx"]
            self.vy = p["vy"]
            self.vz = p["vz"]
            # MAVLink separe la batterie (SYS_STATUS) de la vitesse -> None ici :
            # ne pas ecraser la derniere tension connue.
            if p.get("battery") is not None:
                self.battery = p["battery"]
            self.ts_speed = p.get("ts")
            self.speed_t = time.time()
            if self.tel:
                self.tel.log_rx(self._rx_prefix + "speed", vx=self.vx, vy=self.vy,
                                vz=self.vz, batt=self.battery)
        elif kind == "battery":
            self.battery = p["battery"]
        elif kind == "imu":
            self.roll = p["roll"]
            self.pitch = p["pitch"]
            self.yaw = p["yaw"]
            self.ts_imu = p.get("ts")
            self.imu_t = time.time()
            if self.tel:
                self.tel.log_rx(self._rx_prefix + "imu", roll=self.roll,
                                pitch=self.pitch, yaw=self.yaw)
        elif kind == "mag":
            self.mag = (p["mx"], p["my"], p["mz"])
            self.heading = p["heading"]
            self.ts_mag = p.get("ts")
            self.mag_t = time.time()
            if self.tel:
                self.tel.log_rx(self._rx_prefix + "mag", mx=p["mx"], my=p["my"],
                                mz=p["mz"], heading=self.heading)
        elif kind == "encoder":
            self.encoders = list(p["m"])
            self.ts_enc = p.get("ts")
            self.enc_t = time.time()
            self.enc_hist.append((time.time(), list(self.encoders)))
            if self.tel:
                self.tel.log_rx(self._rx_prefix + "encoder", m=list(self.encoders))
        # --- rapports requete/reponse (lus par getCarType/getWheelGeom/getPid) ---
        elif kind == "car_type":
            self.car_type = p["value"]
            self.car_type_t = time.time()
        elif kind == "wheel_geom":
            self.wheel_geom = dict(p)
            self.wheel_geom_t = time.time()
        elif kind == "pid":
            self.pid[int(p["index"])] = (dict(p), time.time())
        elif kind == "heartbeat":
            # MAVLink seul : sert a la decouverte (sysid). Pas d'etat metrique.
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
        """Copie coherente des dernieres metriques (sous verrou).
        ts_* = horloge interne carte (ms) prefixee des trames ; *_age = fraicheur
        cote hote (secondes depuis la derniere reception), None si jamais recu -- meme
        convention que GroveComSerial.snapshot."""
        now = time.time()
        with self.lock:
            return {
                "battery": self.battery, "yaw": self.yaw,
                "roll": self.roll, "pitch": self.pitch,
                "vx": self.vx, "vy": self.vy, "vz": self.vz,
                "encoders": list(self.encoders) if self.encoders else None,
                "mag": self.mag, "heading": self.heading,
                "ts_speed": self.ts_speed, "ts_imu": self.ts_imu, "ts_enc": self.ts_enc,
                "ts_mag": self.ts_mag,
                "speed_age": (now - self.speed_t) if self.speed_t else None,
                "imu_age": (now - self.imu_t) if self.imu_t else None,
                "enc_age": (now - self.enc_t) if self.enc_t else None,
                "mag_age": (now - self.mag_t) if self.mag_t else None,
                "ok": self.ok, "bad": self.bad,
            }

    # --- envoi de commandes -------------------------------------------------
    def _write(self, frame):
        if self.ser is None or not frame:
            return False
        try:
            self.ser.write(frame)
            return True
        except Exception as e:
            self.last_err = str(e)
            self._closeSer()
            if self.tel:
                self.tel.log("event", msg="com_lost", err=str(e))
            return False

    def sendMotor(self, m1, m2, m3, m4):
        """Consigne PWM brute : 4 valeurs signees en % (-100..100), boucle ouverte."""
        vals = [max(-100, min(100, int(v))) for v in (m1, m2, m3, m4)]
        frame = self._codec.motor(*vals)
        ok = self._write(frame)
        if self.tel:
            self.tel.log("tx_motor", m=vals, hex=frame.hex(), ok=ok)
        return ok

    def stop(self):
        """Arret franc : PWM nuls, envoye plusieurs fois pour fiabilite."""
        ok = False
        for _ in range(3):
            ok = self.sendMotor(0, 0, 0, 0) or ok
            time.sleep(0.005)
        return ok

    def sendServo(self, sid, angle):
        """Positionne un servo : id 1..4, angle 0..180 (borne cote firmware aussi)."""
        sid = int(sid)
        angle = max(0, min(180, int(round(angle))))
        if sid not in (1, 2, 3, 4):
            return False
        frame = self._codec.servo(sid, angle)
        ok = self._write(frame)
        if self.tel:
            self.tel.log("tx_servo", id=sid, angle=angle, hex=frame.hex(), ok=ok)
        return ok

    def sendServoAll(self, a1, a2, a3, a4):
        """Positionne les 4 servos (S1..S4), angles 0..180 d'un seul coup."""
        vals = [max(0, min(180, int(round(a)))) for a in (a1, a2, a3, a4)]
        frame = self._codec.servo_all(*vals)
        ok = self._write(frame)
        if self.tel:
            self.tel.log("tx_servo_all", a=vals, hex=frame.hex(), ok=ok)
        return ok

    def motorRaw(self, vals):
        """Envoi brut des 4 PWM signes (deja bornes par l'appelant), SANS log
        telemetrie : destine aux rafales anti-watchdog de motor_drive (40+ envois).
        Pour le teleop clavier ponctuel, utiliser sendMotor (qui journalise)."""
        v = [max(-100, min(100, int(x))) for x in vals]
        return self._write(self._codec.motor(*v))

    def sendCmdVel(self, linear_x, angular_z):
        """Consigne de vitesse aux conventions ROS (Twist).

        La kinematics differentielle + le PID par roue tournent SUR la carte : on
        n'envoie que le Twist (linear.x m/s, angular.z rad/s ; Vy=0). La mise a
        l'echelle vers le format du fil est faite par le codec (mm/s en yahboom,
        SI en MAVLink). Tout a zero -> arret franc cote carte.
        """
        frame = self._codec.cmd_vel(linear_x, angular_z)
        ok = self._write(frame)
        if self.tel:
            self.tel.log("tx_motion", vx=linear_x, vz=angular_z, ok=ok)
        return ok

    # --- lectures instantanees derivees ------------------------------------
    def encSpeed(self, window=1.0):
        """Vitesse instantanee par moteur (tics/s) sur la fenetre glissante, ou None."""
        now = time.time()
        with self.lock:
            pts = [p for p in self.enc_hist if now - p[0] <= window]
        if len(pts) < 2:
            return None
        dt = pts[-1][0] - pts[0][0]
        if dt < 1e-3:
            return None
        return [(pts[-1][1][i] - pts[0][1][i]) / dt for i in range(4)]

    # --- requete/reponse (lecture d'un rapport apres emission de la requete) ---
    def _requestReport(self, req_frame, read, timeout=1.5):
        """Ecrit la (les) trame(s) de requete puis attend une valeur fraiche via read().

        read() renvoie la valeur decodee si un rapport PLUS RECENT que l'appel est
        arrive, sinon None. Retourne la valeur ou None sur timeout / port ferme.
        """
        if not self._write(req_frame):
            return None
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = read()
            if r is not None:
                return r
            time.sleep(0.05)
        return None

    def getCarType(self, timeout=1.5):
        """Type de chassis (0x01..0x06). Applique le cpr correspondant si connu."""
        with self.lock:
            base_t = self.car_type_t

        def read():
            with self.lock:
                if self.car_type is not None and self.car_type_t > base_t:
                    return self.car_type
            return None

        ct = self._requestReport(self._codec.request_car_type(), read, timeout)
        if ct is not None:
            cpr, _ = CAR_TYPE_CPR.get(ct, (None, None))
            if cpr:
                self.cpr = cpr
        return ct

    def setCarType(self, car_type, save=True):
        """Ecrit le type de chassis (persiste en flash si save)."""
        return self._write(self._codec.set_car_type(car_type, save=save))

    def getWheelGeom(self, timeout=1.5):
        """Geometrie roue courante (dict cpr/circ/diam/APB)."""
        with self.lock:
            base_t = self.wheel_geom_t

        def read():
            with self.lock:
                if self.wheel_geom is not None and self.wheel_geom_t > base_t:
                    return dict(self.wheel_geom)
            return None

        return self._requestReport(self._codec.request_wheel_geom(), read, timeout)

    def setWheelGeom(self, cpr, circ_mm, apb_mm, save=True):
        """Ecrit la geometrie roue. Retourne (ok, err) : err non nul si valeur invalide."""
        frame, err = self._codec.set_wheel_geom(cpr, circ_mm, apb_mm, save=save)
        if err is not None:
            return False, err
        return self._write(frame), None

    def getPid(self, index, timeout=1.5):
        """PID courant (dict {index,kp,ki,kd}) : index 1..4 = moteur M1..M4, 5 = yaw."""
        with self.lock:
            prev = self.pid.get(index)
            base_t = prev[1] if prev else 0.0

        def read():
            with self.lock:
                cur = self.pid.get(index)
                if cur and cur[1] > base_t:
                    return dict(cur[0])
            return None

        return self._requestReport(self._codec.request_pid(index), read, timeout)

    def setMotorPid(self, kp, ki, kd, save=False, motor_id=0, disable=False):
        """Regle le PID moteur (motor_id : 0 = les 4, 1..4 = un moteur M1..M4).

        disable=True (Yahboom/STM32) : desactive le PID de ce moteur (encodeur HS)
          et le cale en recopie sur son voisin ; ignore pour motor_id=0 et sans
          equivalent MAVLink (le codec l'ignore, mode yahboom par defaut).
        save=True : persiste en flash (RAM seule sinon).
        """
        return self._write(self._codec.set_motor_pid(kp, ki, kd, save=save,
                                                      motor_id=motor_id, disable=disable))

    def setYawPid(self, kp, ki, kd, save=False):
        """PID de cap/yaw (persiste en flash si save)."""
        return self._write(self._codec.set_yaw_pid(kp, ki, kd, save=save))

    def enterBootloader(self):
        """Fait sauter la carte dans son bootloader (plusieurs envois pour robustesse)
        puis FERME le port pour liberer COM (upload/flash). Retourne True si au moins
        un envoi a reussi."""
        frame = self._codec.enter_bootloader()
        sent = False
        for _ in range(3):
            sent = self._write(frame) or sent
            time.sleep(0.05)
        time.sleep(0.2)
        self._stop = True          # stoppe le thread lecteur (ne pas rouvrir COM)
        self._closeSer()
        if self.tel:
            self.tel.log("event", msg="bootloader", ok=sent)
        return sent

    def close(self):
        self._stop = True
        try:
            self.stop()
        except Exception:
            pass
        self._closeSer()
        if self.tel:
            self.tel.log("event", msg="com_close", port=self.port)

r"""RobotComSerial - Liaison serie avec la carte STM32 du Bamboo v4.

Portage de l'ancien board_link.BoardLink vers le style Bambou4WD_python (classe
RobotXxx, methodes camelCase). Comportement INCHANGE : ouvre COM4 (@115200), lit
en continu les trames auto-report (thread lecteur) et expose l'envoi des trames
de commande (moteurs, servos). Le protocole (trame [0xFF][0xFC][LEN][FUNC][...][CHK])
et ses decodeurs sont REUTILISES tels quels depuis
firmware/stm32_bamboo/tools/ros_monitor.py (source unique du protocole).

Modele : les ECRITURES se font dans le thread appelant, les LECTURES dans le
thread lecteur -> pas de conflit read/write sur le handle.

/!\ Un seul programme peut tenir COM4 a la fois : couper le serveur MCP avant.
"""
import os
import sys
import threading
import time
from collections import deque

import serial
from serial.tools import list_ports

# --- Pont vers le protocole partage (firmware/stm32_bamboo/tools) --------------
# Ce module est dans <repo>/tools/robot_control/communication/ ; le protocole STM32
# vit sous <repo>/firmware/stm32_bamboo/tools -> remonter de 3 niveaux.
_HERE = os.path.dirname(os.path.abspath(__file__))
_STM32_TOOLS = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "firmware", "stm32_bamboo", "tools"))
if _STM32_TOOLS not in sys.path:
    sys.path.insert(0, _STM32_TOOLS)

from ros_monitor import (  # noqa: E402  (import apres modif sys.path)
    build_frame, FrameParser,
    decode_speed, decode_imu_att, decode_encoder,
    decode_pid, decode_wheel_geom, CAR_TYPE_CPR,
    FUNC_REQUEST_DATA, FUNC_CAR_TYPE, FUNC_SET_WHEEL_GEOM,
    FUNC_SET_MOTOR_PID, FUNC_SET_YAW_PID,
)

# Codes fonction des commandes (miroir de ros_mcp_server.py)
FUNC_MOTOR = 0x10             # [m1 m2 m3 m4] int8 signe (% PWM)
FUNC_PWM_SERVO = 0x03         # [id_1based(1..4), angle(0..180)]
FUNC_PWM_SERVO_ALL = 0x04     # [s1 s2 s3 s4] 4 angles d'un coup
FUNC_ENTER_BOOTLOADER = 0xA3  # saut vers le bootloader ROM (flash sans BOOT0)
SAVE_VERIFY = 0x5F            # octet de garde : ecrit en flash si egal, sinon RAM

# Codes fonction auto-report (miroir de ros_monitor.py)
REPORT_SPEED = 0x0A        # Vx,Vy,Vz + batterie
REPORT_IMU_ATT = 0x0C      # roll, pitch, yaw
REPORT_ENCODER = 0x0D      # M1..M4 comptage cumulatif

DEFAULT_CPR = 1320.0       # tics/tour par defaut (Mecanum/4-roues 330RPM ; cf CAR_TYPE_CPR)


class RobotComSerial:
    """Liaison serie non bloquante avec la carte. Reconnexion automatique."""

    def __init__(self, port="COM4", baud=115200, telemetry=None, cpr=DEFAULT_CPR):
        self.port = port
        self.baud = baud
        self.tel = telemetry
        self.cpr = float(cpr)          # tics/tour (conversion tics/s -> tr/min)
        self.ser = None
        self.lock = threading.RLock()
        self.last_err = None

        # Derniers etats decodes (thread-safe via self.lock)
        self.battery = None            # V
        self.vx = self.vy = self.vz = None
        self.yaw = None                # deg
        self.roll = self.pitch = None  # deg
        self.encoders = None           # [M1..M4]
        self.ok = 0
        self.bad = 0

        # Rapports requete/reponse (REQUEST_DATA 0x50 -> report) + horodatage :
        # une lecture ecrit la requete puis attend un horodatage plus recent.
        self.car_type = None           # octet type de chassis (0x01..0x06)
        self.car_type_t = 0.0
        self.wheel_geom = None         # dict decode_wheel_geom
        self.wheel_geom_t = 0.0
        self.pid = {}                  # index (1..5) -> (dict decode_pid, t)

        # Historique encodeur (vitesse instantanee) + baseline de calibration
        self.enc_hist = deque(maxlen=600)   # (t, [M1..M4])
        self.baseline = None                # [M1..M4] de reference (calibrate)

        self._stop = False
        self._reader = threading.Thread(target=self._readLoop, daemon=True)
        self._reader.start()

    # --- connexion / thread lecteur ----------------------------------------
    def _candidates(self):
        """Autres ports COM disponibles (le port prefere exclu, deja tente)."""
        try:
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        return [p for p in ports if p != self.port]

    def _probe(self, port, timeout=0.8):
        """Ouvre un port candidat et compte les trames Yahboom valides.

        Le port de la carte auto-emet en continu : >=2 trames decodees en
        <timeout> => c'est bien la carte. Retourne (nb_ok, handle_ouvert|None) ;
        le handle n'est garde ouvert que si le port est identifie (deja
        synchronise, pas de reouverture).
        """
        try:
            s = serial.Serial(port, self.baud, timeout=0.1)
        except Exception:
            return 0, None
        parser = FrameParser()
        ok = 0
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                chunk = s.read(128)
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
        # 1) port prefere : on l'ouvre et on lui fait confiance s'il s'ouvre.
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.last_err = None
            if self.tel:
                self.tel.log("event", msg="com_open", port=self.port, mode="prefere")
            return
        except Exception as e:                     # port occupe / absent / errone
            self.last_err = str(e)
            self.ser = None
        # 2) scan auto : on cherche le port qui PARLE le protocole Yahboom.
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
        parser = FrameParser()
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
            for func, data, ok, _raw in parser.feed(chunk):
                with self.lock:
                    if not ok:
                        self.bad += 1
                        continue
                    self.ok += 1
                    self._apply(func, data)

    def _apply(self, func, data):
        """Met a jour l'etat a partir d'une trame decodee (sous verrou)."""
        if func == REPORT_SPEED and len(data) >= 7:
            d = decode_speed(data)
            self.vx = d["Vx (mm/s)"]
            self.vy = d["Vy (mm/s)"]
            self.vz = d["Vz (rad/s)"]
            self.battery = d["Batterie (V)"]
            if self.tel:
                self.tel.log_rx("speed", vx=self.vx, vy=self.vy, vz=self.vz,
                                batt=self.battery)
        elif func == REPORT_IMU_ATT and len(data) >= 6:
            d = decode_imu_att(data)
            self.roll = d["Roll (deg)"]
            self.pitch = d["Pitch (deg)"]
            self.yaw = d["Yaw (deg)"]
            if self.tel:
                self.tel.log_rx("imu", roll=self.roll, pitch=self.pitch,
                                yaw=self.yaw)
        elif func == REPORT_ENCODER and len(data) >= 16:
            d = decode_encoder(data)
            self.encoders = [d[f"M{i + 1}"] for i in range(4)]
            self.enc_hist.append((time.time(), list(self.encoders)))
            if self.tel:
                self.tel.log_rx("encoder", m=list(self.encoders))
        # --- rapports requete/reponse (lus par getCarType/getWheelGeom/getPid) ---
        elif func == FUNC_CAR_TYPE and len(data) >= 1:
            self.car_type = data[0]
            self.car_type_t = time.time()
        elif func == FUNC_SET_WHEEL_GEOM and len(data) >= 6:
            self.wheel_geom = decode_wheel_geom(data)
            self.wheel_geom_t = time.time()
        elif func in (FUNC_SET_MOTOR_PID, FUNC_SET_YAW_PID) and len(data) >= 7:
            pd = decode_pid(data)
            self.pid[int(pd["index"])] = (pd, time.time())

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
        """Copie coherente des dernieres metriques (sous verrou)."""
        with self.lock:
            return {
                "battery": self.battery, "yaw": self.yaw,
                "vx": self.vx, "vy": self.vy, "vz": self.vz,
                "encoders": list(self.encoders) if self.encoders else None,
                "ok": self.ok, "bad": self.bad,
            }

    # --- envoi de commandes -------------------------------------------------
    def _write(self, frame):
        if self.ser is None:
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
        """FUNC_MOTOR : 4 PWM signes en % (int8, -100..100)."""
        vals = [max(-100, min(100, int(v))) for v in (m1, m2, m3, m4)]
        frame = build_frame(FUNC_MOTOR, bytes((v & 0xFF) for v in vals))
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
        """FUNC_PWM_SERVO : servo id 1..4, angle 0..180 (borne cote firmware aussi)."""
        sid = int(sid)
        angle = max(0, min(180, int(round(angle))))
        if sid not in (1, 2, 3, 4):
            return False
        frame = build_frame(FUNC_PWM_SERVO, bytes([sid, angle]))
        ok = self._write(frame)
        if self.tel:
            self.tel.log("tx_servo", id=sid, angle=angle, hex=frame.hex(), ok=ok)
        return ok

    def sendServoAll(self, a1, a2, a3, a4):
        """FUNC_PWM_SERVO_ALL : 4 angles 0..180 d'un seul coup (S1..S4)."""
        vals = [max(0, min(180, int(round(a)))) for a in (a1, a2, a3, a4)]
        frame = build_frame(FUNC_PWM_SERVO_ALL, bytes(vals))
        ok = self._write(frame)
        if self.tel:
            self.tel.log("tx_servo_all", a=vals, hex=frame.hex(), ok=ok)
        return ok

    def motorRaw(self, vals):
        """Envoi brut des 4 PWM signes (deja bornes par l'appelant), SANS log
        telemetrie : destine aux rafales anti-watchdog de motor_drive (40+ envois).
        Pour le teleop clavier ponctuel, utiliser sendMotor (qui journalise)."""
        v = [max(-100, min(100, int(x))) for x in vals]
        return self._write(build_frame(FUNC_MOTOR, bytes((x & 0xFF) for x in v)))

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

    # --- requete/reponse (REQUEST_DATA 0x50 -> report) ---------------------
    def _requestReport(self, req_params, read, timeout=1.5):
        """Ecrit REQUEST_DATA(req_params) puis attend une valeur fraiche via read().

        read() renvoie la valeur decodee si un rapport PLUS RECENT que l'appel est
        arrive, sinon None. Retourne la valeur ou None sur timeout / port ferme.
        """
        if not self._write(build_frame(FUNC_REQUEST_DATA, req_params)):
            return None
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = read()
            if r is not None:
                return r
            time.sleep(0.05)
        return None

    def getCarType(self, timeout=1.5):
        """Type de chassis (0x01..0x06) via REQUEST_DATA(0x15). Applique cpr si connu."""
        with self.lock:
            base_t = self.car_type_t

        def read():
            with self.lock:
                if self.car_type is not None and self.car_type_t > base_t:
                    return self.car_type
            return None

        ct = self._requestReport(bytes([FUNC_CAR_TYPE, 0x00]), read, timeout)
        if ct is not None:
            cpr, _ = CAR_TYPE_CPR.get(ct, (None, None))
            if cpr:
                self.cpr = cpr
        return ct

    def setCarType(self, car_type, save=True):
        """Ecrit le type de chassis (FUNC_CAR_TYPE = [type, verify])."""
        verify = SAVE_VERIFY if save else 0x00
        return self._write(build_frame(FUNC_CAR_TYPE,
                                       bytes([int(car_type) & 0xFF, verify])))

    def getWheelGeom(self, timeout=1.5):
        """Geometrie roue courante (dict decode_wheel_geom) via REQUEST_DATA(0x16)."""
        with self.lock:
            base_t = self.wheel_geom_t

        def read():
            with self.lock:
                if self.wheel_geom is not None and self.wheel_geom_t > base_t:
                    return dict(self.wheel_geom)
            return None

        return self._requestReport(bytes([FUNC_SET_WHEEL_GEOM, 0x00]), read, timeout)

    def setWheelGeom(self, cpr, circ_mm, apb_mm, save=True):
        """Ecrit la geometrie roue (cpr entier ; circ_mm/apb_mm en mm -> stockes *10).

        Retourne (ok, err) : err non nul si une valeur sort de la plage u16.
        """
        try:
            cpr_i = int(round(float(cpr)))
            circ10 = int(round(float(circ_mm) * 10.0))
            apb10 = int(round(float(apb_mm) * 10.0))
        except (TypeError, ValueError):
            return False, "valeurs non numeriques (cpr / circ_mm / apb_mm)."
        if not (0 < cpr_i <= 0xFFFF and 0 < circ10 <= 0xFFFF and 0 < apb10 <= 0xFFFF):
            return False, ("hors plage : cpr=%d circ10=%d apb10=%d "
                           "(1..65535 ; circ/APB <= 6553.5 mm)." % (cpr_i, circ10, apb10))
        verify = SAVE_VERIFY if save else 0x00
        payload = bytes([cpr_i & 0xFF, (cpr_i >> 8) & 0xFF,
                         circ10 & 0xFF, (circ10 >> 8) & 0xFF,
                         apb10 & 0xFF, (apb10 >> 8) & 0xFF, verify])
        return self._write(build_frame(FUNC_SET_WHEEL_GEOM, payload)), None

    def getPid(self, index, timeout=1.5):
        """PID courant (dict kp/ki/kd) : index 1..4 = moteur (partage), 5 = yaw."""
        func = FUNC_SET_YAW_PID if index == 5 else FUNC_SET_MOTOR_PID
        with self.lock:
            prev = self.pid.get(index)
            base_t = prev[1] if prev else 0.0

        def read():
            with self.lock:
                cur = self.pid.get(index)
                if cur and cur[1] > base_t:
                    return dict(cur[0])
            return None

        return self._requestReport(bytes([func, index & 0xFF]), read, timeout)

    @staticmethod
    def _pidPayload(kp, ki, kd, verify):
        """[kp*1000][ki*1000][kd*1000] u16 little-endian + octet verify."""
        def u16(x):
            v = int(round(x * 1000.0)) & 0xFFFF
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        return u16(kp) + u16(ki) + u16(kd) + bytes([verify])

    def setMotorPid(self, kp, ki, kd, save=False):
        """PID moteur UNIQUE partage par les 4 moteurs (FUNC_SET_MOTOR_PID)."""
        verify = SAVE_VERIFY if save else 0x00
        return self._write(build_frame(FUNC_SET_MOTOR_PID,
                                       self._pidPayload(kp, ki, kd, verify)))

    def setYawPid(self, kp, ki, kd, save=False):
        """PID de cap/yaw (FUNC_SET_YAW_PID)."""
        verify = SAVE_VERIFY if save else 0x00
        return self._write(build_frame(FUNC_SET_YAW_PID,
                                       self._pidPayload(kp, ki, kd, verify)))

    def enterBootloader(self):
        """Fait sauter la carte dans son bootloader ROM (FUNC 0xA3), plusieurs
        envois pour robustesse, puis FERME le port pour liberer COM (upload/flash).
        Retourne True si au moins un envoi a reussi."""
        frame = build_frame(FUNC_ENTER_BOOTLOADER, bytes([SAVE_VERIFY]))
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

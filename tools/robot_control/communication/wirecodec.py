r"""wirecodec - Codecs de protocole enfichables pour RobotComSerial.

RobotComSerial expose un SEAM public stable (snapshot / encSpeed / sendCmdVel /
getPid / ...) ; ce module fournit les deux implementations du protocole de fil
selectionnables a l'execution par le parametre `protocol` :

  - "yahboom" (DEFAUT, historique) : trames binaires maison [0xFF][0xFC][LEN][FUNC]
    [payload][CHK], decodeurs reutilises tels quels depuis ros_monitor.py.
  - "mavlink" : MAVLink v2, dialecte `bamboo` (module genere communication/mav/bamboo.py),
    partage avec le firmware des 3 cartes (STM32 / ESP32 WaveShare / Teensy).

L'ancien COM reste donc TOUJOURS disponible (choix de configuration, pas un
remplacement) : cf. directive projet. Chaque codec expose une interface uniforme :

  feed(chunk)       -> (events, n_ok, n_bad)   # decodage RX -> evenements normalises
  <builder>(...)    -> bytes                    # trame(s) TX prete(s) a ecrire

Evenements normalises (kind, payload) consommes par RobotComSerial._applyEvent :
  ("speed",   {vx,vy,vz,battery,ts})   vx/vy en mm/s, vz en rad/s, battery en V|None
  ("battery", {battery})               V (MAVLink : SYS_STATUS separe de la vitesse)
  ("imu",     {roll,pitch,yaw,ts})     degres
  ("imu_raw", {accel,gyro,ts})         accel/gyro bruts GrovePi - yahboom uniquement (le codec MAVLink ne l'emet plus)
  ("ultra",   {id,mm,ts})              telemetre ultrason id 0..3, mm|None - GrovePi
  ("ir",      {id,mm,ts})              telemetre IR id 4, mm|None - GrovePi
  ("mag",     {mx,my,mz,heading,ts})   uT + cap boussole deg
  ("encoder", {m:[M1..M4],ts})         comptage cumulatif
  ("motor_rpm", {rpm:[M1..M4],req:[M1..M4],ts})  RPM mesure + demande, vus par le PID embarque
  ("car_type",{value})                 octet type chassis
  ("wheel_geom", {"cpr (tics/tour)","circ (mm)","diam (mm)","APB (mm)"})
  ("pid",     {index,kp,ki,kd})        index 1..4 = moteur, 5 = yaw
  ("heartbeat", {sysid})               MAVLink seul (decouverte / rx_prefix)
"""
import math
import os
import sys

# --- Pont vers le protocole partage (firmware/stm32_bamboo/tools) --------------
# Ce module vit dans <repo>/tools/robot_control/communication/ ; le protocole
# Yahboom (ros_monitor.py) est sous <repo>/firmware/stm32_bamboo/tools.
_HERE = os.path.dirname(os.path.abspath(__file__))
_STM32_TOOLS = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "firmware", "stm32_bamboo", "tools"))
if _STM32_TOOLS not in sys.path:
    sys.path.insert(0, _STM32_TOOLS)

from ros_monitor import (  # noqa: E402  (import apres modif sys.path)
    build_frame, FrameParser, u32,
    decode_speed, decode_imu_att, decode_encoder,
    decode_pid, decode_wheel_geom, CAR_TYPE_CPR,
    FUNC_REQUEST_DATA, FUNC_CAR_TYPE, FUNC_SET_WHEEL_GEOM,
    FUNC_SET_MOTOR_PID, FUNC_SET_YAW_PID,
)

# Re-export pour RobotComSerial (getCarType applique le cpr du type de chassis).
__all__ = ["make_codec", "YahboomCodec", "MavlinkCodec", "CAR_TYPE_CPR"]

# --- Codes fonction des commandes (miroir de ros_mcp_server.py) ----------------
FUNC_MOTOR = 0x10             # [m1 m2 m3 m4] int8 signe (% PWM) : PWM brut, boucle ouverte
FUNC_MOTION = 0x12            # [parm, Vx, Vy, Vz] int16 LE : consigne vitesse (kinematics + PID carte)
FUNC_PWM_SERVO = 0x03         # [id_1based(1..4), angle(0..180)]
FUNC_PWM_SERVO_ALL = 0x04     # [s1 s2 s3 s4] 4 angles d'un coup
FUNC_ENTER_BOOTLOADER = 0xA3  # saut vers le bootloader ROM (flash sans BOOT0)
SAVE_VERIFY = 0x5F            # octet de garde : ecrit en flash si egal, sinon RAM

# --- Codes fonction auto-report (miroir de ros_monitor.py) ---------------------
REPORT_SPEED = 0x0A        # Vx,Vy,Vz + batterie
REPORT_MAG = 0x0B          # mx,my,mz champ magnetique (extension ESP32 WaveShare)
REPORT_IMU_ATT = 0x0C      # roll, pitch, yaw
REPORT_ENCODER = 0x0D      # M1..M4 comptage cumulatif


# ===========================================================================
#  Codec Yahboom (trames binaires maison) - protocole historique, DEFAUT
# ===========================================================================
class YahboomCodec:
    """Encode/decode le protocole de trames binaires [0xFF][0xFC][LEN][FUNC][...][CHK].

    Enveloppe fine autour des fonctions de ros_monitor (source unique du protocole
    v1). Le parser de flux est STATEFUL : une instance par liaison, alimentee par
    le seul thread lecteur (pas de garde de concurrence).
    """

    def __init__(self):
        self._parser = FrameParser()

    # --- RX : flux d'octets -> evenements normalises -----------------------
    def feed(self, chunk):
        events = []
        n_ok = 0
        n_bad = 0
        for func, data, good, _raw in self._parser.feed(chunk):
            if not good:
                n_bad += 1
                continue
            n_ok += 1
            ev = self._decode(func, data)
            if ev is not None:
                events.append(ev)
        return events, n_ok, n_bad

    @staticmethod
    def _decode(func, data):
        if func == REPORT_SPEED and len(data) >= 11:
            d = decode_speed(data)
            return ("speed", {"vx": d["Vx (mm/s)"], "vy": d["Vy (mm/s)"],
                              "vz": d["Vz (rad/s)"], "battery": d["Batterie (V)"],
                              "ts": u32(data, 0)})
        if func == REPORT_IMU_ATT and len(data) >= 10:
            d = decode_imu_att(data)
            return ("imu", {"roll": d["Roll (deg)"], "pitch": d["Pitch (deg)"],
                            "yaw": d["Yaw (deg)"], "ts": u32(data, 0)})
        if func == REPORT_MAG and len(data) >= 10:
            # 0x0B (ESP32) : [ts u32][mx i16][my i16][mz i16] en 0.1 uT.
            mx = int.from_bytes(data[4:6], "little", signed=True) / 10.0
            my = int.from_bytes(data[6:8], "little", signed=True) / 10.0
            mz = int.from_bytes(data[8:10], "little", signed=True) / 10.0
            heading = math.degrees(math.atan2(my, mx)) % 360.0
            return ("mag", {"mx": mx, "my": my, "mz": mz, "heading": heading,
                            "ts": u32(data, 0)})
        if func == REPORT_ENCODER and len(data) >= 20:
            d = decode_encoder(data)
            return ("encoder", {"m": [d["M%d" % (i + 1)] for i in range(4)],
                                "ts": u32(data, 0)})
        if func == FUNC_CAR_TYPE and len(data) >= 1:
            return ("car_type", {"value": data[0]})
        if func == FUNC_SET_WHEEL_GEOM and len(data) >= 6:
            return ("wheel_geom", decode_wheel_geom(data))
        if func in (FUNC_SET_MOTOR_PID, FUNC_SET_YAW_PID) and len(data) >= 7:
            pd = decode_pid(data)
            return ("pid", {"index": int(pd["index"]), "kp": pd["kp"],
                            "ki": pd["ki"], "kd": pd["kd"]})
        return None

    # --- TX : builders de trames (bytes prets a ecrire) --------------------
    @staticmethod
    def motor(m1, m2, m3, m4):
        vals = [max(-100, min(100, int(v))) for v in (m1, m2, m3, m4)]
        return build_frame(FUNC_MOTOR, bytes((v & 0xFF) for v in vals))

    @staticmethod
    def servo(sid, angle):
        return build_frame(FUNC_PWM_SERVO, bytes([int(sid), int(angle)]))

    @staticmethod
    def servo_all(a1, a2, a3, a4):
        vals = [max(0, min(180, int(round(a)))) for a in (a1, a2, a3, a4)]
        return build_frame(FUNC_PWM_SERVO_ALL, bytes(vals))

    @staticmethod
    def cmd_vel(linear_x, angular_z):
        """FUNC_MOTION : Vx (mm/s), Vy=0, Vz (mrad/s) int16 LE ; parm=0 (pas de tenue cap)."""
        def s16le(x):
            v = int(round(x)) & 0xFFFF
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        vx = max(-1000, min(1000, int(round(linear_x * 1000.0))))
        vz = max(-2000, min(2000, int(round(angular_z * 1000.0))))
        payload = bytes([0x00]) + s16le(vx) + s16le(0) + s16le(vz)
        return build_frame(FUNC_MOTION, payload)

    @staticmethod
    def set_car_type(car_type, save=True):
        verify = SAVE_VERIFY if save else 0x00
        return build_frame(FUNC_CAR_TYPE, bytes([int(car_type) & 0xFF, verify]))

    @staticmethod
    def set_wheel_geom(cpr, circ_mm, apb_mm, save=True):
        """Retourne (bytes|None, err) : circ_mm/apb_mm stockes *10 (u16)."""
        try:
            cpr_i = int(round(float(cpr)))
            circ10 = int(round(float(circ_mm) * 10.0))
            apb10 = int(round(float(apb_mm) * 10.0))
        except (TypeError, ValueError):
            return None, "valeurs non numeriques (cpr / circ_mm / apb_mm)."
        if not (0 < cpr_i <= 0xFFFF and 0 < circ10 <= 0xFFFF and 0 < apb10 <= 0xFFFF):
            return None, ("hors plage : cpr=%d circ10=%d apb10=%d "
                          "(1..65535 ; circ/APB <= 6553.5 mm)." % (cpr_i, circ10, apb10))
        verify = SAVE_VERIFY if save else 0x00
        payload = bytes([cpr_i & 0xFF, (cpr_i >> 8) & 0xFF,
                         circ10 & 0xFF, (circ10 >> 8) & 0xFF,
                         apb10 & 0xFF, (apb10 >> 8) & 0xFF, verify])
        return build_frame(FUNC_SET_WHEEL_GEOM, payload), None

    @staticmethod
    def _pid_args(motor_id, save):
        return ((int(motor_id) & 0x0F) << 4) | (0x0F if save else 0x00)

    @staticmethod
    def _pid_payload(kp, ki, kd, args):
        def u16(x):
            v = int(round(x * 1000.0)) & 0xFFFF
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        return u16(kp) + u16(ki) + u16(kd) + bytes([args & 0xFF])

    def set_motor_pid(self, kp, ki, kd, save=False, motor_id=0, disable=False):
        args = self._pid_args(motor_id, save)
        if disable and motor_id != 0:
            # Sentinelle brute : 3 mots a 0xFFFF -> la carte desactive le PID de ce
            # moteur (encodeur HS) et le cale en recopie sur son voisin.
            payload = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, args & 0xFF])
            return build_frame(FUNC_SET_MOTOR_PID, payload)
        return build_frame(FUNC_SET_MOTOR_PID, self._pid_payload(kp, ki, kd, args))

    def set_yaw_pid(self, kp, ki, kd, save=False):
        verify = SAVE_VERIFY if save else 0x00
        return build_frame(FUNC_SET_YAW_PID, self._pid_payload(kp, ki, kd, verify))

    @staticmethod
    def request_car_type():
        return build_frame(FUNC_REQUEST_DATA, bytes([FUNC_CAR_TYPE, 0x00]))

    @staticmethod
    def request_wheel_geom():
        return build_frame(FUNC_REQUEST_DATA, bytes([FUNC_SET_WHEEL_GEOM, 0x00]))

    @staticmethod
    def request_pid(index):
        func = FUNC_SET_YAW_PID if index == 5 else FUNC_SET_MOTOR_PID
        return build_frame(FUNC_REQUEST_DATA, bytes([func, int(index) & 0xFF]))

    @staticmethod
    def enter_bootloader():
        return build_frame(FUNC_ENTER_BOOTLOADER, bytes([SAVE_VERIFY]))


# ===========================================================================
#  Codec MAVLink v2 (dialecte bamboo) - protocole cible commun aux 3 cartes
# ===========================================================================
# Table de parametres : ORDRE IDENTIQUE au firmware (mav_protocol.c STM32,
# ConnectorMavlink.cpp ESP32, MavFrame.cpp Teensy) -> l'index adresse le meme
# parametre sur les 3 cartes. Necessaire pour les PARAM_REQUEST_READ par index.
_PARAM_NAMES = [
    "MOT1_KP", "MOT1_KI", "MOT1_KD",
    "MOT2_KP", "MOT2_KI", "MOT2_KD",
    "MOT3_KP", "MOT3_KI", "MOT3_KD",
    "MOT4_KP", "MOT4_KI", "MOT4_KD",
    "YAW_KP", "YAW_KI", "YAW_KD",
    "WHEEL_CPR", "WHEEL_CIRC", "WHEEL_APB",
    "CAR_TYPE",
    # Index 19, AJOUTE APRES le contrat fige 0-18 : un hote qui l'ignore lit les 19
    # premiers index a l'identique. Seuil de severite du journal de la carte
    # (MAV_SEVERITY 0..7), ecrit par set_log_level ci-dessous. Volontairement en
    # ECRITURE SEULE cote hote : l'echo PARAM_VALUE "LOG_LEVEL" n'appartient a aucun
    # groupe de reassemblage, donc _on_param le laisse tomber -- c'est voulu, la valeur
    # qui fait foi est celle que l'hote vient d'ecrire.
    "LOG_LEVEL",
]
_IDX = {name: i for i, name in enumerate(_PARAM_NAMES)}


class MavlinkCodec:
    """Encode/decode MAVLink v2 (dialecte bamboo), miroir hote du firmware.

    Le protocole PARAM remplace les get/set de PID / geometrie / type de chassis :
    une lecture emet plusieurs PARAM_REQUEST_READ et le codec REASSEMBLE les
    PARAM_VALUE en un evenement normalise ("pid"/"wheel_geom"/"car_type") quand le
    groupe est complet -> le seam getPid/getWheelGeom de RobotComSerial est inchange.

    Adressage : hote sysid=255, compid=190 (MAV_COMP_ID_MISSIONPLANNER, cote GCS).
    La cible (target_system/component) est laissee a 0 (broadcast) : sur une liaison
    serie point a point, le firmware route par msgid sans filtrer la cible.
    """

    def __init__(self, target_system=0, target_component=0):
        try:
            from .mav import bamboo as _b            # importe en tant que package
        except ImportError:                          # importe a plat (dossier sur sys.path)
            from mav import bamboo as _b
        self._b = _b
        # Deux instances : parsing RX (thread lecteur) et packing TX (threads
        # appelants) ne partagent aucun etat mutable (seq/buffer disjoints).
        self._rx = _b.MAVLink(None, srcSystem=255, srcComponent=190)
        self._rx.robust_parsing = True               # tolerance au bruit serie
        self._tx = _b.MAVLink(None, srcSystem=255, srcComponent=190)
        self._tgt_sys = target_system
        self._tgt_comp = target_component
        # Accumulateurs de groupes PARAM (reassemblage des reponses fragmentees).
        self._pid_acc = {}       # index 1..5 -> {"KP","KI","KD"}
        self._geom_acc = {}      # {"cpr","circ","apb"}

    # --- RX : flux d'octets -> evenements normalises -----------------------
    def feed(self, chunk):
        events = []
        n_ok = 0
        msgs = self._rx.parse_buffer(chunk)
        if msgs:
            for m in msgs:
                n_ok += 1
                events.extend(self._decode(m))
        # robust_parsing laisse tomber le bruit silencieusement -> pas de compteur bad.
        return events, n_ok, 0

    def _decode(self, m):
        t = m.get_type()
        if t == "HEARTBEAT":
            return [("heartbeat", {"sysid": m.get_srcSystem()})]
        if t == "ATTITUDE":
            return [("imu", {"roll": math.degrees(m.roll),
                             "pitch": math.degrees(m.pitch),
                             "yaw": math.degrees(m.yaw), "ts": m.time_boot_ms})]
        # SCALED_IMU (#26, accel/gyro bruts) : plus emis par le GrovePi (gyro non
        # calibre -> bruite, jamais consomme). Le firmware n'envoie que ATTITUDE ;
        # on n'a donc plus de branche imu_raw ici. Un SCALED_IMU inattendu est ignore.
        if t == "DISTANCE_SENSOR":
            # Carte capteurs GrovePi : id 0..3 = HC-SR04 (ultrason), id 4 = Sharp IR.
            # current_distance en cm ; 0 = pas de mesure (DIST_NONE cote firmware).
            mm = None if m.current_distance == 0 else m.current_distance * 10
            kind = "ir" if m.id == 4 else "ultra"
            return [(kind, {"id": m.id, "mm": mm, "ts": m.time_boot_ms})]
        if t == "SYS_STATUS":
            return [("battery", {"battery": m.voltage_battery / 1000.0})]
        if t == "BAMBOO_WHEEL_STATE":
            # vx/vy en m/s -> mm/s (convention snapshot) ; wz en rad/s. Batterie via
            # SYS_STATUS separe (None ici pour ne pas ecraser).
            return [("speed", {"vx": m.vx * 1000.0, "vy": m.vy * 1000.0,
                               "vz": m.wz, "battery": None, "ts": m.time_boot_ms})]
        if t == "BAMBOO_ENCODERS":
            return [("encoder", {"m": [int(c) for c in m.counts], "ts": m.time_boot_ms})]
        if t == "BAMBOO_MOTOR_RPM":
            # Les deux nombres que le PID embarque compare, en RPM et sans retouche : c'est
            # tout l'interet du message (cf. sa description dans bamboo.xml). Les recopies
            # M3/M4 du RPM mesure sont laissees telles quelles, le consommateur doit savoir
            # que cette carte n'a que deux encodeurs.
            return [("motor_rpm", {"rpm": [float(v) for v in m.rpm],
                                   "req": [float(v) for v in m.rpm_req],
                                   "ts": m.time_boot_ms})]
        if t == "BAMBOO_MAG":
            heading = math.degrees(math.atan2(m.my, m.mx)) % 360.0
            return [("mag", {"mx": m.mx, "my": m.my, "mz": m.mz,
                             "heading": heading, "ts": m.time_boot_ms})]
        if t == "STATUSTEXT":
            # Journal de la carte. severity = MAV_SEVERITY (0 EMERGENCY .. 7 DEBUG), donc
            # directement transposable en niveaux ROS par le driver. Le texte arrive en
            # bytes ou en str selon la version de pymavlink, et peut etre complete de zeros.
            txt = m.text
            if isinstance(txt, (bytes, bytearray)):
                txt = txt.decode("utf-8", "replace")
            return [("log", {"severity": int(m.severity),
                             "text": txt.rstrip("\x00").strip()})]
        if t == "PARAM_VALUE":
            return self._on_param(m.param_id, m.param_value)
        return []

    def _on_param(self, name, value):
        # PID moteur : MOTn_Kx (n=1..4).
        if len(name) >= 7 and name[:3] == "MOT" and name[3] in "1234" and name[4] == "_":
            idx = int(name[3])
            key = name[5:]                      # KP / KI / KD
            acc = self._pid_acc.setdefault(idx, {})
            acc[key] = value
            return self._flush_pid(idx, acc)
        if name in ("YAW_KP", "YAW_KI", "YAW_KD"):
            acc = self._pid_acc.setdefault(5, {})
            acc[name[4:]] = value               # KP / KI / KD apres "YAW_"
            return self._flush_pid(5, acc)
        if name == "WHEEL_CPR":
            self._geom_acc["cpr"] = value
            return self._flush_geom()
        if name == "WHEEL_CIRC":
            self._geom_acc["circ"] = value
            return self._flush_geom()
        if name == "WHEEL_APB":
            self._geom_acc["apb"] = value
            return self._flush_geom()
        if name == "CAR_TYPE":
            return [("car_type", {"value": int(round(value))})]
        return []

    def _flush_pid(self, idx, acc):
        if all(k in acc for k in ("KP", "KI", "KD")):
            ev = ("pid", {"index": idx, "kp": acc["KP"], "ki": acc["KI"], "kd": acc["KD"]})
            self._pid_acc[idx] = {}             # pret pour la lecture suivante
            return [ev]
        return []

    def _flush_geom(self):
        g = self._geom_acc
        if all(k in g for k in ("cpr", "circ", "apb")):
            circ = g["circ"]
            ev = ("wheel_geom", {"cpr (tics/tour)": g["cpr"], "circ (mm)": circ,
                                 "diam (mm)": circ / math.pi, "APB (mm)": g["apb"]})
            self._geom_acc = {}
            return [ev]
        return []

    # --- packing bas niveau ------------------------------------------------
    def _pack(self, msg):
        buf = msg.pack(self._tx)
        self._tx.seq = (self._tx.seq + 1) & 0xFF    # seq non auto-incremente hors send()
        return buf

    def _param_set(self, name, value):
        msg = self._tx.param_set_encode(self._tgt_sys, self._tgt_comp,
                                        name.encode("ascii"), float(value),
                                        self._b.MAV_PARAM_TYPE_REAL32)
        return self._pack(msg)

    def _param_req(self, index):
        msg = self._tx.param_request_read_encode(self._tgt_sys, self._tgt_comp,
                                                 b"", int(index))
        return self._pack(msg)

    def _cmd(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        msg = self._tx.command_long_encode(self._tgt_sys, self._tgt_comp, command,
                                           0, p1, p2, p3, p4, p5, p6, p7)
        return self._pack(msg)

    def _storage(self):
        # MAV_CMD_PREFLIGHT_STORAGE param1=1 : commit flash (STM32 ; ESP32/Teensy
        # repondent UNSUPPORTED sans persistance -> inoffensif).
        return self._cmd(self._b.MAV_CMD_PREFLIGHT_STORAGE, p1=1.0)

    # --- TX : builders (bytes prets a ecrire) ------------------------------
    def cmd_vel(self, linear_x, angular_z):
        # BAMBOO_CMD_VEL : vx/vy m/s, wz rad/s (SI, pas de mise a l'echelle).
        return self._pack(self._tx.bamboo_cmd_vel_encode(float(linear_x), 0.0,
                                                         float(angular_z)))

    def motor(self, m1, m2, m3, m4):
        pwm = [max(-100, min(100, int(v))) for v in (m1, m2, m3, m4)]
        return self._pack(self._tx.bamboo_motor_pwm_encode(pwm))

    def servo(self, sid, angle):
        # MAV_CMD_DO_SET_SERVO : param1 = numero de servo (1-based), param2 = angle.
        return self._cmd(self._b.MAV_CMD_DO_SET_SERVO, p1=float(int(sid)),
                         p2=float(int(angle)))

    def servo_all(self, a1, a2, a3, a4):
        out = b""
        for i, a in enumerate((a1, a2, a3, a4)):
            out += self.servo(i + 1, max(0, min(180, int(round(a)))))
        return out

    def set_car_type(self, car_type, save=True):
        out = self._param_set("CAR_TYPE", int(car_type))
        if save:
            out += self._storage()
        return out

    def set_wheel_geom(self, cpr, circ_mm, apb_mm, save=True):
        """Retourne (bytes|None, err) : params WHEEL_* en float32 (mm bruts, pas *10)."""
        try:
            cpr_f, circ_f, apb_f = float(cpr), float(circ_mm), float(apb_mm)
        except (TypeError, ValueError):
            return None, "valeurs non numeriques (cpr / circ_mm / apb_mm)."
        out = (self._param_set("WHEEL_CPR", cpr_f)
               + self._param_set("WHEEL_CIRC", circ_f)
               + self._param_set("WHEEL_APB", apb_f))
        if save:
            out += self._storage()
        return out, None

    def set_log_level(self, level):
        """Seuil de journal de la carte (PARAM LOG_LEVEL, idx 19). Retourne (bytes, err).

        Le firmware filtre A LA SOURCE (`severity > logLevel_` -> rien n'est emis) : ce
        reglage economise donc de la bande passante sur l'UART, partage avec la
        telemetrie -- il ne fait pas que masquer des lignes cote hote.
        """
        try:
            lv = int(level)
        except (TypeError, ValueError):
            return None, "niveau non entier."
        if not 0 <= lv <= 7:
            return None, "hors plage : %d (MAV_SEVERITY 0 EMERGENCY .. 7 DEBUG)." % lv
        return self._param_set("LOG_LEVEL", float(lv)), None

    def set_motor_pid(self, kp, ki, kd, save=False, motor_id=0, disable=False):
        # `disable` (recopie sur le voisin) est une specificite Yahboom/STM32 non
        # portee par le protocole PARAM : on ecrit les gains normalement.
        if motor_id == 0:
            targets = range(1, 5)               # les 4 moteurs
        else:
            targets = (int(motor_id),)
        out = b""
        for n in targets:
            out += (self._param_set("MOT%d_KP" % n, kp)
                    + self._param_set("MOT%d_KI" % n, ki)
                    + self._param_set("MOT%d_KD" % n, kd))
        if save:
            out += self._storage()
        return out

    def set_yaw_pid(self, kp, ki, kd, save=False):
        out = (self._param_set("YAW_KP", kp)
               + self._param_set("YAW_KI", ki)
               + self._param_set("YAW_KD", kd))
        if save:
            out += self._storage()
        return out

    def request_car_type(self):
        return self._param_req(_IDX["CAR_TYPE"])

    def request_wheel_geom(self):
        return (self._param_req(_IDX["WHEEL_CPR"])
                + self._param_req(_IDX["WHEEL_CIRC"])
                + self._param_req(_IDX["WHEEL_APB"]))

    def request_pid(self, index):
        if index == 5:
            base = _IDX["YAW_KP"]
        else:
            base = (int(index) - 1) * 3         # MOTn_KP a l'index (n-1)*3
        return self._param_req(base) + self._param_req(base + 1) + self._param_req(base + 2)

    def enter_bootloader(self):
        # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=3 : saut bootloader (STM32).
        return self._cmd(self._b.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, p1=3.0)


def make_codec(protocol="yahboom", **kwargs):
    """Fabrique le codec du protocole demande ("yahboom" par defaut, ou "mavlink")."""
    p = (protocol or "yahboom").lower()
    if p in ("yahboom", "yb", "v1", "serial_frame"):
        return YahboomCodec()
    if p in ("mavlink", "mav", "v2"):
        return MavlinkCodec(**kwargs)
    raise ValueError("protocole inconnu : %r (attendu 'yahboom' ou 'mavlink')" % protocol)

r"""board_link - Liaison serie avec la carte STM32 du Bamboo v4.

Ouvre COM4 (@115200), lit en continu les trames auto-report (thread lecteur) et
expose l'envoi des trames de commande (moteurs, servos). Le protocole (trame
[0xFF][0xFC][LEN][FUNC][...][CHK]) et ses decodeurs sont REUTILISES tels quels
depuis firmware/stm32_bamboo/tools/ros_monitor.py (source unique du protocole).

Modele : la classe Board de ros_mcp_server.py (thread lecteur + verrou). Les
ECRITURES se font dans le thread appelant, les LECTURES dans le thread lecteur :
pas de conflit read/write sur le handle (idem serveur MCP).

/!\ Un seul programme peut tenir COM4 a la fois : couper le serveur MCP avant.
"""
import os
import sys
import threading
import time

import serial
from serial.tools import list_ports

# --- Pont vers le protocole partage (firmware/stm32_bamboo/tools) --------------
# board_link.py est dans <repo>/tools/robot_control/old/ (version figee) ; le
# protocole STM32 vit sous <repo>/firmware/stm32_bamboo/tools -> ../../../firmware/...
_HERE = os.path.dirname(os.path.abspath(__file__))
_STM32_TOOLS = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "firmware", "stm32_bamboo", "tools"))
if _STM32_TOOLS not in sys.path:
    sys.path.insert(0, _STM32_TOOLS)

from ros_monitor import (  # noqa: E402  (import apres modif sys.path)
    build_frame, FrameParser,
    decode_speed, decode_imu_att, decode_encoder,
)

# Codes fonction des commandes (miroir de ros_mcp_server.py)
FUNC_MOTOR = 0x10          # [m1 m2 m3 m4] int8 signe (% PWM)
FUNC_PWM_SERVO = 0x03      # [id_1based(1..4), angle(0..180)]

# Codes fonction auto-report (miroir de ros_monitor.py)
REPORT_SPEED = 0x0A        # Vx,Vy,Vz + batterie
REPORT_IMU_ATT = 0x0C      # roll, pitch, yaw
REPORT_ENCODER = 0x0D      # M1..M4 comptage cumulatif


class BoardLink:
    """Liaison serie non bloquante avec la carte. Reconnexion automatique."""

    def __init__(self, port="COM4", baud=115200, telemetry=None):
        self.port = port
        self.baud = baud
        self.tel = telemetry
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

        self._stop = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
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

    def _read_loop(self):
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
                self._close_ser()
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
            if self.tel:
                self.tel.log_rx("encoder", m=list(self.encoders))

    def _close_ser(self):
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
            self._close_ser()
            if self.tel:
                self.tel.log("event", msg="com_lost", err=str(e))
            return False

    def send_motor(self, m1, m2, m3, m4):
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
            ok = self.send_motor(0, 0, 0, 0) or ok
            time.sleep(0.005)
        return ok

    def send_servo(self, sid, angle):
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

    def close(self):
        self._stop = True
        try:
            self.stop()
        except Exception:
            pass
        self._close_ser()
        if self.tel:
            self.tel.log("event", msg="com_close", port=self.port)

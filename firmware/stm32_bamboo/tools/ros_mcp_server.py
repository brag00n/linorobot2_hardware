#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_mcp_server.py - Serveur MCP (stdio) exposant les metriques LIVE de la carte
                    YB-ERF01-V3.0 (Yahboom STM32 ROS) a un client MCP.

Zero dependance hors pyserial : le protocole MCP (JSON-RPC 2.0 sur stdin/stdout,
messages delimites par des sauts de ligne) est implemente directement ici.

Le serveur ouvre le port serie EN CONTINU (proprietaire unique du port) dans un
thread lecteur, maintient un etat a jour (dernieres valeurs decodees, comptages
encodeur cumulatifs, frequences), et se reconnecte tout seul si le port est
occupe au demarrage. Les outils MCP se contentent de lire cet etat.

  -> Fermez tout autre moniteur serie (VSCode/PlatformIO, ros_monitor.py) :
     un seul programme peut tenir COM4 a la fois.

Enregistrement cote client (.mcp.json a la racine du projet) :
    {
      "mcpServers": {
        "bambou-board": {
          "command": "C:\\\\Users\\\\...\\\\python.exe",
          "args": ["tools/ros_mcp_server.py"],
          "env": {"BAMBOU_PORT": "COM4", "BAMBOU_BAUD": "115200", "BAMBOU_CPR": "1320"}
        }
      }
    }

Variables d'environnement : BAMBOU_PORT (defaut COM4), BAMBOU_BAUD (115200),
BAMBOU_CPR (1320 = tics/tour pour la conversion tr/min).

Test manuel (sans client MCP) :
    python tools/ros_mcp_server.py selftest      # ouvre le port, affiche l'etat, quitte
"""

import json
import os
import subprocess
import sys
import threading
import time
from collections import defaultdict, deque

try:
    import serial
except ImportError:
    sys.exit("pyserial manquant (lancez avec le Python de PlatformIO).")

# Reutilise les decodeurs de l'outil (meme dossier tools/, ajoute a sys.path[0]).
from ros_monitor import (CAR_TYPE_CPR, FUNC_CAR_TYPE, FUNC_REQUEST_DATA,
                         FrameParser, build_frame, decode_encoder,
                         decode_icm_raw, decode_imu_att, decode_speed)

FUNC_MOTOR = 0x10                     # pilotage PWM direct : [m1 m2 m3 m4] int8 (%)
FUNC_PWM_SERVO = 0x03                 # servo PWM : [id_1based(1..4) angle(0..180)]
FUNC_PWM_SERVO_ALL = 0x04             # 4 servos d'un coup : [s1 s2 s3 s4] angles
FUNC_ENTER_BOOTLOADER = 0xA3          # saut vers le bootloader ROM (flash sans BOOT0)
SAVE_VERIFY = 0x5F                    # octet de garde (evite tout declenchement fortuit)

SERVER_NAME = "bambou-board"
SERVER_VERSION = "1.0.0"
DEFAULT_PROTOCOL = "2025-06-18"


def log(*a):
    """Journaux vers stderr uniquement (stdout est reserve au JSON-RPC)."""
    print("[bambou-mcp]", *a, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# Etat de la carte, alimente par un thread lecteur du port serie
# ---------------------------------------------------------------------------
class Board:
    def __init__(self, port, baud, cpr):
        self.port = port
        self.baud = baud
        self.cpr = cpr
        self.ser = None
        self.lock = threading.RLock()
        self.latest = {}                              # func -> (dict, t)
        self.times = defaultdict(lambda: deque(maxlen=400))  # func -> timestamps
        self.ok = 0
        self.bad = 0
        self.car_type = None
        self.car_type_t = 0.0
        self.enc_hist = deque(maxlen=600)             # (t, [M1..M4])
        self.baseline = None                          # [M1..M4] de reference
        self.start_t = time.time()
        self.last_err = None
        self._release = False                         # True => libere COM (pour pio upload)
        threading.Thread(target=self._reader, daemon=True).start()

    # --- thread lecteur -----------------------------------------------------
    def _open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.last_err = None
            log(f"port {self.port} @ {self.baud} ouvert")
        except Exception as e:                        # port occupe / absent
            self.ser = None
            self.last_err = str(e)

    def _reader(self):
        parser = FrameParser()
        while True:
            if self._release:                         # laisse COM libre pour pio upload
                if self.ser is not None:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.3)
                continue
            if self.ser is None:
                self._open()
                if self.ser is None:
                    time.sleep(2.0)                   # reessai reconnexion
                    continue
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.last_err = str(e)
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                continue
            if not chunk:
                continue
            for func, data, ok, raw in parser.feed(chunk):
                t = time.time()
                with self.lock:
                    if not ok:
                        self.bad += 1
                        continue
                    self.ok += 1
                    self.times[func].append(t)
                    if func == 0x0A and len(data) >= 7:
                        self.latest[func] = (decode_speed(data), t)
                    elif func == 0x0C and len(data) >= 6:
                        self.latest[func] = (decode_imu_att(data), t)
                    elif func == 0x0E and len(data) >= 18:
                        self.latest[func] = (decode_icm_raw(data), t)
                    elif func == 0x0D and len(data) >= 16:
                        d = decode_encoder(data)
                        self.latest[func] = (d, t)
                        self.enc_hist.append((t, [d[f"M{i + 1}"] for i in range(4)]))
                    elif func == FUNC_CAR_TYPE and len(data) >= 1:
                        self.car_type = data[0]
                        self.car_type_t = t

    # --- calculs derives ----------------------------------------------------
    def hz(self, func, window=5.0):
        now = time.time()
        ts = [x for x in self.times[func] if now - x <= window]
        if len(ts) < 2:
            return 0.0
        span = ts[-1] - ts[0]
        return (len(ts) - 1) / span if span > 1e-6 else 0.0

    def enc_speed(self, window=1.0):
        """Vitesse instantanee par moteur (tics/s) sur la fenetre glissante."""
        now = time.time()
        pts = [p for p in self.enc_hist if now - p[0] <= window]
        if len(pts) < 2:
            return None
        dt = pts[-1][0] - pts[0][0]
        if dt < 1e-3:
            return None
        return [(pts[-1][1][i] - pts[0][1][i]) / dt for i in range(4)]

    def detect_car_type(self, timeout=1.5):
        """Envoie la requete FUNC_REQUEST_DATA(0x15) et attend la reponse 0x15."""
        if self.ser is None:
            return None
        with self.lock:
            base_t = self.car_type_t
        try:
            self.ser.write(build_frame(FUNC_REQUEST_DATA, bytes([FUNC_CAR_TYPE, 0x00])))
        except Exception as e:
            self.last_err = str(e)
            return None
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if self.car_type is not None and self.car_type_t > base_t:
                    return self.car_type
            time.sleep(0.05)
        return None

    def encoder_counts(self):
        with self.lock:
            v = self.latest.get(0x0D)
            return ([v[0][f"M{i + 1}"] for i in range(4)], v[1]) if v else (None, None)


# ---------------------------------------------------------------------------
# Outils MCP : (board, arguments) -> texte lisible
# ---------------------------------------------------------------------------
def t_status(board, args):
    with board.lock:
        conn = board.ser is not None
        up = time.time() - board.start_t
        lines = [f"Port {board.port} @ {board.baud} : "
                 f"{'CONNECTE' if conn else 'DECONNECTE'}"]
        if board.last_err:
            lines.append(f"  derniere erreur : {board.last_err}")
        lines.append(f"  uptime {up:.0f}s   trames OK={board.ok}  KO={board.bad}")
        for f, name in ((0x0A, "vitesse"), (0x0C, "attitude"),
                        (0x0E, "imu_brut"), (0x0D, "encodeurs")):
            lines.append(f"  0x{f:02X} {name:<9}: {board.hz(f):5.1f} Hz")
        lines.append(f"  cpr={board.cpr:g}   car_type="
                     f"{('0x%02X' % board.car_type) if board.car_type is not None else '?'}")
    return "\n".join(lines)


def t_metrics(board, args):
    seconds = float(args.get("seconds") or 0)          # 0 = derniere valeur
    with board.lock:
        snap = {f: (v[0], v[1]) for f, v in board.latest.items()}
    lines = []
    v = snap.get(0x0A)
    if v:
        d = v[0]
        lines.append("Vitesse & batterie : "
                     + "  ".join(f"{k}={d[k]:.2f}" for k in d))
    v = snap.get(0x0C)
    if v:
        d = v[0]
        lines.append("Attitude (deg)     : "
                     + "  ".join(f"{k.split()[0]}={d[k]:+.1f}" for k in d))
    v = snap.get(0x0E)
    if v:
        d = v[0]
        g = "  ".join(f"{a}={d['gyro_' + a]:+.3f}" for a in "xyz")
        ac = "  ".join(f"{a}={d['accel_' + a]:+.3f}" for a in "xyz")
        lines.append(f"Gyro (rad/s)       : {g}")
        lines.append(f"Accel (m/s2)       : {ac}")
    v = snap.get(0x0D)
    if v:
        d = v[0]
        lines.append("Encodeurs (tics)   : "
                     + "  ".join(f"M{i + 1}={d[f'M{i + 1}']:.0f}" for i in range(4)))
    sp = board.enc_speed()
    if sp:
        lines.append("Vitesse moteurs    : "
                     + "  ".join(f"M{i + 1}={sp[i]:+.0f}t/s"
                                 f"({sp[i] * 60.0 / board.cpr:+.1f}tr/min)"
                                 for i in range(4)))
    if not lines:
        return "Aucune metrique recue pour l'instant (carte connectee ?)."
    return "\n".join(lines)


def t_encoders(board, args):
    counts, t = board.encoder_counts()
    if counts is None:
        return "Aucune trame encodeur recue."
    lines = ["Comptage cumulatif (tics) : "
             + "  ".join(f"M{i + 1}={counts[i]:.0f}" for i in range(4))]
    sp = board.enc_speed()
    if sp:
        lines.append("Vitesse instantanee       : "
                     + "  ".join(f"M{i + 1}={sp[i]:+.1f} t/s "
                                 f"({sp[i] * 60.0 / board.cpr:+.1f} tr/min)"
                                 for i in range(4)))
    else:
        lines.append("Vitesse instantanee       : (immobile / trop peu d'echantillons)")
    return "\n".join(lines)


def t_car_type(board, args):
    ct = board.detect_car_type()
    if ct is None:
        return "Pas de reponse de la carte pour le type de chassis."
    cpr, label = CAR_TYPE_CPR.get(ct, (None, "type inconnu"))
    if cpr:
        board.cpr = cpr
    return (f"car_type=0x{ct:02X} ({label})"
            + (f"  =>  cpr={cpr:g} tics/tour (applique)" if cpr else
               "  (non repertorie ; utilisez la calibration)"))


def t_calibrate_baseline(board, args):
    counts, t = board.encoder_counts()
    if counts is None:
        return "Aucune trame encodeur : impossible de fixer la baseline."
    with board.lock:
        board.baseline = counts
    return ("Baseline fixee : "
            + "  ".join(f"M{i + 1}={counts[i]:.0f}" for i in range(4))
            + "\nTournez maintenant la roue du nombre de tours voulu, "
              "puis appelez calibrate_read.")


def t_calibrate_read(board, args):
    turns = float(args.get("turns") or 1.0)
    if turns <= 0:
        return "turns doit etre > 0."
    with board.lock:
        base = board.baseline
    if base is None:
        return "Appelez d'abord calibrate_baseline."
    counts, t = board.encoder_counts()
    if counts is None:
        return "Aucune trame encodeur."
    lines = [f"Delta depuis la baseline, sur {turns:g} tour(s) :"]
    for i in range(4):
        dl = counts[i] - base[i]
        sens = "+ (A avance B)" if dl >= 0 else "- (B avance A)"
        cpr_mes = abs(dl) / turns
        lines.append(f"  M{i + 1} : {dl:+.0f} tics  =>  {cpr_mes:8.1f} tics/tour"
                     f"   sens {sens}")
    lines.append("Table firmware de reference : 1320 / 2464 / 1040 / 836.")
    return "\n".join(lines)


def t_motor_drive(board, args):
    """Pilote UN moteur a faible PWM un court instant et mesure le delta encodeur.
    L'ecriture se fait dans le thread serveur, la lecture dans le thread lecteur :
    pas de conflit read/write sur le meme handle (contrairement a un script mono-thread).
    Bornes de securite : |PWM| <= 50 %, duree <= 2 s. STOP toujours envoye a la fin.
    -> Roues surelevees imperativement.
    """
    if board.ser is None:
        return "Port non connecte : impossible de piloter."
    try:
        motor = int(args.get("motor") or 0)
    except (TypeError, ValueError):
        motor = 0
    if motor not in (1, 2, 3, 4):
        return "motor doit valoir 1, 2, 3 ou 4."
    try:
        pwm = int(args.get("pwm") or 0)
    except (TypeError, ValueError):
        pwm = 0
    pwm = max(-50, min(50, pwm))                       # securite : +-50 % max
    seconds = float(args.get("seconds") or 1.0)
    seconds = max(0.1, min(2.0, seconds))              # securite : 2 s max

    base, _ = board.encoder_counts()
    if base is None:
        return "Aucune trame encodeur : mesure impossible."
    base = [int(round(x)) for x in base]

    vals = [0, 0, 0, 0]
    vals[motor - 1] = pwm
    frame = build_frame(FUNC_MOTOR, bytes((v & 0xFF) for v in vals))
    stop = build_frame(FUNC_MOTOR, bytes([0, 0, 0, 0]))

    peak = list(base)
    try:
        t0 = time.time()
        while time.time() - t0 < seconds:
            board.ser.write(frame)                     # repete (anti-watchdog firmware)
            time.sleep(0.05)
            cur, _ = board.encoder_counts()
            if cur:
                for i in range(4):
                    ci = int(round(cur[i]))
                    if abs(ci - base[i]) > abs(peak[i] - base[i]):
                        peak[i] = ci
    finally:
        for _ in range(5):                             # STOP franc, toujours execute
            try:
                board.ser.write(stop)
            except Exception:
                pass
            time.sleep(0.02)

    time.sleep(0.3)
    final, _ = board.encoder_counts()
    final = [int(round(x)) for x in final] if final else peak

    lines = [f"Pilotage M{motor} a {pwm} % pendant {seconds:g}s :"]
    for i in range(4):
        dl = final[i] - base[i]
        pk = peak[i] - base[i]
        tag = "   <-- pilote" if i == motor - 1 else ""
        lines.append(f"  M{i + 1} : delta {dl:+d}  (pic {pk:+d}){tag}")
    dd = final[motor - 1] - base[motor - 1]
    lines.append("")
    if abs(dd) > 20:
        lines.append(f"=> M{motor} a compte {dd:+d} tics sous tension : "
                     f"ENCODEUR FONCTIONNEL.")
    else:
        lines.append(f"=> M{motor} n'a quasiment pas compte ({dd:+d}) : "
                     f"roue bloquee, ou defaut de la chaine encodeur.")
    return "\n".join(lines)


def t_motor_drive_all(board, args):
    """Pilote les QUATRE moteurs simultanement avec controle INDEPENDANT
    de vitesse et de sens de chacun (PWM signe par moteur).
    Deux modes d'appel :
      - explicite : m1, m2, m3, m4 (chacun -40..40 %, defaut 0) ;
      - raccourci : pwm (magnitude) applique au vecteur marche avant du chassis
        M1+ M2- M3- M4+ (option reverse) si AUCUN m1..m4 n'est fourni.
    Bornes de securite : |PWM| <= 40 % par moteur, duree <= 5 s.
    STOP toujours envoye a la fin. -> Roues surelevees imperativement.
    """
    if board.ser is None:
        return "Port non connecte : impossible de piloter."

    def _clip(x):
        try:
            return max(-40, min(40, int(x)))           # securite : +-40 % par moteur
        except (TypeError, ValueError):
            return 0

    seconds = float(args.get("seconds") or 3.0)
    seconds = max(0.1, min(5.0, seconds))              # securite : 5 s max

    per_motor = any(k in args and args.get(k) is not None
                    for k in ("m1", "m2", "m3", "m4"))
    if per_motor:                                      # controle independant explicite
        vals = [_clip(args.get(f"m{i + 1}") or 0) for i in range(4)]
    else:                                              # raccourci vecteur marche avant
        try:
            pwm = abs(int(args.get("pwm") or 15))
        except (TypeError, ValueError):
            pwm = 15
        pwm = min(40, pwm)
        fwd = [1, -1, -1, 1]                           # M1+ M2- M3- M4+
        sgn = -1 if bool(args.get("reverse")) else 1
        vals = [sgn * s * pwm for s in fwd]

    base, _ = board.encoder_counts()
    if base is None:
        return "Aucune trame encodeur : mesure impossible."
    base = [int(round(x)) for x in base]

    frame = build_frame(FUNC_MOTOR, bytes((v & 0xFF) for v in vals))
    stop = build_frame(FUNC_MOTOR, bytes([0, 0, 0, 0]))

    peak = list(base)
    try:
        t0 = time.time()
        while time.time() - t0 < seconds:
            board.ser.write(frame)                     # repete (anti-watchdog firmware)
            time.sleep(0.05)
            cur, _ = board.encoder_counts()
            if cur:
                for i in range(4):
                    ci = int(round(cur[i]))
                    if abs(ci - base[i]) > abs(peak[i] - base[i]):
                        peak[i] = ci
    finally:
        for _ in range(5):                             # STOP franc, toujours execute
            try:
                board.ser.write(stop)
            except Exception:
                pass
            time.sleep(0.02)

    time.sleep(0.3)
    final, _ = board.encoder_counts()
    final = [int(round(x)) for x in final] if final else peak

    lines = [f"Pilotage simultane des 4 moteurs pendant {seconds:g}s :",
             f"  commandes envoyees : M1={vals[0]:+d} M2={vals[1]:+d} "
             f"M3={vals[2]:+d} M4={vals[3]:+d} (%)"]
    for i in range(4):
        dl = final[i] - base[i]
        pk = peak[i] - base[i]
        note = "  (encodeur HS)" if i == 1 else ""     # M2 = voie encodeur morte
        lines.append(f"  M{i + 1} : delta {dl:+d}  (pic {pk:+d}){note}")
    return "\n".join(lines)


def t_enter_bootloader(board, args):
    """Fait sauter la carte dans son bootloader ROM (flash sans BOOT0+RESET) puis
    LIBERE le port COM pour que `pio run -t upload` puisse le prendre.
    Necessite le firmware avec le handler FUNC_ENTER_BOOTLOADER (0xA3)."""
    if board.ser is None:
        return ("Port non connecte : impossible d'envoyer la commande. "
                "Si le serveur MCP a deja libere le port, reconnecte-le (/mcp).")

    frame = build_frame(FUNC_ENTER_BOOTLOADER, bytes([SAVE_VERIFY]))
    sent = False
    try:
        for _ in range(3):                            # quelques envois : robuste au bruit
            board.ser.write(frame)
            sent = True
            time.sleep(0.05)
    except Exception as e:
        board.last_err = str(e)

    time.sleep(0.2)
    board._release = True                             # libere COM4 pour l'upload
    time.sleep(0.5)                                   # laisse le reader fermer le handle

    if not sent:
        return "Echec d'envoi de la commande bootloader (write KO)."
    return ("\n".join([
        "Commande bootloader envoyee (FUNC 0xA3). La carte a du redemarrer dans son",
        "bootloader ROM et le port COM est maintenant LIBERE par le serveur MCP.",
        "",
        "Etape suivante (dans un terminal) :",
        "    pio run -t upload",
        "",
        "Apres le flash, la carte redemarre sur le nouveau firmware. Reconnecte",
        "ensuite le serveur MCP (/mcp) pour reprendre la main sur le port.",
        "",
        "Si l'upload dit que COM4 est occupe : le firmware actuel n'a pas encore le",
        "handler 0xA3 (il faut l'installer une premiere fois via BOOT0+RESET).",
    ]))


def t_flash_firmware(board, args):
    """Met a jour l'application par UART (IAP) sans BOOT0 ni RESET : libere le
    port, lance tools/iap_flash.py (qui envoie 0xA3 -> bootloader, puis
    ERASE/WRITE/VERIFY/GO), puis rouvre le port. Necessite le bootloader
    resident installe une premiere fois (voir README)."""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    flasher = os.path.join(here, "iap_flash.py")
    binpath = args.get("bin") or os.path.join(
        root, ".pio", "build", "genericSTM32F103RC", "firmware.bin")
    if not os.path.isfile(flasher):
        return f"outil introuvable : {flasher}"
    if not os.path.isfile(binpath):
        return (f"image introuvable : {binpath}\n"
                "Compile d'abord l'application avec `pio run`.")

    # Libere COM pour le sous-processus, laisse le thread lecteur fermer le handle.
    board._release = True
    time.sleep(0.6)

    cmd = [sys.executable, flasher, "--port", board.port,
           "--baud", str(board.baud), "--bin", binpath]
    if args.get("no_enter"):
        cmd.append("--no-enter")                       # deja dans le bootloader
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        out = (proc.stdout or "") + (proc.stderr or "")
        ok = proc.returncode == 0
    except subprocess.TimeoutExpired:
        out, ok = "iap_flash.py : timeout (180 s).", False
    except Exception as e:
        out, ok = f"echec lancement iap_flash.py : {e}", False
    finally:
        board._release = False                         # le MCP reprend le port

    head = "Flash IAP OK." if ok else "Flash IAP ECHEC."
    return head + "\n\n" + out.strip()


def t_pwm_servo(board, args):
    """Positionne un servo PWM (SG90) en angle absolu, ou balaie pour un test.

    Cablage carte : S1 = id 1 (PC3), S2 = id 2 (PC2), S3 = id 3 (PC1), S4 = id 4 (PC0).
    Trame appli FUNC_PWM_SERVO (0x03) = [id_1based, angle]. Le firmware borne
    l'angle a 0..180 deg (500..2480 us a 50 Hz). Un SG90 se cale en ~0,3 s.

    Args : id (1..4), angle (0..180). Option sweep=true -> balayage
    90->0->180->90 sur ce servo pour verifier qu'il bouge.
    """
    if board.ser is None:
        return ("Port non connecte : impossible de piloter le servo. "
                "Reconnecte le serveur MCP (/mcp) si besoin.")
    try:
        sid = int(args.get("id") or 0)
    except (TypeError, ValueError):
        sid = 0
    if sid not in (1, 2, 3, 4):
        return "id doit valoir 1 (S1), 2 (S2), 3 (S3) ou 4 (S4)."
    try:
        angle = int(args.get("angle"))
    except (TypeError, ValueError):
        angle = 90
    angle = max(0, min(180, angle))                    # firmware borne aussi 0..180

    def _set(a):
        board.ser.write(build_frame(FUNC_PWM_SERVO, bytes([sid, a])))

    try:
        if args.get("sweep"):
            steps = [90, 0, 90, 180, 90]               # centre -> extremes -> centre
            for a in steps:
                _set(a)
                time.sleep(0.5)                        # laisse le SG90 atteindre la position
            return (f"Servo S{sid} : balayage {steps} termine "
                    f"(repos a 90 deg). FUNC 0x03.")
        _set(angle)
    except Exception as e:
        board.last_err = str(e)
        return f"Ecriture servo KO : {e}"
    return f"Servo S{sid} -> {angle} deg (FUNC 0x03)."


TOOLS = [
    {"name": "flash_firmware",
     "description": "Met a jour l'application par UART (IAP) sans presser BOOT0 "
                    "ni RESET : libere le port, envoie 0xA3 pour basculer dans le "
                    "bootloader resident, ecrit firmware.bin a 0x08004000, verifie "
                    "(CRC32) et saute a l'application. Compiler avant avec `pio run`. "
                    "Necessite le bootloader resident installe une premiere fois.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "bin": {"type": "string",
                                 "description": "chemin de l'image (defaut : "
                                                ".pio/build/genericSTM32F103RC/firmware.bin)"},
                         "no_enter": {"type": "boolean",
                                      "description": "true si la carte est deja dans le bootloader"}}}},
    {"name": "enter_bootloader",
     "description": "Fait sauter la carte dans son bootloader ROM (flash logiciel "
                    "sans presser BOOT0+RESET) et libere le port COM pour "
                    "`pio run -t upload`. Necessite le firmware avec le handler "
                    "FUNC_ENTER_BOOTLOADER (0xA3). Apres flash, reconnecter le MCP.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "motor_drive_all",
     "description": "Pilote les 4 moteurs SIMULTANEMENT avec controle independant "
                    "de vitesse et sens (PWM signe par moteur, borne +-40 %). "
                    "Fournir m1,m2,m3,m4 pour un controle explicite, ou pwm "
                    "(+reverse) pour le raccourci marche avant M1+ M2- M3- M4+. "
                    "Duree <=5 s. ROUES SURELEVEES obligatoire. STOP garanti.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "m1": {"type": "integer",
                                "description": "PWM signe M1 en % (-40..40)"},
                         "m2": {"type": "integer",
                                "description": "PWM signe M2 en % (-40..40)"},
                         "m3": {"type": "integer",
                                "description": "PWM signe M3 en % (-40..40)"},
                         "m4": {"type": "integer",
                                "description": "PWM signe M4 en % (-40..40)"},
                         "pwm": {"type": "integer",
                                 "description": "raccourci : magnitude marche avant (0..40, defaut 15) si m1..m4 absents"},
                         "seconds": {"type": "number",
                                     "description": "duree en s (0.1..5, defaut 3)"},
                         "reverse": {"type": "boolean",
                                     "description": "raccourci : true = marche arriere"}}}},
    {"name": "motor_drive",
     "description": "Pilote UN moteur (1..4) a faible PWM (borne +-50 %) pendant "
                    "une courte duree (borne 2 s) et mesure le delta de comptage "
                    "encodeur. ROUES SURELEVEES obligatoire. STOP envoye a la fin.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "motor": {"type": "integer",
                                   "description": "moteur a piloter : 1, 2, 3 ou 4"},
                         "pwm": {"type": "integer",
                                 "description": "PWM en % signe (-50..50)"},
                         "seconds": {"type": "number",
                                     "description": "duree en s (0.1..2, defaut 1)"}},
                     "required": ["motor"]}},
    {"name": "pwm_servo",
     "description": "Positionne un servo PWM (SG90) en angle absolu. S1=id 1 (PC3), "
                    "S2=id 2 (PC2), S3=id 3 (PC1), S4=id 4 (PC0). Angle borne 0..180 deg "
                    "(500..2480 us a 50 Hz). Option sweep=true : balayage de test "
                    "90->0->180->90 sur le servo choisi.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "id": {"type": "integer",
                                "description": "servo : 1 (S1), 2 (S2), 3 (S3) ou 4 (S4)"},
                         "angle": {"type": "integer",
                                   "description": "angle vise en degres (0..180, defaut 90)"},
                         "sweep": {"type": "boolean",
                                   "description": "true = balayage de test au lieu d'un angle fixe"}},
                     "required": ["id"]}},
    {"name": "status",
     "description": "Etat de la connexion serie : port, frequences par type de "
                    "trame, trames OK/KO, cpr, type de chassis.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "metrics",
     "description": "Dernieres metriques decodees : vitesse & batterie, attitude "
                    "(roll/pitch/yaw en deg), gyro/accel, comptage encodeur et "
                    "vitesse moteurs (tics/s et tr/min).",
     "inputSchema": {"type": "object",
                     "properties": {"seconds": {"type": "number",
                                                "description": "reserve (moyenne)"}}}},
    {"name": "encoders",
     "description": "Comptage encodeur cumulatif par moteur (M1..M4) et vitesse "
                    "instantanee (tics/s, tr/min).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "car_type",
     "description": "Interroge la carte sur son type de chassis (protocole "
                    "FUNC_REQUEST_DATA 0x50 -> 0x15) et en deduit les tics/tour "
                    "(cpr), applique automatiquement.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "calibrate_baseline",
     "description": "Fixe le comptage encodeur courant comme zero de reference, "
                    "avant de tourner une roue a la main.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "calibrate_read",
     "description": "Lit la variation de comptage depuis calibrate_baseline et en "
                    "deduit les tics/tour par moteur (delta / turns) et le sens.",
     "inputSchema": {"type": "object",
                     "properties": {"turns": {"type": "number",
                                              "description": "nb de tours effectues (defaut 1)"}}}},
]

HANDLERS = {
    "flash_firmware": t_flash_firmware,
    "enter_bootloader": t_enter_bootloader,
    "motor_drive_all": t_motor_drive_all,
    "motor_drive": t_motor_drive,
    "pwm_servo": t_pwm_servo,
    "status": t_status,
    "metrics": t_metrics,
    "encoders": t_encoders,
    "car_type": t_car_type,
    "calibrate_baseline": t_calibrate_baseline,
    "calibrate_read": t_calibrate_read,
}


# ---------------------------------------------------------------------------
# Boucle JSON-RPC (stdio, messages delimites par \n)
# ---------------------------------------------------------------------------
def _send(obj):
    data = json.dumps(obj, separators=(",", ":")).encode("utf-8") + b"\n"
    sys.stdout.buffer.write(data)                     # bytes -> pas de conversion \r\n
    sys.stdout.buffer.flush()


def _result_text(mid, text, is_error=False):
    _send({"jsonrpc": "2.0", "id": mid,
           "result": {"content": [{"type": "text", "text": text}],
                      "isError": is_error}})


def serve(board):
    for raw in sys.stdin.buffer:                      # une ligne = un message
        line = raw.decode("utf-8", "replace").strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except Exception:
            continue
        mid = msg.get("id")
        method = msg.get("method")
        if method == "initialize":
            pv = (msg.get("params") or {}).get("protocolVersion", DEFAULT_PROTOCOL)
            _send({"jsonrpc": "2.0", "id": mid,
                   "result": {"protocolVersion": pv,
                              "capabilities": {"tools": {}},
                              "serverInfo": {"name": SERVER_NAME,
                                             "version": SERVER_VERSION}}})
        elif method == "notifications/initialized":
            pass                                       # notification : pas de reponse
        elif method == "ping":
            _send({"jsonrpc": "2.0", "id": mid, "result": {}})
        elif method == "tools/list":
            _send({"jsonrpc": "2.0", "id": mid, "result": {"tools": TOOLS}})
        elif method == "tools/call":
            p = msg.get("params") or {}
            name = p.get("name")
            args = p.get("arguments") or {}
            handler = HANDLERS.get(name)
            if handler is None:
                _send({"jsonrpc": "2.0", "id": mid,
                       "error": {"code": -32602, "message": f"outil inconnu: {name}"}})
                continue
            try:
                text = handler(board, args)
                _result_text(mid, text)
            except Exception as e:
                _result_text(mid, f"erreur outil {name}: {e}", is_error=True)
        elif mid is not None:                          # requete de methode inconnue
            _send({"jsonrpc": "2.0", "id": mid,
                   "error": {"code": -32601, "message": f"methode inconnue: {method}"}})
        # sinon : notification inconnue -> ignore


def main():
    port = os.environ.get("BAMBOU_PORT", "COM4")
    baud = int(os.environ.get("BAMBOU_BAUD", "115200"))
    cpr = float(os.environ.get("BAMBOU_CPR", "1320"))
    board = Board(port, baud, cpr)

    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        log(f"selftest : lecture de {port} pendant 3 s...")
        time.sleep(3.0)
        log(t_status(board, {}).replace("\n", " | "))
        log(t_metrics(board, {}).replace("\n", " | "))
        return

    log(f"demarrage (port={port}, baud={baud}, cpr={cpr:g})")
    serve(board)


if __name__ == "__main__":
    main()

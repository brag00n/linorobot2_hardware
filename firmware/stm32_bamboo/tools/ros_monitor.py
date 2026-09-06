#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_monitor.py - Decodeur lisible du protocole serie de la carte
                 YB-ERF01-V3.0 (Yahboom STM32 ROS Robot Control Panel).

La carte emet en continu (auto-report ~24 Hz) des trames BINAIRES sur son
USART1 (CH340 / port micro-USB). Un moniteur serie classique les affiche donc
comme des caracteres illisibles. Ce script parse ces trames et affiche leur
contenu en clair.

--- Affichage agrege ---
L'affichage n'est PAS rafraichi a chaque trame : il l'est a intervalle fixe
(--interval, defaut 30 s). Pendant chaque intervalle, toutes les trames recues
sont accumulees, et c'est la MOYENNE de chaque metrique sur l'intervalle qui est
affichee (avec le nombre d'echantillons et la frequence mesuree).

Format de trame (little-endian) :
    [0xFF][0xFB][LEN][FUNC][donnees...][CHECKSUM]
      |     |     |    |                   |
      |     |     |    |                   checksum = somme(octets[2..fin-1]) & 0xFF
      |     |     |    code fonction (voir src/protocol.h)
      |     |     LEN  = nombre d'octets a partir de LEN inclus (= taille_totale - 2)
      |     ID du peripherique (PTO_DEVICE_ID - 1 = 0xFC - 1 = 0xFB)
      entete PTO_HEAD = 0xFF

Trames auto-report decodees :
    0x0A REPORT_SPEED    : Vx,Vy,Vz (int16 LE) + batterie (uint8, V*10)
    0x0E REPORT_ICM_RAW  : gyro[3], accel[3], mag[3] (int16 LE, valeur*1000)
    0x0C REPORT_IMU_ATT  : roll,pitch,yaw (int16 LE, radians*10000)
    0x0D REPORT_ENCODER  : M1..M4 (int32 LE, comptage cumulatif)

Le comptage encodeur etant cumulatif, la VITESSE de rotation de chaque moteur
est derivee = (comptage_fin - comptage_debut) / duree_fenetre, affichee en tics/s
et en tr/min (tr/min = tics/s * 60 / --cpr, cpr = tics par tour de roue).

Usage :
    python tools/ros_monitor.py                    # dashboard, moyenne / 30 s
    python tools/ros_monitor.py --interval 5       # moyenne toutes les 5 s
    python tools/ros_monitor.py --interval 0.5     # quasi temps reel (2 Hz)
    python tools/ros_monitor.py --cpr 2464         # X3 PLUS (moteur 205RPM)
    python tools/ros_monitor.py --mode log         # une ligne agregee / intervalle
    python tools/ros_monitor.py --hex              # ajoute la derniere trame brute
    python tools/ros_monitor.py --detect           # demande le type de chassis -> cpr
    python tools/ros_monitor.py --calibrate --turns 5   # mesure des tics/tour
    python tools/ros_monitor.py --port COM5
    python tools/ros_monitor.py --list             # liste les ports serie

Prerequis : pyserial (deja fourni avec PlatformIO).
    ~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py
"""

import argparse
import math
import sys
import time

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    sys.exit("pyserial manquant. Lancez le script avec le Python de PlatformIO,\n"
             "  ex: ~/.platformio/penv/Scripts/python.exe tools/ros_monitor.py\n"
             "  ou:  pip install pyserial")

# ---------------------------------------------------------------------------
# Constantes du protocole (miroir de src/protocol.h)
# ---------------------------------------------------------------------------
PTO_HEAD = 0xFF
PTO_ID_TX = 0xFC - 1          # 0xFB : ID emis par la carte vers l'hote
PTO_ID_RX = 0xFC              # 0xFC : ID des trames hote -> carte
FUNC_REQUEST_DATA = 0x50      # commande "demande de donnee"
FUNC_CAR_TYPE = 0x15          # type de chassis (requete + reponse)
FUNC_SET_WHEEL_GEOM = 0x16    # geometrie roue : cpr + circonference + APB (set + read)
FUNC_SET_MOTOR_PID = 0x13     # PID moteur (set + read)
FUNC_SET_YAW_PID = 0x14       # PID yaw   (set + read)
SAVE_VERIFY = 0x5F            # octet de garde : ecrire en flash si egal, sinon RAM seule

# Type de chassis -> (tics par tour de roue, libelle). Table figee du firmware
# (src/app_motion.c: Motion_Get_Circle_Pulse + src/app_motion.h).
CAR_TYPE_CPR = {
    0x01: (1320.0, "Mecanum X3 (moteur 330RPM)"),
    0x02: (2464.0, "Mecanum X3 PLUS (moteur 205RPM)"),
    0x03: (1320.0, "Mecanum mini (defaut 330RPM)"),
    0x04: (1320.0, "4 roues X1 (moteur 330RPM)"),
    0x05: (836.0,  "Ackermann R2 (moteur 550RPM)"),
    0x06: (1040.0, "Sunrise (moteur 450RPM)"),
}

FUNC_NAMES = {
    0x01: "AUTO_REPORT", 0x02: "BEEP", 0x03: "PWM_SERVO", 0x04: "PWM_SERVO_ALL",
    0x05: "RGB", 0x06: "RGB_EFFECT",
    0x0A: "REPORT_SPEED", 0x0B: "REPORT_MPU_RAW", 0x0C: "REPORT_IMU_ATT",
    0x0D: "REPORT_ENCODER", 0x0E: "REPORT_ICM_RAW", 0x0F: "RESET_STATE",
    0x10: "MOTOR", 0x11: "CAR_RUN", 0x12: "MOTION", 0x13: "SET_MOTOR_PID",
    0x14: "SET_YAW_PID", 0x15: "CAR_TYPE", 0x16: "SET_WHEEL_GEOM",
    0x51: "VERSION", 0x52: "NOW_YAW",
}


def s16(lo, hi):
    """Reconstruit un entier signe 16 bits little-endian."""
    v = lo | (hi << 8)
    return v - 0x10000 if v & 0x8000 else v


def s32(b0, b1, b2, b3):
    """Reconstruit un entier signe 32 bits little-endian."""
    v = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    return v - 0x100000000 if v & 0x80000000 else v


def build_frame(func, params=b""):
    """Construit une trame hote -> carte : [0xFF][0xFC][LEN][FUNC][params][CHK].
    LEN = taille_totale - 2 ; CHK = somme(octets[2..fin-1]) & 0xFF."""
    length = 3 + len(params)                   # LEN = 1(len)+1(func)+len(params)+1(chk) - 1
    frame = bytearray([PTO_HEAD, PTO_ID_RX, length, func]) + bytearray(params)
    frame.append(sum(frame[2:]) & 0xFF)
    return bytes(frame)


# ---------------------------------------------------------------------------
# Decodeurs : renvoient un dict {label: scalaire}. Les scalaires permettent
# de moyenner metrique par metrique sur l'intervalle d'agregation.
# ---------------------------------------------------------------------------
def decode_speed(d):
    return {
        "Vx (mm/s)":    float(s16(d[0], d[1])),
        "Vy (mm/s)":    float(s16(d[2], d[3])),
        "Vz (rad/s)":   s16(d[4], d[5]) / 1000.0,
        "Batterie (V)": d[6] / 10.0,
    }


def decode_icm_raw(d):
    axes = ("x", "y", "z")
    out = {}
    for i, a in enumerate(axes):                     # gyro
        out[f"gyro_{a}"] = s16(d[2 * i], d[2 * i + 1]) / 1000.0
    for i, a in enumerate(axes):                     # accel
        out[f"accel_{a}"] = s16(d[6 + 2 * i], d[6 + 2 * i + 1]) / 1000.0
    for i, a in enumerate(axes):                     # mag
        out[f"mag_{a}"] = s16(d[12 + 2 * i], d[12 + 2 * i + 1]) / 1000.0
    return out


def decode_imu_att(d):
    rad = [s16(d[2 * i], d[2 * i + 1]) / 10000.0 for i in range(3)]
    deg = [r * 180.0 / math.pi for r in rad]
    return {"Roll (deg)": deg[0], "Pitch (deg)": deg[1], "Yaw (deg)": deg[2]}


def decode_encoder(d):
    return {f"M{i + 1}": float(s32(d[4 * i], d[4 * i + 1],
                                   d[4 * i + 2], d[4 * i + 3])) for i in range(4)}


def decode_wheel_geom(d):
    """Report FUNC_SET_WHEEL_GEOM : cpr (u16le), circ*10 (u16le), apb*10 (u16le)."""
    def u16(lo, hi):
        return lo | (hi << 8)
    circ_mm = u16(d[2], d[3]) / 10.0
    return {
        "cpr (tics/tour)": float(u16(d[0], d[1])),
        "circ (mm)":       circ_mm,
        "diam (mm)":       circ_mm / math.pi,
        "APB (mm)":        u16(d[4], d[5]) / 10.0,
    }


def decode_pid(d):
    """Report FUNC_SET_MOTOR_PID/SET_YAW_PID : index (u8), kp/ki/kd (u16le / 1000)."""
    def u16(lo, hi):
        return lo | (hi << 8)
    return {
        "index":  float(d[0]),
        "kp":     u16(d[1], d[2]) / 1000.0,
        "ki":     u16(d[3], d[4]) / 1000.0,
        "kd":     u16(d[5], d[6]) / 1000.0,
    }


# ---------------------------------------------------------------------------
# Formateurs : dict moyenne -> lignes affichables
# ---------------------------------------------------------------------------
def fmt_kv(avg):
    return [f"    {k:<16}: {v:10.3f}" for k, v in avg.items()]


def fmt_icm(avg):
    def row(prefix, label):
        vals = [avg[k] for k in avg if k.startswith(prefix)]
        return "    " + f"{label:<16}: " + "  ".join(f"{v:+9.3f}" for v in vals)
    return [row("gyro_", "Gyro  (rad/s)"),
            row("accel_", "Accel (m/s2)"),
            row("mag_", "Mag   (uT)")]


def fmt_encoder(avg):
    return ["    " + f"{'Comptage (tics)':<16}: "
            + "   ".join(f"{k}={v:11.1f}" for k, v in avg.items())]


def encoder_speed_lines(stats, cpr, prefix="    "):
    """Lignes de vitesse de rotation par moteur, derivees du comptage cumulatif.
    vitesse = (comptage_fin - comptage_debut) / temps_fenetre."""
    sp = stats.get("enc_speed")
    if not sp:
        return [prefix + "Vitesse         : (>= 2 trames requises dans la fenetre)"]
    tps = (prefix + f"{'Vitesse (tics/s)':<16}: "
           + "   ".join(f"M{i + 1}={sp[i]:+9.1f}" for i in range(4)))
    rpm = (prefix + f"{'Vitesse (tr/min)':<16}: "
           + "   ".join(f"M{i + 1}={sp[i] * 60.0 / cpr:+7.1f}" for i in range(4)))
    return [tps, rpm]


# func -> (titre, decode_fn, format_fn, taille_min_donnees)
DECODERS = {
    0x0A: ("Vitesse & batterie",   decode_speed,   fmt_kv,      7),
    0x0C: ("Attitude (RPY)",       decode_imu_att, fmt_kv,      6),
    0x0E: ("IMU brut (ICM20948)",  decode_icm_raw, fmt_icm,    18),
    0x0D: ("Encodeurs",            decode_encoder, fmt_encoder, 16),
}
DISPLAY_ORDER = (0x0A, 0x0C, 0x0E, 0x0D)


# ---------------------------------------------------------------------------
# Machine a etats de lecture des trames
# ---------------------------------------------------------------------------
class FrameParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk):
        """Ajoute des octets recus et renvoie la liste des trames completes.
        Chaque trame = (func, data_bytes, checksum_ok, raw_bytes)."""
        self.buf.extend(chunk)
        frames = []
        while True:
            start = self._find_header()
            if start is None:
                break
            if start > 0:
                del self.buf[:start]           # jette les octets parasites
            if len(self.buf) < 3:
                break                          # pas encore le champ LEN
            length = self.buf[2]               # octets a partir de l'index 2
            total = length + 2                 # + entete + id
            if length < 2 or total > 64:       # trame invalide -> resync
                del self.buf[:2]
                continue
            if len(self.buf) < total:
                break                          # trame incomplete, on attend
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            func = raw[3]
            data = raw[4:total - 1]
            chk = raw[total - 1]
            calc = sum(raw[2:total - 1]) & 0xFF
            frames.append((func, data, chk == calc, raw))
        return frames

    def _find_header(self):
        b = self.buf
        for i in range(len(b) - 1):
            if b[i] == PTO_HEAD and b[i + 1] == PTO_ID_TX:
                return i
        if b and b[-1] == PTO_HEAD:            # garder un 0xFF en attente
            return len(b) - 1
        return None


# ---------------------------------------------------------------------------
# Agregateur : accumule sur la fenetre courante, calcule les moyennes au flush
# ---------------------------------------------------------------------------
class Aggregator:
    def __init__(self):
        self.total_ok = 0
        self.total_bad = 0
        self._reset_window()

    def _reset_window(self):
        self.sums = {}          # func -> {label: somme}
        self.n = {}             # func -> nb de trames dans la fenetre
        self.last_raw = {}      # func -> derniere trame brute (pour --hex)
        self.ok = 0             # trames valides dans la fenetre
        self.bad = 0            # checksums KO dans la fenetre
        self.enc_first = None   # (t, [M1..M4]) premiere trame encodeur de la fenetre
        self.enc_last = None    # (t, [M1..M4]) derniere trame encodeur de la fenetre
        self.win_start = time.time()

    def add(self, func, fields, ok, raw):
        if not ok:
            self.bad += 1
            self.total_bad += 1
            return
        self.ok += 1
        self.total_ok += 1
        if not fields:
            return
        s = self.sums.setdefault(func, {})
        for k, v in fields.items():
            s[k] = s.get(k, 0.0) + v
        self.n[func] = self.n.get(func, 0) + 1
        self.last_raw[func] = raw
        if func == 0x0D:                    # encodeurs : bornes pour la vitesse
            t = time.time()
            counts = [fields.get(f"M{i + 1}", 0.0) for i in range(4)]
            if self.enc_first is None:
                self.enc_first = (t, counts)
            self.enc_last = (t, counts)

    def flush(self):
        """Renvoie (results, window_stats) et reinitialise la fenetre.
        results = { func: (avg_dict, n, hz, last_raw) }."""
        elapsed = max(time.time() - self.win_start, 1e-6)
        results = {}
        for func, s in self.sums.items():
            n = self.n[func]
            avg = {k: s[k] / n for k in s}
            results[func] = (avg, n, n / elapsed, self.last_raw.get(func))
        enc_speed = None                    # tics/s par moteur sur la fenetre
        if self.enc_first and self.enc_last:
            dt = self.enc_last[0] - self.enc_first[0]
            if dt > 1e-3:
                enc_speed = [(self.enc_last[1][i] - self.enc_first[1][i]) / dt
                             for i in range(4)]
        stats = {"ok": self.ok, "bad": self.bad, "elapsed": elapsed,
                 "total_ok": self.total_ok, "total_bad": self.total_bad,
                 "enc_speed": enc_speed}
        self._reset_window()
        return results, stats


# ---------------------------------------------------------------------------
# Affichages
# ---------------------------------------------------------------------------
class DashboardRenderer:
    """Redessine un bloc fixe en place (sequences ANSI)."""

    def __init__(self, show_hex, interval, cpr):
        self.show_hex = show_hex
        self.interval = interval
        self.cpr = cpr
        self._prev_lines = 0
        _enable_vt()

    def render(self, results, stats):
        lines = []
        lines.append(f"=== YB-ERF01 ROS monitor ===  moyenne / {self.interval:g}s"
                     f"  (fenetre {stats['elapsed']:.1f}s)")
        lines.append(f"    trames OK: {stats['ok']:5d}  checksum KO: {stats['bad']:3d}"
                     f"   | total OK: {stats['total_ok']}  KO: {stats['total_bad']}")
        lines.append("")
        for func in DISPLAY_ORDER:
            title, _, fmt, _ = DECODERS[func]
            if func not in results:
                lines.append(f"[{func:#04x}] {title}  -- aucune trame --")
                lines.append("")
                continue
            avg, n, hz, raw = results[func]
            lines.append(f"[{func:#04x}] {title}  (moyenne de {n} trames, {hz:4.1f} Hz)")
            lines.extend(fmt(avg))
            if func == 0x0D:
                lines.extend(encoder_speed_lines(stats, self.cpr))
            if self.show_hex and raw is not None:
                lines.append("    derniere trame : " + raw.hex(" "))
            lines.append("")
        self._draw(lines)

    def _draw(self, lines):
        out = []
        if self._prev_lines:
            out.append(f"\x1b[{self._prev_lines}F")   # remonte le curseur
        for ln in lines:
            out.append("\x1b[2K" + ln + "\n")          # efface la ligne + texte
        sys.stdout.write("".join(out))
        sys.stdout.flush()
        self._prev_lines = len(lines)


class LogRenderer:
    """Defilement : un bloc horodate par intervalle."""

    def __init__(self, show_hex, interval, cpr):
        self.show_hex = show_hex
        self.interval = interval
        self.cpr = cpr

    def render(self, results, stats):
        ts = time.strftime("%H:%M:%S")
        print(f"--- {ts}  moyenne/{self.interval:g}s  "
              f"OK={stats['ok']} KO={stats['bad']} ---")
        for func in DISPLAY_ORDER:
            title, _, fmt, _ = DECODERS[func]
            if func not in results:
                print(f"  [{func:#04x}] {title}: aucune trame")
                continue
            avg, n, hz, raw = results[func]
            body = "  ".join(f"{k}={v:.3f}" for k, v in avg.items())
            print(f"  [{func:#04x}] {title} (n={n}, {hz:.1f} Hz): {body}")
            if func == 0x0D:
                for l in encoder_speed_lines(stats, self.cpr, prefix="      "):
                    print(l)
            if self.show_hex and raw is not None:
                print("      " + raw.hex(" "))
        sys.stdout.flush()


def _enable_vt():
    """Active le traitement des sequences ANSI dans la console Windows."""
    if sys.platform == "win32":
        try:
            import ctypes
            k = ctypes.windll.kernel32
            k.SetConsoleMode(k.GetStdHandle(-11), 7)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Modes de determination de l'odometre
# ---------------------------------------------------------------------------
def do_detect(ser):
    """Interroge la carte sur son type de chassis (protocole FUNC_REQUEST_DATA
    0x50 -> FUNC_CAR_TYPE 0x15) et en deduit les tics/tour. Lecture seule."""
    req = build_frame(FUNC_REQUEST_DATA, bytes([FUNC_CAR_TYPE, 0x00]))
    print(f"Envoi de la requete type de chassis : {req.hex(' ')}")
    for attempt in range(1, 6):
        parser = FrameParser()
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        ser.write(req)
        t0 = time.time()
        while time.time() - t0 < 0.6:
            chunk = ser.read(256)
            if not chunk:
                continue
            for func, data, ok, raw in parser.feed(chunk):
                if func == FUNC_CAR_TYPE and ok and len(data) >= 1:
                    ct = data[0]
                    cpr, label = CAR_TYPE_CPR.get(ct, (None, "type inconnu"))
                    print(f"\nType de chassis renvoye : 0x{ct:02X} ({label})")
                    if cpr:
                        print(f"  => tics par tour de roue (cpr) = {cpr:g}")
                        print(f"  Relancez par ex. : "
                              f"python tools/ros_monitor.py --cpr {cpr:g}")
                    else:
                        print("  Type non repertorie ; utilisez --calibrate pour mesurer.")
                    return
        print(f"  (essai {attempt}/5 : pas de reponse, nouvel essai...)")
    print("\nAucune reponse de la carte au bout de 5 essais.\n"
          "  - Verifiez que la carte est alimentee et sur le bon port.\n"
          "  - Sinon, mesurez directement avec --calibrate.")


def _calibration_bilan(base, delta, turns):
    print("\n\n=== Bilan calibration ===")
    if base is None:
        print("Aucune trame encodeur recue.")
        return
    for i in range(4):
        sens = "+ (A avance B)" if delta[i] >= 0 else "- (B avance A)"
        print(f"  M{i + 1} : delta = {delta[i]:+.0f} tics sur {turns:g} tour(s)"
              f"  =>  {abs(delta[i]) / turns:8.1f} tics/tour   sens {sens}")
    print("\nComparez avec la table firmware : 1320 / 2464 / 1040 / 836.")


def do_calibrate(ser, turns, seconds=0.0):
    """Mesure empirique des tics/tour : tournez une roue d'un nombre exact de
    tours a la main ; l'outil affiche en direct la variation du comptage. Au
    bilan, tics/tour = |delta| / nb_tours par moteur. Le signe du delta donne
    le sens de comptage (voies A/B). Le bilan est affiche a l'arret (Ctrl+C) ou
    apres 'seconds' secondes si > 0."""
    print(f"=== Calibration encodeurs : {turns:g} tour(s) de roue ===")
    print("1. Laissez les roues IMMOBILES (baseline prise a la 1re trame).")
    print(f"2. Tournez la/les roue(s) d'EXACTEMENT {turns:g} tour(s), a la main.")
    if seconds > 0:
        print(f"3. Lisez la colonne 'tics/tour' ; bilan automatique dans {seconds:g}s "
              "(ou Ctrl+C avant).\n")
    else:
        print("3. Lisez la colonne 'tics/tour', puis Ctrl+C pour le bilan.\n")
    parser = FrameParser()
    base = None
    delta = [0.0] * 4
    last_disp = 0.0
    t_start = time.time()
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                for func, data, ok, raw in parser.feed(chunk):
                    if func == 0x0D and ok and len(data) >= 16:
                        c = [decode_encoder(data)[f"M{i + 1}"] for i in range(4)]
                        if base is None:
                            base = c
                        delta = [c[i] - base[i] for i in range(4)]
            now = time.time()
            if now - last_disp > 0.15 and base is not None:
                line = " | ".join(
                    f"M{i + 1} d={delta[i]:+7.0f} ({delta[i] / turns:+8.1f}/tr)"
                    for i in range(4))
                sys.stdout.write("\r" + line)
                sys.stdout.flush()
                last_disp = now
            if seconds > 0 and now - t_start >= seconds:
                _calibration_bilan(base, delta, turns)
                return
    except KeyboardInterrupt:
        _calibration_bilan(base, delta, turns)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Decodeur serie ROS Yahboom YB-ERF01.")
    ap.add_argument("--port", default="COM4", help="port serie (defaut COM4)")
    ap.add_argument("--baud", type=int, default=115200, help="debit (defaut 115200)")
    ap.add_argument("--interval", type=float, default=30.0,
                    help="intervalle d'affichage en secondes ; les metriques recues "
                         "pendant l'intervalle sont moyennees (defaut 30)")
    ap.add_argument("--mode", choices=["dashboard", "log"], default="dashboard",
                    help="dashboard = vue live en place ; log = defilement")
    ap.add_argument("--cpr", type=float, default=1320.0,
                    help="tics d'encodeur par tour de roue, pour la vitesse en tr/min. "
                         "1320 = moteur 330RPM (Mecanum X3 / 4-roues X1, defaut), "
                         "2464 = 205RPM (X3 PLUS), 1040 = 450RPM (Sunrise), "
                         "836 = 550RPM (Ackermann)")
    ap.add_argument("--hex", action="store_true",
                    help="afficher aussi la derniere trame brute de chaque type")
    ap.add_argument("--list", action="store_true", help="lister les ports serie et quitter")
    ap.add_argument("--detect", action="store_true",
                    help="interroger la carte sur son type de chassis (=> tics/tour) et quitter")
    ap.add_argument("--calibrate", action="store_true",
                    help="mesurer les tics/tour en tournant une roue a la main, puis quitter")
    ap.add_argument("--turns", type=float, default=1.0,
                    help="nombre de tours de roue effectues pour --calibrate (defaut 1)")
    ap.add_argument("--cal-seconds", type=float, default=0.0,
                    help="--calibrate : bilan automatique apres N secondes (0 = Ctrl+C)")
    args = ap.parse_args()

    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("Aucun port serie detecte.")
        for p in ports:
            print(f"  {p.device:<8} {p.description}")
        return

    if args.interval <= 0:
        sys.exit("--interval doit etre > 0.")
    if args.turns <= 0:
        sys.exit("--turns doit etre > 0.")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"Impossible d'ouvrir {args.port}: {e}\n"
                 "  -> Fermez le moniteur serie de VSCode/PlatformIO "
                 "(un seul programme peut tenir le port).")

    if args.detect:
        try:
            do_detect(ser)
        finally:
            ser.close()
        return
    if args.calibrate:
        try:
            do_calibrate(ser, args.turns, args.cal_seconds)
        finally:
            ser.close()
        return

    print(f"Lecture de {args.port} @ {args.baud} bauds. "
          f"Affichage moyenne toutes les {args.interval:g}s. Ctrl+C pour quitter.\n")

    parser = FrameParser()
    agg = Aggregator()
    if args.mode == "dashboard":
        renderer = DashboardRenderer(args.hex, args.interval, args.cpr)
    else:
        renderer = LogRenderer(args.hex, args.interval, args.cpr)
    last_flush = time.time()

    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                for func, data, ok, raw in parser.feed(chunk):
                    dec = DECODERS.get(func)
                    if dec and ok and len(data) >= dec[3]:
                        fields = dec[1](data)
                    else:
                        fields = {}
                    agg.add(func, fields, ok, raw)
            now = time.time()
            if now - last_flush >= args.interval:
                results, stats = agg.flush()
                renderer.render(results, stats)
                last_flush = now
    except KeyboardInterrupt:
        print("\nArret.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

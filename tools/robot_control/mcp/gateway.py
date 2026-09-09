r"""gateway.py - Pont partage app <-> serveur MCP action.

Deux roles complementaires dans UN module (importe des deux cotes) :

  1. Protocole socket loopback (127.0.0.1) : une ligne JSON {cmd, args} en requete,
     {ok, data|error} en reponse. Cote MCP : send_command() (client). Cote app :
     CommandServer (thread d'ecoute) + drain() (execution sur le thread principal).
  2. Executeur de commandes carte handle_command(cmd, args, link, motion, cpr) :
     traduit une commande en trames via RobotComSerial (etendu) / RobotMotorDrive.
     Utilise A L'IDENTIQUE par l'app (relais, dans sa boucle) ET par le repli
     COM4-direct du serveur action -> zero duplication.

L'app est l'unique proprietaire de COM4 : quand elle tourne, le MCP lui relaie les
commandes par socket et l'app les execute sur son thread principal (acces COM4
serialise). Quand l'app est arretee, le MCP ouvre COM4 lui-meme (RobotComSerial +
RobotMotorDrive) et appelle le MEME handle_command.

Garde-fous materiels (memoire projet) repris a l'identique : motor_drive borne
+-50 %/2 s, motor_drive_all borne +-40 %/5 s, STOP franc x5, M2 encodeur HS.
-> Roues surelevees imperativement si un pilotage moteur est declenche.
"""
import json
import socket
import threading
import time
from collections import deque

# CAR_TYPE_CPR est re-exporte par le module de liaison (qui l'importe de ros_monitor
# apres avoir ajoute firmware/stm32_bamboo/tools a sys.path). On passe par lui pour
# ne pas dupliquer la resolution de chemin du protocole.
from ..communication.RobotComSerial import CAR_TYPE_CPR

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8787

# Commandes de CONFIGURATION (n'ont de sens que si l'app tourne : elles pilotent le
# suivi/vision, pas la carte). Les autres commandes sont des commandes CARTE.
CONFIG_CMDS = {"set_detector", "set_track_mode", "set_predict_mode", "set_tracking"}

CAR_TYPE_LABELS = {
    0x01: "CAR_MECANUM", 0x02: "CAR_MECANUM_MAX", 0x03: "CAR_MECANUM_MINI",
    0x04: "CAR_FOURWHEEL", 0x05: "CAR_ACKERMAN", 0x06: "CAR_SUNRISE",
}


def config_from(cmd, args):
    """Normalise une commande de config en dict applique par l'app (on_config)."""
    if cmd == "set_detector":
        return {"detector": (args.get("detector") or "").strip().lower()}
    if cmd == "set_track_mode":
        return {"track_mode": (args.get("track_mode") or "").strip().lower()}
    if cmd == "set_predict_mode":
        return {"predict_mode": (args.get("predict_mode") or "").strip().lower()}
    if cmd == "set_tracking":
        return {"active": bool(args.get("on"))}
    return {}


# ===========================================================================
# Protocole socket : client (MCP) et serveur (app)
# ===========================================================================
def parse_addr(spec, default_host=DEFAULT_HOST, default_port=DEFAULT_PORT):
    """'127.0.0.1:8787' | '8787' | '' -> (host, port)."""
    if not spec:
        return default_host, default_port
    spec = str(spec).strip()
    if ":" in spec:
        h, _, p = spec.rpartition(":")
        return (h or default_host), int(p)
    if spec.isdigit():
        return default_host, int(spec)
    return spec, default_port


def ping(host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=1.0):
    """True si l'app ecoute (COM4 lui appartient) ; False si connexion refusee.

    N'execute AUCUNE commande : sert a decider du transport / a interdire les
    operations de maintenance (bootloader/flash) tant que l'app tient le port.
    """
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        return True
    except OSError:
        return False


def send_command(cmd, args, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=15.0):
    """Envoie {cmd,args} a l'app et renvoie (ok, payload, alive).

    alive=False si la connexion est refusee (app arretee) -> l'appelant bascule en
    COM4 direct. payload = texte resultat si ok, message d'erreur sinon.
    """
    try:
        s = socket.create_connection((host, port), timeout=2.0)
    except (ConnectionRefusedError, OSError):
        return False, "app non joignable (%s:%s)" % (host, port), False
    try:
        s.settimeout(timeout)
        s.sendall((json.dumps({"cmd": cmd, "args": args or {}}) + "\n").encode("utf-8"))
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        line = buf.split(b"\n", 1)[0].decode("utf-8", "replace")
        resp = json.loads(line)
        if resp.get("ok"):
            return True, resp.get("data", ""), True
        return False, resp.get("error", "erreur inconnue"), True
    except Exception as e:
        return False, "erreur socket: %s" % e, True
    finally:
        try:
            s.close()
        except Exception:
            pass


class CommandServer:
    """Thread d'ecoute cote app : accepte les connexions MCP, met les requetes en
    file, attend que la boucle app les draine (execution sur le thread principal),
    puis renvoie la reponse. Une requete par connexion (simple et robuste)."""

    def __init__(self, on_config, link, motion, cpr=1320.0,
                 host=DEFAULT_HOST, port=DEFAULT_PORT):
        self.on_config = on_config
        self.link = link
        self.motion = motion
        self.cpr = cpr
        self.host = host
        self.port = port
        self._srv = None
        self._q = deque()                 # (req, Event, slot)
        self._qlock = threading.Lock()
        self._stop = False
        self._thread = None

    def start(self):
        try:
            self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._srv.bind((self.host, self.port))
            self._srv.listen(4)
            self._srv.settimeout(0.5)
        except OSError as e:
            print("[gateway] CommandServer NON demarre (%s:%s) : %s"
                  % (self.host, self.port, e))
            self._srv = None
            return self
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        print("[gateway] CommandServer en ecoute sur %s:%s" % (self.host, self.port))
        return self

    def _accept_loop(self):
        while not self._stop:
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._handle_conn, args=(conn,), daemon=True).start()

    def _handle_conn(self, conn):
        try:
            conn.settimeout(12.0)
            buf = b""
            while b"\n" not in buf:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                buf += chunk
            req = json.loads(buf.split(b"\n", 1)[0].decode("utf-8", "replace"))
            ev = threading.Event()
            slot = {}
            with self._qlock:
                self._q.append((req, ev, slot))
            if ev.wait(timeout=20.0):
                resp = slot.get("resp", {"ok": False, "error": "pas de reponse"})
            else:
                resp = {"ok": False, "error": "timeout execution (app occupee ?)"}
            conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def drain(self):
        """A appeler une fois par tour de boucle app (thread principal) : execute les
        commandes en file sur COM4 et debloque les connexions en attente."""
        while True:
            with self._qlock:
                if not self._q:
                    return
                req, ev, slot = self._q.popleft()
            cmd = req.get("cmd")
            args = req.get("args") or {}
            try:
                if cmd in CONFIG_CMDS:
                    text = self.on_config(config_from(cmd, args))
                else:
                    text = handle_command(cmd, args, self.link, self.motion, self.cpr)
                slot["resp"] = {"ok": True, "data": text}
            except Exception as e:
                slot["resp"] = {"ok": False, "error": "%s: %s" % (cmd, e)}
            ev.set()

    def stop(self):
        self._stop = True
        if self._srv is not None:
            try:
                self._srv.close()
            except Exception:
                pass


# ===========================================================================
# Executeur de commandes carte (partage app-relais / MCP-direct)
# ===========================================================================
def handle_command(cmd, args, link, motion, cpr=1320.0):
    """Execute une commande CARTE sur `link` (RobotComSerial etendu) / `motion`
    (RobotMotorDrive) et renvoie un texte lisible. Leve ValueError si commande
    inconnue. cpr sert aux conversions tics/s -> tr/min (surchargeable)."""
    fn = _BOARD_HANDLERS.get(cmd)
    if fn is None:
        raise ValueError("commande carte inconnue: %s" % cmd)
    return fn(args, link, motion, cpr)


def _f(v, default):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _i(v, default=0):
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def _snap_encoders(link):
    enc = link.snapshot().get("encoders")
    return None if enc is None else [int(round(x)) for x in enc]


# --- pilotage moteurs (garde-fous a l'identique) ---------------------------
def _cmd_motor_drive(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible de piloter."
    motor = _i(args.get("motor"))
    if motor not in (1, 2, 3, 4):
        return "motor doit valoir 1, 2, 3 ou 4."
    pwm = max(-50, min(50, _i(args.get("pwm"))))          # securite : +-50 %
    seconds = max(0.1, min(2.0, _f(args.get("seconds"), 1.0)))  # securite : 2 s
    base = _snap_encoders(link)
    if base is None:
        return "Aucune trame encodeur : mesure impossible."
    vals = [0, 0, 0, 0]
    vals[motor - 1] = pwm
    peak = list(base)
    try:
        t0 = time.time()
        while time.time() - t0 < seconds:
            link.motorRaw(vals)                          # repete (anti-watchdog)
            time.sleep(0.05)
            cur = _snap_encoders(link)
            if cur:
                for i in range(4):
                    if abs(cur[i] - base[i]) > abs(peak[i] - base[i]):
                        peak[i] = cur[i]
    finally:
        for _ in range(5):                               # STOP franc, toujours
            link.motorRaw([0, 0, 0, 0])
            time.sleep(0.02)
    time.sleep(0.3)
    final = _snap_encoders(link) or peak
    lines = ["Pilotage M%d a %d %% pendant %gs :" % (motor, pwm, seconds)]
    for i in range(4):
        tag = "   <-- pilote" if i == motor - 1 else ""
        lines.append("  M%d : delta %+d  (pic %+d)%s"
                     % (i + 1, final[i] - base[i], peak[i] - base[i], tag))
    dd = final[motor - 1] - base[motor - 1]
    lines.append("")
    if abs(dd) > 20:
        lines.append("=> M%d a compte %+d tics : ENCODEUR FONCTIONNEL." % (motor, dd))
    else:
        lines.append("=> M%d n'a quasiment pas compte (%+d) : roue bloquee "
                     "ou defaut de la chaine encodeur." % (motor, dd))
    return "\n".join(lines)


def _cmd_motor_drive_all(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible de piloter."
    seconds = max(0.1, min(5.0, _f(args.get("seconds"), 3.0)))  # securite : 5 s

    def clip(x):
        return max(-40, min(40, _i(x)))                  # securite : +-40 % / moteur

    per_motor = any(args.get(k) is not None for k in ("m1", "m2", "m3", "m4"))
    if per_motor:
        vals = [clip(args.get("m%d" % (i + 1))) for i in range(4)]
    else:
        pwm = min(40, abs(_i(args.get("pwm"), 15)))
        sgn = -1 if bool(args.get("reverse")) else 1
        vals = [sgn * s * pwm for s in (1, -1, -1, 1)]   # marche avant M1+ M2- M3- M4+
    base = _snap_encoders(link)
    if base is None:
        return "Aucune trame encodeur : mesure impossible."
    peak = list(base)
    try:
        t0 = time.time()
        while time.time() - t0 < seconds:
            link.motorRaw(vals)
            time.sleep(0.05)
            cur = _snap_encoders(link)
            if cur:
                for i in range(4):
                    if abs(cur[i] - base[i]) > abs(peak[i] - base[i]):
                        peak[i] = cur[i]
    finally:
        for _ in range(5):
            link.motorRaw([0, 0, 0, 0])
            time.sleep(0.02)
    time.sleep(0.3)
    final = _snap_encoders(link) or peak
    lines = ["Pilotage simultane des 4 moteurs pendant %gs :" % seconds,
             "  commandes : M1=%+d M2=%+d M3=%+d M4=%+d (%%)"
             % (vals[0], vals[1], vals[2], vals[3])]
    for i in range(4):
        note = "  (encodeur HS)" if i == 1 else ""       # M2 = voie encodeur morte
        lines.append("  M%d : delta %+d  (pic %+d)%s"
                     % (i + 1, final[i] - base[i], peak[i] - base[i], note))
    return "\n".join(lines)


# --- servos ----------------------------------------------------------------
def _cmd_pwm_servo(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible de piloter le servo."
    sid = _i(args.get("id"))
    if sid not in (1, 2, 3, 4):
        return "id doit valoir 1 (S1), 2 (S2), 3 (S3) ou 4 (S4)."
    if args.get("sweep"):
        for a in (90, 0, 90, 180, 90):
            link.sendServo(sid, a)
            time.sleep(0.5)
        return "Servo S%d : balayage 90->0->180->90 termine (repos a 90 deg)." % sid
    angle = max(0, min(180, _i(args.get("angle"), 90)))
    ok = link.sendServo(sid, angle)
    return "Servo S%d -> %d deg%s." % (sid, angle, "" if ok else " (ECHEC ecriture)")


# --- PID -------------------------------------------------------------------
def _pid_args(args):
    try:
        return float(args.get("kp")), float(args.get("ki")), float(args.get("kd")), None
    except (TypeError, ValueError):
        return None, None, None, "kp, ki et kd sont requis et numeriques."


def _cmd_set_motor_pid(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible d'ecrire le PID moteur."
    kp, ki, kd, err = _pid_args(args)
    if err:
        return err
    save = bool(args.get("save"))                        # defaut RAM
    if not link.setMotorPid(kp, ki, kd, save=save):
        return "Ecriture PID moteur KO."
    time.sleep(0.2)
    got = link.getPid(1)
    dest = "FLASH (persistant)" if save else "RAM (perdu au reset)"
    head = "PID moteur ecrit -> kp=%.3f ki=%.3f kd=%.3f vers %s." % (kp, ki, kd, dest)
    if got is None:
        return head + "\n  Pas de relecture : verifie avec get_pid."
    return head + "\n  relecture (partage) : kp=%.3f ki=%.3f kd=%.3f" % (
        got["kp"], got["ki"], got["kd"])


def _cmd_set_yaw_pid(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible d'ecrire le PID yaw."
    kp, ki, kd, err = _pid_args(args)
    if err:
        return err
    save = bool(args.get("save"))
    if not link.setYawPid(kp, ki, kd, save=save):
        return "Ecriture PID yaw KO."
    time.sleep(0.2)
    got = link.getPid(5)
    dest = "FLASH (persistant)" if save else "RAM (perdu au reset)"
    head = "PID yaw ecrit -> kp=%.3f ki=%.3f kd=%.3f vers %s." % (kp, ki, kd, dest)
    if got is None:
        return head + "\n  Pas de relecture : verifie avec get_pid."
    return head + "\n  relecture : kp=%.3f ki=%.3f kd=%.3f" % (
        got["kp"], got["ki"], got["kd"])


def _cmd_get_pid(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible de lire les PID."
    m = link.getPid(1)
    y = link.getPid(5)
    lines = []
    lines.append("PID moteur (partage) : kp=%.3f ki=%.3f kd=%.3f"
                 % (m["kp"], m["ki"], m["kd"]) if m else "PID moteur : pas de reponse.")
    lines.append("PID yaw              : kp=%.3f ki=%.3f kd=%.3f"
                 % (y["kp"], y["ki"], y["kd"]) if y else "PID yaw : pas de reponse.")
    return "\n".join(lines)


# --- geometrie roue --------------------------------------------------------
def _fmt_geom(g):
    return ("cpr=%.0f tics/tour   circ=%.1f mm (diam=%.1f mm)   APB=%.1f mm"
            % (g["cpr (tics/tour)"], g["circ (mm)"], g["diam (mm)"], g["APB (mm)"]))


def _cmd_get_wheel_geom(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible de lire la geometrie roue."
    g = link.getWheelGeom()
    if g is None:
        return "Pas de reponse a REQUEST_DATA(0x16). Firmware a jour ? Carte connectee ?"
    return "Geometrie roue : " + _fmt_geom(g)


def _cmd_set_wheel_geom(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible d'ecrire la geometrie roue."
    cur = link.getWheelGeom()                            # peut etre None

    def pick(key, cur_key):
        v = args.get(key)
        if v is None:
            return cur[cur_key] if cur else None
        return _f(v, "ERR")

    cpr_f = pick("cpr", "cpr (tics/tour)")
    circ_f = pick("circ_mm", "circ (mm)")
    apb_f = pick("apb_mm", "APB (mm)")
    if "ERR" in (cpr_f, circ_f, apb_f):
        return "Argument invalide (cpr / circ_mm / apb_mm doivent etre numeriques)."
    if cpr_f is None or circ_f is None or apb_f is None:
        return ("Valeur manquante : la carte n'a pas repondu a la relecture, "
                "fournissez explicitement cpr, circ_mm ET apb_mm.")
    save = args.get("save")
    save = True if save is None else bool(save)
    ok, err = link.setWheelGeom(cpr_f, circ_f, apb_f, save=save)
    if err:
        return err
    if not ok:
        return "Ecriture geometrie roue KO (port ?)."
    time.sleep(0.2)
    got = link.getWheelGeom()
    dest = "FLASH (persistant)" if save else "RAM (perdu au reset)"
    head = ("Geometrie roue ecrite -> cpr=%d, circ=%.1f mm, APB=%.1f mm vers %s."
            % (int(round(cpr_f)), circ_f, apb_f, dest))
    if got is None:
        return head + "\n  Pas de relecture : verifie avec get_wheel_geom."
    return head + "\n  relecture : " + _fmt_geom(got)


# --- type de chassis -------------------------------------------------------
def _cmd_car_type(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible de lire le type de chassis."
    ct = link.getCarType()
    if ct is None:
        return "Pas de reponse de la carte pour le type de chassis."
    c, label = CAR_TYPE_CPR.get(ct, (None, "type inconnu"))
    return ("car_type=0x%02X (%s)" % (ct, label)
            + ("  =>  cpr=%g tics/tour (applique)" % c if c else
               "  (non repertorie ; utilisez la calibration)"))


def _cmd_set_car_type(args, link, motion, cpr):
    if not link.connected:
        return "Port non connecte : impossible d'ecrire le type de chassis."
    ct = _i(args.get("car_type"), -1)
    if ct < 0x01 or ct >= 0x07:
        return "car_type hors plage (1..6 ; 4 = CAR_FOURWHEEL)."
    save = args.get("save")
    save = True if save is None else bool(save)
    if not link.setCarType(ct, save=save):
        return "Ecriture car_type KO (port ?)."
    time.sleep(0.2)
    got = link.getCarType()
    label = CAR_TYPE_LABELS.get(ct, "type inconnu")
    dest = "FLASH (persistant)" if save else "RAM (perdu au reset)"
    head = "car_type ecrit -> 0x%02X (%s) vers %s." % (ct, label, dest)
    if got is None:
        return head + "\n  Pas de relecture (carte muette) : verifie avec car_type."
    verdict = "OK" if got == ct else "MISMATCH"
    c, glabel = CAR_TYPE_CPR.get(got, (None, "type inconnu"))
    return (head + "\n  relecture : 0x%02X (%s) -> %s" % (got, glabel, verdict)
            + ("  cpr=%g tics/tour (applique)" % c if c else ""))


# --- metriques / encodeurs live -------------------------------------------
def _cmd_metrics(args, link, motion, cpr):
    s = link.snapshot()
    lines = []
    if s.get("vx") is not None:
        lines.append("Vitesse : Vx=%.1f Vy=%.1f mm/s  Vz=%.3f rad/s  batt=%s V"
                     % (s["vx"], s["vy"], s["vz"],
                        "%.1f" % s["battery"] if s.get("battery") is not None else "?"))
    if s.get("yaw") is not None:
        lines.append("Attitude : yaw=%+.1f deg" % s["yaw"])
    if s.get("encoders"):
        lines.append("Encodeurs (tics) : "
                     + "  ".join("M%d=%.0f" % (i + 1, s["encoders"][i]) for i in range(4)))
    sp = link.encSpeed()
    c = link.cpr or cpr
    if sp:
        lines.append("Vitesse moteurs : "
                     + "  ".join("M%d=%+.0ft/s(%+.1ftr/min)"
                                 % (i + 1, sp[i], sp[i] * 60.0 / c) for i in range(4)))
    if not lines:
        return "Aucune metrique recue pour l'instant (carte connectee ?)."
    return "\n".join(lines)


def _cmd_encoders(args, link, motion, cpr):
    counts = link.snapshot().get("encoders")
    if counts is None:
        return "Aucune trame encodeur recue."
    lines = ["Comptage cumulatif (tics) : "
             + "  ".join("M%d=%.0f" % (i + 1, counts[i]) for i in range(4))]
    sp = link.encSpeed()
    c = link.cpr or cpr
    if sp:
        lines.append("Vitesse instantanee       : "
                     + "  ".join("M%d=%+.1f t/s (%+.1f tr/min)"
                                 % (i + 1, sp[i], sp[i] * 60.0 / c) for i in range(4)))
    else:
        lines.append("Vitesse instantanee       : (immobile / trop peu d'echantillons)")
    return "\n".join(lines)


# --- calibration encodeur (baseline sur le link partage) -------------------
def _cmd_calibrate_baseline(args, link, motion, cpr):
    counts = _snap_encoders(link)
    if counts is None:
        return "Aucune trame encodeur : impossible de fixer la baseline."
    link.baseline = counts
    return ("Baseline fixee : " + "  ".join("M%d=%d" % (i + 1, counts[i]) for i in range(4))
            + "\nTournez la roue du nombre de tours voulu, puis calibrate_read.")


def _cmd_calibrate_read(args, link, motion, cpr):
    turns = _f(args.get("turns"), 1.0)
    if turns <= 0:
        return "turns doit etre > 0."
    base = link.baseline
    if base is None:
        return "Appelez d'abord calibrate_baseline."
    counts = _snap_encoders(link)
    if counts is None:
        return "Aucune trame encodeur."
    lines = ["Delta depuis la baseline, sur %g tour(s) :" % turns]
    for i in range(4):
        dl = counts[i] - base[i]
        sens = "+ (A avance B)" if dl >= 0 else "- (B avance A)"
        lines.append("  M%d : %+d tics  =>  %8.1f tics/tour   sens %s"
                     % (i + 1, dl, abs(dl) / turns, sens))
    lines.append("Table firmware de reference : 1320 / 2464 / 1040 / 836.")
    return "\n".join(lines)


_BOARD_HANDLERS = {
    "motor_drive": _cmd_motor_drive,
    "motor_drive_all": _cmd_motor_drive_all,
    "pwm_servo": _cmd_pwm_servo,
    "set_motor_pid": _cmd_set_motor_pid,
    "set_yaw_pid": _cmd_set_yaw_pid,
    "get_pid": _cmd_get_pid,
    "get_wheel_geom": _cmd_get_wheel_geom,
    "set_wheel_geom": _cmd_set_wheel_geom,
    "car_type": _cmd_car_type,
    "set_car_type": _cmd_set_car_type,
    "metrics": _cmd_metrics,
    "encoders": _cmd_encoders,
    "calibrate_baseline": _cmd_calibrate_baseline,
    "calibrate_read": _cmd_calibrate_read,
}

# Noms des commandes carte (pour le serveur action : relais OU repli direct).
BOARD_CMDS = frozenset(_BOARD_HANDLERS)

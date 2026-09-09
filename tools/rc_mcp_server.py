#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
rc_mcp_server.py - Serveur MCP (stdio) d'ANALYSE de l'outil robot_control.

L'app robot_control.main detient COM4 ET la camera : aucun autre process ne peut
les ouvrir. Ce serveur n'ouvre NI le port NI la camera : il lit seulement les
fichiers de telemetrie ecrits par l'app (voir robot_control/telemetry.py) et
peut donc tourner EN MEME TEMPS que l'app, pour analyser son comportement.

  logs/state.json          : photo instantanee (dernier de chaque type + compteurs)
  logs/robot_control.jsonl : un enregistrement JSON par ligne, horodate (t)
  logs/latest.jpg          : derniere image annotee (capture pour analyse)

Zero dependance (stdlib seule). Protocole MCP (JSON-RPC 2.0 sur stdin/stdout,
messages delimites par des sauts de ligne) implemente directement ici, sur le
meme modele que ros_mcp_server.py.

Variable d'environnement : RC_LOG_DIR (defaut : robot_control/logs a cote de ce
fichier).

Test manuel (sans client MCP) :
    python tools/rc_mcp_server.py selftest
"""

import json
import math
import os
import sys
import time

SERVER_NAME = "robot-control"
SERVER_VERSION = "1.0.0"
DEFAULT_PROTOCOL = "2025-06-18"

_HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.environ.get(
    "RC_LOG_DIR", os.path.join(_HERE, "robot_control", "logs"))
JSONL_PATH = os.path.join(LOG_DIR, "robot_control.jsonl")
STATE_PATH = os.path.join(LOG_DIR, "state.json")
SNAPSHOT_PATH = os.path.join(LOG_DIR, "latest.jpg")
CURSOR_PATH = os.path.join(LOG_DIR, "mcp_cursor.json")
CONTROL_PATH = os.path.join(LOG_DIR, "control.json")

DETECTORS = ("haar", "dnn", "yunet")


def log(*a):
    print("[rc-mcp]", *a, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# Lecture des fichiers de telemetrie (aucune ouverture de port/camera)
# ---------------------------------------------------------------------------
def _read_state():
    try:
        with open(STATE_PATH, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def _read_records(limit=None, types=None, since=None):
    """Lit les enregistrements JSONL (les plus recents en dernier).

    limit : ne garde que les N derniers ; types : set de types a conserver ;
    since : ne garde que t >= since. Robuste aux lignes tronquees (app en cours
    d'ecriture) : les lignes non parsables sont ignorees.
    """
    recs = []
    try:
        with open(JSONL_PATH, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                if types is not None and r.get("type") not in types:
                    continue
                if since is not None and r.get("t", 0) < since:
                    continue
                recs.append(r)
    except FileNotFoundError:
        return []
    except Exception:
        return recs
    if limit is not None and len(recs) > limit:
        recs = recs[-limit:]
    return recs


def _load_cursor():
    try:
        with open(CURSOR_PATH, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def _fmt_age(path):
    try:
        return round(time.time() - os.path.getmtime(path), 1)
    except Exception:
        return None


def _running(state):
    """L'app est-elle vivante ? (state.json mis a jour il y a < 5 s)."""
    if not state:
        return False
    try:
        return (time.time() - state.get("updated", 0)) < 5.0
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Outils
# ---------------------------------------------------------------------------
def t_status(args):
    state = _read_state()
    if state is None:
        return ("Aucune telemetrie trouvee (%s absent).\n"
                "L'app robot_control est-elle lancee (sans --no-telemetry) ?"
                % STATE_PATH)
    last = state.get("last", {})
    return _status_text(state, last)


def _status_text(state, last):
    alive = _running(state)
    lines = [
        "app       : %s (pid %s)" % ("EN COURS" if alive else "arretee/silencieuse",
                                     state.get("pid")),
        "uptime    : %ss   (state maj il y a %ss)"
        % (state.get("uptime"), round(time.time() - state.get("updated", 0), 1)),
        "compteurs : %s" % json.dumps(state.get("counts", {}), separators=(",", ":")),
    ]
    hb = last.get("heartbeat")
    if hb:
        lines.append(
            "live      : P1=%sfps P2=%sfps suivi=%s pan=%s tilt=%s COM=%s "
            "batt=%s yaw=%s ok=%s bad=%s"
            % (hb.get("disp_fps"), hb.get("det_fps"), hb.get("tracking"),
               hb.get("pan"), hb.get("tilt"), hb.get("connected"),
               hb.get("batt"), hb.get("yaw"), hb.get("ok"), hb.get("bad")))
    det = last.get("detect")
    if det:
        lines.append("detect    : seq=%s visages=%s nx=%s ny=%s aire=%s%%"
                     % (det.get("seq"), det.get("faces"), det.get("nx"),
                        det.get("ny"), det.get("area")))
    for sub in ("speed", "imu", "encoder"):
        r = last.get("rx:" + sub)
        if r:
            fields = {k: v for k, v in r.items() if k not in ("t", "type", "sub")}
            lines.append("rx.%-7s: %s" % (sub, json.dumps(fields, separators=(",", ":"))))
    snap_age = _fmt_age(SNAPSHOT_PATH)
    if snap_age is not None:
        lines.append("image     : %s (il y a %ss)" % (SNAPSHOT_PATH, snap_age))
    return "\n".join(lines)


def t_tail(args):
    n = int(args.get("n", 20))
    types = args.get("types")
    types = set(types) if types else None
    since = _load_cursor().get("t") if args.get("since_mark") else None
    recs = _read_records(limit=n, types=types, since=since)
    if not recs:
        return "Aucun enregistrement (filtre types=%s, since_mark=%s)." % (
            args.get("types"), bool(args.get("since_mark")))
    now = time.time()
    out = []
    for r in recs:
        age = round(now - r.get("t", now), 1)
        rest = {k: v for k, v in r.items() if k not in ("t", "type")}
        out.append("-%5ss %-9s %s" % (age, r.get("type"),
                                      json.dumps(rest, separators=(",", ":"))))
    return "\n".join(out)


def t_board_tx(args):
    n = int(args.get("n", 20))
    recs = _read_records(limit=n, types={"tx_motor", "tx_servo"})
    if not recs:
        return "Aucune trame emise enregistree (moteurs/servos)."
    now = time.time()
    out = []
    for r in recs:
        age = round(now - r.get("t", now), 1)
        if r["type"] == "tx_motor":
            out.append("-%5ss MOTOR m=%s ok=%s [%s]"
                       % (age, r.get("m"), r.get("ok"), r.get("hex")))
        else:
            out.append("-%5ss SERVO id=%s angle=%s ok=%s [%s]"
                       % (age, r.get("id"), r.get("angle"), r.get("ok"), r.get("hex")))
    return "\n".join(out)


def t_board_rx(args):
    state = _read_state()
    if not state:
        return "Aucune telemetrie (state.json absent)."
    last = state.get("last", {})
    out = []
    for sub in ("speed", "imu", "encoder"):
        r = last.get("rx:" + sub)
        if r:
            age = round(time.time() - r.get("t", time.time()), 1)
            fields = {k: v for k, v in r.items() if k not in ("t", "type", "sub")}
            out.append("rx.%-7s (-%ss) %s"
                       % (sub, age, json.dumps(fields, separators=(",", ":"))))
    return "\n".join(out) if out else "Aucune trame recue de la carte pour l'instant."


def t_detection(args):
    n = int(args.get("n", 15))
    recs = _read_records(limit=n, types={"detect"})
    if not recs:
        return "Aucune detection enregistree."
    now = time.time()
    out = []
    for r in recs:
        age = round(now - r.get("t", now), 1)
        out.append("-%5ss seq=%s visages=%s nx=%s ny=%s aire=%s%% suivi=%s"
                   % (age, r.get("seq"), r.get("faces"), r.get("nx"),
                      r.get("ny"), r.get("area"), r.get("tracking")))
    return "\n".join(out)


def t_tracking(args):
    n = int(args.get("n", 20))
    recs = _read_records(limit=n, types={"track"})
    if not recs:
        return "Aucune correction de suivi enregistree (suivi jamais actif ?)."
    now = time.time()
    out = []
    for r in recs:
        age = round(now - r.get("t", now), 1)
        if r.get("skipped"):
            out.append("-%5ss %-4s err=%s -> %s"
                       % (age, r.get("axis"), r.get("err"), r.get("skipped")))
        else:
            out.append("-%5ss %-4s err=%s gain=%s pas=%s(brut %s) %s->%s clamp=%s envoye=%s"
                       % (age, r.get("axis"), r.get("err"), r.get("gain"),
                          r.get("step"), r.get("step_raw"), r.get("before"),
                          r.get("after"), r.get("clamped"), r.get("sent")))
    return "\n".join(out)


def _stats(vals):
    vals = [v for v in vals if isinstance(v, (int, float))]
    if not vals:
        return None
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    return {"n": n, "min": round(min(vals), 4), "max": round(max(vals), 4),
            "moy": round(mean, 4), "ecart": round(math.sqrt(var), 4)}


def _osc_ratio(steps):
    """Taux de changement de signe des pas consecutifs (proxy d'oscillation).

    ~0 = corrections dans un sens constant (convergence) ; ~1 = le servo fait
    l'aller-retour a chaque mesure (pompage/hunting).
    """
    steps = [s for s in steps if isinstance(s, (int, float)) and s != 0]
    if len(steps) < 2:
        return None, len(steps)
    changes = sum(1 for a, b in zip(steps, steps[1:]) if (a > 0) != (b > 0))
    return round(changes / (len(steps) - 1), 3), len(steps)


def t_analyze(args):
    window = float(args.get("window_s", 10.0))
    since = time.time() - window
    if args.get("since_mark"):
        since = _load_cursor().get("t", since)
    dets = _read_records(types={"detect"}, since=since)
    tracks = _read_records(types={"track"}, since=since)
    lines = ["Analyse sur %s enregistrements (fenetre %ss) :"
             % (len(dets) + len(tracks), round(time.time() - since, 1))]

    if dets:
        span = max(1e-3, dets[-1]["t"] - dets[0]["t"])
        lines.append("detections : %d (%.1f/s)" % (len(dets), (len(dets) - 1) / span))
        nx = _stats([d.get("nx") for d in dets])
        ny = _stats([d.get("ny") for d in dets])
        if nx:
            lines.append("  nx  %s" % json.dumps(nx, separators=(",", ":")))
        if ny:
            lines.append("  ny  %s" % json.dumps(ny, separators=(",", ":")))
        faces = [d.get("faces", 0) for d in dets]
        lost = sum(1 for f in faces if not f)
        lines.append("  visage perdu : %d/%d mesures (%.0f%%)"
                     % (lost, len(faces), 100.0 * lost / len(faces)))
    else:
        lines.append("detections : aucune sur la fenetre.")

    for axis in ("pan", "tilt"):
        aset = [r for r in tracks if r.get("axis") == axis]
        if not aset:
            continue
        sent = [r for r in aset if r.get("sent")]
        skipped = sum(1 for r in aset if r.get("skipped"))
        clamped = sum(1 for r in aset if r.get("clamped"))
        ratio, nsteps = _osc_ratio([r.get("step") for r in sent])
        st = _stats([abs(r.get("step", 0)) for r in sent])
        verdict = ""
        if ratio is not None:
            verdict = " <- OSCILLE" if ratio >= 0.5 else " (stable)"
        lines.append(
            "%-4s : %d corr., %d envoyees, %d en butee de pas, %d en zone morte"
            % (axis, len(aset), len(sent), clamped, skipped))
        lines.append("       oscillation=%s%s  |pas| moy=%s"
                     % (ratio, verdict, st.get("moy") if st else "?"))
    if not tracks:
        lines.append("suivi : aucune correction (F pour activer dans l'app).")
    return "\n".join(lines)


def t_capture(args):
    age = _fmt_age(SNAPSHOT_PATH)
    if age is None:
        return ("Aucune image (%s absent). L'app ecrit une image annotee "
                "periodiquement quand elle tourne." % SNAPSHOT_PATH)
    try:
        size = os.path.getsize(SNAPSHOT_PATH)
    except Exception:
        size = "?"
    return ("Derniere image annotee :\n  chemin : %s\n  age    : %ss\n  taille : %s octets\n"
            "(image capturee par l'app : visages, croix centre, nx/ny, angles servo)"
            % (SNAPSHOT_PATH, age, size))


def t_set_detector(args):
    """Ecrit une commande de bascule de detecteur (control.json) que l'app relit.

    C'est le SEUL outil qui ecrit une commande vers l'app (canal decouple par
    fichier, comme la telemetrie) ; l'app applique et valide (fichiers presents).
    """
    det = (args.get("detector") or "").strip().lower()
    if det not in DETECTORS:
        return "detecteur invalide '%s'. Attendu : %s." % (det, ", ".join(DETECTORS))
    t = time.time()
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        tmp = CONTROL_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"detector": det, "t": t,
                       "iso": time.strftime("%H:%M:%S")}, f)
        os.replace(tmp, CONTROL_PATH)
    except Exception as e:
        return "Impossible d'ecrire la commande : %s" % e
    state = _read_state()
    warn = ""
    if not _running(state):
        warn = ("\nATTENTION : l'app ne semble pas active (state.json vieux) ; "
                "la bascule prendra effet au prochain demarrage/lecture.")
    return ("Bascule demandee -> %s (control.json ecrit a %s).%s\n"
            "Verifier l'effet : tail types=[\"event\"] (msg=detector_switch, ok=true)."
            % (det, time.strftime("%H:%M:%S"), warn))


def t_mark(args):
    """Pose un repere temporel : tail/analyze avec since_mark partent d'ici."""
    t = time.time()
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(CURSOR_PATH, "w", encoding="utf-8") as f:
            json.dump({"t": t, "iso": time.strftime("%H:%M:%S")}, f)
    except Exception as e:
        return "Impossible d'ecrire le repere : %s" % e
    return "Repere pose a %s. tail/analyze --since_mark repartiront d'ici." \
        % time.strftime("%H:%M:%S")


# ---------------------------------------------------------------------------
# Declaration MCP
# ---------------------------------------------------------------------------
_N = {"type": "integer", "description": "nombre d'enregistrements (defaut variable)"}
_SINCE = {"type": "boolean", "description": "repartir du dernier repere (mark)"}

TOOLS = [
    {"name": "status",
     "description": "Etat de synthese de l'outil robot_control (lecture des logs, "
                    "n'ouvre ni COM ni camera) : app vivante ?, compteurs, dernier "
                    "heartbeat (fps, suivi, pan/tilt, COM, batterie, yaw), derniere "
                    "detection et dernieres trames recues.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "tail",
     "description": "Derniers enregistrements bruts du journal (tous types ou "
                    "filtres). Types : event, tx_motor, tx_servo, rx, detect, "
                    "track, heartbeat.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "n": _N,
                         "types": {"type": "array", "items": {"type": "string"},
                                   "description": "filtre par types"},
                         "since_mark": _SINCE}}},
    {"name": "board_tx",
     "description": "Dernieres trames ENVOYEES a la carte (moteurs & servos) avec "
                    "valeurs, hex de la trame et succes d'ecriture.",
     "inputSchema": {"type": "object", "properties": {"n": _N}}},
    {"name": "board_rx",
     "description": "Dernieres trames RECUES de la carte (vitesse+batterie, IMU "
                    "roll/pitch/yaw, encodeurs M1..M4).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "detection",
     "description": "Dernieres detections camera : position normalisee du visage "
                    "(nx,ny dans [-1,+1]), nb de visages, aire, fps detection.",
     "inputSchema": {"type": "object", "properties": {"n": _N}}},
    {"name": "tracking",
     "description": "Dernieres corrections pan/tilt : erreur, gain, pas (brut et "
                    "borne), angle avant/apres, saturation, envoye ou zone morte.",
     "inputSchema": {"type": "object", "properties": {"n": _N}}},
    {"name": "analyze",
     "description": "Analyse le comportement sur une fenetre : cadence de detection, "
                    "stats nx/ny, taux de perte du visage, et par axe (pan/tilt) un "
                    "indice d'OSCILLATION (taux de changement de signe des pas : ~0 "
                    "convergence, >=0.5 pompage) + |pas| moyen.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "window_s": {"type": "number",
                                      "description": "fenetre d'analyse en s (defaut 10)"},
                         "since_mark": _SINCE}}},
    {"name": "capture",
     "description": "Chemin et age de la derniere image annotee ecrite par l'app "
                    "(a lire avec l'outil de lecture d'image du client).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "mark",
     "description": "Pose un repere temporel ; tail/analyze avec since_mark "
                    "repartiront de cet instant (pour isoler un essai).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "set_detector",
     "description": "Change le detecteur de visage de l'app A CHAUD (ecrit "
                    "logs/control.json, relu par l'app). haar=leger mais decroche "
                    "de profil ; dnn=res10 robuste au profil (defaut, fichiers "
                    "presents) ; yunet=le plus robuste (necessite le modele onnx). "
                    "Verifier l'effet via tail types=[\"event\"] (detector_switch).",
     "inputSchema": {"type": "object",
                     "properties": {
                         "detector": {"type": "string",
                                      "enum": list(DETECTORS),
                                      "description": "haar | dnn | yunet"}},
                     "required": ["detector"]}},
]

HANDLERS = {
    "status": t_status,
    "tail": t_tail,
    "board_tx": t_board_tx,
    "board_rx": t_board_rx,
    "detection": t_detection,
    "tracking": t_tracking,
    "analyze": t_analyze,
    "capture": t_capture,
    "mark": t_mark,
    "set_detector": t_set_detector,
}


# ---------------------------------------------------------------------------
# Boucle JSON-RPC (stdio, messages delimites par \n)
# ---------------------------------------------------------------------------
def _send(obj):
    data = json.dumps(obj, separators=(",", ":")).encode("utf-8") + b"\n"
    sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()


def _result_text(mid, text, is_error=False):
    _send({"jsonrpc": "2.0", "id": mid,
           "result": {"content": [{"type": "text", "text": text}],
                      "isError": is_error}})


def serve():
    for raw in sys.stdin.buffer:
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
            pass
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
                       "error": {"code": -32602, "message": "outil inconnu: %s" % name}})
                continue
            try:
                _result_text(mid, handler(args))
            except Exception as e:
                _result_text(mid, "erreur outil %s: %s" % (name, e), is_error=True)
        elif mid is not None:
            _send({"jsonrpc": "2.0", "id": mid,
                   "error": {"code": -32601, "message": "methode inconnue: %s" % method}})


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        log("selftest : lecture de %s" % LOG_DIR)
        log(t_status({}).replace("\n", " | "))
        log(t_analyze({}).replace("\n", " | "))
        return
    log("demarrage (log_dir=%s)" % LOG_DIR)
    serve()


if __name__ == "__main__":
    main()

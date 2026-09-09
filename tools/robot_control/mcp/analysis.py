#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""analysis.py - Serveur MCP (stdio) d'ANALYSE, strictement LECTURE SEULE.

Repris a l'identique de l'ancien tools/rc_mcp_server.py, MOINS l'outil d'ecriture
set_detector (migre vers le serveur d'action). Ce serveur n'ouvre NI COM4 NI la
camera : il lit seulement les fichiers de telemetrie ecrits par l'app
(robot_control ou robot_controlv3) et peut donc tourner EN MEME TEMPS que l'app.

  logs/state.json          : photo instantanee (dernier de chaque type + compteurs)
  logs/robot_control.jsonl : un enregistrement JSON par ligne, horodate (t)
  logs/latest.jpg          : derniere image annotee (capture pour analyse)

Dossier de logs : RC_LOG_DIR (defaut robot_control/logs). Zero dependance (stdlib).

Lancement (via .mcp.json) :
    python -m robot_control.mcp.analysis
Test manuel :
    python -m robot_control.mcp.analysis selftest
"""
import json
import math
import os
import sys
import time

from . import _rpc

SERVER_NAME = "robot-analysis"

_LOG = _rpc.make_log("rc-analysis")
LOGS = _rpc.Logs(_rpc.resolve_log_dir())


# ---------------------------------------------------------------------------
# Outils (tous en LECTURE SEULE)
# ---------------------------------------------------------------------------
def t_status(args):
    state = LOGS.read_state()
    if state is None:
        return ("Aucune telemetrie trouvee (%s absent).\n"
                "L'app robot_control est-elle lancee (sans --no-telemetry) ?"
                % LOGS.state)
    return _status_text(state, state.get("last", {}))


def _status_text(state, last):
    alive = LOGS.running(state)
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
    snap_age = LOGS.fmt_age(LOGS.snapshot)
    if snap_age is not None:
        lines.append("image     : %s (il y a %ss)" % (LOGS.snapshot, snap_age))
    return "\n".join(lines)


def t_tail(args):
    n = int(args.get("n", 20))
    types = args.get("types")
    types = set(types) if types else None
    since = LOGS.load_cursor().get("t") if args.get("since_mark") else None
    recs = LOGS.read_records(limit=n, types=types, since=since)
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
    recs = LOGS.read_records(limit=n, types={"tx_motor", "tx_servo", "tx_servo_all"})
    if not recs:
        return "Aucune trame emise enregistree (moteurs/servos)."
    now = time.time()
    out = []
    for r in recs:
        age = round(now - r.get("t", now), 1)
        if r["type"] == "tx_motor":
            out.append("-%5ss MOTOR m=%s ok=%s [%s]"
                       % (age, r.get("m"), r.get("ok"), r.get("hex")))
        elif r["type"] == "tx_servo_all":
            out.append("-%5ss SERVO4 a=%s ok=%s [%s]"
                       % (age, r.get("a"), r.get("ok"), r.get("hex")))
        else:
            out.append("-%5ss SERVO id=%s angle=%s ok=%s [%s]"
                       % (age, r.get("id"), r.get("angle"), r.get("ok"), r.get("hex")))
    return "\n".join(out)


def t_board_rx(args):
    state = LOGS.read_state()
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
    recs = LOGS.read_records(limit=n, types={"detect"})
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
    recs = LOGS.read_records(limit=n, types={"track"})
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
        since = LOGS.load_cursor().get("t", since)
    dets = LOGS.read_records(types={"detect"}, since=since)
    tracks = LOGS.read_records(types={"track"}, since=since)
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
    age = LOGS.fmt_age(LOGS.snapshot)
    if age is None:
        return ("Aucune image (%s absent). L'app ecrit une image annotee "
                "periodiquement quand elle tourne." % LOGS.snapshot)
    try:
        size = os.path.getsize(LOGS.snapshot)
    except Exception:
        size = "?"
    return ("Derniere image annotee :\n  chemin : %s\n  age    : %ss\n  taille : %s octets\n"
            "(image capturee par l'app : visages, croix centre, nx/ny, angles servo)"
            % (LOGS.snapshot, age, size))


def t_mark(args):
    """Pose un repere temporel : tail/analyze avec since_mark partent d'ici.

    Ecriture d'un simple curseur de lecture (mcp_cursor.json) : ce n'est PAS une
    commande vers l'app ni vers la carte, seulement un marque-page d'analyse.
    """
    t = time.time()
    try:
        os.makedirs(LOGS.dir, exist_ok=True)
        with open(LOGS.cursor, "w", encoding="utf-8") as f:
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
}


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        _LOG("selftest : lecture de %s" % LOGS.dir)
        _LOG(t_status({}).replace("\n", " | "))
        _LOG(t_analyze({}).replace("\n", " | "))
        return
    _LOG("demarrage (log_dir=%s)" % LOGS.dir)
    _rpc.serve(SERVER_NAME, TOOLS, HANDLERS)


if __name__ == "__main__":
    main()

"""Analyse live du suivi pan/tilt : lit robot_control.jsonl, calcule sur une
fenetre glissante l'oscillation (ratio de changements de signe du pas par axe),
la saturation du PAS de correction (step ecrete a max_step -- ce N'EST PAS une
butee d'angle servo), la perte de visage, le detecteur actif et le repos courant.
Emet UNE ligne compacte par tick ; prefixe [!] si quelque chose merite l'oeil.
Sortie = flux d'evenements pour le Monitor. Lecture seule (coexiste avec l'app).
"""
import json
import os
import time

LOG = os.path.join(os.path.dirname(__file__), "robot_control.jsonl")
WIN = 8.0     # fenetre d'analyse (s)
TAIL = 4000   # nb de lignes lues en fin de log (fenetre glissante)


def _tail_lines(n):
    """n dernieres lignes du journal, backup .1 inclus (survit a une rotation).

    La telemetrie tourne en fenetre glissante : le fichier vif est plafonne puis
    bascule en .1. Juste apres une bascule le fichier vif est court ; on complete
    donc avec la fin du .1 pour garder une fenetre d'analyse continue.
    """
    live = []
    try:
        with open(LOG, "r", encoding="utf-8", errors="ignore") as f:
            live = f.readlines()
    except FileNotFoundError:
        return None
    if len(live) >= n:
        return live[-n:]
    prev = []
    try:
        with open(LOG + ".1", "r", encoding="utf-8", errors="ignore") as f:
            prev = f.readlines()
    except FileNotFoundError:
        pass
    return (prev + live)[-n:]


def snapshot_tail(reason):
    """Fige les TAIL dernieres lignes du log dans un fichier horodate.

    Appele sur la transition suivi ON->off : les lignes de la session de suivi
    qui vont sortir de la fenetre glissante (poussees par les lignes d'apres-off)
    sont ainsi conservees pour revue, sans toucher au log d'origine.
    Retourne le chemin ecrit, ou None.
    """
    lines = _tail_lines(TAIL)
    if lines is None:
        return None
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(__file__), f"session_{ts}.jsonl")
    try:
        with open(out, "w", encoding="utf-8") as f:
            f.writelines(lines)
    except Exception:
        return None
    return out


def sign_change_ratio(steps):
    xs = [s for s in steps if abs(s) > 1e-6]
    if len(xs) < 2:
        return 0.0
    ch = sum(1 for a, b in zip(xs, xs[1:]) if (a > 0) != (b > 0))
    return ch / (len(xs) - 1)


def osc_index(steps):
    """Indice d'oscillation = frequence d'inversion x amplitude moyenne du pas (deg).

    Combine les deux signatures du pompage : des inversions de sens FREQUENTES
    (ratio de changement de signe) ET de forte AMPLITUDE. Un micro-jitter (petits
    pas qui s'inversent) donne un ratio eleve mais un indice faible ; une vraie
    oscillation (gros pas alternes) ressort. Repere : <0.3 calme, >1.0 pompage net.
    Retourne (indice, ratio, amplitude_moy).
    """
    xs = [s for s in steps if abs(s) > 1e-6]
    if len(xs) < 2:
        return 0.0, 0.0, 0.0
    ratio = sign_change_ratio(xs)
    amp = sum(abs(s) for s in xs) / len(xs)
    return ratio * amp, ratio, amp


def read_window(now):
    tracks = {"pan": [], "tilt": []}
    satur = {"pan": 0, "tilt": 0}   # pas de correction ecrete a max_step
    detects = []
    locks = []                      # etat de verrou du tracker (detect-then-track)
    raw_hits = []                   # detecteur BRUT pendant le lock (True=hit / False=miss)
    unlock_reasons = {}             # motif -> compte des de-verrouillages dans la fenetre
    pred_phases = []                # suite des phases de prediction (off|lock|coast|home)
    pred_errs = []                  # erreurs de prediction (innovation) pendant le lock
    last_hb = None
    detector = "?"
    trk = "?"                       # mode de suivi courant (none/mil/vit)
    lines = _tail_lines(TAIL)
    if lines is None:
        return None
    for ln in lines:
        try:
            r = json.loads(ln)
        except Exception:
            continue
        t = r.get("t", 0)
        ty = r.get("type")
        if ty == "heartbeat":
            last_hb = r
        if ty == "event" and r.get("msg") in ("detector", "detector_switch"):
            # detecteur demarre (detector) ou bascule reussie (detector_switch ok)
            if r.get("msg") == "detector":
                detector = r.get("detector", detector)
            elif r.get("ok"):
                detector = r.get("to", detector)
        if ty == "event" and r.get("msg") == "track_mode" and r.get("ok"):
            trk = r.get("to", trk)
        if ty == "event" and r.get("msg") == "track_unlock" and now - t <= WIN:
            why = r.get("reason") or "?"
            unlock_reasons[why] = unlock_reasons.get(why, 0) + 1
        if r.get("trk"):                 # detect/heartbeat portent le mode courant
            trk = r.get("trk")
        if now - t > WIN:
            continue
        if ty == "track" and "step" in r:
            ax = r.get("axis")
            if ax in tracks:
                tracks[ax].append(r["step"])
                if r.get("clamped"):
                    satur[ax] += 1
        elif ty == "detect":
            detects.append(bool(r.get("faces", 0)))
            if r.get("locked") is not None:
                locks.append(bool(r.get("locked")))
                # gain profil : pendant le lock, le detecteur BRUT voit-il le visage ?
                if r.get("locked") and r.get("raw_det") is not None:
                    raw_hits.append(bool(r.get("raw_det")))
            if r.get("predict") is not None:
                pred_phases.append(r.get("predict"))
            if r.get("pred_err") is not None:
                pred_errs.append(float(r.get("pred_err")))
    return (tracks, satur, detects, locks, raw_hits, unlock_reasons,
            pred_phases, pred_errs, last_hb, detector, trk)


def coast_stats(phases):
    """Compte les coasts et leur issue depuis la suite des phases de prediction.

    Un 'coast' (visage perdu, on extrapole) se termine soit par 'lock' (visage
    RETROUVE -> coast utile) soit par 'home' (abandon -> retour centre). Retourne
    (n_coast_frames, n_home_frames, recovered, failed).
    """
    n_coast = sum(1 for p in phases if p == "coast")
    n_home = sum(1 for p in phases if p == "home")
    recovered = failed = 0
    in_coast = False
    for p in phases:
        if p == "coast":
            in_coast = True
        elif in_coast and p == "lock":
            recovered += 1
            in_coast = False
        elif in_coast and p == "home":
            failed += 1
            in_coast = False
    return n_coast, n_home, recovered, failed


_prev_det = None
_known_det = "?"       # dernier detecteur reellement connu (survit a l'age-out)
_known_trk = "?"       # dernier mode de suivi connu (survit a l'age-out)
_prev_tracking = None  # etat suivi au tick precedent (pour detecter ON->off)
_ticks = 0
while True:
    now = time.time()
    _ticks += 1
    data = read_window(now)
    if data is None:
        print("en attente du log...", flush=True)
        time.sleep(4)
        continue
    (tracks, satur, detects, locks, raw_hits, unlock_reasons,
     pred_phases, pred_errs, hb, detector, trk) = data
    # si l'event detecteur a defile hors fenetre (det='?'), garder le dernier connu
    if detector == "?":
        detector = _known_det
    else:
        _known_det = detector
    if trk == "?":
        trk = _known_trk
    else:
        _known_trk = trk
    op, rp, ap_ = osc_index(tracks["pan"])
    ot, rt, at_ = osc_index(tracks["tilt"])
    np_, nt = len(tracks["pan"]), len(tracks["tilt"])
    loss = 0.0
    if detects:
        loss = 1.0 - sum(detects) / len(detects)
    # detect-then-track : taux de DE-verrouillage (cible perdue par le tracker)
    unlock = 0.0
    if locks:
        unlock = 1.0 - sum(locks) / len(locks)
    # gain profil = part des images VERROUILLEES ou le detecteur brut RATAIT le visage
    # (le tracker a tenu la ou le detecteur seul aurait lache -> le but recherche).
    profile_gain = None
    if raw_hits:
        profile_gain = 1.0 - sum(raw_hits) / len(raw_hits)
    pan = hb.get("pan") if hb else None
    tilt = hb.get("tilt") if hb else None
    conn = hb.get("connected") if hb else None
    tracking = hb.get("tracking") if hb else None
    suivi = "ON" if tracking else ("off" if tracking is not None else "?")
    # transition suivi ON->off : figer les 4000 lignes d'avant le off
    saved = None
    if _prev_tracking and tracking is False:
        saved = snapshot_tail("track_off")
    _prev_tracking = tracking
    flags = []
    # indice d'oscillation = ratio d'inversion x amplitude (deg) ; >1.0 = pompage net
    if op >= 1.0 and np_ >= 4:
        flags.append(f"OSC-PAN idx={op:.2f}(r{rp:.2f}x{ap_:.1f}deg)")
    if ot >= 1.0 and nt >= 4:
        flags.append(f"OSC-TILT idx={ot:.2f}(r{rt:.2f}x{at_:.1f}deg)")
    if satur["pan"] >= 5:
        flags.append(f"PAS-SATURE-PAN x{satur['pan']}")
    if satur["tilt"] >= 5:
        flags.append(f"PAS-SATURE-TILT x{satur['tilt']}")
    # Perte : selon le mode de suivi.
    #  - mode 'none' (detecteur seul) : miss-detecteur ; ~40-60% est le regime
    #    normal du profil en mouvement -> alerter seulement sur perte SEVERE (>=70%).
    #  - mode tracker (mil/vit) : le tracker tient a travers les trous du detecteur,
    #    donc c'est le taux de DE-verrouillage (unlock) qui compte.
    if tracking and trk == "none" and loss >= 0.7 and len(detects) >= 5:
        flags.append(f"PERTE-VISAGE-SEVERE {loss:.0%}")
    if tracking and trk not in ("none", "?") and unlock >= 0.7 and len(locks) >= 5:
        flags.append(f"CIBLE-PERDUE {unlock:.0%}(trk={trk})")
    if conn is False:
        flags.append("DECONNECTE")
    pre = "[!] " + " ".join(flags) if flags else "[ok]"
    # n'emettre que si notable (flag), changement de detecteur, ou battement /20s
    det_changed = detector != _prev_det
    _prev_det = detector
    if saved:
        print(f"[SAVE] suivi->off : {TAIL} lignes figees dans "
              f"{os.path.basename(saved)}", flush=True)
    # battement periodique seulement si le suivi est actif (silence au repos)
    heartbeat = tracking and (_ticks % 5 == 0)
    if flags or det_changed or saved or heartbeat:
        tag = " <bascule>" if det_changed else ""
        if trk not in ("none", "?"):
            perte_txt = f"unlock={unlock:.0%}({len(locks)} lk)"
            if profile_gain is not None:
                perte_txt += f" gain-profil={profile_gain:.0%}({len(raw_hits)})"
            if unlock_reasons:
                top = sorted(unlock_reasons.items(), key=lambda kv: -kv[1])
                perte_txt += " why[" + ",".join(f"{k}:{v}" for k, v in top) + "]"
        else:
            perte_txt = f"perte={loss:.0%}({len(detects)} det)"
        # prediction : temps en coast/home et issue des coasts (retrouve vs abandon)
        n_coast, n_home, recov, failed = coast_stats(pred_phases)
        if n_coast or n_home or recov or failed:
            perte_txt += (f" pred[coast={n_coast} home={n_home} "
                          f"retrouve={recov} abandon={failed}]")
        # erreur de prediction (innovation) : moyenne et pic sur la fenetre ->
        # petite = l'anticipation colle bien au mouvement reel.
        if pred_errs:
            emean = sum(pred_errs) / len(pred_errs)
            perte_txt += f" err~{emean:.3f}(max {max(pred_errs):.3f} n{len(pred_errs)})"
        print(
            f"{pre}{tag} | suivi={suivi} det={detector} trk={trk} pan={pan} tilt={tilt} | "
            f"osc-idx pan={op:.2f}({np_}) tilt={ot:.2f}({nt}) | "
            f"{perte_txt} conn={conn}",
            flush=True,
        )
    time.sleep(4)

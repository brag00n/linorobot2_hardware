#!/usr/bin/env python3
r"""main - Outil de controle du Bamboo v4 : teleop clavier + suivi de visage.

Pipeline P1 (capture + affichage + clavier) sur le thread principal ; la
detection de visage tourne dans vision.FaceTracker (P2). Les commandes moteurs
(motion) et servos pan/tilt (pan_tilt) partent vers la carte STM32 via board_link
(COM4, protocole Yahboom v4).

/!\ Couper le serveur MCP avant (il tient COM4). Premiers essais ROUES SURELEVEES.

Lancement depuis le dossier tools/ (a la racine du repo, tooling multi-cartes) :
  .venv/Scripts/python.exe -m robot_control.main --no-motion   # vision + servos
  .venv/Scripts/python.exe -m robot_control.main               # + moteurs
(installer le venv d'abord : voir tools/README.md ou tools/install.ps1)

Clavier (AZERTY) :
  Z/S       avancer / reculer           Q/D   rotation gauche / droite
  Espace    STOP moteurs                0-9   niveau de vitesse
  Fleches   pan/tilt camera (S1/S2)     J/L I/K  idem (secours)
  C         recentrer camera            F     (des)activer le suivi de visage
  M         changer de detecteur visage (haar/dnn/yunet)
  T         changer de tracker visuel   (none/mil/vit ; detect-then-track)
  P         mode prediction : off / prediction / prediction si perte (coast)
  Echap     quitter
"""
import argparse
import json
import os
import sys
import time

import cv2

from .board_link import BoardLink
from .motion import Motion
from .pan_tilt import PanTilt
from .telemetry import Telemetry
from .vision import FaceTracker, apply_flip, apply_rotate

BACKENDS = {"msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW, "any": cv2.CAP_ANY}
MOVE_WATCHDOG_S = 0.35        # arret auto si aucune touche mouvement recente

# Codes des fleches renvoyes par cv2.waitKeyEx : varient selon le build OpenCV.
# On accepte plusieurs familles connues (Windows highgui, Qt/GTK, et la variante
# ou seuls les octets bas 0x25..0x28 remontent). L'appui reel est aussi logue
# (type "key") pour pouvoir ajouter un code manquant au besoin.
KEYS_LEFT = {2424832, 65361, 0x250000, 37, 63234}
KEYS_UP = {2490368, 65362, 0x260000, 38, 63232}
KEYS_RIGHT = {2555904, 65363, 0x270000, 39, 63235}
KEYS_DOWN = {2621440, 65364, 0x280000, 40, 63233}
SERVO_STEP = 2                # deg par appui fleche (reglage cam manuel)


def _list_devices():
    """Noms des cameras dans l'ordre des index (DirectShow via pygrabber).

    Retourne [] si pygrabber est absent : on retombe alors sur un scan brut.
    Cet ordre correspond aux index CAP_DSHOW ; il s'aligne en pratique avec
    CAP_MSMF (et de toute facon chaque index est teste par lecture reelle).
    """
    try:
        from pygrabber.dshow_graph import FilterGraph
        return FilterGraph().get_input_devices()
    except Exception:
        return []


def _camera_order(scan_max, prefer=None, skip=None):
    """Ordre d'essai des index : prefere `prefer`, repousse `skip` en dernier.

    Sert a distinguer la camera USB pan-tilt de la webcam integree du PC.
    Retourne [(index, nom_ou_None), ...]. Sans noms dispo -> 0..scan_max-1.
    """
    names = _list_devices()
    skip = [s.strip().lower() for s in (skip or []) if s.strip()]
    prefer = (prefer or "").strip().lower()
    if not names:
        return [(i, None) for i in range(scan_max)]
    indexed = list(enumerate(names))

    def is_skip(n):
        return any(s in n.lower() for s in skip)

    def is_pref(n):
        return bool(prefer) and prefer in n.lower()

    preferred = [(i, n) for i, n in indexed if is_pref(n)]
    neutral = [(i, n) for i, n in indexed if not is_pref(n) and not is_skip(n)]
    skipped = [(i, n) for i, n in indexed if not is_pref(n) and is_skip(n)]
    return preferred + neutral + skipped


def _configure_cap(cap, w, h, fps):
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)


def _try_open(index, backend, w, h, fps):
    """Ouvre + configure + LIT une image. Retourne le cap si valide, sinon None.

    Un cap peut s'ouvrir sans jamais delivrer d'image (backend indisponible
    pour cet index) : on exige une lecture reussie pour valider la camera.
    """
    cap = cv2.VideoCapture(index, BACKENDS[backend])
    if not cap.isOpened():
        cap.release()
        return None
    _configure_cap(cap, w, h, fps)
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap


def open_camera(index, backend, w, h, fps, scan_max=6,
                prefer_name=None, skip_names=None, tel=None):
    """Ouvre la camera. `index` peut etre "auto" ou un entier.

    1) si un index precis est demande, on l'essaie sur le backend demande ;
    2) sinon (ou en cas d'echec) on SCANNE les cameras, dans un ORDRE guide par
       les noms (prefere `prefer_name`, ignore `skip_names` comme la webcam
       integree), et on retient la premiere qui ouvre ET delivre une image.
    Retourne (cap, index_utilise, backend_utilise) ou (None, None, None).
    """
    backends = [backend] + [b for b in ("msmf", "dshow", "any") if b != backend]
    if str(index).lower() != "auto":
        idx = int(index)
        cap = _try_open(idx, backend, w, h, fps)
        if cap is not None:
            if tel:
                tel.log("event", msg="camera_open", index=idx,
                        backend=backend, mode="demande")
            return cap, idx, backend
        print(f"Camera index {idx} (backend {backend}) indisponible -> scan auto...")
    order = _camera_order(scan_max, prefer_name, skip_names)
    listing = ", ".join(f"{i}:{n or '?'}" for i, n in order)
    print(f"Cameras (ordre d'essai) : {listing}")
    # Camera en boucle EXTERNE : on epuise tous les backends de la camera
    # preferee avant de passer a la suivante (sinon un backend qui echoue sur
    # la bonne cam mais marche sur la webcam PC ferait un mauvais choix).
    for idx, name in order:
        for b in backends:
            cap = _try_open(idx, b, w, h, fps)
            if cap is not None:
                print(f"Camera retenue : index {idx} ({name or '?'}) backend {b}.")
                if tel:
                    tel.log("event", msg="camera_open", index=idx, name=name,
                            backend=b, mode="auto")
                return cap, idx, b
    if tel:
        tel.log("event", msg="camera_fail", scanned=scan_max, devices=listing)
    return None, None, None


def parse_args():
    ap = argparse.ArgumentParser(description="Controle Bamboo v4 : teleop + suivi visage")
    ap.add_argument("--port", default="COM4", help="port serie carte STM32 (defaut COM4)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--index", default="auto",
                    help="index camera USB, ou 'auto' pour scanner (defaut auto)")
    ap.add_argument("--camera-name", default="USB",
                    help="sous-chaine du nom de la cam a PREFERER en auto (defaut 'USB')")
    ap.add_argument("--skip-name", default="Integrated,IR Camera",
                    help="noms a eviter en auto, separes par des virgules "
                         "(defaut cameras integrees du PC)")
    ap.add_argument("--size", default="1280x720", help="resolution P1 (ex 640x480)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--backend", default="msmf", choices=list(BACKENDS))
    ap.add_argument("--det-width", type=int, default=320, help="largeur detection P2 (px)")
    ap.add_argument("--min-size", type=int, default=24, help="taille min visage en P2 (px)")
    ap.add_argument("--detector", default="yunet", choices=["haar", "dnn", "yunet"],
                    help="detecteur visage : haar (leger), dnn (res10, robuste profil), "
                         "yunet (le plus robuste, defaut ; modele deja sur disque)")
    ap.add_argument("--det-conf", type=float, default=0.6,
                    help="seuil de confiance dnn/yunet (defaut 0.6 ; YuNet score ~0.9 de face)")
    ap.add_argument("--yunet-model", default=None,
                    help="chemin du modele YuNet .onnx (defaut : face_detection_yunet_2023mar.onnx "
                         "dans resources/Other/face_detection_model)")
    # --- suivi detect-then-track (tracker visuel par-dessus le detecteur) ---
    ap.add_argument("--track-mode", default="auto", choices=["none", "mil", "vit", "auto"],
                    help="tracker visuel : none (detecteur seul), mil, vit, "
                         "auto (vit si dispo sinon mil ; defaut). Suit le visage meme de profil")
    ap.add_argument("--vit-model", default=None,
                    help="chemin du modele VitTrack .onnx (defaut : object_tracking_vittrack_2023sep.onnx "
                         "dans resources/Other/face_detection_model)")
    ap.add_argument("--redetect-ms", type=float, default=400.0,
                    help="periode de re-detection pendant le verrou (re-ancrage/anti-derive)")
    ap.add_argument("--track-score-min", type=float, default=0.30,
                    help="score de suivi (Vit) sous lequel la cible est perdue -> re-acquisition")
    ap.add_argument("--track-hold-ms", type=float, default=3000.0,
                    help="duree max de suivi sans reconfirmation detecteur (profil) avant relache")
    ap.add_argument("--track-max-area", type=float, default=0.5,
                    help="aire max de la box suivie (fraction du cadre) avant perte (anti-grossissement)")
    ap.add_argument("--track-max-grow", type=float, default=3.0,
                    help="facteur de grossissement max de la box depuis l'ancrage avant perte")
    # --- prediction de trajectoire (Kalman) : coast + anticipation ---
    ap.add_argument("--predict-mode", default="anticip",
                    choices=["off", "coast", "anticip"],
                    help="prediction de trajectoire : off, coast (continue sur la "
                         "vitesse a la PERTE seulement), anticip (coast + anticipation "
                         "continue ; defaut). Touche P pour cycler")
    ap.add_argument("--predict-ms", type=float, default=700.0,
                    help="duree max (ms) de poursuite predite apres perte avant retour au centre")
    ap.add_argument("--predict-lead-ms", type=float, default=120.0,
                    help="avance temporelle (ms) de l'anticipation (position + vitesse x lead)")
    ap.add_argument("--predict-min-speed", type=float, default=0.4,
                    help="vitesse normalisee/s mini pour declencher un coast a la perte")
    ap.add_argument("--flip", default="v", choices=["none", "v", "h", "180"])
    ap.add_argument("--rotate", type=float, default=0.0,
                    help="rotation libre de l'image en deg (convention OpenCV : "
                         "positif = anti-horaire), pour redresser une camera de "
                         "travers. Appliquee apres --flip, dimensions conservees")
    ap.add_argument("--pan-gain", type=float, default=10.0)
    ap.add_argument("--tilt-gain", type=float, default=6.0)
    ap.add_argument("--deadzone", type=float, default=0.14,
                    help="demi-cote de la SURFACE centrale visee (rectangle) : "
                         "tant que le visage y est, aucune correction (anti-oscillation)")
    ap.add_argument("--dead-hyst", type=float, default=0.05,
                    help="marge d'hysteresis : une fois stabilise dans la surface, "
                         "il faut ressortir de (deadzone+marge) pour re-enclencher")
    ap.add_argument("--max-step", type=float, default=4.0,
                    help="deplacement max en deg VISE par detection (borne la cible)")
    ap.add_argument("--max-vel", type=float, default=120.0,
                    help="vitesse angulaire max du servo en deg/s (lissage anti-flou)")
    ap.add_argument("--max-accel", type=float, default=400.0,
                    help="acceleration max en deg/s^2 (demarrage/freinage progressifs)")
    ap.add_argument("--no-smooth", action="store_true",
                    help="desactive le lissage : saut immediat a la cible (ancien comportement)")
    ap.add_argument("--invert-pan", action="store_true", help="inverser le sens du pan")
    ap.add_argument("--invert-tilt", action="store_true", help="inverser le sens du tilt")
    ap.add_argument("--pan-min", type=float, default=17.0)
    ap.add_argument("--pan-max", type=float, default=178.0)
    ap.add_argument("--pan-home", type=float, default=88.0,
                    help="repos pan = cam de face (mesure : 88, centre de la course)")
    ap.add_argument("--tilt-min", type=float, default=20.0)
    ap.add_argument("--tilt-max", type=float, default=70.0,
                    help="butee haute du tilt (course verticale ; a ajuster au montage)")
    ap.add_argument("--tilt-home", type=float, default=42.0,
                    help="repos tilt = cam de face (mesure : 42)")
    ap.add_argument("--max-pwm", type=int, default=30, help="borne PWM moteur en %% (defaut 30)")
    ap.add_argument("--no-motion", action="store_true",
                    help="desactive les moteurs (vision + servos seulement)")
    ap.add_argument("--no-telemetry", action="store_true",
                    help="desactive le journal de telemetrie (logs/ pour le MCP)")
    ap.add_argument("--log-dir", default=None,
                    help="dossier des logs telemetrie (defaut robot_control/logs)")
    ap.add_argument("--log-budget-mb", type=float, default=10.0,
                    help="budget disque total du journal en Mo (fenetre glissante "
                         "= fichier vif + 1 backup ; defaut 10)")
    return ap.parse_args()


def draw_overlay(frame, faces, main, nx, ny, area_pct, det_fps, disp_fps,
                 tracking, pt, snap, motion_on, detector="", tstate=None):
    fh, fw = frame.shape[:2]
    tstate = tstate or {}
    locked = bool(tstate.get("locked"))
    src = tstate.get("src")
    # couleur de la cible verrouillee selon son ORIGINE :
    #  - "track" (prediction tracker seul, ex. profil) -> MAGENTA = zone tracker
    #  - detect/redetect/reanchor/recenter (recalee sur une detection) -> CYAN = zone detect
    on_tracker = locked and src == "track"
    main_color = (255, 0, 255) if on_tracker else (255, 200, 0)
    for (x, y, w, h) in faces:
        is_main = (main is not None and (x, y, w, h) == main)
        if is_main and locked:
            color = main_color
            thick = 3
        else:
            color = (0, 255, 0) if is_main else (0, 180, 255)
            thick = 2
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, thick)
    # indicateur "surface detectee" : box detecteur BRUTE pendant le lock (fin, jaune).
    # Permet de VOIR ou le detecteur voit le visage vs ou le tracker le suit.
    raw_box = tstate.get("raw_box")
    if locked and raw_box is not None:
        rx, ry, rw, rh = raw_box
        cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 255), 1)
        cv2.putText(frame, "det", (rx, max(0, ry - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    if main is not None:
        x, y, w, h = main
        cx, cy = int(x + w / 2), int(y + h / 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(frame, f"nx={nx:+.2f} ny={ny:+.2f} aire={area_pct:.1f}%",
                    (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    # croix centre
    cv2.line(frame, (fw // 2, 0), (fw // 2, fh), (80, 80, 80), 1)
    cv2.line(frame, (0, fh // 2), (fw, fh // 2), (80, 80, 80), 1)
    # surface centrale visee (rectangle zone morte) : cible du suivi
    dzx = int(pt.deadzone * fw / 2.0)
    dzy = int(pt.deadzone * fh / 2.0)
    in_zone = (main is not None and nx is not None
               and abs(nx) <= pt.deadzone and abs(ny) <= pt.deadzone)
    zc = (0, 200, 0) if in_zone else (0, 165, 255)
    cv2.rectangle(frame, (fw // 2 - dzx, fh // 2 - dzy),
                  (fw // 2 + dzx, fh // 2 + dzy), zc, 1)

    # --- point PREDIT (Kalman) : affiche des qu'il existe, dans tous les modes ---
    # orange, distinct du point rouge (mesure) ; fleche = direction/amplitude du
    # deplacement anticipe. En coast (visage perdu), part du centre de l'image.
    pnx, pny = tstate.get("pred_nx"), tstate.get("pred_ny")
    if pnx is not None and pny is not None:
        px = int(round(fw / 2.0 + pnx * fw / 2.0))
        py = int(round(fh / 2.0 + pny * fh / 2.0))
        # borne au cadre : un point predit hors champ (|.|>1) reste visible au bord
        px = max(6, min(fw - 6, px))
        py = max(6, min(fh - 6, py))
        orange = (0, 140, 255)
        if main is not None:
            ox, oy, ow, oh = main
            src_pt = (int(ox + ow / 2), int(oy + oh / 2))   # depuis le visage suivi
        else:
            src_pt = (fw // 2, fh // 2)                      # coast : depuis le centre
        cv2.arrowedLine(frame, src_pt, (px, py), orange, 2, tipLength=0.3)
        cv2.circle(frame, (px, py), 6, orange, 2)
        perr = tstate.get("pred_err")
        lbl = "pred" if perr is None else f"pred err={perr:.3f}"
        cv2.putText(frame, lbl, (px + 8, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, orange, 1)

    batt = f"{snap['battery']:.1f}V" if snap.get("battery") is not None else "?"
    yaw = f"{snap['yaw']:+.0f}deg" if snap.get("yaw") is not None else "?"
    link = "COM OK" if snap.get("ok") else "COM --"
    trk = tstate.get("mode", "none")
    sc = tstate.get("score")
    trk_txt = f"trk={trk}"
    if trk != "none":
        zone = "TRACK" if on_tracker else "DETECT"
        trk_txt += f" lock={'ON(' + str(src) + '/' + zone + ')' if locked else 'off'}"
        if sc is not None:
            trk_txt += f" sc={sc:.2f}"
        rd = tstate.get("raw_det")
        if locked and rd is not None:
            trk_txt += f" det:{'hit' if rd else 'miss'}"
    pm = tstate.get("predict_mode", "off")
    if pm != "off":                       # prediction active meme sans tracker visuel
        psp = tstate.get("pred_speed") or 0.0
        trk_txt += f" pred={pm}:{tstate.get('predict', 'off')} v={psp:.2f}"
        perr = tstate.get("pred_err")     # erreur de prediction (innovation) si dispo
        if perr is not None:
            trk_txt += f" err={perr:.3f}"
    l1 = (f"P1={disp_fps:.0f}fps P2={det_fps:.0f}fps  visages={len(faces)}  "
          f"det={detector}  {trk_txt}  suivi={'ON' if tracking else 'off'}  "
          f"moteurs={'ON' if motion_on else 'OFF'}")
    l2 = (f"pan(S1)={pt.angleH:.0f}  tilt(S2)={pt.angleV:.0f}  "
          f"batt={batt}  yaw={yaw}  {link}")
    l3 = ("Z/S avance  Q/D rotation  Espace STOP  Fleches pan/tilt  C centre  "
          "F suivi  M detecteur  T tracker  P prediction  0-9 vit  Echap")
    cv2.putText(frame, l1, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, l2, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    # indicateur de FLUIDITE : plus grand pas entre 2 positions calculees (deg/image).
    # vert = doux, orange = soutenu, rouge = coup de butoir (saut brusque, ex. lag dt).
    ms = pt.motion_stats()
    step_max = ms["step_max"]
    vmax = max(abs(ms["vel_pan"]), abs(ms["vel_tilt"]))
    mv_col = (0, 220, 0) if step_max < 2.0 else (0, 165, 255) if step_max < 4.0 else (0, 0, 255)
    cv2.putText(frame, f"pas-max={step_max:.1f}deg  v={vmax:.0f}deg/s", (10, 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, mv_col, 2)
    cv2.putText(frame, l3, (10, fh - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def main():
    args = parse_args()
    try:
        w, h = (int(v) for v in args.size.lower().split("x"))
    except ValueError:
        sys.exit(f"--size invalide : {args.size!r}")

    motion_on = not args.no_motion

    tel = Telemetry(log_dir=args.log_dir, enabled=not args.no_telemetry,
                    max_bytes=int(max(1.0, args.log_budget_mb) * 1_000_000 / 2))
    tel.log("event", msg="start", motion=motion_on, port=args.port,
            index=str(args.index), backend=args.backend, size=f"{w}x{h}",
            pan_gain=args.pan_gain, tilt_gain=args.tilt_gain,
            deadzone=args.deadzone, dead_hyst=args.dead_hyst, max_step=args.max_step,
            invert_pan=args.invert_pan, invert_tilt=args.invert_tilt)

    link = BoardLink(args.port, args.baud, telemetry=tel)
    time.sleep(0.3)                          # laisse le thread lecteur s'ouvrir
    motion = Motion(link, max_pwm=args.max_pwm)
    pt = PanTilt(link, pan_gain=args.pan_gain, tilt_gain=args.tilt_gain,
                 invert_pan=args.invert_pan, invert_tilt=args.invert_tilt,
                 deadzone=args.deadzone, dead_hyst=args.dead_hyst, max_step=args.max_step,
                 pan_min=args.pan_min, pan_max=args.pan_max, pan_home=args.pan_home,
                 tilt_min=args.tilt_min, tilt_max=args.tilt_max, tilt_home=args.tilt_home,
                 max_vel=args.max_vel, max_accel=args.max_accel, smooth=not args.no_smooth,
                 telemetry=tel)
    tracker = FaceTracker(det_width=args.det_width, min_size=args.min_size,
                          detector=args.detector, conf=args.det_conf,
                          yunet_model=args.yunet_model,
                          track_mode=args.track_mode, vit_model=args.vit_model,
                          redetect_ms=args.redetect_ms, score_min=args.track_score_min,
                          hold_ms=args.track_hold_ms,
                          max_area_frac=args.track_max_area, max_grow=args.track_max_grow,
                          predict_mode=args.predict_mode, predict_ms=args.predict_ms,
                          predict_lead_ms=args.predict_lead_ms,
                          predict_min_speed=args.predict_min_speed)
    ok, msg = tracker.ready()
    if not ok:
        tel.log("event", msg="fatal", err=f"detecteur:{msg}")
        tel.close()
        link.close()
        sys.exit(f"Detecteur '{args.detector}' indisponible : {msg}")
    avail = tracker.available_detectors()
    tel.log("event", msg="detector", detector=args.detector, available=avail)
    print(f"Detecteur : {args.detector}  (disponibles : {', '.join(avail)})")
    # Garde suivi : si le tracker resolu n'est pas chargeable, repli sur 'none'
    # (la detection seule reste utile ; ne pas planter).
    tok, tmsg = tracker.track_ready()
    if not tok:
        print(f"[main] tracker '{tracker.track_mode}' indisponible ({tmsg}) -> mode none")
        tracker.set_track_mode("none")
    tavail = tracker.available_trackers()
    tel.log("event", msg="track_mode", to=tracker.track_mode, ok=True,
            available=tavail, info="init")
    print(f"Suivi : {tracker.track_mode}  (trackers disponibles : {', '.join(tavail)})")
    tracker.start()

    cap, used_index, used_backend = open_camera(
        args.index, args.backend, w, h, args.fps,
        prefer_name=args.camera_name,
        skip_names=args.skip_name.split(","), tel=tel)
    if cap is None:
        tracker.stop()
        link.close()
        tel.close()
        sys.exit("Aucune camera exploitable trouvee (scan index 0..5, "
                 "backends msmf/dshow/any). Verifier le branchement.")
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera index {used_index} : {aw}x{ah} (backend {used_backend}, "
          f"flip {args.flip}, rotate {args.rotate})")
    print(f"Carte : {args.port} @ {args.baud}  |  moteurs {'ACTIFS' if motion_on else 'desactives'}")
    print("Fenetre ouverte. F=suivi, Echap=quitter.")

    pt.center()                              # pan 88 / tilt 42 (cam de face)

    tracking = False
    last_seq = -1                            # derniere detection asservie
    last_locked = None                       # dernier etat de verrou (transitions)
    moving = False
    last_move_ts = 0.0
    disp_t0 = time.time()
    disp_n = 0
    disp_fps = 0.0
    read_fail = 0
    hb_t0 = time.time()                      # cadence heartbeat telemetrie
    slew_t = time.time()                     # horloge du lisseur de mouvement (dt)
    win = "Bamboo v4 - controle (teleop + suivi visage)"

    # Bascule de detecteur : touche M (cycle) ET fichier de controle (MCP).
    detect_cycle = tracker.available_detectors()   # ordre de cycle (disponibles)
    track_cycle = tracker.available_trackers()      # cycle des trackers (touche T)
    ctrl_path = os.path.join(tel.log_dir, "control.json")
    ctrl_mtime = 0.0

    def switch_detector(name, source):
        ok, msg = tracker.set_detector(name)
        tel.log("event", msg="detector_switch", to=name, ok=ok,
                info=msg, source=source)
        if not ok:
            print(f"[main] bascule '{name}' refusee ({source}) : {msg}")
        return ok

    def switch_track(name, source):
        ok, msg = tracker.set_track_mode(name)
        tel.log("event", msg="track_mode", to=name, ok=ok,
                info=msg, source=source)
        if not ok:
            print(f"[main] tracker '{name}' refuse ({source}) : {msg}")
        return ok

    def switch_predict(name, source):
        # name=None -> cycle au mode suivant (touche P) ; sinon mode explicite (MCP)
        ok, res = (tracker.cycle_predict_mode() if name is None
                   else tracker.set_predict_mode(name))
        tel.log("event", msg="predict_mode", to=res, ok=ok, source=source)
        if not ok:
            print(f"[main] mode predict '{name}' refuse ({source}) : {res}")
        return ok

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                read_fail += 1
                if read_fail > 30:
                    print("Lecture camera echouee de facon repetee, arret.")
                    break
                time.sleep(0.005)
                continue
            read_fail = 0
            frame = apply_flip(frame, args.flip)
            frame = apply_rotate(frame, args.rotate)
            tracker.publish(frame)

            faces, main, nx, ny, area_pct, det_fps, seq = tracker.latest()
            tstate = tracker.track_state()

            disp_n += 1
            now = time.time()
            if now - disp_t0 >= 1.0:
                disp_fps = disp_n / (now - disp_t0)
                disp_t0 = now
                disp_n = 0

            # Suivi : asservit pan/tilt UNE fois par nouvelle detection (seq),
            # sinon on corrige plusieurs fois vers une position perimee -> pompage.
            if seq != last_seq:
                score = tstate.get("score")
                pspeed = tstate.get("pred_speed")
                perr = tstate.get("pred_err")
                tel.log("detect", seq=seq, faces=len(faces),
                        nx=None if nx is None else round(nx, 4),
                        ny=None if ny is None else round(ny, 4),
                        area=round(area_pct, 2), det_fps=round(det_fps, 1),
                        tracking=tracking, trk=tstate.get("mode"),
                        locked=tstate.get("locked"), src=tstate.get("src"),
                        raw_det=tstate.get("raw_det"),
                        score=None if score is None else round(score, 3),
                        predict=tstate.get("predict"),
                        pred_speed=None if pspeed is None else round(pspeed, 3),
                        pred_err=None if perr is None else round(perr, 4))
                # transition de verrou (lock/unlock) -> event dedie pour l'analyse
                if tstate.get("mode") != "none" and tstate.get("locked") != last_locked:
                    unlocked = not tstate.get("locked")
                    tel.log("event",
                            msg="track_unlock" if unlocked else "track_lock",
                            src=tstate.get("src"),
                            reason=tstate.get("unlock_reason") if unlocked else None,
                            score=None if score is None else round(score, 3))
                    last_locked = tstate.get("locked")
                if tracking and nx is not None:
                    pt.track(nx, ny)          # cible = mesure, anticipation ou coast
                elif tracking and tstate.get("predict") == "home":
                    pt.return_home()          # coast expire : retour doux au centre
            last_seq = seq

            # Lisseur de mouvement : rapproche le servo de la cible a chaque image,
            # borne en vitesse/acceleration (anti-flou). No-op si deja sur la cible.
            now_s = time.time()
            dt = now_s - slew_t
            slew_t = now_s
            pt.slew(min(dt, 0.1))            # borne dt (evite un saut apres un lag)

            snap = link.snapshot()
            draw_overlay(frame, faces, main, nx, ny, area_pct, det_fps, disp_fps,
                         tracking, pt, snap, motion_on, detector=tracker.detector,
                         tstate=tstate)
            cv2.imshow(win, frame)
            tel.snapshot(frame)              # image annotee (throttlee) pour analyse

            now2 = time.time()
            if now2 - hb_t0 >= 2.0:          # battement periodique (etat vivant)
                hb_t0 = now2
                hb_score = tstate.get("score")
                tel.log("heartbeat", disp_fps=round(disp_fps, 1),
                        det_fps=round(det_fps, 1), tracking=tracking,
                        pan=round(pt.angleH, 1), tilt=round(pt.angleV, 1),
                        connected=link.connected, batt=snap.get("battery"),
                        yaw=snap.get("yaw"), ok=snap.get("ok"),
                        bad=snap.get("bad"), trk=tstate.get("mode"),
                        locked=tstate.get("locked"), src=tstate.get("src"),
                        score=None if hb_score is None else round(hb_score, 3),
                        step_max=round(pt.motion_stats()["step_max"], 2))

            # Commande externe (MCP -> logs/control.json) : bascule detecteur
            try:
                m = os.path.getmtime(ctrl_path)
                if m != ctrl_mtime:
                    ctrl_mtime = m
                    with open(ctrl_path, encoding="utf-8") as f:
                        cmd = json.load(f)
                    d = cmd.get("detector")
                    if d and d != tracker.detector:
                        switch_detector(d, "mcp")
                    tm = cmd.get("track_mode")
                    if tm and tm != tracker.track_mode:
                        switch_track(tm, "mcp")
                    pm = cmd.get("predict_mode")
                    if pm and pm != tracker.predict_mode:
                        switch_predict(pm, "mcp")
            except (OSError, ValueError):
                pass

            key = cv2.waitKeyEx(1)
            moved_now = False
            if key != -1:
                k = key & 0xFF
                c = chr(k).lower() if 32 <= k < 127 else ""
                # Journalise CHAQUE touche : permet de retrouver le code reel
                # des fleches sur ce build (MCP : tail types=["key"]).
                tel.log("key", code=key, k=k, c=c)
                if key == 27:                        # Echap
                    tel.log("event", msg="quit")
                    break
                # --- camera pan/tilt : fleches (primaire, plusieurs codes) ---
                elif key in KEYS_LEFT:
                    pt.nudge_pan(-SERVO_STEP)
                elif key in KEYS_RIGHT:
                    pt.nudge_pan(+SERVO_STEP)
                elif key in KEYS_UP:
                    pt.nudge_tilt(+SERVO_STEP)
                elif key in KEYS_DOWN:
                    pt.nudge_tilt(-SERVO_STEP)
                elif c == "f":
                    tracking = not tracking
                    tel.log("event", msg="tracking", on=tracking)
                elif c == "c":
                    pt.center()
                    tel.log("event", msg="center", pan=round(pt.angleH, 1),
                            tilt=round(pt.angleV, 1))
                elif c == "m":
                    # cycle sur les detecteurs disponibles
                    if detect_cycle:
                        cur = tracker.detector
                        i = detect_cycle.index(cur) if cur in detect_cycle else -1
                        switch_detector(detect_cycle[(i + 1) % len(detect_cycle)], "key")
                elif c == "t":
                    # cycle sur les trackers visuels disponibles (none/mil/vit)
                    if track_cycle:
                        cur = tracker.track_mode
                        i = track_cycle.index(cur) if cur in track_cycle else -1
                        switch_track(track_cycle[(i + 1) % len(track_cycle)], "key")
                elif c == "p":
                    # cycle du mode de prediction (off -> prediction -> prediction si perte)
                    switch_predict(None, "key")
                # --- camera pan/tilt : lettres (secours) ---
                elif c == "j":
                    pt.nudge_pan(-SERVO_STEP)
                elif c == "l":
                    pt.nudge_pan(+SERVO_STEP)
                elif c == "i":
                    pt.nudge_tilt(+SERVO_STEP)
                elif c == "k":
                    pt.nudge_tilt(-SERVO_STEP)
                # --- vitesse ---
                elif c.isdigit():
                    motion.set_speed(int(c))
                # --- STOP ---
                elif k == 32:                        # Espace
                    if motion_on:
                        motion.stop()
                    moving = False
                # --- deplacement (momentane + watchdog) ---
                elif c in ("z", "s", "q", "d"):
                    if motion_on:
                        if c == "z":
                            motion.forward()
                        elif c == "s":
                            motion.backward()
                        elif c == "q":
                            motion.rotate_left()
                        elif c == "d":
                            motion.rotate_right()
                    moving = True
                    moved_now = True
                    last_move_ts = now

            # Watchdog : coupe les moteurs si aucune touche mouvement recente
            if moving and not moved_now and (now - last_move_ts) > MOVE_WATCHDOG_S:
                if motion_on:
                    motion.stop()
                moving = False
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if motion_on:
                link.stop()
        except Exception:
            pass
        tracker.stop()
        cap.release()
        cv2.destroyAllWindows()
        link.close()
        tel.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
r"""RobotMain - Orchestrateur du Bamboo v4 : teleop clavier + suivi de visage.

Equivalent du RobotMain.py de Bambou4WD_python (2018) : cable les couches
transverses et les modules metier, et tient le pipeline P1 (capture + affichage
+ clavier) sur le thread principal. Portage de l'ancien robot_control.main
(comportement INCHANGE ; memes flags CLI) vers l'architecture en couches :

  lib.Telemetry                          journal structure (analyse externe / MCP)
  communication.RobotComSerial           liaison serie STM32 (COM4, Yahboom v4)
  device.motion.RobotMotorDrive          4 moteurs du chassis (teleop clavier)
  device.motion.RobotServoMotor          servos camera pan (S1) / tilt (S2)
  device.sensor.RobotSensorWebCam        camera USB (capture P1 + flip/rotate)
  modules.tracking.RobotWebCamMotorized  suivi de visage (detection P2 + Kalman
                                         + asservissement des servos)

Organisation du fichier :
  - parse_args()                    definition des flags CLI (inchanges) ;
  - _overlay_*() / draw_overlay()   dessin de l'incrustation (fonctions pures) ;
  - RobotControlApp                 orchestrateur : porte l'etat de la boucle P1
                                    (setup, clavier, asservissement, MCP, run) ;
  - main()                          point d'entree.

/!\ Couper le serveur MCP avant (il tient COM4). Premiers essais ROUES SURELEVEES.

Lancement depuis le dossier tools/ (a la racine du repo, tooling multi-cartes) :
  .venv/Scripts/python.exe -m robot_control.RobotMain --no-motion   # vision + servos
  .venv/Scripts/python.exe -m robot_control.RobotMain               # + moteurs
  ... --headless      # sans fenetre ni clavier (suivi + telemetrie + serveur MCP)
  ... --board-only    # pur pilote COM4 pour le MCP (ni camera ni suivi ; force headless)
(la version figee de reference reste lancable : -m robot_control.old ...)

Le serveur de commandes MCP (socket loopback 127.0.0.1, cf. mcp/gateway.py) tourne
dans tous les modes : le serveur d'action MCP relaie ses commandes carte/suivi ici
tant que l'app tient COM4. Port configurable via BAMBOU_MCP_PORT (defaut 8787).

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
import os
import sys
import time

import cv2

from .lib.Telemetry import Telemetry
from .communication.RobotComSerial import RobotComSerial
from .device.motion.RobotMotorDrive import RobotMotorDrive
from .device.motion.RobotServoMotor import RobotServoMotor
from .device.sensor.RobotSensorWebCam import RobotSensorWebCam, BACKENDS
from .modules.tracking.RobotWebCamMotorized import RobotWebCamMotorized
from .mcp import gateway

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


# ===========================================================================
# Ligne de commande
# ===========================================================================
def parse_args():
    """Definit et lit les flags CLI (identiques a robot_control.old)."""
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
    # --- usage sans IHM (pilotage MCP par socket ; voir mcp/gateway.py) ---
    ap.add_argument("--headless", action="store_true",
                    help="sans fenetre ni clavier : garde capture + suivi + "
                         "telemetrie + serveur de commandes MCP (analyse live OK)")
    ap.add_argument("--board-only", action="store_true",
                    help="pur pilote COM4 : ni camera ni suivi, seulement liaison "
                         "serie + telemetrie carte + serveur MCP (force --headless)")
    return ap.parse_args()


# ===========================================================================
# Incrustation (overlay) : fonctions de dessin pures, sans etat
# ===========================================================================
def _overlay_detections(frame, faces, main, locked, main_color, tstate):
    """Dessine les rectangles des visages + la box detecteur brute pendant lock.

    Le visage principal verrouille prend `main_color` (couleur codant l'origine
    de la cible) ; sinon vert (principal non verrouille) ou orange (autre visage).
    La box detecteur BRUTE (fine, jaune, label "det") montre ou le detecteur voit
    le visage vs ou le tracker le suit.
    """
    for (x, y, w, h) in faces:
        is_main = (main is not None and (x, y, w, h) == main)
        if is_main and locked:
            color = main_color
            thick = 3
        else:
            color = (0, 255, 0) if is_main else (0, 180, 255)
            thick = 2
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, thick)
    raw_box = tstate.get("raw_box")
    if locked and raw_box is not None:
        rx, ry, rw, rh = raw_box
        cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 255), 1)
        cv2.putText(frame, "det", (rx, max(0, ry - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)


def _overlay_main_marker(frame, main, nx, ny, area_pct):
    """Marque le centre du visage principal (point rouge) + nx/ny/aire au-dessus."""
    if main is None:
        return
    x, y, w, h = main
    cx, cy = int(x + w / 2), int(y + h / 2)
    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
    cv2.putText(frame, f"nx={nx:+.2f} ny={ny:+.2f} aire={area_pct:.1f}%",
                (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


def _overlay_reticle(frame, pt, main, nx, ny):
    """Dessine la croix de centrage + la SURFACE centrale visee (zone morte).

    La zone morte passe au vert quand le visage y est (aucune correction), sinon
    reste orange.
    """
    fh, fw = frame.shape[:2]
    cv2.line(frame, (fw // 2, 0), (fw // 2, fh), (80, 80, 80), 1)
    cv2.line(frame, (0, fh // 2), (fw, fh // 2), (80, 80, 80), 1)
    dzx = int(pt.deadzone * fw / 2.0)
    dzy = int(pt.deadzone * fh / 2.0)
    in_zone = (main is not None and nx is not None
               and abs(nx) <= pt.deadzone and abs(ny) <= pt.deadzone)
    zc = (0, 200, 0) if in_zone else (0, 165, 255)
    cv2.rectangle(frame, (fw // 2 - dzx, fh // 2 - dzy),
                  (fw // 2 + dzx, fh // 2 + dzy), zc, 1)


def _overlay_prediction(frame, tstate, main):
    """Dessine le point PREDIT (Kalman) et la fleche vers lui, si present.

    Orange, distinct du point rouge (mesure). La fleche part du visage suivi, ou
    du centre de l'image en coast (visage perdu). Le point est borne au cadre pour
    rester visible meme si la prediction sort du champ.
    """
    pnx, pny = tstate.get("pred_nx"), tstate.get("pred_ny")
    if pnx is None or pny is None:
        return
    fh, fw = frame.shape[:2]
    px = int(round(fw / 2.0 + pnx * fw / 2.0))
    py = int(round(fh / 2.0 + pny * fh / 2.0))
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


def _track_status_text(tstate, on_tracker):
    """Construit le fragment texte d'etat du tracker + prediction pour le HUD."""
    trk = tstate.get("mode", "none")
    sc = tstate.get("score")
    trk_txt = f"trk={trk}"
    if trk != "none":
        locked, src = tstate.get("locked"), tstate.get("src")
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
    return trk_txt


def _overlay_hud(frame, faces, detector, tracking, pt, snap, motion_on,
                 disp_fps, det_fps, tstate, on_tracker):
    """Ecrit les lignes d'etat : perf/suivi (l1), servo/carte (l2), fluidite, aide."""
    fh = frame.shape[0]
    batt = f"{snap['battery']:.1f}V" if snap.get("battery") is not None else "?"
    yaw = f"{snap['yaw']:+.0f}deg" if snap.get("yaw") is not None else "?"
    link = "COM OK" if snap.get("ok") else "COM --"
    trk_txt = _track_status_text(tstate, on_tracker)
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
    ms = pt.motionStats()
    step_max = ms["step_max"]
    vmax = max(abs(ms["vel_pan"]), abs(ms["vel_tilt"]))
    mv_col = (0, 220, 0) if step_max < 2.0 else (0, 165, 255) if step_max < 4.0 else (0, 0, 255)
    cv2.putText(frame, f"pas-max={step_max:.1f}deg  v={vmax:.0f}deg/s", (10, 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, mv_col, 2)
    cv2.putText(frame, l3, (10, fh - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def draw_overlay(frame, faces, main, nx, ny, area_pct, det_fps, disp_fps,
                 tracking, pt, snap, motion_on, detector="", tstate=None):
    """Incruste toute l'information de suivi sur `frame` (ordre de dessin fige).

    Orchestre les couches de dessin, du fond vers l'avant : detections, marqueur
    du visage principal, reticule + zone morte, point predit, puis le HUD texte.
    La couleur du visage verrouille code son ORIGINE :
      - "track" (prediction tracker seul, ex. profil) -> MAGENTA = zone tracker ;
      - detect/redetect/reanchor/recenter (recalee) -> CYAN = zone detect.
    """
    tstate = tstate or {}
    locked = bool(tstate.get("locked"))
    src = tstate.get("src")
    on_tracker = locked and src == "track"
    main_color = (255, 0, 255) if on_tracker else (255, 200, 0)

    _overlay_detections(frame, faces, main, locked, main_color, tstate)
    _overlay_main_marker(frame, main, nx, ny, area_pct)
    _overlay_reticle(frame, pt, main, nx, ny)
    _overlay_prediction(frame, tstate, main)
    _overlay_hud(frame, faces, detector, tracking, pt, snap, motion_on,
                 disp_fps, det_fps, tstate, on_tracker)


# ===========================================================================
# Orchestrateur : etat de la boucle P1 (capture + affichage + clavier)
# ===========================================================================
class RobotControlApp:
    """Assemble les couches et deroule le pipeline P1 sur le thread principal.

    Porte l'etat de la boucle (suivi arme, cadences d'affichage, horloges de
    lissage/heartbeat, cycles de bascule clavier) et les organes robot cables au
    setup. La detection tourne en parallele dans RobotWebCamMotorized (P2) ; ici
    on capture, on affiche, on lit le clavier et on asservit le servo une fois par
    nouvelle detection.
    """

    def __init__(self, args):
        self.args = args
        self.motion_on = not args.no_motion
        self.board_only = args.board_only        # pur pilote COM4 (ni cam ni suivi)
        self.headless = args.headless or args.board_only   # sans fenetre ni clavier
        self.win = "Bamboo v4 - controle (teleop + suivi visage)"

        # organes robot (cables par setup())
        self.tel = None            # lib.Telemetry
        self.link = None           # communication.RobotComSerial
        self.motion = None         # device.motion.RobotMotorDrive
        self.pt = None             # device.motion.RobotServoMotor
        self.webcam = None         # modules.tracking.RobotWebCamMotorized
        self.cam = None            # device.sensor.RobotSensorWebCam
        self.w = self.h = 0        # resolution P1 (parsee au setup)

        # --- etat de la boucle P1 -------------------------------------------
        self.tracking = False      # suivi de visage arme (touche F)
        self.last_seq = -1         # derniere detection asservie
        self.last_locked = None    # dernier etat de verrou (detection de transition)
        self.moving = False        # une commande de deplacement est en cours
        self.last_move_ts = 0.0    # date de la derniere touche de mouvement (watchdog)
        self.read_fail = 0         # lectures camera consecutives echouees

        # cadences / horloges (reinitialisees juste avant la boucle)
        self._disp_t0 = 0.0
        self._disp_n = 0
        self.disp_fps = 0.0
        self._hb_t0 = 0.0          # cadence du heartbeat telemetrie
        self._slew_t = 0.0         # horloge du lisseur de mouvement (dt)

        # bascules a chaud (touches M / T)
        self._detect_cycle = []    # ordre de cycle des detecteurs disponibles
        self._track_cycle = []     # ordre de cycle des trackers disponibles

        # commande externe MCP : serveur socket loopback (mcp/gateway.py)
        self.server = None

    # -----------------------------------------------------------------------
    # Mise en place : cablage des couches + ouverture camera
    # -----------------------------------------------------------------------
    def setup(self):
        """Cable telemetrie, liaison serie, moteurs, servos, suivi et camera.

        En cas d'echec fatal (detecteur indisponible, aucune camera), nettoie ce
        qui a deja ete alloue et quitte le process (sys.exit), comme l'ancien main.
        """
        args = self.args
        try:
            self.w, self.h = (int(v) for v in args.size.lower().split("x"))
        except ValueError:
            sys.exit(f"--size invalide : {args.size!r}")

        # 1) telemetrie (journal structure ; budget disque = fichier vif + 1 backup)
        self.tel = Telemetry(log_dir=args.log_dir, enabled=not args.no_telemetry,
                             max_bytes=int(max(1.0, args.log_budget_mb) * 1_000_000 / 2))
        self.tel.log("event", msg="start", motion=self.motion_on, port=args.port,
                     index=str(args.index), backend=args.backend, size=f"{self.w}x{self.h}",
                     pan_gain=args.pan_gain, tilt_gain=args.tilt_gain,
                     deadzone=args.deadzone, dead_hyst=args.dead_hyst, max_step=args.max_step,
                     invert_pan=args.invert_pan, invert_tilt=args.invert_tilt)

        # 2) liaison serie STM32 + moteurs (toujours : coeur du pilotage carte)
        self.link = RobotComSerial(args.port, args.baud, telemetry=self.tel)
        time.sleep(0.3)                          # laisse le thread lecteur s'ouvrir
        self.motion = RobotMotorDrive(self.link, maxPwm=args.max_pwm)

        # 3) vision : servos pan/tilt + suivi + camera (sautes en --board-only)
        if not self.board_only:
            self.pt = RobotServoMotor(
                self.link, panGain=args.pan_gain, tiltGain=args.tilt_gain,
                invertPan=args.invert_pan, invertTilt=args.invert_tilt,
                deadzone=args.deadzone, deadHyst=args.dead_hyst, maxStep=args.max_step,
                panMin=args.pan_min, panMax=args.pan_max, panHome=args.pan_home,
                tiltMin=args.tilt_min, tiltMax=args.tilt_max, tiltHome=args.tilt_home,
                maxVel=args.max_vel, maxAccel=args.max_accel, smooth=not args.no_smooth,
                telemetry=self.tel)
            self.webcam = RobotWebCamMotorized(
                self.pt, det_width=args.det_width, min_size=args.min_size,
                detector=args.detector, conf=args.det_conf, yunet_model=args.yunet_model,
                track_mode=args.track_mode, vit_model=args.vit_model,
                redetect_ms=args.redetect_ms, score_min=args.track_score_min,
                hold_ms=args.track_hold_ms, max_area_frac=args.track_max_area,
                max_grow=args.track_max_grow, predict_mode=args.predict_mode,
                predict_ms=args.predict_ms, predict_lead_ms=args.predict_lead_ms,
                predict_min_speed=args.predict_min_speed)
            self._init_perception()              # ready detecteur + tracker, ou sortie
            self.webcam.start()
            self._open_camera()                  # adopte le cap, ou sortie
            self._detect_cycle = self.webcam.availableDetectors()
            self._track_cycle = self.webcam.availableTrackers()

        # 4) serveur de commandes MCP (socket loopback) : les deux modes l'exposent
        host, port = gateway.parse_addr(os.environ.get("BAMBOU_MCP_PORT"))
        self.server = gateway.CommandServer(
            on_config=self._apply_config, link=self.link, motion=self.motion,
            cpr=self.link.cpr or 1320.0, host=host, port=port).start()

        # 5) traces de demarrage + etat initial des servos
        print(f"Carte : {args.port} @ {args.baud}  |  "
              f"moteurs {'ACTIFS' if self.motion_on else 'desactives'}")
        if self.board_only:
            print("Mode board-only : ni camera ni suivi (pur pilote COM4 pour le MCP).")
        else:
            print("Fenetre %s. F=suivi, Echap=quitter."
                  % ("desactivee (--headless)" if self.headless else "ouverte"))
            self.pt.center()                     # pan 88 / tilt 42 (cam de face)
        return self

    def _init_perception(self):
        """Verifie que le detecteur est pret et resout le tracker (repli 'none').

        Un detecteur indisponible est fatal (nettoyage + sortie) ; un tracker non
        chargeable retombe sur 'none' (la detection seule reste utile, pas de plantage).
        """
        args = self.args
        ok, msg = self.webcam.ready()
        if not ok:
            self.tel.log("event", msg="fatal", err=f"detecteur:{msg}")
            self.tel.close()
            self.link.close()
            sys.exit(f"Detecteur '{args.detector}' indisponible : {msg}")
        avail = self.webcam.availableDetectors()
        self.tel.log("event", msg="detector", detector=args.detector, available=avail)
        print(f"Detecteur : {args.detector}  (disponibles : {', '.join(avail)})")

        tok, tmsg = self.webcam.trackReady()
        if not tok:
            print(f"[main] tracker '{self.webcam.trackMode}' indisponible ({tmsg}) -> mode none")
            self.webcam.setTrackMode("none")
        tavail = self.webcam.availableTrackers()
        self.tel.log("event", msg="track_mode", to=self.webcam.trackMode, ok=True,
                     available=tavail, info="init")
        print(f"Suivi : {self.webcam.trackMode}  (trackers disponibles : {', '.join(tavail)})")

    def _open_camera(self):
        """Ouvre la camera USB (scan auto guide par les noms), ou nettoie et sort."""
        args = self.args
        self.cam = RobotSensorWebCam(
            index=args.index, backend=args.backend, width=self.w, height=self.h,
            fps=args.fps, flip=args.flip, rotate=args.rotate, preferName=args.camera_name,
            skipNames=args.skip_name.split(","), telemetry=self.tel)
        if not self.cam.open():
            self.webcam.stop()
            self.link.close()
            self.tel.close()
            sys.exit("Aucune camera exploitable trouvee (scan index 0..5, "
                     "backends msmf/dshow/any). Verifier le branchement.")
        print(f"Camera index {self.cam.index} : {self.cam.width}x{self.cam.height} "
              f"(backend {self.cam.backend}, flip {args.flip}, rotate {args.rotate})")

    # -----------------------------------------------------------------------
    # Bascules a chaud (touche clavier OU fichier de commande MCP)
    # -----------------------------------------------------------------------
    def _switch_detector(self, name, source):
        """Bascule le detecteur de visage (haar/dnn/yunet) et journalise."""
        ok, msg = self.webcam.setDetector(name)
        self.tel.log("event", msg="detector_switch", to=name, ok=ok, info=msg, source=source)
        if not ok:
            print(f"[main] bascule '{name}' refusee ({source}) : {msg}")
        return ok

    def _switch_track(self, name, source):
        """Bascule le tracker visuel (none/mil/vit) et journalise."""
        ok, msg = self.webcam.setTrackMode(name)
        self.tel.log("event", msg="track_mode", to=name, ok=ok, info=msg, source=source)
        if not ok:
            print(f"[main] tracker '{name}' refuse ({source}) : {msg}")
        return ok

    def _switch_predict(self, name, source):
        """Change le mode de prediction. name=None -> cycle au suivant (touche P)."""
        ok, res = (self.webcam.cyclePredictMode() if name is None
                   else self.webcam.setPredictMode(name))
        self.tel.log("event", msg="predict_mode", to=res, ok=ok, source=source)
        if not ok:
            print(f"[main] mode predict '{name}' refuse ({source}) : {res}")
        return ok

    def _apply_config(self, cfg):
        """Applique une commande de config MCP recue par socket (gateway).

        cfg (issu de gateway.config_from) porte UNE cle : detector / track_mode /
        predict_mode / active. Execute sur le thread principal (via server.drain).
        Renvoie un compte-rendu texte renvoye au client MCP.
        """
        if self.webcam is None:                  # --board-only : pas de vision
            return "Vision desactivee (--board-only) : commande de suivi ignoree."
        d = cfg.get("detector")
        if d:
            if d == self.webcam.detector:
                return "Detecteur deja sur '%s'." % d
            return "Detecteur -> %s : %s" % (d, "OK" if self._switch_detector(d, "mcp")
                                             else "refuse")
        tm = cfg.get("track_mode")
        if tm:
            if tm == self.webcam.trackMode:
                return "Tracker deja sur '%s'." % tm
            return "Tracker -> %s : %s" % (tm, "OK" if self._switch_track(tm, "mcp")
                                           else "refuse")
        pm = cfg.get("predict_mode")
        if pm:
            if pm == self.webcam.predict_mode:
                return "Mode prediction deja sur '%s'." % pm
            return "Prediction -> %s : %s" % (pm, "OK" if self._switch_predict(pm, "mcp")
                                              else "refuse")
        if "active" in cfg:
            self.tracking = bool(cfg["active"])
            self.tel.log("event", msg="tracking", on=self.tracking, source="mcp")
            return "Suivi -> %s." % ("ON" if self.tracking else "off")
        return "Commande de config vide."

    # -----------------------------------------------------------------------
    # Suivi : asservissement par NOUVELLE detection + journalisation
    # -----------------------------------------------------------------------
    def _on_new_detection(self, faces, nx, ny, area_pct, det_fps, seq, tstate):
        """Traite une detection fraiche (seq change) : journal + asservissement.

        Journalise la detection et toute transition de verrou (lock/unlock), puis
        asservit le servo UNE fois vers la cible courante. Asservir une seule fois
        par seq evite de corriger plusieurs fois vers une position perimee (pompage).
        """
        score = tstate.get("score")
        pspeed = tstate.get("pred_speed")
        perr = tstate.get("pred_err")
        self.tel.log("detect", seq=seq, faces=len(faces),
                     nx=None if nx is None else round(nx, 4),
                     ny=None if ny is None else round(ny, 4),
                     area=round(area_pct, 2), det_fps=round(det_fps, 1),
                     tracking=self.tracking, trk=tstate.get("mode"),
                     locked=tstate.get("locked"), src=tstate.get("src"),
                     raw_det=tstate.get("raw_det"),
                     score=None if score is None else round(score, 3),
                     predict=tstate.get("predict"),
                     pred_speed=None if pspeed is None else round(pspeed, 3),
                     pred_err=None if perr is None else round(perr, 4))
        # transition de verrou (lock/unlock) -> event dedie pour l'analyse
        if tstate.get("mode") != "none" and tstate.get("locked") != self.last_locked:
            unlocked = not tstate.get("locked")
            self.tel.log("event", msg="track_unlock" if unlocked else "track_lock",
                         src=tstate.get("src"),
                         reason=tstate.get("unlock_reason") if unlocked else None,
                         score=None if score is None else round(score, 3))
            self.last_locked = tstate.get("locked")
        # asservissement du servo vers la cible courante (delegue au subsystem :
        # cible = mesure, anticipation ou coast ; retour maison si coast expire)
        self.webcam.moveToTrackedArea(self.tracking)

    # -----------------------------------------------------------------------
    # Clavier
    # -----------------------------------------------------------------------
    def _process_key(self, key, now):
        """Traite une touche. Retourne (quit, moved_now).

        quit=True demande la sortie (Echap) ; moved_now=True si la touche est une
        commande de deplacement (rearme le watchdog de mouvement).
        """
        k = key & 0xFF
        c = chr(k).lower() if 32 <= k < 127 else ""
        # Journalise CHAQUE touche : permet de retrouver le code reel des fleches
        # sur ce build OpenCV (MCP : tail types=["key"]).
        self.tel.log("key", code=key, k=k, c=c)

        if key == 27:                            # Echap
            self.tel.log("event", msg="quit")
            return True, False

        # --- camera pan/tilt : fleches (primaire, plusieurs codes possibles) ---
        if key in KEYS_LEFT:
            self.pt.nudgePan(-SERVO_STEP)
        elif key in KEYS_RIGHT:
            self.pt.nudgePan(+SERVO_STEP)
        elif key in KEYS_UP:
            self.pt.nudgeTilt(+SERVO_STEP)
        elif key in KEYS_DOWN:
            self.pt.nudgeTilt(-SERVO_STEP)
        elif c == "f":                           # armer / desarmer le suivi
            self.tracking = not self.tracking
            self.tel.log("event", msg="tracking", on=self.tracking)
        elif c == "c":                           # recentrer la camera
            self.pt.center()
            self.tel.log("event", msg="center", pan=round(self.pt.angleH, 1),
                         tilt=round(self.pt.angleV, 1))
        elif c == "m":                           # cycle sur les detecteurs disponibles
            if self._detect_cycle:
                cur = self.webcam.detector
                i = self._detect_cycle.index(cur) if cur in self._detect_cycle else -1
                self._switch_detector(self._detect_cycle[(i + 1) % len(self._detect_cycle)], "key")
        elif c == "t":                           # cycle sur les trackers (none/mil/vit)
            if self._track_cycle:
                cur = self.webcam.trackMode
                i = self._track_cycle.index(cur) if cur in self._track_cycle else -1
                self._switch_track(self._track_cycle[(i + 1) % len(self._track_cycle)], "key")
        elif c == "p":                           # cycle du mode de prediction
            self._switch_predict(None, "key")
        # --- camera pan/tilt : lettres (secours) ---
        elif c == "j":
            self.pt.nudgePan(-SERVO_STEP)
        elif c == "l":
            self.pt.nudgePan(+SERVO_STEP)
        elif c == "i":
            self.pt.nudgeTilt(+SERVO_STEP)
        elif c == "k":
            self.pt.nudgeTilt(-SERVO_STEP)
        # --- vitesse ---
        elif c.isdigit():
            self.motion.setSpeed(int(c))
        # --- STOP ---
        elif k == 32:                            # Espace
            if self.motion_on:
                self.motion.stop()
            self.moving = False
        # --- deplacement (momentane + watchdog) ---
        elif c in ("z", "s", "q", "d"):
            if self.motion_on:
                if c == "z":
                    self.motion.forward()
                elif c == "s":
                    self.motion.backward()
                elif c == "q":
                    self.motion.rotateLeft()
                elif c == "d":
                    self.motion.rotateRight()
            self.moving = True
            self.last_move_ts = now
            return False, True
        return False, False

    # -----------------------------------------------------------------------
    # Boucle principale P1 + arret
    # -----------------------------------------------------------------------
    def run(self):
        """Point d'entree : setup, boucle (P1 ou board-only), puis arret propre."""
        self.setup()
        try:
            self._loop_board() if self.board_only else self._loop()
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    def _loop_board(self):
        """Boucle pur pilote COM4 (--board-only) : draine les commandes MCP et
        journalise un heartbeat carte periodique. Ni camera ni suivi."""
        print("En ecoute (board-only). Ctrl-C pour quitter.")
        self._hb_t0 = time.time()
        while True:
            self.server.drain()                  # execute les commandes carte MCP
            snap = self.link.snapshot()
            now2 = time.time()
            if now2 - self._hb_t0 >= 2.0:
                self._hb_t0 = now2
                self.tel.log("heartbeat", connected=self.link.connected,
                             batt=snap.get("battery"), yaw=snap.get("yaw"),
                             ok=snap.get("ok"), bad=snap.get("bad"),
                             encoders=snap.get("encoders"))
            time.sleep(0.02)

    def _loop(self):
        """Boucle P1 : capture -> etat suivi -> asservissement -> overlay -> clavier."""
        now = time.time()
        self._disp_t0 = now
        self._disp_n = 0
        self.disp_fps = 0.0
        self._hb_t0 = now
        self._slew_t = now

        while True:
            # 1) capture (flip/rotate appliques dans cam.read()) + garde d'echec
            ok, frame = self.cam.read()
            if not ok or frame is None:
                self.read_fail += 1
                if self.read_fail > 30:
                    print("Lecture camera echouee de facon repetee, arret.")
                    break
                time.sleep(0.005)
                continue
            self.read_fail = 0
            self.webcam.publish(frame)

            # 2) dernier resultat de detection (P2) + etat de suivi
            faces, main, nx, ny, area_pct, det_fps, seq = self.webcam.latest()
            tstate = self.webcam.trackState()

            # 3) cadence d'affichage P1
            self._disp_n += 1
            now = time.time()
            if now - self._disp_t0 >= 1.0:
                self.disp_fps = self._disp_n / (now - self._disp_t0)
                self._disp_t0 = now
                self._disp_n = 0

            # 4) asservissement : une fois par NOUVELLE detection (seq)
            if seq != self.last_seq:
                self._on_new_detection(faces, nx, ny, area_pct, det_fps, seq, tstate)
            self.last_seq = seq

            # 5) lisseur de mouvement : rapproche le servo de la cible (dt borne),
            #    borne en vitesse/acceleration (anti-flou). No-op si deja sur la cible.
            now_s = time.time()
            dt = now_s - self._slew_t
            self._slew_t = now_s
            self.pt.slew(min(dt, 0.1))           # borne dt (evite un saut apres un lag)

            # 6) rendu de l'incrustation + snapshot telemetrie
            snap = self.link.snapshot()
            draw_overlay(frame, faces, main, nx, ny, area_pct, det_fps, self.disp_fps,
                         self.tracking, self.pt, snap, self.motion_on,
                         detector=self.webcam.detector, tstate=tstate)
            if not self.headless:
                cv2.imshow(self.win, frame)      # fenetre coupee en --headless
            self.tel.snapshot(frame)             # image annotee (throttlee) pour analyse

            # 7) heartbeat periodique (etat vivant pour l'analyse)
            self._maybe_heartbeat(snap, tstate, det_fps)

            # 8) commandes MCP en file (config suivi + carte) -> thread principal
            self.server.drain()

            # 9) clavier + watchdog de mouvement (clavier coupe en --headless)
            key = cv2.waitKeyEx(1) if not self.headless else -1
            moved_now = False
            if key != -1:
                quit_now, moved_now = self._process_key(key, now)
                if quit_now:
                    break
            if self.headless:
                time.sleep(0.005)                # sans waitKey : evite la boucle folle
            if self.moving and not moved_now and (now - self.last_move_ts) > MOVE_WATCHDOG_S:
                if self.motion_on:
                    self.motion.stop()
                self.moving = False

    def _maybe_heartbeat(self, snap, tstate, det_fps):
        """Emet un battement telemetrie toutes les 2 s (perf, servo, carte, suivi)."""
        now2 = time.time()
        if now2 - self._hb_t0 < 2.0:
            return
        self._hb_t0 = now2
        hb_score = tstate.get("score")
        self.tel.log("heartbeat", disp_fps=round(self.disp_fps, 1),
                     det_fps=round(det_fps, 1), tracking=self.tracking,
                     pan=round(self.pt.angleH, 1), tilt=round(self.pt.angleV, 1),
                     connected=self.link.connected, batt=snap.get("battery"),
                     yaw=snap.get("yaw"), ok=snap.get("ok"), bad=snap.get("bad"),
                     trk=tstate.get("mode"), locked=tstate.get("locked"),
                     src=tstate.get("src"),
                     score=None if hb_score is None else round(hb_score, 3),
                     step_max=round(self.pt.motionStats()["step_max"], 2))

    def _shutdown(self):
        """Arret propre : serveur MCP, moteurs coupes, threads/camera, journaux."""
        if self.server is not None:
            self.server.stop()
        try:
            if self.motion_on and self.link is not None:
                self.link.stop()
        except Exception:
            pass
        if self.webcam is not None:
            self.webcam.stop()
        if self.cam is not None:
            self.cam.release()
        if not self.headless:
            cv2.destroyAllWindows()
        if self.link is not None:
            self.link.close()
        if self.tel is not None:
            self.tel.close()


def main():
    RobotControlApp(parse_args()).run()


if __name__ == "__main__":
    main()

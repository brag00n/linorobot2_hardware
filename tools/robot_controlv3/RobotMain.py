#!/usr/bin/env python3
r"""RobotMain (v3) - Core du banc d'essai « nodes ROS2 » : hote executeur + HMI.

Meme fonction que robot_control.RobotMain (teleop clavier + suivi de visage
pan/tilt + prediction Kalman), mais l'architecture est refondue en NODES simules :
le Core n'execute plus le traitement, il ORCHESTRE des nodes via le micro-framework
roslite (voir roslite/). Objectif : preparer la migration ROS2 en isolant chaque
traitement (camera, suivi, servo, carte) derriere une interface standard testable
sans queue ni environnement ROS.

Pipeline (ordre d'appel = ordre ROS futur) :
  CameraNode --/camera/image--> TrackingNode --/servo/cmd--> ServoNode
                                     |  \--/tracking/result---> (Core: overlay+log)
                                     |   \-/tracking/metrics--> (Core: HUD)
  BoardNode --/board/telemetry--> (Core: HUD/log)   ServoNode --/servo/state--> (Core)
  Core --/tracking/config--> TrackingNode   Core --/servo/cmd--> ServoNode (clavier)

Le Core reste responsable des SERVICES : affichage/overlay (HMI), pilotage clavier,
telemetrie + serveur de commandes MCP (socket loopback, cf. robot_control.mcp.gateway),
pilotage moteurs. Les ALGORITHMES (detection, Kalman, slew, mix drive, protocole
serie) sont ceux de robot_control, reutilises par import via les nodes -- rien n'est reecrit.

L'app est l'unique proprietaire de COM4 : le serveur d'action MCP lui relaie ses
commandes carte/suivi par socket tant qu'elle tourne. Port : BAMBOU_MCP_PORT (defaut 8787).

Lancement depuis le dossier tools/ :
  .venv/Scripts/python.exe -m robot_controlv3.RobotMain --no-motion --index 1 --flip h
  ... --headless      # sans fenetre ni clavier (suivi + telemetrie + serveur MCP)
  ... --board-only    # pur pilote COM4 (BoardNode seul ; force headless)
(l'app monolithique de reference reste lancable : -m robot_control.RobotMain ...)

Clavier : O active/coupe les moteurs (securite), fleches = moteurs (haut/bas avance/
recule, gauche/droite rotation), Maj+fleches = camera pan/tilt, C centre, Page-Up/Down =
vitesse 0-9, Espace STOP, F suivi, M detecteur, T tracker, P prediction, R reco, Echap.
"""
import math
import os
import time

import cv2
import numpy as np

# --- reutilisation par import : CLI, overlay et constantes de robot_control -----
# (memes flags, meme incrustation, memes codes clavier -> comportement identique)
from robot_control.RobotMain import (
    parse_args,
    _overlay_detections, _overlay_main_marker, _overlay_reticle, _overlay_prediction,
    KEYS_LEFT, KEYS_UP, KEYS_RIGHT, KEYS_DOWN,
    KEYS_PGUP, KEYS_PGDN, SERVO_STEP, TARGET_STEP, clamp_target,
)

# Periode de republication cmd_vel (modele etat combine type ROS) : ~10 Hz.
# Tolerance : le watchdog cmd_vel firmware coupe apres ~400 ms sans trame.
CMDVEL_PERIOD_S = 0.1
from robot_control.lib.Telemetry import Telemetry
from robot_control.version import APP_VERSION
from robot_control import robot_profile
from robot_control.communication.RobotComSerial import RobotComSerial, DEFAULT_CPR
from robot_control.communication.Esp32ComSerial import (
    Esp32ComSerial, ESP32_DEFAULT_CPR, ESP32_DEFAULT_BAUD, ESP32_VID_PID)
from robot_control.communication.TeensyComSerial import (
    TeensyComSerial, TEENSY_DEFAULT_CPR, TEENSY_DEFAULT_BAUD, TEENSY_VID_PID)
from robot_control.device.motion.RobotMotorDrive import RobotMotorDrive
from robot_control.modules.tracking.RobotWebCamMotorized import PREDICT_MODES
from robot_control.mcp import gateway

from .roslite import Executor
from .metrics import MetricsConfig
from .nodes import (CameraNode, TrackingNode, ServoNode, BoardNode, GrovePiNode,
                    GamepadNode, WSEsp32Node, TeensyNode, FaceRecogNode, FaceTrainNode,
                    WheelTachoNode)
from .nodes.GamepadNode import (
    AX_LX, AX_LY, AX_RX, AX_RY, AX_LT, AX_RT,
    BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_BACK, BTN_START, BTN_L3, BTN_R3,
    HAT_DPAD)
from .msgs import (TrackingConfig, ServoCmd, TrackingResult, TrackingMetrics,
                   ServoState, GrovePiTelemetry, Esp32Telemetry, JoyMsg, RecognitionConfig,
                   RecognitionResult, TrainState, WheelTachoConfig)


# ===========================================================================
# HUD v3 : 3 cartes materielles (CAM / STM32 / GROVE), une par coin, chacune dans
# un rectangle pointille de TAILLE FIXE (cadre grise + « -- » si materiel absent).
# Les marqueurs centraux (detections/reticule/predit) restent ceux de robot_control
# (via _draw_image_markers) ; seul le HUD TEXTE est remplace par ces cartes, ou
# chaque champ occupe une colonne fixe (une valeur qui change ne decale rien).
# ===========================================================================
def _blank_frame(w, h):
    """Toile noire (ndarray BGR) servant de support d'affichage sans camera.

    Permet a l'app de rester une IHM VISUELLE meme camera absente : on y incruste
    les capteurs GrovePi et l'etat des composants (exigence « demarrer sans camera,
    afficher les capteurs en temps reel »)."""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(frame, "Pas de camera - capteurs en temps reel", (10, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 80), 2)
    return frame


# --- palette commune des cartes HUD ----------------------------------------
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_C_BORDER = (95, 95, 95)          # cadre pointille (carte presente)
_C_BORDER_OFF = (55, 55, 55)      # cadre pointille (carte absente)
_C_TITLE = (235, 235, 235)
_C_TITLE_OFF = (110, 110, 110)
_C_LABEL = (150, 150, 150)        # libelles (gris)
_C_VAL = (235, 235, 235)          # valeurs (blanc)
_C_ON = (0, 220, 0)               # etat actif / present
_C_OFF = (120, 120, 120)          # etat inactif / absent
_C_WARN = (0, 165, 255)           # orange (soutenu / silencieux)
_C_BAD = (0, 0, 255)              # rouge (coup de butoir)
_C_IMU = (0, 255, 255)            # cyan (IMU)

# duree de vie d'episode (s) au-dela de laquelle le verrou est juge assez STABLE
# pour declencher une reconnaissance de personne (age vire au vert dans la carte CAM).
_LOCK_STABLE_S = 2.0


def _put(frame, x, y, text, col, scale=0.45, thick=1):
    cv2.putText(frame, text, (int(x), int(y)), _FONT, scale, col, thick, cv2.LINE_AA)


def _dashed_line(frame, p1, p2, col, dash=7, gap=5, thick=1):
    x1, y1 = p1
    x2, y2 = p2
    dist = int(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5)
    if dist == 0:
        return
    for i in range(0, dist, dash + gap):
        a, b = i / dist, min(i + dash, dist) / dist
        cv2.line(frame, (int(x1 + (x2 - x1) * a), int(y1 + (y2 - y1) * a)),
                 (int(x1 + (x2 - x1) * b), int(y1 + (y2 - y1) * b)), col, thick)


def _dashed_rect(frame, x, y, w, h, col):
    _dashed_line(frame, (x, y), (x + w, y), col)
    _dashed_line(frame, (x + w, y), (x + w, y + h), col)
    _dashed_line(frame, (x + w, y + h), (x, y + h), col)
    _dashed_line(frame, (x, y + h), (x, y), col)


def _card_bg(frame, x, y, w, h, alpha=0.55):
    """Fond semi-transparent FONCE derriere la carte : melange un rectangle noir
    avec l'image (via addWeighted sur la ROI) pour que le texte clair reste lisible
    par-dessus le flux camera. Borne la ROI aux limites de l'image."""
    fh, fw = frame.shape[:2]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(fw, x + w), min(fh, y + h)
    if x1 <= x0 or y1 <= y0:
        return
    roi = frame[y0:y1, x0:x1]
    roi[:] = cv2.addWeighted(roi, 1.0 - alpha, np.zeros_like(roi), 0.0, 0.0)


def _card_frame(frame, x, y, w, h, title, port, present, active, port_col=None):
    """Cadre pointille (TAILLE FIXE) + en-tete « ● TITRE  port ». Rend l'y du 1er
    contenu. present -> cadre/titre clairs ; active -> pastille verte. `port_col`
    force la couleur de l'etiquette port (ex. orange sur trames erronees) ; None =
    couleur par defaut selon present."""
    _card_bg(frame, x, y, w, h)
    _dashed_rect(frame, x, y, w, h, _C_BORDER if present else _C_BORDER_OFF)
    hy = y + 18
    cv2.circle(frame, (x + 13, hy - 5), 5, _C_ON if active else _C_OFF, -1)
    _put(frame, x + 25, hy, title, _C_TITLE if present else _C_TITLE_OFF, 0.5)
    if port:
        col = port_col if port_col is not None else (_C_LABEL if present else _C_TITLE_OFF)
        _put(frame, x + 92, hy, port, col, 0.45)
    return y + 38


def _row(frame, x, y, fields):
    """Une ligne = suite de (dx, texte, couleur) a colonnes FIXES (dx depuis x).
    Chaque champ est dessine independamment -> une valeur qui change de longueur
    ne decale JAMAIS ses voisins (exigence : format stable)."""
    for dx, text, col in fields:
        _put(frame, x + dx, y, text, col)


def _hmi(mcfg, group):
    """Le groupe de metriques `group` doit-il etre dessine (cible HMI) ? (None = oui)."""
    return mcfg is None or mcfg.enabled(group, "hmi")


def _draw_cam_card(frame, x, y, cam_ok, camnode, active, met, n_faces, p1_fps,
                   tstate, mcfg=None):
    """Carte CAM (haut-gauche) : source + etats suivi/detecteur/tracker + perf + pred."""
    present = bool(camnode.available)
    if present:
        idx = getattr(camnode.cam, "index", "?")
        port = f"idx{idx} {camnode.width}x{camnode.height}"
    else:
        port = "absente"
    yc = _card_frame(frame, x, y, 320, 160, "CAM", port, present, active)

    locked = bool(tstate.get("locked"))
    src = tstate.get("src")
    on_tracker = locked and src == "track"
    sc = tstate.get("score")
    rd = tstate.get("raw_det")
    trk = tstate.get("mode", "none")
    pm = tstate.get("predict_mode", "off")

    lock_txt = f"ON({src})" if locked else "off"
    # --- etat detection/suivi (groupe cam_detect) ----------------------------
    if _hmi(mcfg, "cam_detect"):
        _row(frame, x, yc, [
            (10, "suivi", _C_LABEL), (78, "ON" if active else "off", _C_ON if active else _C_OFF),
            (150, "det", _C_LABEL), (200, met.detector or "?", _C_VAL)])
        _row(frame, x, yc + 19, [
            (10, "trk", _C_LABEL), (78, trk, _C_VAL),
            (150, "lock", _C_LABEL), (200, lock_txt, _C_ON if locked else _C_OFF)])
        _row(frame, x, yc + 38, [
            (10, "sc", _C_LABEL), (78, f"{sc:.2f}" if sc is not None else "--", _C_VAL),
            (150, "det", _C_LABEL),
            (200, ("hit" if rd else "miss") if (locked and rd is not None) else "--",
             _C_ON if rd else _C_OFF)])
    # --- perf (groupe cam_fps) : P1/P2/vis ------------------------------------
    if _hmi(mcfg, "cam_fps"):
        _row(frame, x, yc + 57, [
            (10, "P1", _C_LABEL), (48, f"{p1_fps:4.0f}", _C_VAL),
            (110, "P2", _C_LABEL), (148, f"{met.det_fps:4.0f}", _C_VAL),
            (210, "vis", _C_LABEL), (258, f"{n_faces:d}", _C_VAL)])
    if _hmi(mcfg, "cam_detect"):
        if pm != "off":
            v = tstate.get("pred_speed") or 0.0
            err = tstate.get("pred_err")
            _row(frame, x, yc + 76, [
                (10, "pred", _C_LABEL), (78, pm, _C_VAL),
                (150, "v", _C_LABEL), (172, f"{v:.2f}", _C_VAL),
                (230, "err", _C_LABEL), (272, f"{err:.3f}" if err is not None else "--", _C_VAL)])
        else:
            _row(frame, x, yc + 76, [(10, "pred", _C_LABEL), (78, "off", _C_OFF)])

        # episode de suivi : id (handle pour une reconnaissance) + duree de vie (critere).
        # Age vire au vert au-dela d'un seuil = episode assez stable pour identifier.
        if locked:
            lid = tstate.get("lock_id") or 0
            age = tstate.get("lock_age") or 0.0
            _row(frame, x, yc + 95, [
                (10, "id", _C_LABEL), (48, f"#{lid}", _C_VAL),
                (110, "age", _C_LABEL),
                (150, f"{age:5.1f}s", _C_ON if age >= _LOCK_STABLE_S else _C_VAL)])
        else:
            _row(frame, x, yc + 95, [(10, "id", _C_LABEL), (48, "--", _C_OFF)])


def _draw_stm_card(frame, x, y, present, port_name, snap, pt, motion_on, smooth,
                   mcfg=None, cmdvel=None):
    """Carte STM32 (haut-droit) : etats moteurs/lissage + servos + fluidite + IMU carte
    + vitesses de rotation moteurs (barres bipolaires). Chaque sous-bloc est dessine
    seulement si son groupe de metriques est actif (cible HMI)."""
    port = f"{port_name} {'OK' if snap.get('ok') else '--'}"
    yc = _card_frame(frame, x, y, 320, 212, "STM32", port, present, present)

    ms = pt.motionStats() or {}
    step_max = ms.get("step_max", 0.0)
    vmax = max(abs(ms.get("vel_pan", 0.0)), abs(ms.get("vel_tilt", 0.0)))
    mcol = _C_ON if step_max < 2.0 else _C_WARN if step_max < 4.0 else _C_BAD
    batt = f"{snap['battery']:.1f}V" if snap.get("battery") is not None else "--"

    def _ang(v):
        return f"{v:+6.1f}" if v is not None else "    --"

    if _hmi(mcfg, "stm32_motor"):
        _row(frame, x, yc, [
            (10, "moteurs", _C_LABEL), (98, "ON" if motion_on else "OFF",
                                        _C_ON if motion_on else _C_OFF),
            (170, "lissage", _C_LABEL), (258, "ON" if smooth else "off",
                                         _C_ON if smooth else _C_OFF)])
    if _hmi(mcfg, "stm32_servo"):
        _row(frame, x, yc + 19, [
            (10, "pan  S1", _C_LABEL), (98, f"{pt.angleH:4.0f}deg", _C_VAL),
            (170, "tilt S2", _C_LABEL), (258, f"{pt.angleV:4.0f}deg", _C_VAL)])
        _row(frame, x, yc + 38, [
            (10, "pas-max", _C_LABEL), (98, f"{step_max:4.1f}deg", mcol),
            (170, "v", _C_LABEL), (258, f"{vmax:4.0f}deg/s", mcol)])
    if _hmi(mcfg, "stm32_batt"):
        # cmd_vel courant (modele ROS) : linear.x (m/s) / angular.z (rad/s)
        lin, ang = (cmdvel if cmdvel is not None else (0.0, 0.0))
        cvcol = _C_ON if (lin or ang) else _C_OFF
        _row(frame, x, yc + 57, [
            (10, "batt", _C_LABEL), (98, batt, _C_VAL),
            (150, "cmd_vel", _C_LABEL),
            (222, f"{lin:+.2f}m/s {ang:+.2f}r/s", cvcol)])
    if _hmi(mcfg, "stm32_imu"):
        # IMU de la carte STM32 (attitude roll/pitch/yaw, trame 0x0C)
        _row(frame, x, yc + 76, [
            (10, "IMU roll", _C_LABEL), (95, _ang(snap.get("roll")), _C_IMU),
            (170, "pitch", _C_LABEL), (228, _ang(snap.get("pitch")), _C_IMU)])
        _row(frame, x, yc + 95, [
            (10, "    yaw", _C_LABEL), (95, _ang(snap.get("yaw")), _C_IMU)])
    if _hmi(mcfg, "stm32_rpm"):
        _draw_rpm_bars(frame, x + 10, yc + 112, snap.get("rpm"))


def _draw_rpm_bars(frame, x, y, rpm, hs_m2=False):
    """4 barres BIPOLAIRES de vitesse de rotation (tours/MINUTE) : 0 au CENTRE,
    remplissage a DROITE si rpm>0 (avant, vert) / a GAUCHE si rpm<0 (arriere, orange).
    Echelle commune normalisee sur max(|rpm|) des moteurs VALIDES avec plancher.
    `hs_m2` : interrupteur PAR ROBOT, a lever si la voie encodeur de M2 est morte
    -> barre PLEINE grise + « HS », et M2 exclu de l'echelle. DEFAUT False : sur le
    robot STM32 YahBoom la carte a ete REMPLACEE (2026-09-25) et les 4 voies sont
    saines (cf. bamboo-v4-hardware-faults) ; les cartes ESP32 WaveShare et Teensy
    n'ont jamais eu ce defaut."""
    _put(frame, x, y, "rpm", _C_LABEL, 0.42, 1)
    bx, bw = x + 24, 210
    cxb = bx + bw // 2
    half = bw // 2
    # echelle commune : max |rpm| des moteurs valides, plancher 30 rpm. Les 4
    # moteurs comptent ; M2 (indice 1) n'en sort que si hs_m2 est leve.
    valid_idx = (0, 2, 3) if hs_m2 else (0, 1, 2, 3)
    if rpm is not None:
        live = [abs(rpm[i]) for i in valid_idx if i < len(rpm) and rpm[i] is not None]
        scale = max(30.0, max(live) if live else 30.0)
    else:
        scale = 30.0
    for i in range(4):
        ry = y + 14 + i * 14
        top, bot = ry - 9, ry - 2
        _put(frame, x, ry, "M%d" % (i + 1), _C_LABEL, 0.42, 1)
        cv2.rectangle(frame, (bx, top), (bx + bw, bot), (60, 60, 60), 1)   # rail
        cv2.line(frame, (cxb, top), (cxb, bot), (90, 90, 90), 1)           # repere 0
        if hs_m2 and i == 1:                         # voie M2 declaree morte -> HS
            cv2.rectangle(frame, (bx, top), (bx + bw, bot), (90, 90, 90), -1)
            _put(frame, bx + bw + 8, ry, "HS", _C_OFF, 0.42, 1)
            continue
        v = rpm[i] if (rpm is not None and i < len(rpm) and rpm[i] is not None) else None
        if v is None:
            _put(frame, bx + bw + 8, ry, "  --", _C_OFF, 0.42, 1)
            continue
        frac = max(-1.0, min(1.0, v / scale))
        fl = int(half * abs(frac))
        if fl > 0:
            col = (0, 210, 0) if frac >= 0 else (0, 165, 255)   # vert=avant / orange=arriere
            if frac >= 0:
                cv2.rectangle(frame, (cxb, top), (cxb + fl, bot), col, -1)
            else:
                cv2.rectangle(frame, (cxb - fl, top), (cxb, bot), col, -1)
        _put(frame, bx + bw + 8, ry, "%+5.0f" % v, _C_VAL, 0.42, 1)


def _draw_wsesp32_card(frame, x, y, port_name, es, mcfg=None):
    """Carte WSESP32 (carte de controle ESP32 WaveShare) : vitesses mesurees +
    batterie + IMU (roll/pitch/yaw) + vitesses de rotation moteurs (barres bipolaires).

    Calquee sur _draw_stm_card mais SANS bloc servo (servo ST3215 bus serie hors
    perimetre) : la carte est un pur emetteur de telemetrie cote app (le pilotage
    passe par cmd_vel). `es` = message Esp32Telemetry (None si node absent). Chaque
    sous-bloc est gate par son groupe de metriques (cible HMI)."""
    present = bool(es is not None and es.connected)
    fresh = bool(present and es.speed_age is not None and es.speed_age < 1.5)
    # Etiquette du port : orange (alerte) des que la carte a accumule > 2 trames
    # erronees (bruit ligne / desync), sinon OK / -- selon reception. Remplace
    # l'ancien champ « trames X/Y » (demande utilisateur).
    bad = es.bad if present else 0
    port = f"{port_name} {'OK' if (present and es.ok) else '--'}" if present else "absente"
    port_col = _C_WARN if bad > 2 else None
    yc = _card_frame(frame, x, y, 320, 190, "WSESP32", port, present, fresh, port_col)

    def _ang(v):
        return f"{v:+6.1f}" if v is not None else "    --"

    if _hmi(mcfg, "esp32_motor"):
        # vitesses mesurees (odometrie carte) : Vx m/s, Vz rad/s (Vy nul en differentiel)
        # present peut etre vrai (HEARTBEAT recu) avant la 1re trame de vitesse
        # (WHEEL_STATE en MAVLink) -> vx/vz encore None : on retombe sur 0.0.
        vx = es.vx if (present and es.vx is not None) else 0.0
        vz = es.vz if (present and es.vz is not None) else 0.0
        vcol = _C_ON if (present and (vx or vz)) else _C_OFF
        _row(frame, x, yc, [
            (10, "vitesse", _C_LABEL), (98, f"{vx:+.2f}m/s", vcol),
            (185, "wz", _C_LABEL), (222, f"{vz:+.2f}r/s", vcol)])
    if _hmi(mcfg, "esp32_batt"):
        batt = f"{es.battery:.1f}V" if (present and es.battery is not None) else "--"
        _row(frame, x, yc + 19, [(10, "batt", _C_LABEL), (98, batt, _C_VAL)])
    if _hmi(mcfg, "esp32_mag"):
        # cap boussole (magnetometre AK09918, trame 0x0B) : degres [0..360[
        fresh_mag = bool(present and es.mag_age is not None and es.mag_age < 1.5)
        cap = (f"{es.heading:5.1f}deg"
               if (fresh_mag and es.heading is not None) else "    --")
        _row(frame, x, yc + 19, [
            (170, "cap", _C_LABEL), (258, cap, _C_IMU if fresh_mag else _C_OFF)])
    if _hmi(mcfg, "esp32_imu"):
        r = es.roll if present else None
        p = es.pitch if present else None
        yw = es.yaw if present else None
        _row(frame, x, yc + 38, [
            (10, "IMU roll", _C_LABEL), (95, _ang(r), _C_IMU),
            (170, "pitch", _C_LABEL), (228, _ang(p), _C_IMU)])
        _row(frame, x, yc + 57, [
            (10, "    yaw", _C_LABEL), (95, _ang(yw), _C_IMU)])
    if _hmi(mcfg, "esp32_rpm"):
        # hs_m2=False : les moteurs ESP32 (encodeurs A/B) sont tous valides.
        # C'est le DEFAUT partout depuis le remplacement de la carte STM32
        # (2026-09-25) : plus aucune voie encodeur morte sur le parc.
        _draw_rpm_bars(frame, x + 10, yc + 74, es.rpm if present else None, hs_m2=False)


def _draw_teensy_card(frame, x, y, port_name, ts, mcfg=None):
    """Carte TEENSY (3e carte de controle) : vitesses mesurees + batterie + IMU
    (roll/pitch/yaw) + vitesses de rotation moteurs (barres bipolaires).

    Calquee sur _draw_wsesp32_card mais SANS ligne magneto/cap : l'IMU MPU6050 n'a
    pas de magnetometre (yaw relatif/derivant). `ts` = message TeensyTelemetry (None
    si node absent). Les 4 encodeurs Teensy sont REELS -> hs_m2=False, 4 barres RPM
    valides. Chaque sous-bloc est gate par son groupe de metriques (cible HMI)."""
    present = bool(ts is not None and ts.connected)
    fresh = bool(present and ts.speed_age is not None and ts.speed_age < 1.5)
    bad = ts.bad if present else 0
    port = f"{port_name} {'OK' if (present and ts.ok) else '--'}" if present else "absente"
    port_col = _C_WARN if bad > 2 else None
    yc = _card_frame(frame, x, y, 320, 190, "TEENSY", port, present, fresh, port_col)

    def _ang(v):
        return f"{v:+6.1f}" if v is not None else "    --"

    if _hmi(mcfg, "teensy_motor"):
        # vitesses mesurees (odometrie carte) : Vx m/s, Vz rad/s (Vy nul en differentiel)
        # present peut etre vrai (HEARTBEAT recu) avant la 1re trame de vitesse
        # (WHEEL_STATE en MAVLink) -> vx/vz encore None : on retombe sur 0.0.
        vx = ts.vx if (present and ts.vx is not None) else 0.0
        vz = ts.vz if (present and ts.vz is not None) else 0.0
        vcol = _C_ON if (present and (vx or vz)) else _C_OFF
        _row(frame, x, yc, [
            (10, "vitesse", _C_LABEL), (98, f"{vx:+.2f}m/s", vcol),
            (185, "wz", _C_LABEL), (222, f"{vz:+.2f}r/s", vcol)])
    if _hmi(mcfg, "teensy_batt"):
        batt = f"{ts.battery:.1f}V" if (present and ts.battery is not None) else "--"
        _row(frame, x, yc + 19, [(10, "batt", _C_LABEL), (98, batt, _C_VAL)])
    if _hmi(mcfg, "teensy_imu"):
        # yaw RELATIF (integration gyro-z, pas de magneto) -> derive dans le temps
        r = ts.roll if present else None
        p = ts.pitch if present else None
        yw = ts.yaw if present else None
        _row(frame, x, yc + 38, [
            (10, "IMU roll", _C_LABEL), (95, _ang(r), _C_IMU),
            (170, "pitch", _C_LABEL), (228, _ang(p), _C_IMU)])
        _row(frame, x, yc + 57, [
            (10, "    yaw", _C_LABEL), (95, _ang(yw), _C_IMU)])
    if _hmi(mcfg, "teensy_rpm"):
        # hs_m2=False : les 4 encodeurs quadrature Teensy sont reels et valides.
        _draw_rpm_bars(frame, x + 10, yc + 74, ts.rpm if present else None, hs_m2=False)


def _draw_grove_card(frame, x, y, port_name, gp, mcfg=None):
    """Carte GROVE (bas-droit) : 4 ultrasons (mm + barre) + IMU roll/pitch +
    telemetre IR (distance + barre de proximite). Sous-blocs gates par groupe (HMI)."""
    present = bool(gp is not None and gp.connected)
    fresh = bool(present and gp.ultra_age is not None and gp.ultra_age < 1.5)
    ver = (gp.version if (gp and gp.version) else "") if present else ""
    port = f"{port_name} {ver}".strip() if present else "absente"
    yc = _card_frame(frame, x, y, 320, 176, "GROVE", port, present, fresh)

    if _hmi(mcfg, "grove_ultra"):
        # --- ultrasons : 2x2, valeur mm (largeur fixe) + barre de proximite ---
        ultra = gp.ultra if present else [None, None, None, None]
        for i, d in enumerate(ultra):
            cx = x + 10 + (i % 2) * 160
            cy = yc + (i // 2) * 22
            if d is None:
                txt, col, fill = "  --", _C_OFF, 0
            else:
                f = max(0.0, min(1.0, (d - 300) / 1200.0))     # 0=proche 1=loin
                col = (0, int(80 + 140 * f), int(220 - 140 * f))
                txt = f"{d:4d}"
                fill = int(46 * (1.0 - f))
            _put(frame, cx, cy, f"S{i}", _C_LABEL)
            _put(frame, cx + 28, cy, f"{txt}mm", col)
            bx = cx + 100
            cv2.rectangle(frame, (bx, cy - 9), (bx + 46, cy - 3), (60, 60, 60), 1)
            if fill > 0:
                cv2.rectangle(frame, (bx, cy - 9), (bx + fill, cy - 3), col, -1)

    if _hmi(mcfg, "grove_imu"):
        # --- IMU : roll/pitch fusionnes (filtre complementaire MPU6050) -------
        # Les accel/gyro bruts (ex-lignes « brut a » / « g ») ne sont plus affiches :
        # gyro non calibre sur cette carte -> valeurs tres bruitees, sans valeur ajoutee.
        iy = yc + 48
        roll = f"{gp.roll:+7.1f}" if (present and gp.roll is not None) else "     --"
        pitch = f"{gp.pitch:+7.1f}" if (present and gp.pitch is not None) else "     --"
        _row(frame, x, iy, [
            (10, "IMU roll", _C_LABEL), (95, roll, _C_IMU),
            (185, "pitch", _C_LABEL), (245, pitch, _C_IMU)])

    if _hmi(mcfg, "grove_irdist"):
        # --- telemetre IR Sharp : distance seule + barre de proximite ---------
        iry = yc + 105
        d = gp.ir_dist if present else None
        if d is None:
            txt, col, fill = "  --", _C_OFF, 0
        else:
            f = max(0.0, min(1.0, (d - 100) / 600.0))      # 0=proche 1=loin
            col = (0, int(80 + 140 * f), int(220 - 140 * f))
            txt = f"{d:4d}"
            fill = int(46 * (1.0 - f))
        _put(frame, x + 10, iry, "IR", _C_LABEL)
        _put(frame, x + 38, iry, f"{txt}mm", col)
        bx = x + 110
        cv2.rectangle(frame, (bx, iry - 9), (bx + 46, iry - 3), (60, 60, 60), 1)
        if fill > 0:
            cv2.rectangle(frame, (bx, iry - 9), (bx + fill, iry - 3), col, -1)


def _draw_tacho_card(frame, x, y, th):
    """Carte TACHY (tachymetre optique des roues) : RPM signe par roue mesure a la
    camera. Independante de la carte de controle -- c'est tout l'interet : BambooWS
    n'a que 2 encodeurs (un par cote) et recopie les voies 3/4 sur 1/2, donc seule
    une mesure optique temoigne que la 2e roue d'un cote tourne vraiment."""
    act = bool(th and th.active)
    cal = bool(th and th.calibrated)
    yc = _card_frame(frame, x, y, 320, 112, "TACHY", "cam", act, act and cal)
    if th is None or not act:
        _put(frame, x + 12, yc, "desarme (W)", _C_OFF, 0.45)
        return
    if not cal:
        _put(frame, x + 12, yc, th.note or "calibration...", _C_WARN, 0.45)
        return
    # une ligne par roue : id, angle courant, rpm signe, qualite du marqueur
    for i, w in enumerate(th.wheels):
        ry = yc + i * 16
        rpm = w.get("rpm")
        q = w.get("quality") or 0.0
        col = _C_VAL if w.get("ok") else _C_OFF
        _row(frame, x + 12, ry, [
            (0, "roue %s" % w.get("id", "?"), _C_LABEL),
            (62, ("%3.0f deg" % w["theta"]) if w.get("theta") is not None else "  -- ",
             _C_LABEL),
            (132, ("%+6.1f rpm" % rpm) if rpm is not None else "    -- rpm", col),
            (232, "q%3.0f" % q, _C_LABEL if q >= 20 else _C_WARN)])
    ry = yc + 2 * 16 + 4
    rec = getattr(th, "recording", 0.0) or 0.0
    _row(frame, x + 12, ry, [
        (0, "%4.1f fps" % th.fps, _C_LABEL),
        (80, ("REC %4.1fs" % rec) if rec > 0 else "", _C_BAD)])


def _draw_tacho_overlay(frame, th):
    """Trace la ROI et l'anneau de mesure de chaque roue : c'est le seul retour
    visuel qui permet de juger d'un coup d'oeil si la detection Hough a pris la
    bonne roue et le bon rayon."""
    if th is None or not th.active:
        return
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = th.roi
    cv2.rectangle(frame, (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h)),
                  (110, 110, 110), 1)
    for wh in th.wheels:
        cx, cy = int(wh["cx"]), int(wh["cy"])
        col = _C_ON if wh.get("ok") else _C_WARN
        cv2.circle(frame, (cx, cy), int(wh["r"]), (140, 140, 140), 1)
        cv2.circle(frame, (cx, cy), int(wh["r_mark"]), col, 1)
        th_deg = wh.get("theta")
        if th_deg is not None:
            a = math.radians(th_deg)
            # meme convention que WheelTachoNode._ring_index : (cos a, -sin a)
            px = int(cx + wh["r_mark"] * math.cos(a))
            py = int(cy - wh["r_mark"] * math.sin(a))
            cv2.line(frame, (cx, cy), (px, py), col, 2)
        _put(frame, cx - 6, cy - int(wh["r"]) - 6, str(wh.get("id", "")), col, 0.5)


def _draw_recog_badge(frame, recog, x=8, y=176, w=320):
    """Bandeau reconnaissance SOUS la carte CAM : ligne d'etat (mode + nom/id_pred +
    score + stabilite) puis VIGNETTE du visage capture (crop 112x112 redresse par
    alignCrop = exactement la vue fournie au recogniseur).

    known -> vert si stable, orange si episode instable (stab < 60%) ; unknown ->
    orange ; idle/off -> gris. `score` = cosinus lisse (EMA), `stability` = taux de
    frames « known » sur l'episode (indice anti-flicker)."""
    if recog is None or getattr(recog, "mode", "off") == "off":
        return
    if recog.status == "known":
        pct = int(round(max(0.0, min(1.0, recog.score)) * 100))
        stab = int(round(max(0.0, min(1.0, recog.stability)) * 100))
        txt = "RECO #%s %s  %d%%  stab %d%%" % (recog.id_pred, recog.name, pct, stab)
        col = (120, 220, 120) if recog.stability >= 0.6 else (0, 170, 255)
    elif recog.status == "unknown":
        txt = "RECO inconnu (%.2f)" % recog.score
        col = (0, 170, 255)
    else:
        txt = "reco %s : en attente" % recog.mode
        col = (170, 170, 170)
    thumb = getattr(recog, "thumb", None)
    has_thumb = thumb is not None and getattr(thumb, "size", 0)
    ts = 112                                          # cote de la vignette affichee
    h = 22 + (ts + 10 if has_thumb else 0)
    _card_bg(frame, x, y, w, h, alpha=0.55)
    _put(frame, x + 8, y + 15, "%s   [%s]" % (txt, recog.mode), col, 0.45, 1)
    if not has_thumb:
        return
    try:
        vig = cv2.resize(thumb, (ts, ts), interpolation=cv2.INTER_NEAREST)
        vx, vy = x + 8, y + 24
        frame[vy:vy + ts, vx:vx + ts] = vig
        cv2.rectangle(frame, (vx - 1, vy - 1), (vx + ts, vy + ts), col, 1)
        _put(frame, vx + ts + 12, vy + 16, "visage capture", _C_LABEL, 0.42, 1)
        _put(frame, vx + ts + 12, vy + 36, "vue recogniseur", _C_TITLE_OFF, 0.36, 1)
        _put(frame, vx + ts + 12, vy + 52, "112x112 redresse", _C_TITLE_OFF, 0.36, 1)
    except Exception:
        pass


def _draw_train_log(frame, tr):
    """Panneau LOG d'apprentissage (rectangle pointille + fond semi-transparent),
    ancre cote DROITE-CENTRE (bande libre entre les cartes STM32 et GROVE). Affiche
    le log FENETRE du batch (dernieres lignes). Cadre rouge + pastille pleine pendant
    l'apprentissage, gris ensuite. `tr` = message TrainState (/recognition/train_state).
    La VISIBILITE (pendant + ~20 s apres) est decidee par l'appelant."""
    running = bool(getattr(tr, "running", False))
    lines = getattr(tr, "lines", None) or []
    if not lines and not running:
        return
    fw, fh = frame.shape[1], frame.shape[0]
    m, w, lh = 8, 380, 15
    band_top, band_bot = 226, fh - 196          # entre STM32 (haut-D) et GROVE (bas-D)
    avail = max(60, band_bot - band_top)
    maxlines = max(3, min(12, (avail - 46) // lh))
    show = lines[-maxlines:]
    h = 34 + max(1, len(show)) * lh + 8
    x = fw - w - m
    y = band_top + max(0, (avail - h) // 2)
    _card_bg(frame, x, y, w, h, alpha=0.6)
    _dashed_rect(frame, x, y, w, h, _C_BAD if running else _C_BORDER)
    cv2.circle(frame, (x + 13, y + 16), 5, _C_BAD if running else _C_OFF, -1)
    hdr = "APPRENTISSAGE" + ("  (en cours)" if running else "")
    _put(frame, x + 25, y + 21, hdr, _C_TITLE, 0.5)
    ly = y + 34 + 11
    for ln in show:
        _put(frame, x + 10, ly, ln[:58], (190, 205, 190), 0.38, 1)
        ly += lh


def _draw_image_markers(frame, faces, main, nx, ny, area_pct, pt, tstate):
    """Marqueurs LIES A L'IMAGE (detections, cible, reticule, point predit) — le HUD
    texte de robot_control est remplace par les cartes v3, mais ces marqueurs
    centraux (position visage) sont reutilises tels quels."""
    locked = bool(tstate.get("locked"))
    on_tracker = locked and tstate.get("src") == "track"
    main_color = (255, 0, 255) if on_tracker else (255, 200, 0)
    _overlay_detections(frame, faces, main, locked, main_color, tstate)
    _overlay_main_marker(frame, main, nx, ny, area_pct, tstate)
    _overlay_reticle(frame, pt, main, nx, ny)
    _overlay_prediction(frame, tstate, main)


# aide clavier sous forme de BOUTONS, regroupes en MATRICE par type. Chaque bouton
# porte un index STABLE (voir _help_btn_for_key) ; il s'eclaire a la pression de sa
# touche. Groupes = colonnes empilees, alignees en bas a gauche.
_BTN_CAM = 11                        # index du bouton bascule camera (interne/externe)
_BTN_TARGET = 12                     # index du bouton taille de surface cible (+/-)
_BTN_SPEED = 9                       # index du bouton vitesse moteur (Page-Up/Down)
_BTN_GAMEPAD = 17                    # index du bouton manette (pastille etat connexion)
_BTN_TACHO = 18                      # index du bouton tachymetre optique des roues (W)
_HELP_GROUPS = [
    ("MOTION", [(16, "O", "moteurs"), (0, "Fleches", "direction"),
                (2, "Espace", "STOP"), (9, "PgU/D", "vitesse"),
                (_BTN_TACHO, "W", "tachy")]),
    ("CAMERA", [(3, "Maj+Fl", "pan/tilt"), (4, "C", "centre"), (11, "V", "cam"),
                (12, "+/-", "cible")]),
    ("SUIVI", [(5, "F", "suivi"), (6, "M", "detecteur"),
               (7, "T", "tracker"), (8, "P", "prediction")]),
    ("RECO", [(13, "R", "reco"), (15, "A", "acquis."), (14, "G", "apprend.")]),
    ("SYSTEME", [(17, "Manette", "BT"), (10, "Echap", "quitter")]),
]


def _shift_down():
    """True si Maj (Shift) est enfonce a l'instant. Sous Windows on interroge
    GetKeyState(VK_SHIFT) : cv2.waitKeyEx ne remonte pas l'etat des modificateurs
    (les fleches renvoient le meme code avec ou sans Shift). Ailleurs : False."""
    try:
        import ctypes
        return bool(ctypes.windll.user32.GetKeyState(0x10) & 0x8000)
    except Exception:
        return False


# Codes touches virtuelles Windows des fleches (VK_LEFT/UP/RIGHT/DOWN).
_VK_ARROWS = (0x26, 0x28, 0x25, 0x27)     # (haut, bas, gauche, droite)


def _arrows_down():
    """Etat PHYSIQUE des 4 fleches a l'instant -> (haut, bas, gauche, droite).
    cv2.waitKeyEx ne fournit que du keydown (auto-repete OS), jamais de keyup : pour
    un pilotage « tenue » (presser=bouger / relacher=stop) on sonde l'etat reel des
    touches via GetAsyncKeyState (bit 0x8000). Windows uniquement ; ailleurs -> tout
    False (le teleop a fenetre ne tourne que sur le bureau Windows ; --headless/Linux
    n'a pas de clavier)."""
    try:
        import ctypes
        g = ctypes.windll.user32.GetAsyncKeyState
        return tuple(bool(g(vk) & 0x8000) for vk in _VK_ARROWS)
    except Exception:
        return (False, False, False, False)


def _window_focused(title):
    """True si la fenetre nommee `title` est au premier plan (garde-fou : ne pas
    piloter les moteurs quand une AUTRE application a le focus alors qu'une fleche
    est physiquement enfoncee). Si le HWND est introuvable (fenetre pas encore creee,
    hors Windows) -> True (repli permissif, coherent avec la detection Shift)."""
    try:
        import ctypes
        u = ctypes.windll.user32
        hwnd = u.FindWindowW(None, title)
        if not hwnd:
            return True
        return u.GetForegroundWindow() == hwnd
    except Exception:
        return True


def _help_btn_for_key(key):
    """Index du bouton d'aide correspondant a la touche `key` (code cv2), ou None.
    Meme correspondance que _process_key (fleches nues = moteurs, Shift+fleches et
    i/j/k/l = pan/tilt camera ; Page-Up/Down = vitesse ; v = camera)."""
    if key == 27:
        return 10
    if key == 32:
        return 2
    if key in KEYS_PGUP or key in KEYS_PGDN:
        return _BTN_SPEED
    if key in KEYS_LEFT or key in KEYS_RIGHT or key in KEYS_UP or key in KEYS_DOWN:
        return 3 if _shift_down() else 0         # Shift -> camera, nu -> moteurs
    k = key & 0xFF
    c = chr(k).lower() if 32 <= k < 127 else ""
    if c in ("i", "j", "k", "l"):
        return 3
    if c in ("+", "=", "-"):
        return _BTN_TARGET
    return {"c": 4, "f": 5, "m": 6, "t": 7, "p": 8, "v": _BTN_CAM,
            "r": 13, "a": 15, "g": 14, "o": 16, "w": _BTN_TACHO}.get(c, None)


def _draw_help_matrix(frame, active, cam_src, target_size=None, accent=None, speed=None,
                      gamepad=None):
    """Matrice de boutons d'aide (bas-gauche), groupee par type : chaque GROUPE est
    une colonne (en-tete + boutons empiles, alignes en bas). Un bouton s'eclaircit
    quand sa touche est pressee (`active` = index eclaires). Geometrie deterministe
    -> stable, seule la couleur change. `cam_src` ('ext'/'int') annote le bouton V.
    `accent` = {index: couleur BGR} : bouton a etat COLLANT (allume tant que l'etat
    dure, teinte de la couleur) -> F allume quand le suivi est arme, G rouge pendant
    l'apprentissage. Prioritaire sur le flash `active`.
    `gamepad` : etat manette pour la pastille du bouton MANETTE -> True (vert,
    connectee), False (rouge, absente), None (gris, node desactive)."""
    accent = accent or {}
    fh = frame.shape[0]
    x0, bottom = 8, fh - 8
    sl, sd, sh = 0.45, 0.4, 0.4      # echelles libelle / description / en-tete
    pad, lg, bh, vg, colgap = 8, 6, 20, 4, 10
    dot_r, dot_gap = 4, 8            # pastille etat (bouton MANETTE)
    s = bh + vg
    x = x0
    for header, btns in _HELP_GROUPS:
        # largeur de colonne = plus large bouton du groupe
        labels = []
        colw = 0
        for idx, key, desc in btns:
            d = desc
            if idx == _BTN_CAM:
                d = f"{desc} {cam_src}"
            elif idx == _BTN_TARGET and target_size is not None:
                d = f"{desc} {target_size:.2f}"
            elif idx == _BTN_SPEED and speed is not None:
                d = f"{desc} {speed}"
            (lw, _), _ = cv2.getTextSize(key, _FONT, sl, 2)
            (dw, _), _ = cv2.getTextSize(d, _FONT, sd, 1)
            # le bouton MANETTE reserve de la place a droite pour sa pastille d'etat
            extra = (dot_gap + 2 * dot_r) if idx == _BTN_GAMEPAD else 0
            labels.append((idx, key, d, lw, dw))
            colw = max(colw, lw + lg + dw + extra + 2 * pad)
        k = len(btns)
        start_y = bottom - (k - 1) * s - bh
        # en-tete du groupe
        _put(frame, x + 1, start_y - 7, header, (140, 140, 140), sh, 1)
        for i, (idx, key, d, lw, dw) in enumerate(labels):
            by = start_y + i * s
            acc = accent.get(idx)
            on = idx in active
            lit = on or acc is not None
            roi = frame[by:by + bh, x:x + colw]
            if roi.size:
                fill = np.empty_like(roi)
                if acc is not None:
                    fill[:] = tuple(int(c * 0.6) for c in acc)   # accent assombri (fond)
                    a = 0.8
                else:
                    fill[:] = (105, 105, 105) if on else (45, 45, 45)
                    a = 0.85 if on else 0.55
                roi[:] = cv2.addWeighted(roi, 1.0 - a, fill, a, 0.0)
            border = acc if acc is not None else ((170, 170, 170) if on else (75, 75, 75))
            cv2.rectangle(frame, (x, by), (x + colw, by + bh), border, 1)
            ty = by + bh - 6
            _put(frame, x + pad, ty, key, (255, 255, 255) if lit else (215, 215, 215), sl, 2)
            _put(frame, x + pad + lw + lg, ty, d,
                 (200, 210, 200) if lit else (160, 160, 160), sd, 1)
            if idx == _BTN_GAMEPAD:
                # pastille : vert=connectee, rouge=absente, gris=node desactive
                col = _C_OFF if gamepad is None else (_C_ON if gamepad else _C_BAD)
                dx = x + pad + lw + lg + dw + dot_gap + dot_r
                cv2.circle(frame, (dx, by + bh // 2), dot_r, col, -1)
        x += colw + colgap
    return x - colgap                              # bord droit du dernier groupe dessine


def _draw_hud_cards(frame, cam_ok, camnode, link, port_name, gp_port, snap, pt,
                    motion_on, smooth, active, met, n_faces, p1_fps, tstate, gp,
                    key_flash=None, btn_accent=None, mcfg=None, speed=None, cmdvel=None,
                    es=None, esp32_port=None, ts=None, teensy_port=None, gamepad=None):
    """Dispose les cartes materielles aux zones dediees de l'ecran + la matrice
    de boutons (bas-gauche).

    Zones (exigence : « afficher toutes les cartes connectees ») :
      - CAM      : haut-gauche ;
      - STM32    : haut-droit (carte de controle historique, si `port_name`) ;
      - GROVE    : bas-droit (carte capteurs) ;
      - WSESP32  : BAS-CENTRE (carte de controle ESP32 WaveShare, si `esp32_port`) --
                   zone propre et bien visible, demandee explicitement.
    Sur un robot mono-carte, seule sa carte de controle + GROVE apparaissent ; sur un
    banc STM32+ESP32, les DEUX s'affichent (STM32 haut-droit, ESP32 bas-centre) sans se
    chevaucher. Chaque carte garde sa gestion present/absent (cadre grise + « -- »).
    `key_flash` = index de boutons a eclairer (touches recemment pressees)."""
    fw, fh = frame.shape[1], frame.shape[0]
    m, gap = 8, 8
    rx = fw - 320 - m
    _draw_cam_card(frame, m, m, cam_ok, camnode, active, met, n_faces, p1_fps,
                   tstate, mcfg=mcfg)

    # --- carte de controle STM32 : haut-droit (si configuree) ---
    if port_name is not None:
        _draw_stm_card(frame, rx, m, bool(link and link.connected), port_name, snap,
                       pt, motion_on, smooth, mcfg=mcfg, cmdvel=cmdvel)

    # --- carte capteurs GROVE : bas-droit ---
    _draw_grove_card(frame, rx, fh - 176 - 16, gp_port, gp, mcfg=mcfg)

    # --- matrice de boutons (bas-gauche) : dessinee d'abord pour connaitre son bord
    #     droit et y adosser la carte ESP32 sans chevauchement. ---
    mx = _draw_help_matrix(frame, key_flash or set(), getattr(camnode, "source", "ext"),
                           getattr(pt, "deadzone", None), accent=btn_accent, speed=speed,
                           gamepad=gamepad)

    # --- carte de controle ESP32 WaveShare : BAS-CENTRE (zone dediee) ---
    # Centree dans la bande libre du bas, entre la matrice (a gauche) et GROVE (a
    # droite), pour un "bas-centre" propre sans mordre sur les boutons ni GROVE.
    if esp32_port is not None:
        band_l = (mx if mx is not None else 0) + gap
        band_r = rx - gap
        ex = band_l + max(0, (band_r - band_l - 320) // 2)
        ey = fh - 190 - 16                         # ancree en bas (carte 190 de haut)
        _draw_wsesp32_card(frame, ex, ey, esp32_port, es, mcfg=mcfg)

    # --- carte de controle TEENSY (3e carte) : BAS-CENTRE, EMPILEE au-dessus de
    #     l'ESP32 si le banc en a une, sinon meme ancrage bas (robot mono-Teensy). ---
    if teensy_port is not None:
        band_l = (mx if mx is not None else 0) + gap
        band_r = rx - gap
        tx = band_l + max(0, (band_r - band_l - 320) // 2)
        ty = fh - 190 - 16
        if esp32_port is not None:
            ty = ty - 190 - gap                    # empilee au-dessus de l'ESP32
        _draw_teensy_card(frame, tx, ty, teensy_port, ts, mcfg=mcfg)


class _ServoView:
    """Adaptateur : expose un ServoState (topic /servo/state) avec l'interface que
    les helpers d'overlay de robot_control attendent d'un servo (`pt`).

    Permet de reutiliser draw_overlay() TEL QUEL : il lit pt.angleH/angleV/deadzone
    et pt.motionStats(). Ici ces valeurs viennent du message, pas de l'objet servo
    (que le Core ne detient pas : il vit dans ServoNode).
    """

    def __init__(self, state: ServoState):
        self.angleH = state.angleH
        self.angleV = state.angleV
        self.deadzone = state.deadzone
        self._motion = state.motion

    def motionStats(self):
        return self._motion


class RobotControlCore:
    """Hote : construit les nodes, deroule la boucle (spin + HMI + clavier + MCP)."""

    def __init__(self, args):
        self.args = args
        self.motion_on = not args.no_motion
        self.board_only = args.board_only        # pur pilote COM4 (BoardNode seul)
        self.headless = args.headless or args.board_only   # sans fenetre ni clavier
        self.win = "Bamboo v4 - controle v3 (nodes ROS2 simules)"

        self.tel = None
        self.link = None
        self.motion = None
        self.executor = None
        self.camera = self.tracking = self.servo = self.board = None
        self.stm32_link = None                   # lien STM32 pour la carte HUD (ou None)
        self.stm32_port = None                   # port STM32 pour l'etiquette carte HUD (ou None)
        self.wsesp32 = None                      # node carte de controle ESP32 WaveShare (optionnel)
        self.esp32_port = None                   # port ESP32 pour l'etiquette carte HUD (ou None)
        self.teensy = None                       # node 3e carte de controle Teensy (optionnel)
        self.teensy_port = None                  # port Teensy pour l'etiquette carte HUD (ou None)
        self.grovepi = None                      # node carte capteurs GrovePi+ (optionnel)
        self.gamepad = None                      # node manette de jeu (optionnel)
        self.recognition = None                  # node reconnaissance de visage (optionnel)
        self.train = None                        # node dedie a l'apprentissage (optionnel)
        self.tacho = None                        # node tachymetre optique des roues (optionnel)
        self.server = None                       # serveur de commandes MCP (gateway)
        self.mcfg = None                         # config metriques (HMI/MCP/log), cf. setup()

        # --- etat de boucle cote Core (HMI/clavier/MCP), comme l'ancien app -----
        self.active = True           # suivi arme par defaut (touche F) -> /tracking/config
        self.last_seq = -1           # derniere detection loguee (event detect)
        self.last_locked = None      # dernier etat de verrou (transition lock/unlock)
        self._cmdvel_t0 = 0.0        # horodatage du dernier republication cmd_vel (~10 Hz)
        self._hold_prev = (0, 0)     # dernier (fwd, turn) sonde -> republication immediate au changement
        self._servo_seq = 0          # sequence des ServoCmd clavier (messages discrets)
        self._recog_seq = 0          # sequence des commandes reco ponctuelles (train/fichier)
        self._tacho_seq = 0          # sequence des consignes tachymetre (commande discrete)
        self._tacho_emit = 0.0       # dernier envoi de la metrique tacho (throttle ~5 Hz)

        # --- etat manette (detection de fronts : une action par appui) ----------
        self._gp_prev_buttons = ()   # dernier vecteur de boutons (0/1) vu sur /joy
        self._gp_prev_hat = (0, 0)   # dernier etat de la croix directionnelle
        self._gp_prev_trig = (0, 0)  # dernier etat (enfonce 0/1) des gachettes ZL/ZR
        self._gp_seq = -1            # derniere trame /joy traitee pour les boutons
        self._gp_driving = False     # la manette pilotait au tour precedent (edge stop)

        self._disp_t0 = 0.0
        self._disp_n = 0
        self.disp_fps = 0.0
        self._hb_t0 = 0.0
        self._btn_flash = {}         # index bouton d'aide -> date de derniere pression
        self._train_done_t = 0.0     # date de fin du dernier batch (panneau log ~20 s apres)

    # -----------------------------------------------------------------------
    # Mise en place : telemetrie, liaison serie, nodes, executeur
    # -----------------------------------------------------------------------
    def setup(self):
        args = self.args
        # 0) banniere version (TOT : avant l'ouverture camera => precede « Camera index »)
        print(f"App robot_control v{APP_VERSION}")
        # config des metriques (HMI/MCP/log) : coupures issues de --no-metric
        self.mcfg = MetricsConfig().parse_no_metric(getattr(args, "no_metric", None))
        # 1) telemetrie (journal structure ; budget = fichier vif + 1 backup)
        self.tel = Telemetry(log_dir=args.log_dir, enabled=not args.no_telemetry,
                             max_bytes=int(max(1.0, args.log_budget_mb) * 1_000_000 / 2),
                             version=APP_VERSION)
        self.tel.log("event", msg="start", app="v3", version=APP_VERSION,
                     robot=getattr(args, "robot", None),
                     motion=self.motion_on, port=args.port,
                     index=str(args.index), backend=args.backend, size=args.size,
                     pan_gain=args.pan_gain, tilt_gain=args.tilt_gain,
                     deadzone=args.deadzone, dead_hyst=args.dead_hyst, max_step=args.max_step,
                     invert_pan=args.invert_pan, invert_tilt=args.invert_tilt)

        # 2) cartes de controle (profil robot) : la carte PRINCIPALE (1re activee)
        #    porte le lien PARTAGE self.link (moteurs + servos + gateway MCP) ; les
        #    cartes secondaires (banc STM32+ESP32 branches ensemble) ont leur propre
        #    node autonome. BoardNode = STM32, WSEsp32Node = ESP32 WaveShare.
        controls = getattr(args, "controls", None) or [
            {"kind": "stm32", "port": args.port, "baud": args.baud,
             "cpr": None, "vid_pid": None, "enabled": True}]
        enabled = [c for c in controls if c.get("enabled", True)]
        primary = enabled[0] if enabled else None
        control_nodes = []
        for c in enabled:
            is_primary = c is primary
            # rappel quand le scan auto trouve la carte sur un COM different du COM
            # configure : on reecrit le port dans le profil robot (self-healing) pour
            # que le prochain lancement la retrouve instantanement (cf. persist_control_port).
            on_resolved = (lambda p, kind=c["kind"]:
                           robot_profile.persist_control_port(
                               getattr(args, "robot", None), kind, p))
            if c["kind"] == "stm32":
                link = RobotComSerial(c["port"], c["baud"], telemetry=self.tel,
                                      cpr=c.get("cpr") or DEFAULT_CPR,
                                      vid_pid=c.get("vid_pid"),
                                      protocol=c.get("protocol", "yahboom"),
                                      on_port_resolved=on_resolved)
                node = BoardNode(link)
                self.board = node
                self.stm32_link = link
                self.stm32_port = c["port"]
                if is_primary:
                    self.link = link
            elif c["kind"] == "esp32":
                self.esp32_port = c["port"]
                if is_primary:
                    # ESP32 = carte principale : le Core POSSEDE le lien (le node le
                    # recoit injecte, comme BoardNode ; ferme par le Core au shutdown).
                    link = Esp32ComSerial(
                        c["port"], c.get("baud") or ESP32_DEFAULT_BAUD,
                        telemetry=self.tel, cpr=c.get("cpr") or ESP32_DEFAULT_CPR,
                        vid_pid=c.get("vid_pid") or ESP32_VID_PID,
                        protocol=c.get("protocol", "yahboom"),
                        on_port_resolved=on_resolved)
                    node = WSEsp32Node(link=link)
                    self.link = link
                else:
                    # carte secondaire : le node possede/ferme son propre lien.
                    # rx_prefix="esp32_" -> rx:esp32_* distinct de la carte primaire.
                    node = WSEsp32Node(port=c["port"],
                                       baud=c.get("baud") or ESP32_DEFAULT_BAUD,
                                       telemetry=self.tel, cpr=c.get("cpr"),
                                       rx_prefix="esp32_",
                                       protocol=c.get("protocol", "yahboom"),
                                       on_port_resolved=on_resolved)
                self.wsesp32 = node
            elif c["kind"] == "teensy":
                self.teensy_port = c["port"]
                if is_primary:
                    # Teensy = carte principale : le Core POSSEDE le lien (le node le
                    # recoit injecte, comme BoardNode ; ferme par le Core au shutdown).
                    link = TeensyComSerial(
                        c["port"], c.get("baud") or TEENSY_DEFAULT_BAUD,
                        telemetry=self.tel, cpr=c.get("cpr") or TEENSY_DEFAULT_CPR,
                        vid_pid=c.get("vid_pid") or TEENSY_VID_PID,
                        protocol=c.get("protocol", "yahboom"),
                        on_port_resolved=on_resolved)
                    node = TeensyNode(link=link)
                    self.link = link
                else:
                    # carte secondaire : le node possede/ferme son propre lien.
                    # rx_prefix="teensy_" -> rx:teensy_* distinct de la carte primaire.
                    node = TeensyNode(port=c["port"],
                                      baud=c.get("baud") or TEENSY_DEFAULT_BAUD,
                                      telemetry=self.tel, cpr=c.get("cpr"),
                                      rx_prefix="teensy_",
                                      protocol=c.get("protocol", "yahboom"),
                                      on_port_resolved=on_resolved)
                self.teensy = node
            else:
                continue
            control_nodes.append(node)
        time.sleep(0.3)                          # laisse le(s) thread(s) lecteur s'ouvrir
        self.motion = RobotMotorDrive(self.link, maxPwm=args.max_pwm)

        # 3) carte capteurs GrovePi+ (ultrasons + IMU) : optionnelle, tolerante a
        #    l'absence (reconnexion auto). Port/baud issus du profil (args.sensors) :
        #    NE PAS reutiliser args.baud (= baud carte de controle, 921600 en ESP32).
        sensors = getattr(args, "sensors", None)
        if sensors and sensors.get("enabled", True):
            self.grovepi = GrovePiNode(port=sensors.get("port"),
                                       baud=sensors.get("baud", 115200),
                                       telemetry=self.tel,
                                       vid_pid=sensors.get("vid_pid"),
                                       protocol=sensors.get("protocol", "yahboom"))

        # 3b) manette de jeu (teleop portable pygame) : optionnelle, utile meme en
        #     --board-only (pilotage sans camera). Absente/pygame manquant -> node
        #     inerte (connected=False), le Core force alors un stop (securite).
        gamepad = getattr(args, "gamepad", None)
        if gamepad and gamepad.get("enabled", True):
            self.gamepad = GamepadNode(index=gamepad.get("index", 0),
                                       deadzone=gamepad.get("deadzone", 0.12),
                                       expo=gamepad.get("expo", 0.35),
                                       probe=bool(getattr(args, "gamepad_probe", False)))

        # 4) nodes + executeur : en --board-only, seules les cartes (ni cam ni suivi)
        self.executor = Executor()
        if self.board_only:
            for node in control_nodes:
                self.executor.add_node(node)
            if self.grovepi is not None:
                self.executor.add_node(self.grovepi)
            if self.gamepad is not None:
                self.executor.add_node(self.gamepad)
        else:
            self.camera = CameraNode(args, telemetry=self.tel)
            self.tracking = TrackingNode(args, telemetry=self.tel)
            # reconnaissance APRES le suivi : consomme /tracking/result (box+landmarks+lock)
            self.recognition = FaceRecogNode(args, telemetry=self.tel)
            # apprentissage : node DEDIE (worker unique) -> /recognition/train_state
            self.train = FaceTrainNode(args, telemetry=self.tel)
            # servos camera pan/tilt : sur la carte de controle principale (self.link).
            self.servo = ServoNode(args, self.link, telemetry=self.tel)
            # tachymetre optique : APRES la camera, AVANT tout le reste. L'executeur
            # deroule SET->PROCESS->GET node par node, donc il voit la frame du tour ;
            # et le HUD n'etant dessine qu'apres spin_once(), il la voit VIERGE.
            self.tacho = WheelTachoNode(active=bool(getattr(args, "tacho", False)),
                                        telemetry=self.tel)
            nodes = [self.camera, self.tacho, self.tracking, self.recognition,
                     self.train, self.servo] + control_nodes
            if self.grovepi is not None:
                nodes.append(self.grovepi)
            if self.gamepad is not None:
                nodes.append(self.gamepad)
            for node in nodes:
                self.executor.add_node(node)
        self.executor.start()                    # ouvre camera (non fatal) + detecteur + P2

        # 4) config initiale du suivi + serveur de commandes MCP (socket loopback)
        if not self.board_only:
            self.executor.publish("/tracking/config", TrackingConfig(active=self.active))
            self.executor.publish("/recognition/config",
                                  RecognitionConfig(mode=self.recognition.mode))
        host, port = gateway.parse_addr(os.environ.get("BAMBOU_MCP_PORT"))
        self.server = gateway.CommandServer(
            on_config=self._apply_config, link=self.link, motion=self.motion,
            cpr=self.link.cpr or 1320.0, host=host, port=port).start()
        label = getattr(args, "robot_label", None) or getattr(args, "robot", "")
        print(f"Robot : {label}  |  carte {args.port} @ {args.baud}  |  "
              f"moteurs {'ACTIFS' if self.motion_on else 'desactives'}")
        if self.board_only:
            print("Mode board-only : ni camera ni suivi (pur pilote COM4 pour le MCP).")
        else:
            print("Fenetre %s. F=suivi, Echap=quitter."
                  % ("desactivee (--headless)" if self.headless else "ouverte"))
        return self

    # -----------------------------------------------------------------------
    # Publication de config suivi (touches F/M/T/P, MCP)
    # -----------------------------------------------------------------------
    def _publish_cfg(self, detector=None, track_mode=None, predict_mode=None):
        """Publie /tracking/config avec l'etat d'armement + un eventuel changement."""
        self.executor.publish("/tracking/config", TrackingConfig(
            active=self.active, detector=detector,
            track_mode=track_mode, predict_mode=predict_mode))

    def _publish_servo(self, kind, delta=0.0):
        """Publie un ServoCmd clavier discret (recentrage / pas manuel pan-tilt)."""
        self._servo_seq += 1
        self.executor.publish("/servo/cmd", ServoCmd(kind=kind, seq=self._servo_seq, delta=delta))

    def _publish_recog(self, mode=None, command=None, path=None, id_lot=None):
        """Publie /recognition/config : changement de mode et/ou commande ponctuelle."""
        if command is not None:
            self._recog_seq += 1
        self.executor.publish("/recognition/config", RecognitionConfig(
            mode=mode, command=command, seq=self._recog_seq, path=path, id_lot=id_lot))

    def _publish_tacho(self, active=None, calibrate=False, record_s=0.0, roi=None):
        """Publie /tacho/config. `seq` incremente a CHAQUE appel : les champs
        impulsionnels (calibrate, record_s) ne doivent declencher qu'une fois."""
        self._tacho_seq += 1
        self.executor.publish("/tacho/config", WheelTachoConfig(
            seq=self._tacho_seq, active=active, calibrate=calibrate,
            record_s=record_s, roi=roi))

    # -----------------------------------------------------------------------
    # Journalisation par NOUVELLE detection (event detect + transition verrou)
    # -----------------------------------------------------------------------
    def _log_detection(self, res: TrackingResult):
        """Reprend le journal per-seq de l'ancien _on_new_detection (analyse/MCP).
        L'enregistrement `detect` releve du groupe metrique `cam_detect` : coupe si
        ce groupe est desactive pour MCP et log (les transitions de verrou restent)."""
        tstate = res.tstate
        score = tstate.get("score")
        pspeed = tstate.get("pred_speed")
        perr = tstate.get("pred_err")
        mc = self.mcfg
        if mc is None or mc.enabled("cam_detect", "mcp") or mc.enabled("cam_detect", "log"):
            self.tel.log("detect", seq=res.seq, faces=len(res.faces),
                     nx=None if res.nx is None else round(res.nx, 4),
                     ny=None if res.ny is None else round(res.ny, 4),
                     area=round(res.area_pct, 2), det_fps=round(res.det_fps, 1),
                     tracking=self.active, trk=tstate.get("mode"),
                     locked=tstate.get("locked"), src=tstate.get("src"),
                     raw_det=tstate.get("raw_det"),
                     score=None if score is None else round(score, 3),
                     lock_id=tstate.get("lock_id"),
                     lock_age=round(tstate.get("lock_age", 0.0), 2),
                     predict=tstate.get("predict"),
                     pred_speed=None if pspeed is None else round(pspeed, 3),
                     pred_err=None if perr is None else round(perr, 4))
        if tstate.get("mode") != "none" and tstate.get("locked") != self.last_locked:
            unlocked = not tstate.get("locked")
            self.tel.log("event", msg="track_unlock" if unlocked else "track_lock",
                         src=tstate.get("src"),
                         reason=tstate.get("unlock_reason") if unlocked else None,
                         lock_id=tstate.get("lock_id"),
                         lock_age=round(tstate.get("lock_age", 0.0), 2),
                         score=None if score is None else round(score, 3))
            self.last_locked = tstate.get("locked")

    # -----------------------------------------------------------------------
    # Commande de config MCP (socket, via gateway) -> /tracking/config
    # -----------------------------------------------------------------------
    def _apply_config(self, cfg):
        """Applique une commande de config MCP recue par socket (gateway.drain).

        cfg (gateway.config_from) porte UNE cle : detector / track_mode /
        predict_mode / active / metrics -> publiee sur /tracking/config (appliquee
        au spin suivant). Renvoie un compte-rendu texte pour le client MCP.
        """
        # --- metriques (HMI/MCP/log) : applicable meme en --board-only ----------
        if "metrics" in cfg and self.mcfg is not None:
            m = cfg["metrics"]                   # {"spec": ..., "on": bool}
            ok, msg = self.mcfg.apply(m.get("spec", ""), bool(m.get("on", False)))
            self.tel.log("event", msg="set_metrics", spec=m.get("spec"),
                         on=bool(m.get("on")), ok=ok)
            return "Metrique : %s" % msg
        # --- tachymetre optique des roues -----------------------------------
        if "tacho" in cfg:
            if self.tacho is None:
                return "Tachymetre indisponible (--board-only : pas de camera)."
            t = cfg["tacho"]                     # {"active":..,"calibrate":..,"record_s":..}
            self._publish_tacho(active=t.get("active"),
                                calibrate=bool(t.get("calibrate")),
                                record_s=float(t.get("record_s") or 0.0),
                                roi=t.get("roi"))
            bits = []
            if t.get("active") is not None:
                bits.append("armement -> %s" % ("ON" if t["active"] else "off"))
            if t.get("calibrate"):
                bits.append("recalibration demandee")
            if t.get("record_s"):
                bits.append("capture de %.1f s de profils" % float(t["record_s"]))
            if t.get("roi"):
                bits.append("ROI -> %s" % (tuple(t["roi"]),))
            return "Tachymetre : %s." % (", ".join(bits) or "rien a faire")
        if self.tracking is None:                # --board-only : pas de node suivi
            return "Vision desactivee (--board-only) : commande de suivi ignoree."
        d = cfg.get("detector")
        if d:
            self._publish_cfg(detector=d)
            return "Detecteur -> %s (publie sur /tracking/config)." % d
        tm = cfg.get("track_mode")
        if tm:
            self._publish_cfg(track_mode=tm)
            return "Tracker -> %s (publie sur /tracking/config)." % tm
        pm = cfg.get("predict_mode")
        if pm:
            self._publish_cfg(predict_mode=pm)
            return "Prediction -> %s (publie sur /tracking/config)." % pm
        if "active" in cfg:
            self.active = bool(cfg["active"])
            self._publish_cfg()
            self.tel.log("event", msg="tracking", on=self.active, source="mcp")
            return "Suivi -> %s." % ("ON" if self.active else "off")
        # --- reconnaissance de visage (node optionnel) ----------------------
        if self.recognition is not None:
            rm = cfg.get("recog_mode")
            if rm:
                self._publish_recog(mode=rm)
                return "Reconnaissance -> %s (publie sur /recognition/config)." % rm
            if cfg.get("train"):
                self._publish_recog(command="train")
                return "Apprentissage (enrolement) lance dans le thread worker."
            if "recognize_image" in cfg:
                self._publish_recog(command="recognize_file", path=cfg["recognize_image"])
                return "Reconnaissance image -> %s." % cfg["recognize_image"]
            if "acquire_image" in cfg:
                self._publish_recog(command="acquire_file", path=cfg["acquire_image"],
                                    id_lot=cfg.get("id_lot"))
                return "Acquisition image -> %s." % cfg["acquire_image"]
        return "Commande de config vide."

    # -----------------------------------------------------------------------
    # Clavier (identique a robot_control ; nudges/center via /servo/cmd)
    # -----------------------------------------------------------------------
    def _drive_from_arrows(self):
        """Pilotage moteurs par TENUE des fleches (modele presser=bouger/relacher=stop).
        Sonde l'etat physique des 4 fleches et fixe l'etat cmd_vel a la vitesse programmee :
        fwd = haut - bas, turn = gauche - droite (les opposees s'annulent, combinaison = arc).
        Republie IMMEDIATEMENT sur changement de (fwd, turn) -> demarrage / arret francs ;
        entre deux, le cycle ~10 Hz de la boucle entretient la trame.

        Neutralise (etat 0) si moteurs desarmes, Shift enfonce (reserve au pan/tilt camera),
        ou fenetre pas au premier plan (garde-fou anti-mouvement fantome)."""
        if (not self.motion_on) or _shift_down() or (not _window_focused(self.win)):
            fwd, turn = 0, 0
        else:
            up, down, left, right = _arrows_down()
            fwd = int(up) - int(down)
            turn = int(left) - int(right)         # gauche=+ (anti-horaire), droite=-
        if (fwd, turn) != self._hold_prev:
            self.motion.holdVelocity(fwd, turn)
            if self.motion_on:
                self.motion.publish()             # transition franche (start/stop immediat)
                self._cmdvel_t0 = time.time()
            self._hold_prev = (fwd, turn)

    # -----------------------------------------------------------------------
    # Manette de jeu (/joy) : jumeau du clavier, en cohabitation
    # -----------------------------------------------------------------------
    def _axis(self, axes, idx):
        """Lit l'axe `idx` (float [-1,1]), applique zone morte + expo -> [-1,1].
        Hors zone morte, renormalise puis courbe l'expo (finesse au centre, pleine
        echelle au bord). En deca de la zone morte : 0 (pas de derive au repos)."""
        v = float(axes[idx]) if 0 <= idx < len(axes) else 0.0
        dz = self.gamepad.deadzone
        m = abs(v)
        if m <= dz:
            return 0.0
        s = (m - dz) / (1.0 - dz)                 # renormalise l'amplitude utile
        e = self.gamepad.expo
        s = (1.0 - e) * s + e * (s ** 3)          # expo : doux au centre
        return s if v >= 0 else -s

    def _drive_from_gamepad(self, joy):
        """Pilotage analogique par le stick gauche (poussee = vitesse), en cohabitation
        avec le clavier. La manette ne prend la main que lorsqu'elle est POUSSEE : au
        relachement (ou deconnexion) elle emet un STOP franc une fois puis rend la main
        (le clavier peut alors piloter). Manette absente = aucune ingerence."""
        driving = False
        fwd = turn = 0.0
        connected = (joy is not None and joy.connected and self.motion_on)
        if connected:
            fwd = -self._axis(joy.axes, AX_LY)    # avant = stick vers le haut (LY < 0)
            turn = -self._axis(joy.axes, AX_LX)   # gauche = + (angular.z anti-horaire)
            driving = (fwd != 0.0 or turn != 0.0)
        if driving:
            self.motion.holdVelocityAnalog(fwd, turn)
            if not self._gp_driving:              # front montant : demarrage franc
                self.motion.publish()
                self._cmdvel_t0 = time.time()
            self._gp_driving = True
        elif self._gp_driving:                    # front descendant / deconnexion : stop franc
            self.motion.holdVelocityAnalog(0.0, 0.0)
            if self.motion_on:
                self.motion.stop()                # cmd_vel(0,0) x3 -> Motion_Stop
            self._gp_driving = False
            self._hold_prev = (0, 0)              # resynchronise l'etat de tenue clavier

    def _gamepad_buttons(self, joy):
        """Actions manette a FRONT MONTANT (une par appui) + camera au stick droit.
        Reprend les memes branches que _process_key : chaque bouton equivaut a sa touche.
        Ne traite chaque trame /joy qu'une fois (garde sur seq)."""
        if joy is None or not joy.connected:
            # deconnexion : oublie l'etat pour ne pas rejouer d'edges au retour
            self._gp_prev_buttons = ()
            self._gp_prev_hat = (0, 0)
            self._gp_prev_trig = (0, 0)
            return
        if joy.seq == self._gp_seq:               # deja traitee (bus profondeur 1)
            return
        self._gp_seq = joy.seq
        btn = joy.buttons
        prev = self._gp_prev_buttons

        def pressed(i):                           # front montant du bouton i
            return (i < len(btn) and btn[i]
                    and not (i < len(prev) and prev[i]))

        # --- deplacement / securite ---------------------------------------
        if pressed(BTN_B):                        # B : STOP immediat (equiv. Espace)
            self.motion.stop()
        if pressed(BTN_L3):                       # clic stick G : armer/desarmer (equiv. O)
            self.motion_on = not self.motion_on
            if not self.motion_on:
                self.motion.stop()
            self.tel.log("event", msg="motion", on=self.motion_on, source="gamepad")
            print(f"Moteurs {'ACTIFS' if self.motion_on else 'desactives'}")

        # --- plage de vitesse : +/- (plancher) ; gachettes ZL/ZR (plafond) --
        if pressed(BTN_START):                    # + : vitesse min +1
            self.motion.bumpSpeedMin(+1)
        if pressed(BTN_BACK):                     # - : vitesse min -1
            self.motion.bumpSpeedMin(-1)
        trig = (1 if self._axis_raw(joy.axes, AX_LT) > 0.5 else 0,
                1 if self._axis_raw(joy.axes, AX_RT) > 0.5 else 0)
        if trig[0] and not self._gp_prev_trig[0]:  # ZL : vitesse max -1
            self.motion.bumpSpeedMax(-1)
        if trig[1] and not self._gp_prev_trig[1]:  # ZR : vitesse max +1
            self.motion.bumpSpeedMax(+1)
        self._gp_prev_trig = trig

        # --- vision / modes (equivalents clavier F/R/A/T/P/M/G) ------------
        if pressed(BTN_X):                        # X : suivi on/off (equiv. F)
            self.active = not self.active
            self._publish_cfg()
            self.tel.log("event", msg="tracking", on=self.active, source="gamepad")
        if pressed(BTN_Y) and self.recognition is not None:   # Y : reconnaissance (equiv. R)
            cur = self.recognition.mode
            self._publish_recog(mode="off" if cur != "off" else "recognition")
        if pressed(BTN_A) and self.recognition is not None:   # A : acquisition (equiv. A)
            cur = self.recognition.mode
            self._publish_recog(mode="recognition" if cur == "acquisition" else "acquisition")
        if pressed(BTN_LB) and self.tracking is not None:     # LB : cycle tracker (equiv. T)
            cyc = self.tracking.webcam.availableTrackers()
            if cyc:
                cur = self.tracking.webcam.trackMode
                i = cyc.index(cur) if cur in cyc else -1
                self._publish_cfg(track_mode=cyc[(i + 1) % len(cyc)])
        if pressed(BTN_RB) and self.tracking is not None:     # RB : cycle prediction (equiv. P)
            cur = self.tracking.webcam.predict_mode
            i = PREDICT_MODES.index(cur) if cur in PREDICT_MODES else 0
            self._publish_cfg(predict_mode=PREDICT_MODES[(i + 1) % len(PREDICT_MODES)])

        # --- croix directionnelle (D-pad) : taille cible / detecteur / train
        hat = joy.hats[HAT_DPAD] if HAT_DPAD < len(joy.hats) else (0, 0)
        ph = self._gp_prev_hat
        if hat[0] == 1 and ph[0] != 1:            # D-pad droite : cible + (equiv. +)
            self._resize_target(+TARGET_STEP)
        if hat[0] == -1 and ph[0] != -1:          # D-pad gauche : cible - (equiv. -)
            self._resize_target(-TARGET_STEP)
        if hat[1] == 1 and ph[1] != 1 and self.tracking is not None:   # haut : detecteur (equiv. M)
            cyc = self.tracking.webcam.availableDetectors()
            if cyc:
                cur = self.tracking.webcam.detector
                i = cyc.index(cur) if cur in cyc else -1
                self._publish_cfg(detector=cyc[(i + 1) % len(cyc)])
        if hat[1] == -1 and ph[1] != -1 and self.recognition is not None:  # bas : train (equiv. G)
            self._publish_recog(command="train")
            self.tel.log("event", msg="train_request", source="gamepad")
        self._gp_prev_hat = hat

        # --- camera au stick droit (nudges continus proportionnels a la poussee) -
        if self.servo is not None:
            pan = self._axis(joy.axes, AX_RX)     # droite = + (comme fleche droite)
            tilt = -self._axis(joy.axes, AX_RY)   # stick haut = tilt + (comme fleche haut)
            if pan != 0.0:
                self._publish_servo("nudge_pan", SERVO_STEP * pan)
            if tilt != 0.0:
                self._publish_servo("nudge_tilt", SERVO_STEP * tilt)
        if pressed(BTN_R3):                       # clic stick D : recentrer (equiv. C)
            self._publish_servo("center")

        self._gp_prev_buttons = tuple(btn)

    @staticmethod
    def _axis_raw(axes, idx):
        """Valeur brute de l'axe (sans zone morte) : gachettes ZL/ZR en detection de front."""
        return float(axes[idx]) if 0 <= idx < len(axes) else -1.0

    def _process_key(self, key, now):
        """Traite une touche. Retourne (quit, moved_now)."""
        k = key & 0xFF
        c = chr(k).lower() if 32 <= k < 127 else ""
        self.tel.log("key", code=key, k=k, c=c)

        if key == 27:                            # Echap
            self.tel.log("event", msg="quit")
            return True, False

        # --- fleches : Maj+fleche = camera pan/tilt ; fleche nue = moteurs -----
        arrow = (key in KEYS_LEFT or key in KEYS_RIGHT
                 or key in KEYS_UP or key in KEYS_DOWN)
        if arrow and _shift_down():              # Maj+fleche : orientation camera
            if key in KEYS_LEFT:
                self._publish_servo("nudge_pan", -SERVO_STEP)
            elif key in KEYS_RIGHT:
                self._publish_servo("nudge_pan", +SERVO_STEP)
            elif key in KEYS_UP:
                self._publish_servo("nudge_tilt", +SERVO_STEP)
            else:
                self._publish_servo("nudge_tilt", -SERVO_STEP)
        elif arrow:                              # fleche nue : pilotage moteurs
            # Modele « tenue » : le mouvement est pilote par le sondage de l'etat
            # PHYSIQUE des fleches dans la boucle principale (_drive_from_arrows),
            # pas par cet evenement keydown (qui ne dit rien du relachement). On ne
            # fait rien ici ; on signale juste « moved » pour l'eclairage du bouton.
            return False, True
        elif key in KEYS_PGUP:                    # Page-Up : vitesse +1 (0..9)
            self.motion.setSpeed(self.motion.speedLevel + 1)
        elif key in KEYS_PGDN:                    # Page-Down : vitesse -1 (0..9)
            self.motion.setSpeed(self.motion.speedLevel - 1)
        elif c == "i":                           # camera pan/tilt : alias clavier
            self._publish_servo("nudge_tilt", +SERVO_STEP)
        elif c == "k":
            self._publish_servo("nudge_tilt", -SERVO_STEP)
        elif c == "j":
            self._publish_servo("nudge_pan", -SERVO_STEP)
        elif c == "l":
            self._publish_servo("nudge_pan", +SERVO_STEP)
        elif c == "f":                           # armer / desarmer le suivi
            self.active = not self.active
            self._publish_cfg()
            self.tel.log("event", msg="tracking", on=self.active)
        elif c == "c":                           # recentrer la camera
            self._publish_servo("center")
            self.tel.log("event", msg="center")
        elif c == "m":                           # cycle detecteurs disponibles
            cyc = self.tracking.webcam.availableDetectors()
            if cyc:
                cur = self.tracking.webcam.detector
                i = cyc.index(cur) if cur in cyc else -1
                self._publish_cfg(detector=cyc[(i + 1) % len(cyc)])
        elif c == "t":                           # cycle trackers (none/mil/vit)
            cyc = self.tracking.webcam.availableTrackers()
            if cyc:
                cur = self.tracking.webcam.trackMode
                i = cyc.index(cur) if cur in cyc else -1
                self._publish_cfg(track_mode=cyc[(i + 1) % len(cyc)])
        elif c == "p":                           # cycle mode de prediction
            cur = self.tracking.webcam.predict_mode
            i = PREDICT_MODES.index(cur) if cur in PREDICT_MODES else 0
            self._publish_cfg(predict_mode=PREDICT_MODES[(i + 1) % len(PREDICT_MODES)])
        elif c == "r":                           # toggle reconnaissance on/off
            if self.recognition is not None:
                cur = self.recognition.mode
                self._publish_recog(mode="off" if cur != "off" else "recognition")
        elif c == "a":                           # toggle acquisition (jeu d'apprentissage)
            if self.recognition is not None:
                cur = self.recognition.mode
                self._publish_recog(
                    mode="recognition" if cur == "acquisition" else "acquisition")
        elif c == "g":                           # lance une passe d'apprentissage (enrolement)
            if self.recognition is not None:
                self._publish_recog(command="train")
                self.tel.log("event", msg="train_request", source="key")
        elif c == "v":                           # bascule camera interne <-> externe
            if self.camera is not None:
                self.camera.switch_camera()
                self.tel.log("event", msg="camera_switch", source=self.camera.source)
        elif c in ("+", "="):                    # agrandir la surface cible
            self._resize_target(+TARGET_STEP)
        elif c == "-":                           # retrecir la surface cible
            self._resize_target(-TARGET_STEP)
        elif c == "w":                           # bascule tachymetre optique des roues
            if self.tacho is not None:
                self._publish_tacho(active=not self.tacho.active)
        elif c == "o":                           # bascule moteurs ON/OFF (securite)
            self.motion_on = not self.motion_on
            if not self.motion_on:               # a la coupure : cmd_vel(0,0) franc
                self.motion.stop()
            self.tel.log("event", msg="motion", on=self.motion_on)
            print(f"Moteurs {'ACTIFS' if self.motion_on else 'desactives'}")
        elif c.isdigit():                        # 0-9 : reglage direct de la vitesse
            self.motion.setSpeed(int(c))
        elif k == 32:                            # Espace : STOP (etat cmd_vel a zero)
            self.motion.stop()
        return False, False

    def _resize_target(self, delta):
        """Ajuste en direct la taille de la surface cible (demi-cote), bornee.
        Agit sur le RobotServoMotor du ServoNode ; ServoState (donc le reticule)
        se met a jour au tour suivant. Sans effet en --board-only (pas de servo)."""
        if self.servo is None:
            return
        m = self.servo.servo
        m.deadzone = clamp_target(m.deadzone + delta)
        self.tel.log("event", msg="target_size", size=round(m.deadzone, 3))

    # -----------------------------------------------------------------------
    # Boucle principale + arret
    # -----------------------------------------------------------------------
    def run(self):
        # setup() dans le bloc protege : un echec d'ouverture camera/detecteur
        # (RuntimeError leve par un node au demarrage) libere quand meme serie/journaux.
        try:
            self.setup()
            self._loop_board() if self.board_only else self._loop()
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    def _loop_board(self):
        """Boucle pur pilote COM4 (--board-only) : spin BoardNode, draine les
        commandes MCP et journalise un heartbeat carte. Ni camera ni suivi."""
        print("En ecoute (board-only). Ctrl-C pour quitter.")
        self._hb_t0 = time.time()
        while True:
            self.executor.spin_once()            # BoardNode (+GrovePi +manette) : lit la telemetrie
            self.server.drain()                  # execute les commandes carte MCP
            snap = _board_snap(self.executor.latest("/board/telemetry"))
            gp = self.executor.latest("/grovepi/telemetry")
            now2 = time.time()
            # manette : pilotage sans camera (stick gauche + B/L3/+-/ZL/ZR seulement ;
            #           vision/servo absents en board-only -> branches gardees no-op).
            if self.gamepad is not None:
                joy = self.executor.latest("/joy")
                self._drive_from_gamepad(joy)
                self._gamepad_buttons(joy)
                if self.motion_on and (now2 - self._cmdvel_t0) >= CMDVEL_PERIOD_S:
                    self.motion.publish()        # ~10 Hz : nourrit le watchdog cmd_vel
                    self._cmdvel_t0 = now2
            if now2 - self._hb_t0 >= 2.0:
                self._hb_t0 = now2
                rpm = snap.get("rpm")
                self.tel.log("heartbeat", connected=self.link.connected,
                             batt=snap.get("battery"), yaw=snap.get("yaw"),
                             ok=snap.get("ok"), bad=snap.get("bad"), rpm=rpm,
                             grove=None if gp is None else gp.connected,
                             grove_ultra=None if gp is None else gp.ultra)
                if rpm is not None:
                    self.tel.set("stm32_rpm_sensor", rpm, cfg=self.mcfg)
                if gp is not None and gp.ir_dist is not None:
                    self.tel.set("grove_irdist_sensor",
                                 {"dist": gp.ir_dist, "adc": gp.ir_adc,
                                  "age": None if gp.ir_age is None else round(gp.ir_age, 2)},
                                 cfg=self.mcfg)
                if self.gamepad is not None:
                    self.tel.set("gamepad_input_actuator",
                                 {"connected": bool(joy and joy.connected),
                                  "name": joy.name if joy else "",
                                  "driving": self._gp_driving,
                                  "speed_min": self.motion.speedMinLevel,
                                  "speed_max": self.motion.speedMaxLevel},
                                 cfg=self.mcfg)
            time.sleep(0.02)

    def _loop(self):
        """Boucle : spin executeur -> lecture bus -> overlay -> heartbeat -> MCP -> clavier."""
        now = time.time()
        self._disp_t0 = now
        self._disp_n = 0
        self.disp_fps = 0.0
        self._hb_t0 = now

        while True:
            # 1) un tour de pipeline (les nodes : set -> process -> get)
            self.executor.spin_once()

            # 2) support d'affichage : image camera si dispo, sinon TOILE NOIRE.
            #    La camera est OPTIONNELLE : l'app tourne et affiche les capteurs
            #    (GrovePi, STM32) meme sans camera, et la reprend a chaud si elle
            #    revient (CameraNode reessaie l'ouverture en tache de fond).
            img = self.executor.latest("/camera/image")
            cam_ok = (self.camera.available and self.camera.ok
                      and img is not None and img.frame is not None)
            frame = img.frame if cam_ok else _blank_frame(self.camera.width, self.camera.height)

            # 3) resultats sur le bus (traitement, metriques, servo, carte, capteurs)
            res = self.executor.latest("/tracking/result")
            met = self.executor.latest("/tracking/metrics") or TrackingMetrics()
            sstate = self.executor.latest("/servo/state") or ServoState(deadzone=self.args.deadzone)
            board = self.executor.latest("/board/telemetry")
            gp = self.executor.latest("/grovepi/telemetry")
            es = self.executor.latest("/wsesp32/telemetry")
            ts = self.executor.latest("/teensy/telemetry")
            joy = self.executor.latest("/joy")
            recog = self.executor.latest("/recognition/result")
            th = self.executor.latest("/tacho/state")

            # 4) cadence d'affichage
            self._disp_n += 1
            now = time.time()
            if now - self._disp_t0 >= 1.0:
                self.disp_fps = self._disp_n / (now - self._disp_t0)
                self._disp_t0 = now
                self._disp_n = 0

            # 5) journal per-seq (nouvelle detection)
            if res is not None and res.seq != self.last_seq:
                self._log_detection(res)
                self.last_seq = res.seq

            # 6) overlay (helpers robot_control reutilises via _ServoView) + snapshot
            faces = res.faces if res is not None else []
            main = res.main if res is not None else None
            nx = res.nx if res is not None else None
            ny = res.ny if res is not None else None
            area_pct = res.area_pct if res is not None else 0.0
            tstate = res.tstate if res is not None else {}
            snap = _board_snap(board)
            pt = _ServoView(sstate)
            # marqueurs centraux (position visage) reutilises de robot_control...
            _draw_image_markers(frame, faces, main, nx, ny, area_pct, pt, tstate)
            # ...puis les 3 cartes materielles v3 (CAM / STM32 / GROVE) + aide clavier.
            # boutons a eclairer = touches pressees dans les 300 ms (flash a la pression)
            flash = {i for i, t in self._btn_flash.items() if now - t < 0.3}
            # boutons a etat COLLANT : F allume tant que le suivi est arme, G rouge
            # tant que le node d'apprentissage tourne (etat lu sur /recognition/train_state).
            accent = {}
            if self.motion_on:
                accent[16] = _C_ON                     # O (index 16) : moteurs armes
            if self.active:
                accent[5] = _C_ON                      # F (index 5) : suivi arme
            if th is not None and th.active:
                # W : vert si les roues sont calibrees, orange tant qu'on cherche
                accent[_BTN_TACHO] = _C_ON if th.calibrated else _C_WARN
            _rmode = getattr(recog, "mode", "off") if recog is not None else "off"
            if _rmode != "off":
                # R : vert si un visage est reconnu, orange sinon (actif mais inconnu)
                known = getattr(recog, "status", "") == "known"
                accent[13] = _C_ON if known else _C_WARN   # R (index 13) : reco active
            if _rmode == "acquisition":
                accent[15] = _C_ON                     # A (index 15) : acquisition active
            tr = self.executor.latest("/recognition/train_state")
            train_running = bool(tr.running) if tr is not None else False
            if train_running:
                accent[14] = _C_BAD                    # G (index 14) : apprentissage en cours
                self._train_done_t = 0.0
            elif tr is not None and tr.summary is not None and self._train_done_t == 0.0:
                self._train_done_t = now               # 1er tick apres la fin du batch
            gp_conn = None
            if self.gamepad is not None:
                gp_conn = bool(joy and joy.connected)  # vert/rouge sur le bouton MANETTE
            _draw_hud_cards(frame, cam_ok, self.camera, self.stm32_link, self.stm32_port,
                            self.args.grovepi_port, snap, pt, self.motion_on,
                            not self.args.no_smooth, self.active, met, len(faces),
                            self.disp_fps, tstate, gp, key_flash=flash, btn_accent=accent,
                            mcfg=self.mcfg, speed=self.motion.speedLevel,
                            cmdvel=(self.motion.lin, self.motion.ang),
                            es=es, esp32_port=self.esp32_port,
                            ts=ts, teensy_port=self.teensy_port, gamepad=gp_conn)
            if _hmi(self.mcfg, "recog_badge"):
                _draw_recog_badge(frame, recog)  # nom/id_pred + score + mode reco
            if _hmi(self.mcfg, "tacho_rpm") and th is not None and th.active:
                # anneaux DANS l'image (reperage visuel) + carte haut-centre, dans la
                # bande libre entre CAM (haut-gauche) et la carte de controle (haut-droit).
                _draw_tacho_overlay(frame, th)
                _tx = 336 + max(0, (frame.shape[1] - 328 - 8 - 336 - 320) // 2)
                _draw_tacho_card(frame, _tx, 8, th)
            # metrique MCP : throttlee a ~5 Hz (state.json n'est de toute facon
            # rafraichi qu'au plus une fois par seconde ; le jsonl, lui, tout garde).
            if th is not None and th.active and now - self._tacho_emit >= 0.2:
                self._tacho_emit = now
                self.tel.set("tacho_rpm_sensor",
                             {"calibrated": bool(th.calibrated),
                              "fps": round(th.fps, 1),
                              "roi": [round(v, 4) for v in th.roi],
                              "recording": round(getattr(th, "recording", 0.0), 1),
                              "dump": getattr(th, "dump", None),
                              "note": th.note,
                              "wheels": [
                                  {"id": w.get("id"),
                                   "rpm": (round(w["rpm"], 2)
                                           if w.get("rpm") is not None else None),
                                   "theta": (round(w["theta"], 1)
                                             if w.get("theta") is not None else None),
                                   "quality": round(w.get("quality") or 0.0, 1),
                                   "ok": bool(w.get("ok")),
                                   "alias": int(w.get("alias") or 0)}
                                  for w in th.wheels]},
                             cfg=self.mcfg)
            # version applicative (coin bas-gauche) : repere de code charge
            _put(frame, 8, frame.shape[0] - 8, "v" + APP_VERSION, _C_TITLE_OFF, 0.4)
            # panneau log d'apprentissage : pendant le batch, puis ~20 s apres
            if (_hmi(self.mcfg, "train_log") and tr is not None
                    and (train_running
                         or (self._train_done_t and now - self._train_done_t < 20.0))):
                _draw_train_log(frame, tr)
            if not self.headless:
                cv2.imshow(self.win, frame)      # fenetre coupee en --headless
            self.tel.snapshot(frame)

            # 7) heartbeat periodique
            self._maybe_heartbeat(snap, tstate, met.det_fps, pt, gp)

            # 8) commandes MCP en file (config suivi + carte) -> thread principal
            self.server.drain()

            # 9) clavier (coupe en --headless)
            key = cv2.waitKeyEx(1) if not self.headless else -1
            if key != -1:
                bi = _help_btn_for_key(key)          # eclaire le bouton d'aide pressé
                if bi is not None:
                    self._btn_flash[bi] = now
                quit_now, _moved = self._process_key(key, now)
                if quit_now:
                    break
            if self.headless:
                time.sleep(0.005)                # sans waitKey : evite la boucle folle
            elif not cam_ok:
                time.sleep(0.02)                 # sans camera : cap ~50 fps (evite 100% CPU)

            # 10) tenue des fleches : sonde l'etat physique -> etat cmd_vel (presser=bouger,
            #     relacher=stop) ; republication immediate au changement (start/stop francs).
            self._drive_from_arrows()

            # 10b) manette : le stick gauche prend la main quand il est pousse (sinon rend
            #      la main au clavier) ; boutons/croix/gachettes = actions a front montant.
            if self.gamepad is not None:
                self._drive_from_gamepad(joy)
                self._gamepad_buttons(joy)

            # 11) republieur cmd_vel ~10 Hz : entretient l'etat de mouvement (modele ROS)
            #     et nourrit le watchdog cmd_vel firmware. A l'arret (etat 0) on emet
            #     quand meme cmd_vel(0,0) tant que les moteurs sont armes (Motion_Stop).
            if self.motion_on and (now - self._cmdvel_t0) >= CMDVEL_PERIOD_S:
                self.motion.publish()
                self._cmdvel_t0 = now

    def _maybe_heartbeat(self, snap, tstate, det_fps, pt, gp=None):
        """Battement telemetrie toutes les 2 s (perf, servo, carte, suivi, capteurs).

        Le record `heartbeat` (vue de synthese historique) est conserve tel quel pour
        le MCP `status`. Les metriques NOUVELLES du framework (rpm moteurs, telemetre
        IR) sont en plus emises comme metriques nommees/typees gatees par groupe."""
        now2 = time.time()
        if now2 - self._hb_t0 < 2.0:
            return
        self._hb_t0 = now2
        mc = self.mcfg
        hb_score = tstate.get("score")
        ms = pt.motionStats()
        rpm = snap.get("rpm")
        self.tel.log("heartbeat", disp_fps=round(self.disp_fps, 1),
                     det_fps=round(det_fps, 1), tracking=self.active,
                     pan=round(pt.angleH, 1), tilt=round(pt.angleV, 1),
                     connected=self.link.connected, batt=snap.get("battery"),
                     yaw=snap.get("yaw"), ok=snap.get("ok"), bad=snap.get("bad"),
                     trk=tstate.get("mode"), locked=tstate.get("locked"),
                     src=tstate.get("src"),
                     lock_id=tstate.get("lock_id"),
                     lock_age=round(tstate.get("lock_age", 0.0), 2),
                     score=None if hb_score is None else round(hb_score, 3),
                     step_max=round(ms.get("step_max", 0.0), 2) if ms else 0.0,
                     rpm=rpm,
                     grove=None if gp is None else gp.connected,
                     grove_ultra=None if gp is None else gp.ultra,
                     grove_roll=None if gp is None else gp.roll,
                     grove_pitch=None if gp is None else gp.pitch)
        # --- metriques nommees/typees (framework) : gatees par groupe (mcp/log) ---
        if rpm is not None:
            self.tel.set("stm32_rpm_sensor", rpm, cfg=mc)
        if gp is not None and gp.ir_dist is not None:
            self.tel.set("grove_irdist_sensor",
                         {"dist": gp.ir_dist, "adc": gp.ir_adc,
                          "age": None if gp.ir_age is None else round(gp.ir_age, 2)},
                         cfg=mc)
        if self.gamepad is not None:
            joy = self.executor.latest("/joy")
            self.tel.set("gamepad_input_actuator",
                         {"connected": bool(joy and joy.connected),
                          "name": joy.name if joy else "",
                          "driving": self._gp_driving,
                          "speed_min": self.motion.speedMinLevel,
                          "speed_max": self.motion.speedMaxLevel},
                         cfg=mc)

    def _shutdown(self):
        """Arret propre : serveur MCP, moteurs coupes, nodes/camera, journaux."""
        if self.server is not None:
            self.server.stop()
        try:
            # Arret fiable meme en pleine boucle fermee : cmd_vel(0,0) -> Motion_Stop
            # (le PWM brut ne remet pas g_start_ctrl=0 et serait ecrase par le PID).
            if self.link is not None and self.motion is not None:
                self.motion.stop()
        except Exception:
            pass
        if self.executor is not None:
            self.executor.stop()                 # camera.release() + webcam.stop()
        if not self.headless:
            cv2.destroyAllWindows()
        if self.link is not None:
            self.link.close()
        if self.tel is not None:
            self.tel.close()


def _board_snap(board):
    """BoardTelemetry (topic) -> dict attendu par les helpers d'overlay/heartbeat."""
    if board is None:
        return {"battery": None, "yaw": None, "roll": None, "pitch": None,
                "rpm": None, "ok": 0, "bad": 0}
    return {"battery": board.battery, "yaw": board.yaw, "roll": board.roll,
            "pitch": board.pitch, "rpm": board.rpm, "ok": board.ok, "bad": board.bad}


def main():
    RobotControlCore(parse_args()).run()


if __name__ == "__main__":
    main()

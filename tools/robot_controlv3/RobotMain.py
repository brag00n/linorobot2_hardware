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

Clavier : identique a robot_control (Z/S/Q/D moteurs, fleches pan/tilt, C centre,
F suivi, M detecteur, T tracker, P prediction, 0-9 vitesse, Espace STOP, Echap).
"""
import os
import time

import cv2

# --- reutilisation par import : CLI, overlay et constantes de robot_control -----
# (memes flags, meme incrustation, memes codes clavier -> comportement identique)
from robot_control.RobotMain import (
    parse_args, draw_overlay,
    MOVE_WATCHDOG_S, KEYS_LEFT, KEYS_UP, KEYS_RIGHT, KEYS_DOWN, SERVO_STEP,
)
from robot_control.lib.Telemetry import Telemetry
from robot_control.communication.RobotComSerial import RobotComSerial
from robot_control.device.motion.RobotMotorDrive import RobotMotorDrive
from robot_control.modules.tracking.RobotWebCamMotorized import PREDICT_MODES
from robot_control.mcp import gateway

from .roslite import Executor
from .nodes import CameraNode, TrackingNode, ServoNode, BoardNode
from .msgs import TrackingConfig, ServoCmd, TrackingResult, TrackingMetrics, ServoState


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
        self.server = None                       # serveur de commandes MCP (gateway)

        # --- etat de boucle cote Core (HMI/clavier/MCP), comme l'ancien app -----
        self.active = False          # suivi arme (touche F) -> publie sur /tracking/config
        self.last_seq = -1           # derniere detection loguee (event detect)
        self.last_locked = None      # dernier etat de verrou (transition lock/unlock)
        self.moving = False          # une commande moteur est en cours
        self.last_move_ts = 0.0
        self._servo_seq = 0          # sequence des ServoCmd clavier (messages discrets)

        self._disp_t0 = 0.0
        self._disp_n = 0
        self.disp_fps = 0.0
        self._hb_t0 = 0.0

    # -----------------------------------------------------------------------
    # Mise en place : telemetrie, liaison serie, nodes, executeur
    # -----------------------------------------------------------------------
    def setup(self):
        args = self.args
        # 1) telemetrie (journal structure ; budget = fichier vif + 1 backup)
        self.tel = Telemetry(log_dir=args.log_dir, enabled=not args.no_telemetry,
                             max_bytes=int(max(1.0, args.log_budget_mb) * 1_000_000 / 2))
        self.tel.log("event", msg="start", app="v3", motion=self.motion_on, port=args.port,
                     index=str(args.index), backend=args.backend, size=args.size,
                     pan_gain=args.pan_gain, tilt_gain=args.tilt_gain,
                     deadzone=args.deadzone, dead_hyst=args.dead_hyst, max_step=args.max_step,
                     invert_pan=args.invert_pan, invert_tilt=args.invert_tilt)

        # 2) liaison serie STM32 (partagee : BoardNode + ServoNode + moteurs Core)
        self.link = RobotComSerial(args.port, args.baud, telemetry=self.tel)
        time.sleep(0.3)                          # laisse le thread lecteur s'ouvrir
        self.motion = RobotMotorDrive(self.link, maxPwm=args.max_pwm)

        # 3) nodes + executeur : en --board-only, seul BoardNode (ni cam ni suivi)
        self.board = BoardNode(self.link)
        self.executor = Executor()
        if self.board_only:
            self.executor.add_node(self.board)
        else:
            self.camera = CameraNode(args, telemetry=self.tel)
            self.tracking = TrackingNode(args, telemetry=self.tel)
            self.servo = ServoNode(args, self.link, telemetry=self.tel)
            for node in (self.camera, self.tracking, self.servo, self.board):
                self.executor.add_node(node)
        self.executor.start()                    # ouvre camera + verifie detecteur + thread P2

        # 4) config initiale du suivi + serveur de commandes MCP (socket loopback)
        if not self.board_only:
            self.executor.publish("/tracking/config", TrackingConfig(active=self.active))
        host, port = gateway.parse_addr(os.environ.get("BAMBOU_MCP_PORT"))
        self.server = gateway.CommandServer(
            on_config=self._apply_config, link=self.link, motion=self.motion,
            cpr=self.link.cpr or 1320.0, host=host, port=port).start()
        print(f"Carte : {args.port} @ {args.baud}  |  "
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

    # -----------------------------------------------------------------------
    # Journalisation par NOUVELLE detection (event detect + transition verrou)
    # -----------------------------------------------------------------------
    def _log_detection(self, res: TrackingResult):
        """Reprend le journal per-seq de l'ancien _on_new_detection (analyse/MCP)."""
        tstate = res.tstate
        score = tstate.get("score")
        pspeed = tstate.get("pred_speed")
        perr = tstate.get("pred_err")
        self.tel.log("detect", seq=res.seq, faces=len(res.faces),
                     nx=None if res.nx is None else round(res.nx, 4),
                     ny=None if res.ny is None else round(res.ny, 4),
                     area=round(res.area_pct, 2), det_fps=round(res.det_fps, 1),
                     tracking=self.active, trk=tstate.get("mode"),
                     locked=tstate.get("locked"), src=tstate.get("src"),
                     raw_det=tstate.get("raw_det"),
                     score=None if score is None else round(score, 3),
                     predict=tstate.get("predict"),
                     pred_speed=None if pspeed is None else round(pspeed, 3),
                     pred_err=None if perr is None else round(perr, 4))
        if tstate.get("mode") != "none" and tstate.get("locked") != self.last_locked:
            unlocked = not tstate.get("locked")
            self.tel.log("event", msg="track_unlock" if unlocked else "track_lock",
                         src=tstate.get("src"),
                         reason=tstate.get("unlock_reason") if unlocked else None,
                         score=None if score is None else round(score, 3))
            self.last_locked = tstate.get("locked")

    # -----------------------------------------------------------------------
    # Commande de config MCP (socket, via gateway) -> /tracking/config
    # -----------------------------------------------------------------------
    def _apply_config(self, cfg):
        """Applique une commande de config MCP recue par socket (gateway.drain).

        cfg (gateway.config_from) porte UNE cle : detector / track_mode /
        predict_mode / active -> publiee sur /tracking/config (appliquee au spin
        suivant). Renvoie un compte-rendu texte pour le client MCP.
        """
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
        return "Commande de config vide."

    # -----------------------------------------------------------------------
    # Clavier (identique a robot_control ; nudges/center via /servo/cmd)
    # -----------------------------------------------------------------------
    def _process_key(self, key, now):
        """Traite une touche. Retourne (quit, moved_now)."""
        k = key & 0xFF
        c = chr(k).lower() if 32 <= k < 127 else ""
        self.tel.log("key", code=key, k=k, c=c)

        if key == 27:                            # Echap
            self.tel.log("event", msg="quit")
            return True, False

        if key in KEYS_LEFT:
            self._publish_servo("nudge_pan", -SERVO_STEP)
        elif key in KEYS_RIGHT:
            self._publish_servo("nudge_pan", +SERVO_STEP)
        elif key in KEYS_UP:
            self._publish_servo("nudge_tilt", +SERVO_STEP)
        elif key in KEYS_DOWN:
            self._publish_servo("nudge_tilt", -SERVO_STEP)
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
        elif c == "j":
            self._publish_servo("nudge_pan", -SERVO_STEP)
        elif c == "l":
            self._publish_servo("nudge_pan", +SERVO_STEP)
        elif c == "i":
            self._publish_servo("nudge_tilt", +SERVO_STEP)
        elif c == "k":
            self._publish_servo("nudge_tilt", -SERVO_STEP)
        elif c.isdigit():
            self.motion.setSpeed(int(c))
        elif k == 32:                            # Espace : STOP moteurs
            if self.motion_on:
                self.motion.stop()
            self.moving = False
        elif c in ("z", "s", "q", "d"):          # deplacement (momentane + watchdog)
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
            self.executor.spin_once()            # BoardNode : lit la telemetrie carte
            self.server.drain()                  # execute les commandes carte MCP
            snap = _board_snap(self.executor.latest("/board/telemetry"))
            now2 = time.time()
            if now2 - self._hb_t0 >= 2.0:
                self._hb_t0 = now2
                self.tel.log("heartbeat", connected=self.link.connected,
                             batt=snap.get("battery"), yaw=snap.get("yaw"),
                             ok=snap.get("ok"), bad=snap.get("bad"))
            time.sleep(0.02)

    def _loop(self):
        """Boucle : spin executeur -> lecture bus -> overlay -> heartbeat -> MCP -> clavier."""
        now = time.time()
        self._disp_t0 = now
        self._disp_n = 0
        self.disp_fps = 0.0
        self._hb_t0 = now

        while True:
            # 1) un tour de pipeline (les 4 nodes : set -> process -> get)
            self.executor.spin_once()

            # 2) garde d'echec camera (sur la DERNIERE lecture), comme l'ancien :
            #    lecture ratee -> on saute ce tour ; 30 echecs consecutifs -> arret.
            if not self.camera.ok:
                if self.camera.read_fail > 30:
                    print("Lecture camera echouee de facon repetee, arret.")
                    break
                time.sleep(0.005)
                continue
            img = self.executor.latest("/camera/image")
            if img is None or img.frame is None:
                time.sleep(0.005)
                continue
            frame = img.frame

            # 3) resultats sur le bus (traitement, metriques, servo, carte)
            res = self.executor.latest("/tracking/result")
            met = self.executor.latest("/tracking/metrics") or TrackingMetrics()
            sstate = self.executor.latest("/servo/state") or ServoState(deadzone=self.args.deadzone)
            board = self.executor.latest("/board/telemetry")

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
            draw_overlay(frame, faces, main, nx, ny, area_pct, met.det_fps, self.disp_fps,
                         self.active, pt, snap, self.motion_on,
                         detector=met.detector, tstate=tstate)
            if not self.headless:
                cv2.imshow(self.win, frame)      # fenetre coupee en --headless
            self.tel.snapshot(frame)

            # 7) heartbeat periodique
            self._maybe_heartbeat(snap, tstate, met.det_fps, pt)

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

    def _maybe_heartbeat(self, snap, tstate, det_fps, pt):
        """Battement telemetrie toutes les 2 s (perf, servo, carte, suivi)."""
        now2 = time.time()
        if now2 - self._hb_t0 < 2.0:
            return
        self._hb_t0 = now2
        hb_score = tstate.get("score")
        ms = pt.motionStats()
        self.tel.log("heartbeat", disp_fps=round(self.disp_fps, 1),
                     det_fps=round(det_fps, 1), tracking=self.active,
                     pan=round(pt.angleH, 1), tilt=round(pt.angleV, 1),
                     connected=self.link.connected, batt=snap.get("battery"),
                     yaw=snap.get("yaw"), ok=snap.get("ok"), bad=snap.get("bad"),
                     trk=tstate.get("mode"), locked=tstate.get("locked"),
                     src=tstate.get("src"),
                     score=None if hb_score is None else round(hb_score, 3),
                     step_max=round(ms.get("step_max", 0.0), 2) if ms else 0.0)

    def _shutdown(self):
        """Arret propre : serveur MCP, moteurs coupes, nodes/camera, journaux."""
        if self.server is not None:
            self.server.stop()
        try:
            if self.motion_on and self.link is not None:
                self.link.stop()
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
        return {"battery": None, "yaw": None, "ok": 0, "bad": 0}
    return {"battery": board.battery, "yaw": board.yaw, "ok": board.ok, "bad": board.bad}


def main():
    RobotControlCore(parse_args()).run()


if __name__ == "__main__":
    main()

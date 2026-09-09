r"""TrackingNode - node de traitement : suivi de visage (detection + prediction Kalman).

Enveloppe le subsystem modules.tracking.RobotWebCamMotorized de robot_control
(reutilise PAR IMPORT : detecteurs haar/dnn/yunet, tracker visuel, machine
coast/anticipation Kalman -- ALGORITHMES STRICTEMENT INCHANGES ; on ne recopie
rien, on consomme la classe telle quelle, thread de detection P2 compris).

Decouplage servo fidele ROS2 : au lieu de laisser le subsystem piloter le servo,
on lui injecte un SERVO-PROXY qui capte ses appels track()/returnHome() (emis par
moveToTrackedArea) et les transforme en messages ServoCmd sur /servo/cmd. Le corps
d'algorithme du subsystem n'est pas touche : c'est exactement la frontiere que ROS2
materialisera par une file/topic.

Entrees : /camera/image, /tracking/config.
Sorties : /tracking/result (par NOUVELLE detection), /tracking/metrics (chaque tour),
          /servo/cmd (cible d'asservissement, par NOUVELLE detection si suivi actif).

Sous ROS2 : subscribers image/config + publishers result/metrics/servo_cmd ; le
subsystem resterait un composant interne du node (thread P2 -> callback ou executor
multi-thread).
"""
from robot_control.modules.tracking.RobotWebCamMotorized import RobotWebCamMotorized

from ..roslite import Node
from ..msgs import ImageMsg, TrackingConfig, TrackingResult, TrackingMetrics, ServoCmd


class _ServoCmdProxy:
    """Faux servo : enregistre l'INTENTION d'asservissement au lieu de bouger.

    Expose l'interface minimale utilisee par RobotWebCamMotorized.moveToTrackedArea
    (track / returnHome). `pending` retient le dernier ordre pour que le node le
    publie sur /servo/cmd (puis le remet a None).
    """

    def __init__(self):
        self.pending = None            # ("track", nx, ny) | ("home", None, None) | None

    def track(self, nx, ny):
        self.pending = ("track", nx, ny)

    def returnHome(self):
        self.pending = ("home", None, None)


class TrackingNode(Node):
    """Detecte + predit ; publie resultat/metriques et la cible servo (sans bouger le servo)."""

    def __init__(self, args, telemetry=None):
        super().__init__("tracking")
        self._args = args
        self._tel = telemetry
        self._proxy = _ServoCmdProxy()
        self.webcam = RobotWebCamMotorized(
            self._proxy, det_width=args.det_width, min_size=args.min_size,
            detector=args.detector, conf=args.det_conf, yunet_model=args.yunet_model,
            track_mode=args.track_mode, vit_model=args.vit_model,
            redetect_ms=args.redetect_ms, score_min=args.track_score_min,
            hold_ms=args.track_hold_ms, max_area_frac=args.track_max_area,
            max_grow=args.track_max_grow, predict_mode=args.predict_mode,
            predict_ms=args.predict_ms, predict_lead_ms=args.predict_lead_ms,
            predict_min_speed=args.predict_min_speed)
        self.active = False            # suivi arme (pilote par /tracking/config)
        self._last_seq = -1            # derniere detection publiee (asservie)
        self._cmd_seq = 0              # sequence des ServoCmd emis (info/debug)

        self._img_in = self.create_input("/camera/image", ImageMsg)
        self._cfg_in = self.create_input("/tracking/config", TrackingConfig)
        self._result_out = self.create_output("/tracking/result", TrackingResult)
        self._metrics_out = self.create_output("/tracking/metrics", TrackingMetrics)
        self._servo_out = self.create_output("/servo/cmd", ServoCmd)

    def on_start(self):
        """Verifie detecteur/tracker (fatal si detecteur KO) et lance le thread P2."""
        ok, msg = self.webcam.ready()
        if not ok:
            raise RuntimeError(f"Detecteur '{self._args.detector}' indisponible : {msg}")
        avail = self.webcam.availableDetectors()
        print(f"Detecteur : {self._args.detector}  (disponibles : {', '.join(avail)})")
        tok, tmsg = self.webcam.trackReady()
        if not tok:
            print(f"[tracking] tracker '{self.webcam.trackMode}' indisponible ({tmsg}) -> mode none")
            self.webcam.setTrackMode("none")
        tavail = self.webcam.availableTrackers()
        print(f"Suivi : {self.webcam.trackMode}  (trackers disponibles : {', '.join(tavail)})")
        self.webcam.start()

    def process(self):
        """Applique la config, pousse l'image, lit le resultat, publie result/metrics/cmd."""
        # 1) config (touches F/M/T/P, MCP) : arme le suivi + bascules a chaud
        self._apply_config(self._cfg_in.get())

        # 2) fournir la derniere image au thread de detection (P2)
        img = self._img_in.get()
        if img is not None and img.frame is not None:
            self.webcam.publish(img.frame)

        # 3) dernier resultat de detection + etat de suivi (asynchrone P2)
        faces, main, nx, ny, area_pct, det_fps, seq = self.webcam.latest()
        tstate = self.webcam.trackState()

        # 4) metriques HUD (a chaque tour)
        self._metrics_out.set(TrackingMetrics(
            det_fps=det_fps, detector=self.webcam.detector,
            track_mode=self.webcam.trackMode, predict_mode=self.webcam.predict_mode,
            active=self.active, n_faces=len(faces)))

        # 5) par NOUVELLE detection seulement (une correction servo par seq -> anti-pompage)
        if seq != self._last_seq:
            self._last_seq = seq
            self._result_out.set(TrackingResult(
                seq=seq, faces=faces, main=main, nx=nx, ny=ny,
                area_pct=area_pct, det_fps=det_fps, tstate=tstate))
            # asservissement : delegue au subsystem (cible = mesure/anticip/coast, ou
            # retour maison si coast expire). Le proxy capte l'intention -> ServoCmd.
            self._proxy.pending = None
            self.webcam.moveToTrackedArea(self.active)
            self._emit_servo_cmd(tstate.get("predict", "off"))

    def _emit_servo_cmd(self, phase):
        """Traduit l'intention captee par le proxy en message ServoCmd sur /servo/cmd."""
        p = self._proxy.pending
        if p is None:
            return
        kind, nx, ny = p
        self._cmd_seq += 1
        self._servo_out.set(ServoCmd(kind=kind, seq=self._cmd_seq, nx=nx, ny=ny, phase=phase))

    def _apply_config(self, cfg):
        """Arme le suivi et applique les bascules detecteur/tracker/prediction sur changement."""
        if cfg is None:
            return
        self.active = cfg.active
        if cfg.detector and cfg.detector != self.webcam.detector:
            ok, msg = self.webcam.setDetector(cfg.detector)
            self._log("detector_switch", to=cfg.detector, ok=ok, info=msg)
        if cfg.track_mode and cfg.track_mode != self.webcam.trackMode:
            ok, msg = self.webcam.setTrackMode(cfg.track_mode)
            self._log("track_mode", to=cfg.track_mode, ok=ok, info=msg)
        if cfg.predict_mode and cfg.predict_mode != self.webcam.predict_mode:
            ok, res = self.webcam.setPredictMode(cfg.predict_mode)
            self._log("predict_mode", to=res, ok=ok)

    def _log(self, msg, **fields):
        if self._tel is not None:
            self._tel.log("event", msg=msg, source="config", **fields)

    def on_stop(self):
        self.webcam.stop()

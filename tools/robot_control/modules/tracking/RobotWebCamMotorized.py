r"""RobotWebCamMotorized - Subsystem : suivi de visage par camera motorisee.

Equivalent du subsystem/RobotWebCamMotorized.py de Bambou4WD_python (2018) :
COMPOSE les couches transverses pour realiser une fonction robot complete.
Reprend l'orchestration de l'ancien vision.FaceTracker (thread de detection P2,
prediction de trajectoire, publication de l'etat) et y ajoute l'asservissement
du servo (moveToTrackedArea), qui etait dans main.py.

Pipeline (comportement INCHANGE) :
  P1 (capture + affichage + clavier) : chez l'orchestrateur RobotMain, sur le
      thread principal (la GUI OpenCV doit y rester). RobotMain appelle publish()
      a chaque image, puis moveToTrackedArea() une fois par NOUVELLE detection.
  P2 (detection + prediction) : le thread interne ici recupere la derniere image
      publiee, la reduit (largeur det_width), delegue la perception a
      interaction.FaceDetection, remappe en pleine resolution, calcule la
      position normalisee (nx, ny) du visage principal, puis la lisse/anticipe/
      extrapole via lib.KalmanPredictor (machine coast/anticipation).
"""
import threading
import time

import cv2

from ...interaction.FaceDetection import FaceDetection
from ...lib.KalmanPredictor import KalmanPredictor

# modes de prediction, dans l'ordre de bascule de la touche P
PREDICT_MODES = ("off", "anticip", "coast")
PREDICT_LABELS = {"off": "off", "anticip": "prediction",
                  "coast": "prediction si perte"}


def _clip(v, lim):
    """Borne v dans [-lim, +lim]."""
    return lim if v > lim else (-lim if v < -lim else v)


class RobotWebCamMotorized:
    """Suivi de visage asynchrone. Publie la derniere position connue et asservit
    le servo pan/tilt vers la cible (mesuree, anticipee ou extrapolee)."""

    def __init__(self, servo, det_width=320, min_size=24,
                 scale_factor=1.1, min_neighbors=5, cascade_path=None,
                 detector="haar", conf=0.5,
                 dnn_proto=None, dnn_model=None, yunet_model=None,
                 track_mode="auto", vit_model=None,
                 redetect_ms=400, score_min=0.30, hold_ms=3000,
                 iou_reanchor=0.20, max_area_frac=0.5, max_grow=3.0,
                 predict_mode="anticip", predict_ms=700,
                 predict_lead_ms=120, predict_min_speed=0.4):
        self._servo = servo               # RobotServoMotor asservi (pan/tilt)
        self.det_width = det_width

        # couche interaction : perception (detecteur commutable + tracker visuel)
        self._det = FaceDetection(
            det_width=det_width, min_size=min_size, scale_factor=scale_factor,
            min_neighbors=min_neighbors, cascade_path=cascade_path,
            detector=detector, conf=conf, dnn_proto=dnn_proto,
            dnn_model=dnn_model, yunet_model=yunet_model,
            track_mode=track_mode, vit_model=vit_model,
            redetect_ms=redetect_ms, score_min=score_min, hold_ms=hold_ms,
            iou_reanchor=iou_reanchor, max_area_frac=max_area_frac,
            max_grow=max_grow)

        # --- prediction de trajectoire (coast + anticipation) -----------------
        self.predict_mode = (predict_mode if predict_mode in PREDICT_MODES
                             else "anticip")
        self.predict_ms = predict_ms              # duree max de coast avant retour maison
        self.predict_lead_ms = predict_lead_ms    # avance temporelle de l'anticipation
        self.predict_min_speed = predict_min_speed  # vitesse mini pour declencher un coast
        self._predictor = KalmanPredictor()
        self._predict_phase = "off"               # off|lock|coast|home
        self._pred_speed = 0.0                    # |vitesse| normalisee/s
        self._pred_nx = None                      # point predit (overlay), None si aucun
        self._pred_ny = None
        self._lost_t = None                       # t du debut de perte (fenetre de coast)
        self._last_pred_t = None                  # t du dernier pas de prediction (dt)
        self._pending_predict = None              # bascule mode predict demandee a chaud
        self._pred_err = None                     # erreur de prediction (innovation, coords norm.)

        self._lock = threading.Lock()
        self._frame = None            # derniere image pleine res (deja flippee)
        self._frame_id = 0

        # Resultats (coords pleine res)
        self._faces = []              # [(x,y,w,h), ...] remappees
        self._main = None             # (x,y,w,h) plus grand visage
        self._nx = None               # position normalisee [-1,+1] (droite +)
        self._ny = None               # position normalisee [-1,+1] (bas +)
        self._area_pct = 0.0
        self._det_fps = 0.0
        self._seq = 0                 # incremente a chaque detection (P2)
        self._raw_box_full = None     # box detecteur brute (pleine res) pendant lock

        self._stop = False
        self._worker = None

    # --- delegations couche interaction (detecteur) -------------------------
    @property
    def detector(self):
        return self._det.detector

    def ready(self, det=None):
        return self._det.ready(det)

    def availableDetectors(self):
        return self._det.availableDetectors()

    def setDetector(self, name):
        return self._det.setDetector(name)

    # --- delegations couche interaction (tracker visuel) --------------------
    @property
    def trackMode(self):
        return self._det.trackMode

    def trackReady(self, mode=None):
        return self._det.trackReady(mode)

    def availableTrackers(self):
        return self._det.availableTrackers()

    def setTrackMode(self, name):
        return self._det.setTrackMode(name)

    # --- prediction de trajectoire : bascule a chaud du mode ----------------
    def setPredictMode(self, name):
        """Demande une bascule du mode de prediction ; appliquee par le worker."""
        if name not in PREDICT_MODES:
            return False, f"mode predict inconnu : {name}"
        self._pending_predict = name
        return True, name

    def cyclePredictMode(self):
        """Bascule au mode de prediction suivant (off -> anticip -> coast -> ...)."""
        i = PREDICT_MODES.index(self.predict_mode) if \
            self.predict_mode in PREDICT_MODES else 0
        nxt = PREDICT_MODES[(i + 1) % len(PREDICT_MODES)]
        return self.setPredictMode(nxt)

    @property
    def predictLabel(self):
        return PREDICT_LABELS.get(self.predict_mode, self.predict_mode)

    # --- etat / echanges avec P1 --------------------------------------------
    def trackState(self):
        """Etat courant du suivi : perception (interaction) + prediction (ici).

        Fusionne FaceDetection.state() (mode, locked, src, score, raw_det,
        unlock_reason) avec la box detecteur brute REMAPPEE pleine res et les
        champs de prediction (phase, mode, vitesse, erreur, point predit).
        """
        st = self._det.state()
        with self._lock:
            raw_box = self._raw_box_full
            pred_nx, pred_ny = self._pred_nx, self._pred_ny
        return {"mode": st["mode"], "locked": st["locked"], "src": st["src"],
                "score": st["score"], "raw_det": st["raw_det"],
                "raw_box": raw_box, "unlock_reason": st["unlock_reason"],
                "predict": self._predict_phase, "predict_mode": self.predict_mode,
                "pred_speed": self._pred_speed, "pred_err": self._pred_err,
                "pred_nx": pred_nx, "pred_ny": pred_ny}

    def start(self):
        ok, msg = self._det.ready()
        if not ok:
            raise RuntimeError(msg)
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()
        return self

    def publish(self, frame):
        """Fournit la derniere image capturee (P1 -> P2)."""
        with self._lock:
            self._frame = frame
            self._frame_id += 1

    def latest(self):
        """Retourne (faces, main_face, nx, ny, area_pct, det_fps, seq)."""
        with self._lock:
            return (list(self._faces), self._main, self._nx, self._ny,
                    self._area_pct, self._det_fps, self._seq)

    def moveToTrackedArea(self, tracking):
        """Asservit le servo vers la cible courante (a appeler par NOUVELLE detection).

        Reprend la logique per-seq de l'ancien main.py : si le suivi est actif et
        qu'une cible existe (nx non nul), on vise (mesure, anticipation ou coast) ;
        si la prediction abandonne (phase 'home', coast expire), retour doux au
        repos. Sinon, rien (le lisseur continue son mouvement en cours cote servo).
        """
        with self._lock:
            nx, ny = self._nx, self._ny
            phase = self._predict_phase
        if tracking and nx is not None:
            self._servo.track(nx, ny)          # cible = mesure, anticipation ou coast
        elif tracking and phase == "home":
            self._servo.returnHome()           # coast expire : retour doux au centre

    def stop(self):
        self._stop = True
        if self._worker is not None:
            self._worker.join(timeout=1.0)

    # --- prediction de trajectoire : machine coast/anticipation -------------
    def _predictStep(self, mnx, mny, now, det_fps):
        """Machine coast/anticipation. mnx/mny = position MESUREE (None si perdue).

        Retourne (out_nx, out_ny, pred_nx, pred_ny, phase, speed) :
          out_*  : cible envoyee au servo (mesuree, anticipee, ou extrapolee) ;
          pred_* : point PREDIT a afficher a l'ecran (None si aucun) ;
          phase  : off|lock|coast|home ; speed : |vitesse| normalisee/s.
        """
        # dt reel entre deux pas P2 (borne dans le filtre)
        if self._last_pred_t is None:
            dt = 1.0 / det_fps if det_fps > 1 else 0.04
        else:
            dt = now - self._last_pred_t
        self._last_pred_t = now

        if self.predict_mode == "off":
            self._lost_t = None
            self._pred_err = None
            return mnx, mny, None, None, "off", 0.0

        lead = self.predict_lead_ms / 1000.0

        if mnx is not None:
            # visage visible : (re)cale le filtre. Redemarrage propre apres un
            # abandon (home) ou une toute premiere acquisition.
            if (not self._predictor.inited) or self._predict_phase == "home":
                self._predictor.reset(mnx, mny)
                px, py, vx, vy = mnx, mny, 0.0, 0.0
                self._pred_err = None            # pas de prediction anterieure a comparer
            else:
                px, py, vx, vy = self._predictor.update(mnx, mny, dt)
                # erreur = ecart prediction anterieure <-> mesure actuelle (innovation)
                self._pred_err = self._predictor.lastErr
            self._lost_t = None
            speed = (vx * vx + vy * vy) ** 0.5
            # point predit (avance temporelle) : affiche toujours, sert au servo en anticip.
            # Base sur la MESURE brute (pas la position lissee, qui retarde sur rampe) +
            # la vitesse estimee -> le point predit devance reellement la mesure.
            lnx = _clip(mnx + vx * lead, 1.5)
            lny = _clip(mny + vy * lead, 1.5)
            if self.predict_mode == "anticip":
                return lnx, lny, lnx, lny, "lock", speed
            # mode coast : servo sur la mesure brute, overlay montre l'anticipation
            return mnx, mny, lnx, lny, "lock", speed

        # visage perdu (on garde _pred_err : derniere precision mesuree connue)
        if self._lost_t is None:
            self._lost_t = now
        speed = 0.0
        if self._predictor.inited:
            _, _, vx, vy = self._predictor.peek()
            speed = (vx * vx + vy * vy) ** 0.5
        within = (now - self._lost_t) * 1000.0 <= self.predict_ms
        if self._predictor.inited and speed >= self.predict_min_speed and within:
            px, py, vx, vy = self._predictor.coast(dt)
            speed = (vx * vx + vy * vy) ** 0.5
            cx, cy = _clip(px, 1.5), _clip(py, 1.5)
            return cx, cy, cx, cy, "coast", speed
        # coast fini / visage immobile a la perte : retour maison, plus de cible
        self._pred_err = None
        return None, None, None, None, "home", speed

    # --- thread de detection P2 ---------------------------------------------
    def _loop(self):
        try:
            self._det.buildImpl()
        except Exception as e:
            print(f"[vision] init detecteur '{self._det.detector}' echoue : {e}")
            return
        print(f"[vision] detecteur actif : {self._det.detector} | "
              f"tracker : {self._det.trackMode}")
        t0 = time.time()
        n = 0
        last_id = -1
        while not self._stop:
            # bascule mode de prediction a chaud demandee ? (les bascules
            # detecteur/tracker sont appliquees par FaceDetection.process)
            if self._pending_predict is not None and self._pending_predict != self.predict_mode:
                self.predict_mode = self._pending_predict
                self._pending_predict = None
                self._predictor.resetUninit()   # oublie la vitesse du mode precedent
                self._predict_phase = "off"
                self._lost_t = None
                self._pred_err = None
                print(f"[vision] mode prediction -> {self.predict_mode} "
                      f"({PREDICT_LABELS.get(self.predict_mode)})")

            with self._lock:
                frame = self._frame
                fid = self._frame_id
            if frame is None or fid == last_id:
                time.sleep(0.002)
                continue
            last_id = fid

            fh, fw = frame.shape[:2]
            scale = fw / float(self.det_width)         # remapping P2 -> P1
            det_h = int(round(fh / scale))
            small = cv2.resize(frame, (self.det_width, det_h),
                               interpolation=cv2.INTER_AREA)
            sh, sw = small.shape[:2]

            # perception (couche interaction) : detecteur seul ou detect-then-track
            main_s, faces_s = self._det.process(small, sw, sh)

            # box detecteur brute (pendant lock) remappee pleine res -> indicateur overlay
            raw_full = None
            raw_small = self._det.state()["raw_box"]
            if raw_small is not None:
                rx, ry, rw, rh = raw_small
                raw_full = (int(rx * scale), int(ry * scale),
                            int(rw * scale), int(rh * scale))

            remapped = [(int(x * scale), int(y * scale),
                         int(w * scale), int(h * scale)) for (x, y, w, h) in faces_s]
            main = None
            if main_s is not None:
                x, y, w, h = main_s
                main = (int(x * scale), int(y * scale), int(w * scale), int(h * scale))

            # position normalisee MESUREE du visage principal (None si perdu)
            mnx = mny = None
            area_pct = 0.0
            if main is not None:
                x, y, w, h = main
                cx, cy = x + w / 2.0, y + h / 2.0
                mnx = (cx - fw / 2.0) / (fw / 2.0)
                mny = (cy - fh / 2.0) / (fh / 2.0)
                area_pct = 100.0 * (w * h) / (fw * fh)

            n += 1
            now = time.time()
            det_fps = self._det_fps
            if now - t0 >= 1.0:
                det_fps = n / (now - t0)
                t0 = now
                n = 0

            # prediction de trajectoire : la cible servo (nx,ny) peut etre anticipee
            # (visage visible) ou extrapolee (coast, visage perdu). pred_* = point
            # a afficher a l'ecran.
            out_nx, out_ny, pred_nx, pred_ny, phase, pspeed = self._predictStep(
                mnx, mny, now, det_fps)

            with self._lock:
                self._faces = remapped
                self._main = main
                self._nx, self._ny = out_nx, out_ny
                self._area_pct = area_pct
                self._det_fps = det_fps
                self._raw_box_full = raw_full
                self._predict_phase = phase
                self._pred_speed = pspeed
                self._pred_nx, self._pred_ny = pred_nx, pred_ny
                self._seq += 1

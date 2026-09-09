"""vision - Detection de visage en pipeline decouple (P2), detecteur commutable.

Rappel de l'architecture 2 pipelines (mise au point dans face_detect.py) :
  P1 (capture + affichage) : reste dans main.py (la GUI OpenCV doit etre sur le
      thread principal). main.py appelle publish(frame) a chaque image.
  P2 (detection) : thread separe ici. Il recupere la derniere image publiee, la
      reduit (largeur det_width), lance le DETECTEUR choisi, remappe les boites
      en pleine resolution et calcule la position normalisee (nx, ny) du visage
      principal. Tourne a sa propre cadence -> l'affichage ne gele jamais.

Detecteurs disponibles (--detector) :
  haar  : cascade de Haar frontale (leger mais decroche des ~15 deg de profil).
  dnn   : SSD res10 (Caffe, cv2.dnn) - robuste au profil/inclinaison jusqu'a
          ~45-60 deg. Fichiers deploy.prototxt + res10_*.caffemodel deja presents
          dans Bambou4WD_python/src/resources/Other/face_detection_model/.
  yunet : cv2.FaceDetectorYN (ONNX) - le plus robuste, donne 5 points de repere
          et un score ; necessite le modele face_detection_yunet_*.onnx.

Suivi detect-then-track (--track-mode) : par-dessus le detecteur, un TRACKER
visuel suit l'apparence de la cible image par image. Le detecteur ACQUIERT le
visage (init du tracker) ; le tracker le SUIT ensuite meme de profil (ou le
detecteur decroche) ; on re-detecte periodiquement pour re-ancrer / reconfirmer.
  none : pas de tracker (detecteur a chaque image, comportement historique).
  mil  : cv2.TrackerMIL (aucun modele externe, pas de score de suivi).
  vit  : cv2.TrackerVit (ONNX object_tracking_vittrack_*, donne un score).

On reutilise find_cascade() et apply_flip() de face_detect.py (dossier parent).
"""
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

# face_detect.py est a la racine tools/ ; ce module est sous robot_control/old/
_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.normpath(os.path.join(_HERE, "..", ".."))   # <repo>/tools
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from face_detect import find_cascade, apply_flip, apply_rotate  # noqa: E402,F401 (reexportes)

# Modeles vision (DNN/YuNet/VitTrack) livres avec l'ancien toolkit Bambou4WD_python
# (partages avec le legacy src/interaction/RobotObject.py -> on ne les deplace pas).
# Depuis <repo>/tools : ../firmware/usbcam_bamboo/Bambou4WD_python/src/resources/...
_DNN_DIR = os.path.normpath(os.path.join(
    _PARENT, "..", "firmware", "usbcam_bamboo", "Bambou4WD_python", "src",
    "resources", "Other", "face_detection_model"))
DEFAULT_DNN_PROTO = os.path.join(_DNN_DIR, "deploy.prototxt")
DEFAULT_DNN_MODEL = os.path.join(_DNN_DIR, "res10_300x300_ssd_iter_140000.caffemodel")
# Modele YuNet (ONNX) telecharge dans le meme dossier (OpenCV Zoo)
DEFAULT_YUNET_MODEL = os.path.join(_DNN_DIR, "face_detection_yunet_2023mar.onnx")
# Modele VitTrack (ONNX) telecharge dans le meme dossier (OpenCV Zoo, ~0.7 Mo)
DEFAULT_VIT_MODEL = os.path.join(_DNN_DIR, "object_tracking_vittrack_2023sep.onnx")


def _iou(a, b):
    """Intersection sur union de deux boites (x, y, w, h). 0 si l'une est None."""
    if a is None or b is None:
        return 0.0
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def _degenerate(box, w, h):
    """La boite suivie est-elle inexploitable (tracker parti) ? (coords small)."""
    if box is None:
        return True
    x, y, bw, bh = box
    if bw <= 2 or bh <= 2:
        return True
    cx, cy = x + bw / 2.0, y + bh / 2.0
    if cx < 0 or cy < 0 or cx > w or cy > h:   # centre hors cadre
        return True
    return False


def _implausible(box, w, h, ref_area, max_frac, max_grow):
    """La boite a-t-elle DERIVE/GROSSI au point de ne plus etre un visage ?

    Mode d'echec classique des trackers (MIL/Vit) : la boite enfle jusqu'a
    couvrir presque tout le cadre tout en gardant un score correct -> centre au
    milieu, nx/ny~0, la camera ne bouge plus. On rejette :
      - une boite occupant presque tout le cadre (>90% d'une dimension) ;
      - une aire > max_frac de l'image (un visage reste petit) ;
      - une aire > max_grow x l'aire a l'ancrage (a trop grossi depuis le lock).
    """
    if box is None:
        return True
    _, _, bw, bh = box
    if bw > w * 0.9 or bh > h * 0.9:
        return True
    area = bw * bh
    if area > max_frac * (w * h):
        return True
    if ref_area > 0 and area > max_grow * ref_area:
        return True
    return False


def _inter_frac(inner, outer):
    """Fraction de l'aire de `inner` contenue dans `outer` (containment)."""
    if inner is None or outer is None:
        return 0.0
    ax, ay, aw, ah = inner
    bx, by, bw, bh = outer
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ia = aw * ah
    return inter / ia if ia > 0 else 0.0


def _clip(v, lim):
    """Borne v dans [-lim, +lim]."""
    return lim if v > lim else (-lim if v < -lim else v)


class MotionPredictor:
    """Filtre de Kalman a vitesse constante sur la position normalisee (nx, ny).

    Etat [nx, ny, vnx, vny], mesure [nx, ny]. La matrice de transition porte le
    pas de temps reel dt (reecrit a chaque pas, la cadence P2 variant ~27 Hz).
    Deux usages :
      - update() (visage visible) : predict + correct -> lisse et estime la vitesse ;
      - coast()  (visage perdu)   : predict seul -> extrapole la trajectoire.
    Unites normalisees : nx,ny in [-1,+1], vitesses en unites/s.
    """

    def __init__(self, proc_var=1e-2, meas_var=1e-1):
        kf = cv2.KalmanFilter(4, 2)
        kf.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
        kf.transitionMatrix = np.eye(4, dtype=np.float32)
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * float(proc_var)
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * float(meas_var)
        self._kf = kf
        self._inited = False
        # erreur de prediction (innovation) : distance entre la position PREDITE
        # au pas precedent et la position REELLE mesuree maintenant. None tant
        # qu'aucune mesure post-init n'est disponible.
        self.last_err = None

    @property
    def inited(self):
        return self._inited

    def reset(self, nx, ny):
        """(Re)demarre le filtre a (nx, ny), vitesse nulle."""
        self._kf.statePost = np.array([[nx], [ny], [0.], [0.]], dtype=np.float32)
        self._kf.statePre = self._kf.statePost.copy()
        self._kf.errorCovPost = np.eye(4, dtype=np.float32)
        self._inited = True
        self.last_err = None

    def reset_uninit(self):
        """Oublie l'etat : le prochain update() repartira de la mesure."""
        self._inited = False

    def _set_dt(self, dt):
        dt = max(1e-3, min(0.2, float(dt)))       # borne le pas (dt-spike)
        self._kf.transitionMatrix[0, 2] = dt
        self._kf.transitionMatrix[1, 3] = dt

    def update(self, nx, ny, dt):
        """Visage visible : predict + correct. Retourne (px, py, vx, vy) lisses.

        Met a jour last_err = innovation = distance entre la position PREDITE a ce
        pas (a priori, depuis l'etat precedent) et la mesure REELLE (nx, ny). C'est
        l'erreur de prediction : petite = le modele anticipe bien le mouvement.
        """
        if not self._inited:
            self.reset(nx, ny)
            return float(nx), float(ny), 0.0, 0.0
        self._set_dt(dt)
        pre = self._kf.predict().ravel()          # a priori (prediction du pas courant)
        self.last_err = math.hypot(float(nx) - float(pre[0]),
                                   float(ny) - float(pre[1]))
        st = self._kf.correct(
            np.array([[float(nx)], [float(ny)]], dtype=np.float32)).ravel()
        return float(st[0]), float(st[1]), float(st[2]), float(st[3])

    def coast(self, dt):
        """Visage perdu : predict seul (extrapolation). Retourne (px, py, vx, vy).

        La prediction devient le nouvel etat courant pour continuer a extrapoler
        au pas suivant (vitesse maintenue, modele a vitesse constante).
        """
        if not self._inited:
            return None, None, 0.0, 0.0
        self._set_dt(dt)
        st = self._kf.predict()
        self._kf.statePost = st.copy()
        st = st.ravel()
        return float(st[0]), float(st[1]), float(st[2]), float(st[3])

    def peek(self):
        """Etat courant sans avancer le filtre : (px, py, vx, vy)."""
        if not self._inited:
            return None, None, 0.0, 0.0
        st = self._kf.statePost.ravel()
        return float(st[0]), float(st[1]), float(st[2]), float(st[3])


# modes de prediction, dans l'ordre de bascule de la touche P
PREDICT_MODES = ("off", "anticip", "coast")
PREDICT_LABELS = {"off": "off", "anticip": "prediction",
                  "coast": "prediction si perte"}


class FaceTracker:
    """Detecteur de visage asynchrone. Publie la derniere position connue."""

    def __init__(self, det_width=320, min_size=24,
                 scale_factor=1.1, min_neighbors=5, cascade_path=None,
                 detector="haar", conf=0.5,
                 dnn_proto=None, dnn_model=None, yunet_model=None,
                 track_mode="auto", vit_model=None,
                 redetect_ms=400, score_min=0.30, hold_ms=3000,
                 iou_reanchor=0.20, max_area_frac=0.5, max_grow=3.0,
                 predict_mode="anticip", predict_ms=700,
                 predict_lead_ms=120, predict_min_speed=0.4):
        self.det_width = det_width
        self.min_size = min_size
        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors
        self.detector = detector          # "haar" | "dnn" | "yunet"
        self.conf = conf                  # seuil de confiance (dnn/yunet)
        self.cascade_path = cascade_path or find_cascade()
        self.dnn_proto = dnn_proto or DEFAULT_DNN_PROTO
        self.dnn_model = dnn_model or DEFAULT_DNN_MODEL
        self.yunet_model = yunet_model or DEFAULT_YUNET_MODEL

        # --- suivi detect-then-track (tracker visuel par-dessus le detecteur) ---
        self.vit_model = vit_model or DEFAULT_VIT_MODEL
        self.redetect_ms = redetect_ms    # periode de re-detection pendant le lock
        self.score_min = score_min        # seuil de perte (Vit) -> re-acquisition
        self.hold_ms = hold_ms            # duree max de HOLD sans reconfirmation detecteur
        self.iou_reanchor = iou_reanchor  # recouvrement mini pour re-ancrer sur une detection
        self.max_area_frac = max_area_frac  # aire max de la box suivie (fraction du cadre)
        self.max_grow = max_grow          # facteur de grossissement max depuis l'ancrage
        self._track_mode = self._resolve_track_mode(track_mode)  # "none"|"mil"|"vit"
        self._trk = None                  # instance tracker OpenCV courante
        self._locked = False              # cible verrouillee ?
        self._lock_src = "off"            # off|detect|track|reanchor|redetect
        self._track_score = None          # dernier score de suivi (Vit)
        self._ref_area = 0.0              # aire (small) de la box a l'ancrage (anti-grossissement)
        self._last_redetect = 0.0         # t derniere re-detection pendant le lock
        self._last_confirm = 0.0          # t derniere confirmation detecteur (lock/reanchor)
        self._pending_track = None        # bascule tracker demandee a chaud
        # instrumentation : hit/miss BRUT du detecteur pendant le lock (gain profil mesurable)
        self._raw_det = None              # dernier resultat detecteur brut pendant lock (True/False/None)
        self._raw_det_box = None          # derniere box detecteur brute pendant lock (coords small)
        # motif du dernier de-verrouillage (pourquoi le tracker a lache) :
        #   update_ko | degenerate | implausible | score | hold_timeout | switch | None
        self._unlock_reason = None

        # --- prediction de trajectoire (coast + anticipation) -----------------
        self.predict_mode = (predict_mode if predict_mode in PREDICT_MODES
                             else "anticip")
        self.predict_ms = predict_ms              # duree max de coast avant retour maison
        self.predict_lead_ms = predict_lead_ms    # avance temporelle de l'anticipation
        self.predict_min_speed = predict_min_speed  # vitesse mini pour declencher un coast
        self._predictor = MotionPredictor()
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
        self._impl = None             # callable(small_bgr) -> [(x,y,w,h) coords small]
        self._pending = None          # detecteur demande a chaud (applique par worker)
        self._yunet = None            # instance FaceDetectorYN (taille memorisee)
        self._yunet_size = None

    def ready(self, det=None):
        """Retourne (ok, message) : le detecteur (det ou courant) est-il chargeable ?"""
        det = det or self.detector
        if det == "haar":
            if self.cascade_path is None:
                return False, "cascade Haar introuvable (haarcascade_frontalface_default.xml)"
            return True, self.cascade_path
        if det == "dnn":
            for p in (self.dnn_proto, self.dnn_model):
                if not os.path.exists(p):
                    return False, f"fichier DNN manquant : {p}"
            return True, self.dnn_model
        if det == "yunet":
            if not self.yunet_model or not os.path.exists(self.yunet_model):
                return False, ("modele YuNet manquant (--yunet-model chemin.onnx). "
                               "Telecharger face_detection_yunet_2023mar.onnx")
            if not hasattr(cv2, "FaceDetectorYN"):
                return False, "cv2.FaceDetectorYN indisponible (opencv trop ancien)"
            return True, self.yunet_model
        return False, f"detecteur inconnu : {det}"

    def available_detectors(self):
        """Liste des detecteurs actuellement chargeables (fichiers presents)."""
        return [d for d in ("haar", "dnn", "yunet") if self.ready(d)[0]]

    def set_detector(self, name):
        """Demande une bascule a chaud ; appliquee par le worker au tour suivant.

        Retourne (ok, message). Refuse tot si le detecteur n'est pas chargeable.
        """
        ok, msg = self.ready(name)
        if not ok:
            return False, msg
        self._pending = name
        return True, name

    # --- suivi detect-then-track : selection / etat du tracker visuel --------
    def _resolve_track_mode(self, requested):
        """Resout 'auto' -> meilleur tracker disponible (vit sinon mil sinon none).

        Un mode explicite est garde tel quel (main.py verifie track_ready et
        peut retomber sur 'none' avec un message si le modele manque).
        """
        if requested != "auto":
            return requested
        if self.track_ready("vit")[0]:
            return "vit"
        if self.track_ready("mil")[0]:
            return "mil"
        return "none"

    def track_ready(self, mode=None):
        """(ok, message) : le tracker (mode ou courant) est-il chargeable ?"""
        mode = mode or self._track_mode
        if mode == "none":
            return True, "none"
        if mode == "mil":
            if not hasattr(cv2, "TrackerMIL_create"):
                return False, "cv2.TrackerMIL_create indisponible"
            return True, "mil"
        if mode == "vit":
            if not hasattr(cv2, "TrackerVit_create"):
                return False, "cv2.TrackerVit_create indisponible (opencv sans module tracking)"
            if not self.vit_model or not os.path.exists(self.vit_model):
                return False, (f"modele Vit manquant : {self.vit_model} "
                               "(object_tracking_vittrack_2023sep.onnx, OpenCV Zoo)")
            return True, self.vit_model
        return False, f"tracker inconnu : {mode}"

    def available_trackers(self):
        """Liste des modes de suivi actuellement chargeables (modeles presents)."""
        return [m for m in ("none", "mil", "vit") if self.track_ready(m)[0]]

    def set_track_mode(self, name):
        """Demande une bascule de tracker a chaud ; appliquee par le worker."""
        ok, msg = self.track_ready(name)
        if not ok:
            return False, msg
        self._pending_track = name
        return True, name

    @property
    def track_mode(self):
        return self._track_mode

    # --- prediction de trajectoire : bascule a chaud du mode ----------------
    def set_predict_mode(self, name):
        """Demande une bascule du mode de prediction ; appliquee par le worker."""
        if name not in PREDICT_MODES:
            return False, f"mode predict inconnu : {name}"
        self._pending_predict = name
        return True, name

    def cycle_predict_mode(self):
        """Bascule au mode de prediction suivant (off -> anticip -> coast -> ...)."""
        i = PREDICT_MODES.index(self.predict_mode) if \
            self.predict_mode in PREDICT_MODES else 0
        nxt = PREDICT_MODES[(i + 1) % len(PREDICT_MODES)]
        return self.set_predict_mode(nxt)

    @property
    def predict_label(self):
        return PREDICT_LABELS.get(self.predict_mode, self.predict_mode)

    def track_state(self):
        """Etat courant du suivi : {mode, locked, src, score, raw_det, raw_box}.

        raw_det : le detecteur a-t-il vu un visage au dernier cycle de re-detection
        pendant le lock (True/False/None) -> mesure du gain profil du tracker.
        raw_box : cette detection brute remappee pleine res (indicateur overlay).
        predict/pred_speed/pred_nx/pred_ny : phase de prediction (off|lock|coast|home),
        module de la vitesse estimee, et point PREDIT a afficher (None si aucun).
        """
        with self._lock:
            raw_box = self._raw_box_full
            pred_nx, pred_ny = self._pred_nx, self._pred_ny
        return {"mode": self._track_mode, "locked": self._locked,
                "src": self._lock_src, "score": self._track_score,
                "raw_det": self._raw_det, "raw_box": raw_box,
                "unlock_reason": self._unlock_reason,
                "predict": self._predict_phase, "predict_mode": self.predict_mode,
                "pred_speed": self._pred_speed, "pred_err": self._pred_err,
                "pred_nx": pred_nx, "pred_ny": pred_ny}

    def _make_tracker(self, mode):
        if mode == "mil":
            return cv2.TrackerMIL_create()
        if mode == "vit":
            p = cv2.TrackerVit_Params()
            p.net = self.vit_model
            return cv2.TrackerVit_create(p)
        raise RuntimeError(f"tracker inconnu : {mode}")

    def _init_tracker(self, img, box):
        """(Re)cree et initialise le tracker sur box (coords small). True si ok."""
        try:
            trk = self._make_tracker(self._track_mode)
            trk.init(img, tuple(int(v) for v in box))
            self._trk = trk
            _, _, bw, bh = box
            self._ref_area = float(bw * bh)   # reference anti-grossissement
            return True
        except Exception as e:
            print(f"[vision] init tracker '{self._track_mode}' echoue : {e}")
            self._trk = None
            return False

    def _reset_lock(self):
        self._locked = False
        self._trk = None
        self._lock_src = "off"
        self._track_score = None
        self._ref_area = 0.0
        self._raw_det = None
        self._raw_det_box = None

    def start(self):
        ok, msg = self.ready()
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

    def stop(self):
        self._stop = True
        if self._worker is not None:
            self._worker.join(timeout=1.0)

    # --- detecteurs : chacun rend une liste de (x,y,w,h) en coords de small ----
    def _build_impl(self):
        if self.detector == "haar":
            cascade = cv2.CascadeClassifier(self.cascade_path)
            if cascade.empty():
                raise RuntimeError(f"echec chargement cascade : {self.cascade_path}")

            def _haar(small):
                gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                dets = cascade.detectMultiScale(
                    gray, scaleFactor=self.scale_factor,
                    minNeighbors=self.min_neighbors,
                    minSize=(self.min_size, self.min_size))
                return [tuple(int(v) for v in d) for d in dets]
            return _haar

        if self.detector == "dnn":
            net = cv2.dnn.readNetFromCaffe(self.dnn_proto, self.dnn_model)

            def _dnn(small):
                h, w = small.shape[:2]
                blob = cv2.dnn.blobFromImage(
                    small, 1.0, (300, 300), (104.0, 177.0, 123.0),
                    swapRB=False, crop=False)
                net.setInput(blob)
                out = net.forward()
                boxes = []
                for i in range(out.shape[2]):
                    if float(out[0, 0, i, 2]) < self.conf:
                        continue
                    x1 = int(out[0, 0, i, 3] * w)
                    y1 = int(out[0, 0, i, 4] * h)
                    x2 = int(out[0, 0, i, 5] * w)
                    y2 = int(out[0, 0, i, 6] * h)
                    bw, bh = x2 - x1, y2 - y1
                    if bw > 0 and bh > 0:
                        boxes.append((x1, y1, bw, bh))
                return boxes
            return _dnn

        if self.detector == "yunet":
            def _yunet(small):
                h, w = small.shape[:2]
                if self._yunet is None or self._yunet_size != (w, h):
                    self._yunet = cv2.FaceDetectorYN.create(
                        self.yunet_model, "", (w, h),
                        score_threshold=self.conf)
                    self._yunet_size = (w, h)
                _, faces = self._yunet.detect(small)
                if faces is None:
                    return []
                return [(int(f[0]), int(f[1]), int(f[2]), int(f[3]))
                        for f in faces]
            return _yunet

        raise RuntimeError(f"detecteur inconnu : {self.detector}")

    def _track_step(self, small, sw, sh):
        """Un pas de la machine detect-then-track (coords small).

        Retourne (main_box | None, faces_boxes) et met a jour l'etat de verrou
        (_locked, _lock_src, _track_score, _trk, _last_*). Voir docstring module.
        """
        now = time.time()
        if not self._locked:
            # ACQUIRE : le detecteur est le portier -> on ne verrouille que sur visage
            dets = self._impl(small)
            main_s = max(dets, key=lambda r: r[2] * r[3]) if dets else None
            if main_s is not None and self._init_tracker(small, main_s):
                self._locked = True
                self._lock_src = "detect"
                self._last_confirm = now
                self._last_redetect = now
                return main_s, [main_s]
            self._lock_src = "off"
            self._track_score = None
            return None, dets

        # LOCKED : le tracker suit l'apparence
        ok, box = self._trk.update(small)
        box = tuple(int(v) for v in box) if box is not None else None
        score = None
        if self._track_mode == "vit":
            try:
                score = float(self._trk.getTrackingScore())
            except Exception:
                score = None
        self._track_score = score
        # perdu = tracker KO, box hors cadre, box invraisemblable (derive/grossissement),
        # ou score Vit trop bas. _implausible rattrape le mode d'echec ou la box enfle
        # jusqu'a couvrir tout le cadre (centre ~= milieu -> la camera ne bougeait plus).
        # On retient le PREMIER motif declencheur (ordre = severite) pour l'instrumentation.
        lost_reason = None
        if not ok:
            lost_reason = "update_ko"
        elif _degenerate(box, sw, sh):
            lost_reason = "degenerate"
        elif _implausible(box, sw, sh, self._ref_area,
                          self.max_area_frac, self.max_grow):
            lost_reason = "implausible"
        elif score is not None and score < self.score_min:
            lost_reason = "score"
        lost = lost_reason is not None

        # re-detection : immediate si perdu, sinon periodique (re-ancrage / anti-derive)
        do_redetect = lost or (now - self._last_redetect) * 1000.0 >= self.redetect_ms
        det_main = None
        if do_redetect:
            dets = self._impl(small)
            det_main = max(dets, key=lambda r: r[2] * r[3]) if dets else None
            self._last_redetect = now
            self._raw_det = det_main is not None      # instrumentation hit/miss brut
            self._raw_det_box = det_main

        if lost:
            if det_main is not None and self._init_tracker(small, det_main):
                self._locked = True
                self._lock_src = "redetect"
                self._last_confirm = now
                return det_main, [det_main]
            self._unlock_reason = lost_reason   # perdu et aucune detection -> relache
            self._reset_lock()
            return None, ([] if det_main is None else [det_main])

        # tracker OK mais une detection est dispo : le detecteur fait FOI -> on recale
        # dessus a CHAQUE cycle de re-detection, meme si la box a derive LOIN du visage
        # (c'est le role de la re-recherche periodique : corriger la derive du tracker).
        if det_main is not None:
            if self._init_tracker(small, det_main):
                far = _iou(box, det_main) < self.iou_reanchor
                self._lock_src = "recenter" if far else "reanchor"
                self._last_confirm = now
                return det_main, [det_main]
        # detecteur muet mais tracker tient (profil) : HOLD borne par hold_ms
        if do_redetect and det_main is None and \
                (now - self._last_confirm) * 1000.0 > self.hold_ms:
            self._unlock_reason = "hold_timeout"
            self._reset_lock()
            return None, []
        self._lock_src = "track"
        return box, [box]

    def _predict_step(self, mnx, mny, now, det_fps):
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
                self._pred_err = self._predictor.last_err
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

    def _loop(self):
        try:
            self._impl = self._build_impl()
        except Exception as e:
            print(f"[vision] init detecteur '{self.detector}' echoue : {e}")
            return
        print(f"[vision] detecteur actif : {self.detector} | tracker : {self._track_mode}")
        t0 = time.time()
        n = 0
        last_id = -1
        while not self._stop:
            # bascule detecteur a chaud demandee ?
            if self._pending and self._pending != self.detector:
                want = self._pending
                self._pending = None
                old = self.detector
                self.detector = want
                try:
                    self._impl = self._build_impl()
                    self._unlock_reason = "switch"
                    self._reset_lock()              # nouveau detecteur -> re-acquisition
                    print(f"[vision] bascule detecteur -> {want}")
                except Exception as e:
                    self.detector = old
                    try:
                        self._impl = self._build_impl()
                    except Exception:
                        pass
                    print(f"[vision] bascule '{want}' echouee, garde '{old}' : {e}")

            # bascule tracker a chaud demandee ?
            if self._pending_track is not None and self._pending_track != self._track_mode:
                self._track_mode = self._pending_track
                self._pending_track = None
                self._unlock_reason = "switch"
                self._reset_lock()
                print(f"[vision] bascule tracker -> {self._track_mode}")

            # bascule mode de prediction a chaud demandee ?
            if self._pending_predict is not None and self._pending_predict != self.predict_mode:
                self.predict_mode = self._pending_predict
                self._pending_predict = None
                self._predictor.reset_uninit()   # oublie la vitesse du mode precedent
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

            if self._track_mode == "none":
                dets = self._impl(small)
                main_s = max(dets, key=lambda r: r[2] * r[3]) if dets else None
                faces_s = dets
                self._lock_src = "off"
                self._track_score = None
                self._raw_det = None
                self._raw_det_box = None
            else:
                main_s, faces_s = self._track_step(small, sw, sh)

            # box detecteur brute (pendant lock) remappee pleine res -> indicateur overlay
            raw_full = None
            if self._raw_det_box is not None:
                rx, ry, rw, rh = self._raw_det_box
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
            out_nx, out_ny, pred_nx, pred_ny, phase, pspeed = self._predict_step(
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

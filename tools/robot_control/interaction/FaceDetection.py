r"""FaceDetection - Detecteurs de visage commutables + suivi detect-then-track.

Extrait de l'ancien vision.FaceTracker (partie perception seule ; le threading
P2, le remap pleine resolution, la normalisation et la prediction Kalman sont
dans le subsystem modules.tracking.RobotWebCamMotorized). Style Bambou4WD_python
(RobotXxx/camelCase). Comportement des algorithmes INCHANGE.

Detecteurs (setDetector) :
  haar  : cascade de Haar frontale (leger mais decroche des ~15 deg de profil).
  dnn   : SSD res10 (Caffe, cv2.dnn) - robuste au profil/inclinaison ~45-60 deg.
  yunet : cv2.FaceDetectorYN (ONNX) - le plus robuste, score + 5 reperes.

Suivi detect-then-track (setTrackMode) : par-dessus le detecteur, un TRACKER
visuel suit l'apparence de la cible image par image. Le detecteur ACQUIERT le
visage (init du tracker) ; le tracker le SUIT ensuite meme de profil ; on
re-detecte periodiquement pour re-ancrer / reconfirmer.
  none : detecteur a chaque image (comportement historique).
  mil  : cv2.TrackerMIL (aucun modele externe, pas de score).
  vit  : cv2.TrackerVit (ONNX object_tracking_vittrack_*, donne un score).

Travaille en coords de l'image REDUITE ('small', largeur det_width). L'appelant
fournit small + ses dimensions (sw, sh) via process() ; buildImpl() prepare le
detecteur courant (a appeler une fois avant la boucle).
"""
import os
import time

import cv2

from ..device.sensor.RobotSensorWebCam import findCascade

# Modeles vision (DNN/YuNet/VitTrack) livres avec l'ancien toolkit Bambou4WD_python
# (partages avec le legacy src/interaction/RobotObject.py -> on ne les deplace pas).
# Ce module est sous <repo>/tools/robot_control/interaction/ -> remonter de 3
# niveaux pour <repo>, puis firmware/usbcam_bamboo/Bambou4WD_python/src/resources/...
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
_DNN_DIR = os.path.normpath(os.path.join(
    _REPO, "firmware", "usbcam_bamboo", "Bambou4WD_python", "src",
    "resources", "Other", "face_detection_model"))
DEFAULT_DNN_PROTO = os.path.join(_DNN_DIR, "deploy.prototxt")
DEFAULT_DNN_MODEL = os.path.join(_DNN_DIR, "res10_300x300_ssd_iter_140000.caffemodel")
DEFAULT_YUNET_MODEL = os.path.join(_DNN_DIR, "face_detection_yunet_2023mar.onnx")
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


class FaceDetection:
    """Perception visage : detecteur commutable + suivi detect-then-track.

    Sans thread ni image publiee : l'orchestrateur (subsystem) fournit l'image
    reduite a process() et exploite le resultat (main_box, faces) + state().
    """

    def __init__(self, det_width=320, min_size=24,
                 scale_factor=1.1, min_neighbors=5, cascade_path=None,
                 detector="haar", conf=0.5,
                 dnn_proto=None, dnn_model=None, yunet_model=None,
                 track_mode="auto", vit_model=None,
                 redetect_ms=400, score_min=0.30, hold_ms=3000,
                 iou_reanchor=0.20, max_area_frac=0.5, max_grow=3.0):
        self.det_width = det_width
        self.min_size = min_size
        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors
        self.detector = detector          # "haar" | "dnn" | "yunet"
        self.conf = conf                  # seuil de confiance (dnn/yunet)
        self.cascade_path = cascade_path or findCascade()
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
        self._track_mode = self._resolveTrackMode(track_mode)  # "none"|"mil"|"vit"
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

        self._impl = None             # callable(small_bgr) -> [(x,y,w,h) coords small]
        self._pending = None          # detecteur demande a chaud (applique par process)
        self._yunet = None            # instance FaceDetectorYN (taille memorisee)
        self._yunet_size = None

    # --- detecteurs : disponibilite / bascule -------------------------------
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

    def availableDetectors(self):
        """Liste des detecteurs actuellement chargeables (fichiers presents)."""
        return [d for d in ("haar", "dnn", "yunet") if self.ready(d)[0]]

    def setDetector(self, name):
        """Demande une bascule a chaud ; appliquee par process() au tour suivant.

        Retourne (ok, message). Refuse tot si le detecteur n'est pas chargeable.
        """
        ok, msg = self.ready(name)
        if not ok:
            return False, msg
        self._pending = name
        return True, name

    # --- suivi detect-then-track : selection / etat du tracker visuel --------
    def _resolveTrackMode(self, requested):
        """Resout 'auto' -> meilleur tracker disponible (vit sinon mil sinon none).

        Un mode explicite est garde tel quel (l'orchestrateur verifie trackReady
        et peut retomber sur 'none' avec un message si le modele manque).
        """
        if requested != "auto":
            return requested
        if self.trackReady("vit")[0]:
            return "vit"
        if self.trackReady("mil")[0]:
            return "mil"
        return "none"

    def trackReady(self, mode=None):
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

    def availableTrackers(self):
        """Liste des modes de suivi actuellement chargeables (modeles presents)."""
        return [m for m in ("none", "mil", "vit") if self.trackReady(m)[0]]

    def setTrackMode(self, name):
        """Demande une bascule de tracker a chaud ; appliquee par process()."""
        ok, msg = self.trackReady(name)
        if not ok:
            return False, msg
        self._pending_track = name
        return True, name

    @property
    def trackMode(self):
        return self._track_mode

    def state(self):
        """Etat courant du suivi : {mode, locked, src, score, raw_det, raw_box}.

        raw_det : le detecteur a-t-il vu un visage au dernier cycle de re-detection
        pendant le lock (True/False/None) -> mesure du gain profil du tracker.
        raw_box : cette detection brute (coords small ; l'orchestrateur remappe).
        """
        return {"mode": self._track_mode, "locked": self._locked,
                "src": self._lock_src, "score": self._track_score,
                "raw_det": self._raw_det, "raw_box": self._raw_det_box,
                "unlock_reason": self._unlock_reason}

    # --- tracker visuel : instanciation / (re)init / reset ------------------
    def _makeTracker(self, mode):
        if mode == "mil":
            return cv2.TrackerMIL_create()
        if mode == "vit":
            p = cv2.TrackerVit_Params()
            p.net = self.vit_model
            return cv2.TrackerVit_create(p)
        raise RuntimeError(f"tracker inconnu : {mode}")

    def _initTracker(self, img, box):
        """(Re)cree et initialise le tracker sur box (coords small). True si ok."""
        try:
            trk = self._makeTracker(self._track_mode)
            trk.init(img, tuple(int(v) for v in box))
            self._trk = trk
            _, _, bw, bh = box
            self._ref_area = float(bw * bh)   # reference anti-grossissement
            return True
        except Exception as e:
            print(f"[vision] init tracker '{self._track_mode}' echoue : {e}")
            self._trk = None
            return False

    def _resetLock(self):
        self._locked = False
        self._trk = None
        self._lock_src = "off"
        self._track_score = None
        self._ref_area = 0.0
        self._raw_det = None
        self._raw_det_box = None

    # --- detecteurs : chacun rend une liste de (x,y,w,h) en coords de small ----
    def buildImpl(self):
        """(Re)construit le callable de detection pour le detecteur courant.

        A appeler une fois avant la boucle. Retourne le callable et le memorise
        dans self._impl (leve RuntimeError si le detecteur ne charge pas).
        """
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
            self._impl = _haar
            return self._impl

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
            self._impl = _dnn
            return self._impl

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
            self._impl = _yunet
            return self._impl

        raise RuntimeError(f"detecteur inconnu : {self.detector}")

    def _applyPending(self):
        """Applique les bascules a chaud (detecteur, tracker) demandees.

        Repris tel quel du debut de l'ancienne boucle P2 : bascule detecteur
        (reconstruit _impl, repli sur l'ancien en cas d'echec) puis tracker
        (re-acquisition). Les bascules de mode de prediction sont gerees par le
        subsystem (elles concernent le predicteur, pas la detection).
        """
        # bascule detecteur a chaud demandee ?
        if self._pending and self._pending != self.detector:
            want = self._pending
            self._pending = None
            old = self.detector
            self.detector = want
            try:
                self.buildImpl()
                self._unlock_reason = "switch"
                self._resetLock()              # nouveau detecteur -> re-acquisition
                print(f"[vision] bascule detecteur -> {want}")
            except Exception as e:
                self.detector = old
                try:
                    self.buildImpl()
                except Exception:
                    pass
                print(f"[vision] bascule '{want}' echouee, garde '{old}' : {e}")

        # bascule tracker a chaud demandee ?
        if self._pending_track is not None and self._pending_track != self._track_mode:
            self._track_mode = self._pending_track
            self._pending_track = None
            self._unlock_reason = "switch"
            self._resetLock()
            print(f"[vision] bascule tracker -> {self._track_mode}")

    def process(self, small, sw, sh):
        """Un pas de perception sur l'image reduite. Retourne (main_box, faces).

        Applique d'abord les bascules a chaud, puis :
          - mode 'none' : detecteur a chaque image (comportement historique) ;
          - sinon : machine detect-then-track (trackStep).
        Coords en pixels de l'image reduite ('small'). L'orchestrateur remappe.
        """
        self._applyPending()
        if self._track_mode == "none":
            dets = self._impl(small)
            main_s = max(dets, key=lambda r: r[2] * r[3]) if dets else None
            faces_s = dets
            self._lock_src = "off"
            self._track_score = None
            self._raw_det = None
            self._raw_det_box = None
            return main_s, faces_s
        return self.trackStep(small, sw, sh)

    def trackStep(self, small, sw, sh):
        """Un pas de la machine detect-then-track (coords small).

        Retourne (main_box | None, faces_boxes) et met a jour l'etat de verrou
        (_locked, _lock_src, _track_score, _trk, _last_*). Voir docstring module.
        """
        now = time.time()
        if not self._locked:
            # ACQUIRE : le detecteur est le portier -> on ne verrouille que sur visage
            dets = self._impl(small)
            main_s = max(dets, key=lambda r: r[2] * r[3]) if dets else None
            if main_s is not None and self._initTracker(small, main_s):
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
            if det_main is not None and self._initTracker(small, det_main):
                self._locked = True
                self._lock_src = "redetect"
                self._last_confirm = now
                return det_main, [det_main]
            self._unlock_reason = lost_reason   # perdu et aucune detection -> relache
            self._resetLock()
            return None, ([] if det_main is None else [det_main])

        # tracker OK mais une detection est dispo : le detecteur fait FOI -> on recale
        # dessus a CHAQUE cycle de re-detection, meme si la box a derive LOIN du visage
        # (c'est le role de la re-recherche periodique : corriger la derive du tracker).
        if det_main is not None:
            if self._initTracker(small, det_main):
                far = _iou(box, det_main) < self.iou_reanchor
                self._lock_src = "recenter" if far else "reanchor"
                self._last_confirm = now
                return det_main, [det_main]
        # detecteur muet mais tracker tient (profil) : HOLD borne par hold_ms
        if do_redetect and det_main is None and \
                (now - self._last_confirm) * 1000.0 > self.hold_ms:
            self._unlock_reason = "hold_timeout"
            self._resetLock()
            return None, []
        self._lock_src = "track"
        return box, [box]

r"""FaceRecogNode - node roslite : reconnaissance de visage (SFace + galerie).

S'appuie sur le SUIVI (TrackingNode) : quand un episode de verrou est STABLE
(lock_age >= seuil), on aligne le visage principal (box + 5 landmarks pleine res
relayes par /tracking/result) et on calcule son embedding SFace pour :
  - recognition : predire (id_pred + nom) ou « unknown » (cosinus < seuil) ;
  - acquisition : idem + SAUVEGARDER le crop aligne 112x112 sous le jeu de visages
    (identified/<id_pred>-<name>/<id_lot>/ si connu, sinon unknown/<id_lot>/) pour
    constituer le jeu d'apprentissage au fil de l'eau ;
  - off : ne rien faire.

Commandes ponctuelles (via /recognition/config, seq discrete) executees dans un
THREAD WORKER pour ne pas bloquer la boucle roslite :
  train          -> passe d'enrolement FaceTrainer (scan/split/enrole/cross-test) ;
  recognize_file -> reconnaissance d'une image sur disque (chemin) ;
  acquire_file   -> acquisition manuelle d'une image sur disque (chemin + id_lot).

Le moteur (FaceRecognizer) et le pipeline (FaceTrainer) sont dans robot_control
(reutilisables hors roslite). Ce node n'est que le CABLAGE ROS.

Entrees : /camera/image, /tracking/result, /recognition/config.
Sortie  : /recognition/result.
"""
import os
import threading
import time

import cv2

from robot_control.interaction.FaceRecognizer import FaceRecognizer
from robot_control.interaction.FaceTrainer import FaceTrainer
from robot_control.interaction.FaceDetection import DEFAULT_YUNET_MODEL

from ..roslite import Node
from ..msgs import ImageMsg, TrackingResult, RecognitionResult, RecognitionConfig

RECOG_MODES = ("off", "recognition", "acquisition")


class FaceRecogNode(Node):
    """Reconnaissance/acquisition du visage suivi + apprentissage (thread worker)."""

    def __init__(self, args, telemetry=None):
        super().__init__("recognition")
        self._args = args
        self._tel = telemetry
        faces_dir = getattr(args, "faces_dir", None) or os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "..", "robot_control", "faces")
        self.faces_dir = os.path.normpath(faces_dir)
        self.rec = FaceRecognizer(
            model_path=getattr(args, "sface_model", None),
            faces_dir=self.faces_dir,
            cos_thr=getattr(args, "recog_cos_thr", 0.363))
        self.mode = getattr(args, "recog_mode", "off") or "off"
        self._stable_s = float(getattr(args, "recog_lock_stable_s", 2.0))
        self._min_ok = float(getattr(args, "recog_min_ok", 0.6))
        self._min_imgs = int(getattr(args, "train_min_imgs", 10))
        # garde-fous acquisition (qualite + debit)
        self._acq_min_size = int(getattr(args, "acq_min_size", 60))
        self._acq_min_sharp = float(getattr(args, "acq_min_sharp", 40.0))
        self._acq_period = 0.3            # au plus ~3 crops/s par episode
        self._acq_cap = 80                # plafond d'images par lot
        # prefixe de session pour rendre les id_lot uniques (lock_id seul repart a 1)
        self._session = time.strftime("%Y%m%d-%H%M%S")

        # etat de reconnaissance par episode (throttle : 1 calcul par nouveau seq)
        self._seq_done = -1
        self._episode_lock = 0
        self._last_result = RecognitionResult(seq=0, status="idle", mode=self.mode)
        self._acq_last_t = 0.0
        self._acq_count = {}              # id_lot -> nb d'images ecrites

        # thread worker : commandes lourdes (train / fichiers) hors boucle
        self._cmd_seq_done = -1
        self._worker = None
        self._pending_out = None          # RecognitionResult a publier (rempli par worker)
        self._need_reload = False
        self._lock = threading.Lock()

        self._img_in = self.create_input("/camera/image", ImageMsg)
        self._trk_in = self.create_input("/tracking/result", TrackingResult)
        self._cfg_in = self.create_input("/recognition/config", RecognitionConfig)
        self._out = self.create_output("/recognition/result", RecognitionResult)

    # --- cycle de vie --------------------------------------------------------
    def on_start(self):
        """Charge le modele SFace + la galerie (NON fatal : degrade en 'off')."""
        ok, msg = self.rec.ready()
        if not ok:
            print(f"[reco] modele SFace indisponible ({msg}) -> reconnaissance OFF")
            self.mode = "off"
            return
        try:
            self.rec.build()
            print(f"[reco] SFace charge  |  galerie : {len(self.rec.gallery)} personne(s)  "
                  f"|  faces_dir : {self.faces_dir}  |  mode : {self.mode}")
        except Exception as e:
            print(f"[reco] echec chargement SFace ({e}) -> reconnaissance OFF")
            self.mode = "off"

    # --- boucle --------------------------------------------------------------
    def process(self):
        self._apply_config(self._cfg_in.get())

        # resultat differe d'un worker (train / commande fichier) : publier + recharger
        with self._lock:
            pend = self._pending_out
            self._pending_out = None
            need_reload = self._need_reload
            self._need_reload = False
        if need_reload and self.rec.loaded:
            self.rec.reload()
        if pend is not None:
            self._last_result = pend
            self._out.set(pend)

        if not self.rec.loaded or self.mode == "off":
            return

        res = self._trk_in.get()
        img = self._img_in.get()
        if res is None or img is None or img.frame is None:
            return
        # ne (re)calculer qu'a chaque NOUVELLE detection (throttle par seq)
        if res.seq == self._seq_done:
            return

        tstate = res.tstate or {}
        stable = (tstate.get("locked") and res.main is not None
                  and tstate.get("lock_age", 0.0) >= self._stable_s)
        if not stable:
            return
        self._seq_done = res.seq

        lock_id = int(tstate.get("lock_id", 0) or 0)
        id_lot = f"{self._session}-{lock_id}"
        landmarks = tstate.get("landmarks")
        emb = self.rec.embed(img.frame, res.main, landmarks)
        id_pred, name, cos = self.rec.match(emb)
        status = "known" if id_pred is not None else "unknown"

        out = RecognitionResult(
            seq=res.seq, status=status, id_pred=id_pred, name=name,
            score=round(float(cos), 3), id_lot=id_lot, lock_id=lock_id, mode=self.mode)

        if self.mode == "acquisition":
            saved = self._acquire(img.frame, res.main, landmarks, tstate,
                                  id_pred, name, id_lot)
            if saved:
                self._log("acquire", id_lot=id_lot, status=status,
                          id_pred=id_pred, n=self._acq_count.get(id_lot))

        # log de reconnaissance : 1 par episode (au changement de lock_id)
        if lock_id != self._episode_lock:
            self._episode_lock = lock_id
            self._log("recognize", status=status, id_pred=id_pred, name=name,
                      score=round(float(cos), 3), lock_id=lock_id)

        self._last_result = out
        self._out.set(out)

    # --- acquisition ---------------------------------------------------------
    def _acquire(self, frame, box, landmarks, tstate, id_pred, name, id_lot):
        """Sauvegarde un crop aligne 112x112 (avec garde-fous qualite + debit)."""
        now = time.time()
        if now - self._acq_last_t < self._acq_period:
            return False
        if self._acq_count.get(id_lot, 0) >= self._acq_cap:
            return False
        # qualite : landmarks presents (redressement fiable) + taille + nettete
        if landmarks is None:
            return False
        x, y, w, h = box
        if min(w, h) < self._acq_min_size:
            return False
        aligned = self.rec.align(frame, box, landmarks)
        sharp = cv2.Laplacian(cv2.cvtColor(aligned, cv2.COLOR_BGR2GRAY),
                              cv2.CV_64F).var()
        if sharp < self._acq_min_sharp:
            return False
        # destination : identified/<id_pred>-<name>/<id_lot>/ si connu, sinon unknown/
        if id_pred is not None:
            d = os.path.join(self.faces_dir, "identified", f"{id_pred}-{name}", id_lot)
        else:
            d = os.path.join(self.faces_dir, "unknown", id_lot)
        os.makedirs(d, exist_ok=True)
        fn = os.path.join(d, f"{int(now * 1000) % 10_000_000:07d}.jpg")
        if not cv2.imwrite(fn, aligned):
            return False
        self._acq_last_t = now
        self._acq_count[id_lot] = self._acq_count.get(id_lot, 0) + 1
        return True

    # --- config / commandes --------------------------------------------------
    def _apply_config(self, cfg):
        """Mode continu (off/recognition/acquisition) + commande ponctuelle (seq)."""
        if cfg is None:
            return
        if cfg.mode and cfg.mode in RECOG_MODES and cfg.mode != self.mode:
            if cfg.mode != "off" and not self.rec.loaded:
                self._log("recog_mode", to=cfg.mode, ok=False, info="SFace indisponible")
            else:
                self.mode = cfg.mode
                self._log("recog_mode", to=self.mode, ok=True)
        if cfg.command and cfg.seq != self._cmd_seq_done:
            self._cmd_seq_done = cfg.seq
            self._startWorker(cfg.command, cfg.path, cfg.id_lot)

    def _startWorker(self, command, path, id_lot):
        """Lance une commande lourde dans un thread (si aucun deja en cours)."""
        if self._worker is not None and self._worker.is_alive():
            self._log("recog_busy", command=command)
            return
        if not self.rec.loaded:
            self._log("recog_cmd", command=command, ok=False, info="SFace indisponible")
            return
        self._worker = threading.Thread(
            target=self._runCommand, args=(command, path, id_lot), daemon=True)
        self._worker.start()

    def _runCommand(self, command, path, id_lot):
        """Corps du thread worker : train / recognize_file / acquire_file."""
        try:
            if command == "train":
                trainer = FaceTrainer(
                    self.faces_dir, recognizer=self.rec, cos_thr=self.rec.cos_thr,
                    min_imgs=self._min_imgs, recog_min_ok=self._min_ok,
                    log=(self._tel.log if self._tel else None))
                summary = trainer.run()
                out = RecognitionResult(seq=0, status="idle", mode=self.mode,
                                        train=summary)
                with self._lock:
                    self._pending_out = out
                    self._need_reload = True
            elif command in ("recognize_file", "acquire_file"):
                self._runFileCommand(command, path, id_lot)
            else:
                self._log("recog_cmd", command=command, ok=False, info="inconnue")
        except Exception as e:
            self._log("recog_cmd", command=command, ok=False, info=str(e))

    def _runFileCommand(self, command, path, id_lot):
        """Reconnaissance/acquisition manuelle d'une image sur disque."""
        if not path or not os.path.exists(path):
            self._log("recog_cmd", command=command, ok=False, info=f"fichier absent : {path}")
            return
        frame = cv2.imread(path)
        if frame is None:
            self._log("recog_cmd", command=command, ok=False, info="lecture image KO")
            return
        box, landmarks = self._detectFile(frame)
        if box is None:
            self._log("recog_cmd", command=command, ok=False, info="aucun visage detecte")
            return
        emb = self.rec.embed(frame, box, landmarks)
        id_pred, name, cos = self.rec.match(emb)
        status = "known" if id_pred is not None else "unknown"
        lot = id_lot or f"manual-{self._session}"
        if command == "acquire_file":
            aligned = self.rec.align(frame, box, landmarks)
            if id_pred is not None:
                d = os.path.join(self.faces_dir, "identified", f"{id_pred}-{name}", lot)
            else:
                d = os.path.join(self.faces_dir, "unknown", lot)
            os.makedirs(d, exist_ok=True)
            cv2.imwrite(os.path.join(d, f"{int(time.time()*1000) % 10_000_000:07d}.jpg"),
                        aligned)
            self._log("acquire", id_lot=lot, status=status, id_pred=id_pred, source="file")
        else:
            self._log("recognize", status=status, id_pred=id_pred, name=name,
                      score=round(float(cos), 3), source="file")
        with self._lock:
            self._pending_out = RecognitionResult(
                seq=0, status=status, id_pred=id_pred, name=name,
                score=round(float(cos), 3), id_lot=lot, mode=self.mode)

    def _detectFile(self, frame):
        """Detection YuNet ponctuelle (fichier) -> (box, 5 landmarks) ou (None, None)."""
        model = getattr(self._args, "yunet_model", None) or DEFAULT_YUNET_MODEL
        if not os.path.exists(model) or not hasattr(cv2, "FaceDetectorYN"):
            # pas de YuNet -> pas de landmarks : reco plein cadre (degrade)
            h, w = frame.shape[:2]
            return (0, 0, w, h), None
        h, w = frame.shape[:2]
        yn = cv2.FaceDetectorYN.create(model, "", (w, h),
                                       score_threshold=getattr(self._args, "det_conf", 0.6))
        _, faces = yn.detect(frame)
        if faces is None or len(faces) == 0:
            return None, None
        f = max(faces, key=lambda r: r[2] * r[3])
        box = (int(f[0]), int(f[1]), int(f[2]), int(f[3]))
        lms = [(float(f[4 + 2 * i]), float(f[5 + 2 * i])) for i in range(5)]
        return box, lms

    # --- divers --------------------------------------------------------------
    def result(self):
        """Dernier resultat de reconnaissance (pour l'HUD du Core)."""
        return self._last_result

    def _log(self, msg, **fields):
        if self._tel is not None:
            self._tel.log("event", msg=msg, source="reco", **fields)

    def on_stop(self):
        pass

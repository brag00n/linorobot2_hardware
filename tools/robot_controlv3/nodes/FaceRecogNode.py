r"""FaceRecogNode - node roslite : reconnaissance de visage (SFace + galerie).

S'appuie sur le SUIVI (TrackingNode) : quand un episode de verrou est STABLE
(lock_age >= seuil), on aligne le visage principal (box + 5 landmarks pleine res
relayes par /tracking/result) et on calcule son embedding SFace pour :
  - recognition : predire (id_pred + nom) ou « unknown » (cosinus < seuil) ;
  - acquisition : idem + SAUVEGARDER le crop aligne 112x112 sous le jeu de visages
    (identified/<id_pred>-<name>/<id_lot>/ si connu, sinon unknown/<id_lot>/) pour
    constituer le jeu d'apprentissage au fil de l'eau ;
  - off : ne rien faire.

Commandes fichier ponctuelles (via /recognition/config, seq discrete) executees
dans un THREAD WORKER pour ne pas bloquer la boucle roslite :
  recognize_file -> reconnaissance d'une image sur disque (chemin) ;
  acquire_file   -> acquisition manuelle d'une image sur disque (chemin + id_lot).

L'APPRENTISSAGE (« train ») est un node DEDIE (FaceTrainNode) : ce node-ci ecoute
/recognition/train_state et RECHARGE sa galerie a la fin d'un batch (done_seq).

Le moteur (FaceRecognizer) est dans robot_control (reutilisable hors roslite).
Ce node n'est que le CABLAGE ROS.

Entrees : /camera/image, /tracking/result, /recognition/config, /recognition/train_state.
Sortie  : /recognition/result.
"""
import collections
import os
import threading
import time

import cv2

from robot_control.interaction.FaceRecognizer import FaceRecognizer
from robot_control.interaction.FaceDetection import DEFAULT_YUNET_MODEL

from ..roslite import Node
from ..msgs import (ImageMsg, TrackingResult, RecognitionResult, RecognitionConfig,
                    TrainState)

RECOG_MODES = ("off", "recognition", "acquisition")


class FaceRecogNode(Node):
    """Reconnaissance/acquisition du visage suivi + apprentissage (thread worker)."""

    def __init__(self, args, telemetry=None):
        super().__init__("recognition")
        self._args = args
        self._tel = telemetry
        faces_dir = getattr(args, "faces_dir", None) or os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "..", "data", "faces")
        self.faces_dir = os.path.normpath(faces_dir)
        self.rec = FaceRecognizer(
            model_path=getattr(args, "sface_model", None),
            faces_dir=self.faces_dir,
            cos_thr=getattr(args, "recog_cos_thr", 0.363))
        self.mode = getattr(args, "recog_mode", "off") or "off"
        self._stable_s = float(getattr(args, "recog_lock_stable_s", 2.0))
        # redressement du visage a la reconnaissance : detecteur YuNet dedie (mis en
        # cache par taille d'image) pour obtenir des landmarks PLEINE RES et aligner
        # (alignCrop redresse la tete penchee). Les landmarks du tracker sont souvent
        # absents (box interpolee VIT) ou en coords reduites -> inexploitables ici.
        self._yunet_model = getattr(args, "yunet_model", None) or DEFAULT_YUNET_MODEL
        # seuil de confiance DEDIE (plus bas que le detecteur de suivi) : YuNet
        # decroche sur les visages penches ; un seuil permissif (0.5) rattrape ces
        # detections -> plus de frames redressees (alignCrop) tete inclinee.
        self._det_conf = float(getattr(args, "recog_det_conf", 0.5))
        self._yn = None
        self._yn_wh = None
        # stabilisation temporelle par episode (anti-flicker) : lissage EMA du
        # cosinus + hysteresis a deux seuils. Entree au seuil « meme identite »
        # recommande (0.363) applique au cosinus LISSE (le lissage tue le bruit
        # image qui faisait clignoter) ; sortie a un seuil nettement plus bas ->
        # une fois verrouille, on TIENT malgre les creux transitoires.
        thr = float(getattr(args, "recog_cos_thr", 0.363))
        _on = getattr(args, "recog_cos_on", None)
        self._cos_on = float(_on) if _on is not None else thr          # entree « connu »
        self._cos_off = float(getattr(args, "recog_cos_off", 0.30))    # sortie « connu »
        self._ema_a = float(getattr(args, "recog_ema", 0.4))           # poids EMA cosinus
        # telemetrie reco LIVE (periodique, moyennee sur fenetre glissante) : rend
        # la stabilite observable PENDANT l'episode (pas seulement a sa fin).
        self._live_period = float(getattr(args, "recog_live_period", 2.0))  # cadence event
        self._live_win = float(getattr(args, "recog_live_win", 3.0))        # fenetre moyenne
        self._live_t = 0.0
        self._live_buf = collections.deque(maxlen=600)  # (t, cos, known, id_pred, name)
        # garde-fous acquisition (qualite + debit)
        self._acq_min_size = int(getattr(args, "acq_min_size", 60))
        self._acq_min_sharp = float(getattr(args, "acq_min_sharp", 40.0))
        self._acq_period = 0.3            # au plus ~3 crops/s par episode
        self._acq_cap = 80                # plafond d'images par lot
        # prefixe de session pour rendre les id_lot uniques (lock_id seul repart a 1)
        self._session = time.strftime("%Y%m%d-%H%M%S")

        # etat de reconnaissance par episode (throttle : 1 calcul par nouveau seq)
        self._seq_done = -1
        # etat de stabilisation de l'episode courant (reinitialise au changement
        # de lock_id) : identite committee par hysteresis + accumulateurs metrique.
        self._ep_lock = 0
        self._ep_ema = 0.0            # cosinus lisse (EMA)
        self._ep_id = None            # id_pred committe (None = inconnu)
        self._ep_name = "unknown"
        self._ep_n = 0                # ticks de l'episode
        self._ep_known = 0            # ticks classes « known »
        self._ep_flips = 0            # bascules known<->unknown
        self._ep_prev = None          # statut du tick precedent
        self._ep_sum = 0.0            # somme des cosinus bruts (moyenne)
        self._ep_sq = 0.0             # somme des carres (ecart-type)
        self._ep_thumb = None         # 1re vignette de l'episode (figee jusqu'au suivant)
        self._last_result = RecognitionResult(seq=0, status="idle", mode=self.mode)
        self._acq_last_t = 0.0
        self._acq_count = {}              # id_lot -> nb d'images ecrites

        # thread worker : commandes fichier ponctuelles (recognize_file/acquire_file)
        # hors boucle. L'APPRENTISSAGE est un node dedie (FaceTrainNode) : ce node
        # ecoute /recognition/train_state et RECHARGE sa galerie a la fin d'un batch.
        self._cmd_seq_done = -1
        self._worker = None
        self._pending_out = None          # RecognitionResult a publier (rempli par worker)
        self._lock = threading.Lock()
        self._train_done_seen = 0         # dernier done_seq traite (declenche le reload)

        self._img_in = self.create_input("/camera/image", ImageMsg)
        self._trk_in = self.create_input("/tracking/result", TrackingResult)
        self._cfg_in = self.create_input("/recognition/config", RecognitionConfig)
        self._train_in = self.create_input("/recognition/train_state", TrainState)
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

        # fin d'un batch d'apprentissage (node dedie) -> recharger la galerie
        ts = self._train_in.get()
        if ts is not None and ts.done_seq != self._train_done_seen:
            self._train_done_seen = ts.done_seq
            if self.rec.loaded:
                self.rec.reload()
                self._log("gallery_reload", persons=len(self.rec.gallery))

        # resultat differe d'une commande fichier (recognize_file/acquire_file)
        with self._lock:
            pend = self._pending_out
            self._pending_out = None
        if pend is not None:
            self._last_result = pend
            self._out.set(pend)

        if not self.rec.loaded or self.mode == "off":
            # reco arretee : publier UNE fois un resultat "idle" pour que le bus
            # (donc le HUD : badge + bouton R) reflete l'arret. Sinon l'ancien
            # resultat reste latche et l'affichage semble « ne pas se desactiver ».
            if self._last_result is None or self._last_result.status != "idle":
                self._flushEpisode()                    # clot proprement l'episode
                self._ep_lock, self._ep_n = None, 0
                idle = RecognitionResult(seq=0, status="idle", mode=self.mode)
                self._last_result = idle
                self._out.set(idle)
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
        # redressement geometrique : detection YuNet fraiche (plein cadre) -> box + 5
        # landmarks pleine res -> alignCrop (redresse les visages penches). Repli sur
        # les landmarks du tracker puis recadrage brut si YuNet ne trouve rien.
        box, landmarks = self._detectLandmarks(img.frame, res.main)
        aligned = box is not None                        # YuNet frais -> alignCrop applique
        if box is None:
            box, landmarks = res.main, tstate.get("landmarks")
        crop = self.rec.align(img.frame, box, landmarks)   # 112x112 redresse (= vue recogniseur)
        emb = self.rec.feature(crop)
        # candidat le PLUS proche, SANS seuil : l'hysteresis/lissage decide en aval.
        id_cand, name_cand, cos = self.rec.nearest(emb)

        # nouvel episode -> clore le precedent (bilan metrique) puis reinitialiser
        if lock_id != self._ep_lock:
            self._flushEpisode()
            self._ep_lock = lock_id
            self._ep_ema = cos
            self._ep_id, self._ep_name = None, "unknown"
            self._ep_n = self._ep_known = self._ep_flips = 0
            self._ep_prev = None
            self._ep_sum = self._ep_sq = 0.0
            self._ep_thumb = crop        # vignette figee sur le 1er visage de l'episode
        else:
            self._ep_ema = self._ema_a * cos + (1.0 - self._ema_a) * self._ep_ema

        # hysteresis a deux seuils sur le cosinus lisse -> identite committee
        ema = self._ep_ema
        if self._ep_id is None:
            if id_cand is not None and ema >= self._cos_on:
                self._ep_id, self._ep_name = id_cand, name_cand
        elif id_cand == self._ep_id and ema >= self._cos_off:
            pass                                              # reste verrouille (bande basse)
        elif id_cand is not None and ema >= self._cos_on:
            self._ep_id, self._ep_name = id_cand, name_cand   # bascule d'identite
        else:
            self._ep_id, self._ep_name = None, "unknown"      # decroche sous cos_off
        status = "known" if self._ep_id is not None else "unknown"

        # metrique d'episode (taux de reco, moyenne/ecart-type cosinus, bascules)
        self._ep_n += 1
        if status == "known":
            self._ep_known += 1
        if self._ep_prev is not None and status != self._ep_prev:
            self._ep_flips += 1
        self._ep_prev = status
        self._ep_sum += cos
        self._ep_sq += cos * cos
        stability = self._ep_known / self._ep_n if self._ep_n else 0.0

        out = RecognitionResult(
            seq=res.seq, status=status, id_pred=self._ep_id, name=self._ep_name,
            score=round(float(ema), 3), raw_score=round(float(cos), 3),
            stability=round(float(stability), 3),
            id_lot=id_lot, lock_id=lock_id, mode=self.mode, thumb=self._ep_thumb)

        if self.mode == "acquisition":
            saved = self._acquire(img.frame, box, landmarks, tstate,
                                  self._ep_id, self._ep_name, id_lot)
            if saved:
                self._log("acquire", id_lot=id_lot, status=status,
                          id_pred=self._ep_id, n=self._acq_count.get(id_lot))

        self._last_result = out
        self._out.set(out)

        # telemetrie live : echantillon + emission periodique moyennee
        now = time.time()
        self._live_buf.append((now, float(cos), status == "known",
                               self._ep_id, self._ep_name, aligned))
        if now - self._live_t >= self._live_period:
            self._live_t = now
            self._emitLive(now)

    def _emitLive(self, now):
        """Emet un event `recog_live` : stats MOYENNEES sur la fenetre glissante
        (`recog_live_win`), pour suivre la stabilite EN COURS d'episode (taux de
        reco, cos moyen/ecart-type, bascules known<->unknown). Lisible via MCP
        robot-analysis `tail types=["event"]` sans attendre la fin de l'episode."""
        cutoff = now - self._live_win
        xs = [s for s in self._live_buf if s[0] >= cutoff]
        if not xs:
            return
        n = len(xs)
        coss = [s[1] for s in xs]
        mean = sum(coss) / n
        var = max(0.0, sum(c * c for c in coss) / n - mean * mean)
        known = sum(1 for s in xs if s[2])
        aligned = sum(1 for s in xs if len(s) > 5 and s[5])   # frames alignCrop (YuNet frais)
        flips = sum(1 for a, b in zip(xs, xs[1:]) if a[2] != b[2])
        # identite committee dominante sur la fenetre (parmi les frames « known »)
        ids = [(s[3], s[4]) for s in xs if s[2] and s[3] is not None]
        maj_id, maj_name = (collections.Counter(ids).most_common(1)[0][0]
                            if ids else (None, "unknown"))
        self._log("recog_live", n=n, recog_rate=round(known / n, 3),
                  aligned_rate=round(aligned / n, 3),
                  cos_mean=round(mean, 3), cos_std=round(var ** 0.5, 3),
                  flips=flips, id_pred=maj_id, name=maj_name,
                  ema=round(float(self._ep_ema), 3),
                  status="known" if self._ep_id is not None else "unknown",
                  lock_id=self._ep_lock)

    def _flushEpisode(self):
        """Bilan metrique de l'episode qui se termine -> event `recog_stability`.

        recog_rate = taux de frames « known » ; cos_mean/cos_std sur les cosinus
        bruts ; flips = nb de bascules known<->unknown (indice de clignotement).
        Consultable via MCP robot-analysis `tail types=["event"]`."""
        if self._ep_n <= 0:
            return
        mean = self._ep_sum / self._ep_n
        var = max(0.0, self._ep_sq / self._ep_n - mean * mean)
        self._log("recog_stability", n=self._ep_n,
                  recog_rate=round(self._ep_known / self._ep_n, 3),
                  cos_mean=round(mean, 3), cos_std=round(var ** 0.5, 3),
                  flips=self._ep_flips, id_pred=self._ep_id, name=self._ep_name,
                  lock_id=self._ep_lock)

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
        # « train » est traite par le node dedie (FaceTrainNode) : on l'ignore ici.
        if (cfg.command and cfg.command != "train"
                and cfg.seq != self._cmd_seq_done):
            self._cmd_seq_done = cfg.seq
            self._startWorker(cfg.command, cfg.path, cfg.id_lot)

    def _startWorker(self, command, path, id_lot):
        """Lance une commande fichier dans un thread (si aucun deja en cours)."""
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
        """Corps du thread worker : recognize_file / acquire_file (fichier disque)."""
        try:
            if command in ("recognize_file", "acquire_file"):
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

    # --- redressement (alignement) live -------------------------------------
    def _detectLandmarks(self, frame, track_box):
        """Detection YuNet plein cadre -> (box, 5 landmarks) PLEINE RES du visage
        recouvrant `track_box`, pour aligner (redresser) le crop comme a l'enrolement.
        Detecteur mis en cache par taille d'image. (None, None) si indispo/aucun visage."""
        if not hasattr(cv2, "FaceDetectorYN") or not os.path.exists(self._yunet_model):
            return None, None
        h, w = frame.shape[:2]
        if self._yn is None or self._yn_wh != (w, h):
            self._yn = cv2.FaceDetectorYN.create(
                self._yunet_model, "", (w, h), score_threshold=self._det_conf)
            self._yn_wh = (w, h)
        _, faces = self._yn.detect(frame)
        if faces is None or len(faces) == 0:
            return None, None
        f = self._pickFace(faces, track_box)
        box = (int(f[0]), int(f[1]), int(f[2]), int(f[3]))
        lms = [(float(f[4 + 2 * i]), float(f[5 + 2 * i])) for i in range(5)]
        return box, lms

    @staticmethod
    def _pickFace(faces, box):
        """Visage detecte le plus proche (par centre) de la box suivie ; le plus grand
        si box absente. Comparaison par centre (et non IoU) car les deux boites viennent
        de detecteurs differents."""
        if box is None:
            return max(faces, key=lambda r: r[2] * r[3])
        bcx, bcy = box[0] + box[2] / 2.0, box[1] + box[3] / 2.0
        return min(faces, key=lambda r: (r[0] + r[2] / 2.0 - bcx) ** 2
                   + (r[1] + r[3] / 2.0 - bcy) ** 2)

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
        self._flushEpisode()          # bilan du dernier episode en cours

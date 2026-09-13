r"""FaceTrainNode - node roslite DEDIE a l'apprentissage (enrolement SFace).

Separe du FaceRecogNode : le batch d'apprentissage (scan / split 50-50 / enrolement /
validation / cross-test / regroupement des lots) est LOURD. Il tourne dans un THREAD
WORKER (UN SEUL a la fois -> pas d'apprentissage concurrent) pour ne jamais bloquer
la boucle roslite ni la reconnaissance temps reel du FaceRecogNode.

Le node possede sa PROPRE instance FaceRecognizer (chargement SFace independant) :
le pipeline calcule des embeddings dans le thread worker sans partager l'objet cv2
avec le thread principal (ou tourne la reconnaissance).

Entree : /recognition/config       (reagit uniquement a command == "train").
Sortie : /recognition/train_state   (running + log fenetre + synthese + done_seq).
Le FaceRecogNode ecoute /recognition/train_state et RECHARGE sa galerie a la fin
d'un batch (quand done_seq s'incremente).
"""
import collections
import os
import threading
import time

from robot_control.interaction.FaceRecognizer import FaceRecognizer
from robot_control.interaction.FaceTrainer import FaceTrainer

from ..roslite import Node
from ..msgs import RecognitionConfig, TrainState


class FaceTrainNode(Node):
    """Apprentissage (enrolement) dans un thread worker unique ; publie son etat."""

    def __init__(self, args, telemetry=None):
        super().__init__("face_train")
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
        self._min_ok = float(getattr(args, "recog_min_ok", 0.6))
        self._min_imgs = int(getattr(args, "train_min_imgs", 10))

        # worker unique + etat partage (worker <-> boucle) protege par _lock
        self._cmd_seq_done = -1
        self._worker = None
        self._lock = threading.Lock()
        self._running = False
        self._log_buf = collections.deque(maxlen=120)   # log fenetre (HUD)
        self._last_t = 0.0
        self._done_seq = 0                               # +1 a chaque fin de batch
        self._summary = None                             # synthese du dernier batch
        self._state_seq = 0                              # version d'etat publiee
        self._published = None                           # derniere signature publiee

        self._cfg_in = self.create_input("/recognition/config", RecognitionConfig)
        self._out = self.create_output("/recognition/train_state", TrainState)

    # --- cycle de vie --------------------------------------------------------
    def on_start(self):
        """Charge une instance SFace propre au node (NON fatal : apprentissage indispo)."""
        ok, msg = self.rec.ready()
        if not ok:
            print(f"[train] modele SFace indisponible ({msg}) -> apprentissage indisponible")
            return
        try:
            self.rec.build()
            print(f"[train] SFace charge  |  faces_dir : {self.faces_dir}")
        except Exception as e:
            print(f"[train] echec chargement SFace ({e}) -> apprentissage indisponible")

    # --- boucle --------------------------------------------------------------
    def process(self):
        cfg = self._cfg_in.get()
        if (cfg is not None and cfg.command == "train"
                and cfg.seq != self._cmd_seq_done):
            self._cmd_seq_done = cfg.seq
            self._start()

        # publie l'etat courant SEULEMENT quand il change (running / log / done)
        with self._lock:
            running = self._running
            lines = list(self._log_buf)
            summary = self._summary
            done_seq = self._done_seq
        sig = (running, len(lines), done_seq)
        if sig != self._published:
            self._published = sig
            self._state_seq += 1
            self._out.set(TrainState(seq=self._state_seq, running=running,
                                     lines=lines, summary=summary, done_seq=done_seq))

    # --- worker --------------------------------------------------------------
    def _start(self):
        """Lance le batch dans le worker (refuse si un batch tourne deja)."""
        if self._worker is not None and self._worker.is_alive():
            self._tlog("train_busy")             # un seul apprentissage a la fois
            return
        if not self.rec.loaded:
            self._tlog("train_error", msg="SFace indisponible")
            with self._lock:
                self._done_seq += 1              # debloque un eventuel attente cote reco
            return
        with self._lock:
            self._summary = None
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        """Corps du thread : enrolement complet via FaceTrainer, puis synthese."""
        with self._lock:
            self._running = True
        self._tlog("train_launch", faces_dir=self.faces_dir, min_imgs=self._min_imgs)
        try:
            trainer = FaceTrainer(
                self.faces_dir, recognizer=self.rec, cos_thr=self.rec.cos_thr,
                min_imgs=self._min_imgs, recog_min_ok=self._min_ok, log=self._tlog)
            summary = trainer.run()
            with self._lock:
                self._summary = summary
        except Exception as e:
            self._tlog("train_error", msg=str(e))
            with self._lock:
                self._summary = {"ok": False, "error": str(e)}
        finally:
            with self._lock:
                self._running = False
                self._done_seq += 1

    def _tlog(self, msg, **fields):
        """Journalise une etape : ligne fenetree (HUD) + relais telemetrie (event)."""
        parts = []
        for k, v in fields.items():
            if k in ("id_lot", "lot"):
                v = str(v).rsplit("-", 1)[-1]     # suffixe court (lock_id) au lieu du chemin
            parts.append("%s=%s" % (k, v))
        line = msg if not parts else "%s  %s" % (msg, " ".join(parts))
        with self._lock:
            self._log_buf.append(line)
            self._last_t = time.time()
        if self._tel is not None:
            self._tel.log("event", msg=msg, source="train", **fields)

    def on_stop(self):
        pass

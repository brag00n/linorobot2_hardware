r"""FaceRecognizer - Reconnaissance de visage SFace (embeddings + galerie).

Miroir de FaceDetection.py cote perception, mais pour l'IDENTITE : enveloppe
cv2.FaceRecognizerSF (ONNX face_recognition_sface_2021dec, natif opencv 4.11/5.0
SANS contrib - verifie) qui produit un embedding 128D par visage. Deux visages
sont la meme personne si la SIMILARITE COSINUS de leurs embeddings depasse un
seuil (recommandation OpenCV : >= 0.363).

L'« apprentissage » ici n'est PAS un re-entrainement de reseau : c'est de
l'ENROLEMENT = calculer et stocker les embeddings de reference d'une personne
(galerie). match() compare un embedding a la galerie.

Alignement : recognizer.alignCrop(frame, faceRow) redresse le visage (rotation
plan + echelle + translation) vers un 112x112 canonique via les 5 landmarks
YuNet (yeux/nez/coins bouche). C'est le format attendu par feature(). Sans
landmarks (detecteur haar/dnn) -> simple recadrage+resize (degrade).

Ce module ne depend PAS de roslite : reutilisable par le noeud v3, le monolithe
v1 ou en CLI (FaceTrainer).
"""
import os

import cv2
import numpy as np

from .FaceDetection import _DNN_DIR

# Modele SFace livre dans le meme dossier que YuNet/Vit (voir FaceDetection).
DEFAULT_SFACE_MODEL = os.path.join(_DNN_DIR, "face_recognition_sface_2021dec.onnx")

# Cote canonique de l'image alignee produite/attendue par SFace.
ALIGN_SIZE = 112


class FaceRecognizer:
    """Moteur SFace + galerie {id_pred: {name, embeddings (N x 128)}}."""

    def __init__(self, model_path=None, faces_dir=None, cos_thr=0.363):
        self.model_path = model_path or DEFAULT_SFACE_MODEL
        self.faces_dir = faces_dir            # racine du jeu de visages (facultatif)
        self.cos_thr = float(cos_thr)         # seuil « meme identite »
        self._rec = None                      # instance cv2.FaceRecognizerSF
        self._gallery = {}                    # id_pred(int) -> {"name":str, "embeddings":ndarray}

    # --- disponibilite / chargement -----------------------------------------
    def ready(self):
        """(ok, message) : le modele SFace est-il chargeable ?"""
        if not hasattr(cv2, "FaceRecognizerSF"):
            return False, "cv2.FaceRecognizerSF indisponible (opencv trop ancien)"
        if not self.model_path or not os.path.exists(self.model_path):
            return False, (f"modele SFace manquant : {self.model_path} "
                           "(face_recognition_sface_2021dec.onnx, OpenCV Zoo)")
        return True, self.model_path

    def build(self):
        """Charge le modele (leve RuntimeError si indisponible) et la galerie."""
        ok, msg = self.ready()
        if not ok:
            raise RuntimeError(msg)
        self._rec = cv2.FaceRecognizerSF.create(self.model_path, "")
        if self.faces_dir:
            self.reload()
        return self._rec

    @property
    def loaded(self):
        return self._rec is not None

    # --- alignement / embedding ---------------------------------------------
    @staticmethod
    def _faceRow(box, landmarks):
        """Ligne de detection (1x15) attendue par alignCrop : box + 5 landmarks + score.

        box = (x,y,w,h) ; landmarks = [(x,y)*5] (yeux D/G, nez, bouche D/G).
        Le score (col 14) n'est pas utilise par alignCrop -> 1.0.
        """
        x, y, w, h = box
        row = [float(x), float(y), float(w), float(h)]
        for (px, py) in landmarks:
            row.extend((float(px), float(py)))
        row.append(1.0)
        return np.asarray([row], dtype=np.float32)

    def _resizeCrop(self, frame, box):
        """Repli sans landmarks : recadre la box et redimensionne en 112x112."""
        if box is None:
            return cv2.resize(frame, (ALIGN_SIZE, ALIGN_SIZE))
        x, y, w, h = (int(v) for v in box)
        fh, fw = frame.shape[:2]
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(fw, x + w), min(fh, y + h)
        if x1 <= x0 or y1 <= y0:
            return cv2.resize(frame, (ALIGN_SIZE, ALIGN_SIZE))
        return cv2.resize(frame[y0:y1, x0:x1], (ALIGN_SIZE, ALIGN_SIZE))

    def align(self, frame, box, landmarks):
        """Redresse le visage vers un 112x112 canonique. Landmarks None -> repli resize."""
        if self._rec is not None and landmarks is not None and box is not None:
            try:
                return self._rec.alignCrop(frame, self._faceRow(box, landmarks))
            except Exception:
                pass
        return self._resizeCrop(frame, box)

    def feature(self, aligned):
        """Embedding 128D (float32, 1D) d'une image DEJA alignee (112x112 BGR)."""
        f = self._rec.feature(aligned)
        return np.asarray(f, dtype=np.float32).reshape(-1)

    def embed(self, frame, box, landmarks):
        """Raccourci align()+feature() a partir d'une frame + box + landmarks."""
        return self.feature(self.align(frame, box, landmarks))

    # --- similarite / correspondance galerie --------------------------------
    @staticmethod
    def cosine(a, b):
        """Similarite cosinus de deux embeddings (>= cos_thr => meme identite)."""
        a = np.asarray(a, dtype=np.float32).reshape(-1)
        b = np.asarray(b, dtype=np.float32).reshape(-1)
        na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
        if na == 0.0 or nb == 0.0:
            return -1.0
        return float(np.dot(a, b) / (na * nb))

    # alias explicite pour le cross-test du trainer
    def matchPair(self, e1, e2):
        return self.cosine(e1, e2)

    def match(self, emb):
        """Compare emb a la galerie -> (id_pred, name, cos) ou (None, 'unknown', best_cos).

        Pour chaque personne on prend le MEILLEUR cosinus sur ses embeddings de
        reference (robuste aux variations de pose du sous-lot d'enrolement).
        """
        best_id, best_name, best_cos = None, "unknown", -1.0
        for id_pred, entry in self._gallery.items():
            embs = entry["embeddings"]
            if embs is None or len(embs) == 0:
                continue
            cos = max(self.cosine(emb, e) for e in embs)
            if cos > best_cos:
                best_id, best_name, best_cos = id_pred, entry["name"], cos
        if best_cos >= self.cos_thr:
            return best_id, best_name, best_cos
        return None, "unknown", best_cos

    def nearest(self, emb):
        """Identite de galerie la PLUS proche, SANS seuil -> (id_pred, name, cos).

        Comme match() mais renvoie toujours le meilleur candidat (meme sous le seuil)
        pour laisser l'appelant appliquer sa propre hysteresis/lissage temporel.
        Renvoie (None, 'unknown', -1.0) si la galerie est vide.
        """
        best_id, best_name, best_cos = None, "unknown", -1.0
        for id_pred, entry in self._gallery.items():
            embs = entry["embeddings"]
            if embs is None or len(embs) == 0:
                continue
            cos = max(self.cosine(emb, e) for e in embs)
            if cos > best_cos:
                best_id, best_name, best_cos = id_pred, entry["name"], cos
        return best_id, best_name, best_cos

    # --- galerie -------------------------------------------------------------
    @staticmethod
    def parseIdentifiedName(dirname):
        """`<id_pred>-<name>` -> (id_pred:int|None, name:str). Tolerant."""
        base = os.path.basename(dirname.rstrip("/\\"))
        head, _, tail = base.partition("-")
        try:
            return int(head), (tail or "unknown")
        except ValueError:
            return None, base

    def reload(self):
        """(Re)charge la galerie depuis <faces_dir>/identified/*/gallery.npz."""
        self._gallery = {}
        if not self.faces_dir:
            return self._gallery
        root = os.path.join(self.faces_dir, "identified")
        if not os.path.isdir(root):
            return self._gallery
        for name in sorted(os.listdir(root)):
            d = os.path.join(root, name)
            npz = os.path.join(d, "gallery.npz")
            if not os.path.isdir(d) or not os.path.exists(npz):
                continue
            id_pred, person = self.parseIdentifiedName(name)
            if id_pred is None:
                continue
            try:
                data = np.load(npz)
                embs = np.asarray(data["embeddings"], dtype=np.float32)
            except Exception:
                continue
            self._gallery[id_pred] = {"name": person, "embeddings": embs}
        return self._gallery

    @property
    def gallery(self):
        return self._gallery

    @staticmethod
    def saveGallery(path, embeddings):
        """Ecrit gallery.npz (embeddings N x 128) pour une personne."""
        embeddings = np.asarray(embeddings, dtype=np.float32)
        if embeddings.ndim == 1:
            embeddings = embeddings.reshape(1, -1)
        np.savez(path, embeddings=embeddings)

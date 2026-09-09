r"""RobotSensorWebCam - Camera USB du Bamboo v4 (pipeline P1 : capture + flip/rotate).

Portage vers le style Bambou4WD_python (RobotXxx, camelCase) de la logique
d'ouverture camera de l'ancien main.py (open_camera + helpers) et des fonctions
image findCascade/applyFlip/applyRotate de face_detect.py. Comportement INCHANGE :
ouverture MSMF/DShow avec scan guide par nom (prefere la cam USB, ecarte la webcam
integree), negociation MJPG (30 fps en 720p), lecture pleine resolution puis
flip + rotate. La detection (P2) est ailleurs (interaction.FaceDetection, orchestree
par modules.tracking.RobotWebCamMotorized).

Les helpers findCascade/applyFlip/applyRotate sont RECOPIES ici (et non importes
de face_detect.py) pour que le paquet robot_control ne depende plus d'un script
frere via sys.path. face_detect.py reste la demo autonome equivalente.
"""
import os

import cv2

# Backends de capture (Windows) : MSMF negocie MJPG -> 30 fps 720p ; DShow reste
# souvent en YUY2 (~4 fps, plafond USB2) ; any = choix OpenCV.
BACKENDS = {"msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW, "any": cv2.CAP_ANY}


def findCascade():
    """Chemin de la cascade Haar frontale : dossier tools/ d'abord, sinon cv2.

    Ce module est sous <repo>/tools/robot_control/device/sensor/ ; la cascade
    livree avec le toolkit est a la racine <repo>/tools/ -> remonter de 3 niveaux.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    tools_root = os.path.normpath(os.path.join(here, "..", "..", ".."))
    local = os.path.join(tools_root, "haarcascade_frontalface_default.xml")
    if os.path.exists(local):
        return local
    builtin = os.path.join(cv2.data.haarcascades,
                           "haarcascade_frontalface_default.xml")
    return builtin if os.path.exists(builtin) else None


def applyFlip(frame, mode):
    if mode == "v":
        return cv2.flip(frame, 0)
    if mode == "h":
        return cv2.flip(frame, 1)
    if mode == "180":
        return cv2.flip(frame, -1)
    return frame


def applyRotate(frame, deg):
    """Rotation libre de l'image autour de son centre (dimensions conservees).

    deg = angle en degres, convention OpenCV : positif = sens ANTI-HORAIRE.
    Sert a redresser une camera montee de travers. Les dimensions restent
    identiques : les coins sortis du cadre sont rognes, les zones decouvertes
    remplies en noir. Appliquee en P1 AVANT la detection pour que tout le
    pipeline (coords normalisees) reste coherent.
    """
    if not deg:
        return frame
    h, w = frame.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), float(deg), 1.0)
    return cv2.warpAffine(frame, m, (w, h), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)


class RobotSensorWebCam:
    """Camera USB : ouverture robuste (scan guide par nom) + lecture flip/rotate."""

    def __init__(self, index="auto", backend="msmf", width=1280, height=720,
                 fps=30, flip="v", rotate=0.0, preferName="USB",
                 skipNames=None, scanMax=6, telemetry=None):
        self.index = index                # "auto" ou entier ; devient l'index reel apres open()
        self.backend = backend            # devient le backend reel apres open()
        self.width = width                # resolution demandee (maj a l'effective apres open)
        self.height = height
        self.fps = fps
        self.flip = flip
        self.rotate = rotate
        self.preferName = preferName
        self.skipNames = skipNames if skipNames is not None else ["Integrated", "IR Camera"]
        self.scanMax = scanMax
        self.tel = telemetry
        self.cap = None
        self.readFail = 0

    # --- enumeration / ordre d'essai des cameras ----------------------------
    def _listDevices(self):
        """Noms des cameras dans l'ordre des index (DirectShow via pygrabber).

        Retourne [] si pygrabber est absent : on retombe alors sur un scan brut.
        Cet ordre correspond aux index CAP_DSHOW ; il s'aligne en pratique avec
        CAP_MSMF (et de toute facon chaque index est teste par lecture reelle).
        """
        try:
            from pygrabber.dshow_graph import FilterGraph
            return FilterGraph().get_input_devices()
        except Exception:
            return []

    def _cameraOrder(self):
        """Ordre d'essai des index : prefere preferName, repousse skipNames en dernier.

        Sert a distinguer la camera USB pan-tilt de la webcam integree du PC.
        Retourne [(index, nom_ou_None), ...]. Sans noms dispo -> 0..scanMax-1.
        """
        names = self._listDevices()
        skip = [s.strip().lower() for s in (self.skipNames or []) if s.strip()]
        prefer = (self.preferName or "").strip().lower()
        if not names:
            return [(i, None) for i in range(self.scanMax)]
        indexed = list(enumerate(names))

        def is_skip(n):
            return any(s in n.lower() for s in skip)

        def is_pref(n):
            return bool(prefer) and prefer in n.lower()

        preferred = [(i, n) for i, n in indexed if is_pref(n)]
        neutral = [(i, n) for i, n in indexed if not is_pref(n) and not is_skip(n)]
        skipped = [(i, n) for i, n in indexed if not is_pref(n) and is_skip(n)]
        return preferred + neutral + skipped

    def _configureCap(self, cap):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

    def _tryOpen(self, index, backend):
        """Ouvre + configure + LIT une image. Retourne le cap si valide, sinon None.

        Un cap peut s'ouvrir sans jamais delivrer d'image (backend indisponible
        pour cet index) : on exige une lecture reussie pour valider la camera.
        """
        cap = cv2.VideoCapture(index, BACKENDS[backend])
        if not cap.isOpened():
            cap.release()
            return None
        self._configureCap(cap)
        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            return None
        return cap

    def open(self):
        """Ouvre la camera (index precis puis scan auto). Retourne True si ouverte.

        1) si un index precis est demande, on l'essaie sur le backend demande ;
        2) sinon (ou en cas d'echec) on SCANNE les cameras, dans un ORDRE guide par
           les noms (prefere preferName, ecarte skipNames comme la webcam integree),
           et on retient la premiere qui ouvre ET delivre une image.
        Met a jour self.cap/index/backend/width/height (resolution effective).
        """
        backend = self.backend
        backends = [backend] + [b for b in ("msmf", "dshow", "any") if b != backend]
        if str(self.index).lower() != "auto":
            idx = int(self.index)
            cap = self._tryOpen(idx, backend)
            if cap is not None:
                self._adopt(cap, idx, backend)
                if self.tel:
                    self.tel.log("event", msg="camera_open", index=idx,
                                 backend=backend, mode="demande")
                return True
            print(f"Camera index {idx} (backend {backend}) indisponible -> scan auto...")
        order = self._cameraOrder()
        listing = ", ".join(f"{i}:{n or '?'}" for i, n in order)
        print(f"Cameras (ordre d'essai) : {listing}")
        # Camera en boucle EXTERNE : on epuise tous les backends de la camera
        # preferee avant de passer a la suivante (sinon un backend qui echoue sur
        # la bonne cam mais marche sur la webcam PC ferait un mauvais choix).
        for idx, name in order:
            for b in backends:
                cap = self._tryOpen(idx, b)
                if cap is not None:
                    print(f"Camera retenue : index {idx} ({name or '?'}) backend {b}.")
                    self._adopt(cap, idx, b)
                    if self.tel:
                        self.tel.log("event", msg="camera_open", index=idx,
                                     name=name, backend=b, mode="auto")
                    return True
        if self.tel:
            self.tel.log("event", msg="camera_fail", scanned=self.scanMax,
                         devices=listing)
        return False

    def _adopt(self, cap, index, backend):
        """Adopte un cap ouvert : memorise index/backend + resolution effective."""
        self.cap = cap
        self.index = index
        self.backend = backend
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # --- capture ------------------------------------------------------------
    @property
    def opened(self):
        return self.cap is not None

    def read(self):
        """Lit une image et applique flip puis rotate. Retourne (ok, frame|None)."""
        if self.cap is None:
            return False, None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.readFail += 1
            return False, None
        self.readFail = 0
        frame = applyFlip(frame, self.flip)
        frame = applyRotate(frame, self.rotate)
        return True, frame

    def release(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None

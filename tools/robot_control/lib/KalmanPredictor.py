r"""KalmanPredictor - Filtre de Kalman a vitesse constante sur (nx, ny).

Extrait de l'ancien vision.MotionPredictor ; algorithme INCHANGE, style aligne
sur Bambou4WD_python (methodes camelCase). Etat [nx, ny, vnx, vny], mesure
[nx, ny]. La matrice de transition porte le pas de temps reel dt (reecrit a
chaque pas, la cadence de detection variant ~27 Hz). Deux usages :
  - update() (visage visible) : predict + correct -> lisse et estime la vitesse ;
  - coast()  (visage perdu)   : predict seul -> extrapole la trajectoire.
Unites normalisees : nx, ny in [-1, +1], vitesses en unites/s.
"""
import math

import cv2
import numpy as np


class KalmanPredictor:
    def __init__(self, procVar=1e-2, measVar=1e-1):
        kf = cv2.KalmanFilter(4, 2)
        kf.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
        kf.transitionMatrix = np.eye(4, dtype=np.float32)
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * float(procVar)
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * float(measVar)
        self._kf = kf
        self._inited = False
        # erreur de prediction (innovation) : distance entre la position PREDITE
        # au pas precedent et la position REELLE mesuree maintenant. None tant
        # qu'aucune mesure post-init n'est disponible.
        self.lastErr = None

    @property
    def inited(self):
        return self._inited

    def reset(self, nx, ny):
        """(Re)demarre le filtre a (nx, ny), vitesse nulle. Fluide (return self)."""
        self._kf.statePost = np.array([[nx], [ny], [0.], [0.]], dtype=np.float32)
        self._kf.statePre = self._kf.statePost.copy()
        self._kf.errorCovPost = np.eye(4, dtype=np.float32)
        self._inited = True
        self.lastErr = None
        return self

    def resetUninit(self):
        """Oublie l'etat : le prochain update() repartira de la mesure."""
        self._inited = False
        return self

    def _setDt(self, dt):
        dt = max(1e-3, min(0.2, float(dt)))       # borne le pas (dt-spike)
        self._kf.transitionMatrix[0, 2] = dt
        self._kf.transitionMatrix[1, 3] = dt

    def update(self, nx, ny, dt):
        """Visage visible : predict + correct. Retourne (px, py, vx, vy) lisses.

        Met a jour lastErr = innovation = distance entre la position PREDITE a ce
        pas (a priori, depuis l'etat precedent) et la mesure REELLE (nx, ny). C'est
        l'erreur de prediction : petite = le modele anticipe bien le mouvement.
        """
        if not self._inited:
            self.reset(nx, ny)
            return float(nx), float(ny), 0.0, 0.0
        self._setDt(dt)
        pre = self._kf.predict().ravel()          # a priori (prediction du pas courant)
        self.lastErr = math.hypot(float(nx) - float(pre[0]),
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
        self._setDt(dt)
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

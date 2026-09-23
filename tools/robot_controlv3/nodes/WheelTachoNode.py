r"""WheelTachoNode - tachymetre OPTIQUE des roues (RPM + sens) par la camera.

POURQUOI CE NODE EXISTE
-----------------------
La carte ESP32 WaveShare n'a que DEUX odometres (un encodeur par cote) et le
firmware RECOPIE les voies 3/4 sur 1/2 : `wheel_rpm` ne peut donc PAS dire si la
deuxieme roue d'un cote tourne reellement (courroie cassee, moteur muet, PWM qui
ne passe pas). Ce node mesure la rotation des DEUX roues visibles du cote droit
DIRECTEMENT dans l'image, ce qui est le seul temoin independant de la carte.

PRINCIPE (2 etapes, volontairement dissymetriques)
--------------------------------------------------
1. CALIBRATION (une seule fois, a l'armement ou sur demande) : HoughCircles dans
   une ROI pour trouver les 2 roues. Les parametres de Hough sont FRAGILES sur
   cette scene (mesure du 2026-09-23 : dp=1.2/p1=120/p2=40 -> exactement les 2
   roues ; dp=1.5/p1=100/p2=30 -> 7 faux positifs ; dp=1.0/p1=150/p2=50 -> 0) ->
   on ne relance JAMAIS Hough par frame, on fige les cercles et on suit la marque.
   Dans la foulee on cherche la MARQUE comme la PLUS GROSSE TACHE BLANCHE du
   disque (les marques sont des carres de papier colles sur les roues) : sa
   distance au centre donne le rayon de l'anneau de mesure. Le critere est la
   SURFACE et non la saillance, parce que le reflet speculaire du moyeu est
   LEGEREMENT PLUS saillant que le papier (162 contre 155 niveaux, mesure du banc)
   tout en etant bien plus petit -- choisir par saillance faisait suivre ce reflet,
   donc une tache IMMOBILE, donc un RPM nul qu'on aurait pris pour une roue
   bloquee.
2. MESURE (a chaque frame) : on echantillonne la luminance sur le CERCLE du rayon
   de marque (N angles) -> profil angulaire 1D ; la marque blanche en est le pic.
   L'argmax (affine par interpolation parabolique) donne un ANGLE ; le deroulage
   entre frames donne un angle cumule signe ; une regression lineaire sur une
   fenetre glissante donne le RPM et son SENS.

Pas de seuillage de luminance absolu, pas de detection de blob : c'est ce qui
rend la mesure insensible a l'eclairage general (on ne regarde qu'un CONTRASTE
local sur un cercle) -- le seuillage absolu avait ete essaye et noye par le texte
du HUD.

BUDGET CPU : POURQUOI CA NE MANGE PAS LA CADENCE CAMERA
-------------------------------------------------------
Le calcul ne lit que 3 rayons x N_ANGLES x 2 roues = ~1080 pixels par frame, par
indexation directe du tableau : quelques dizaines de microsecondes, argmax,
parabole et moindres carres compris. Le piege est ailleurs : convertir toute
l'image en gris (cvtColor sur 1280x720, ~1 ms) pour n'en lire que 1080 pixels
coutait 95 % de la facture -> on echantillonne DIRECTEMENT le BGR et on moyenne
les canaux (seul le CONTRASTE importe, les poids luma sont inutiles). Le gris
n'est calcule que sur la ROI, et seulement a la calibration.

ENREGISTREMENT ET ANALYSE A POSTERIORI (mode `record_s`)
--------------------------------------------------------
Decoupler acquisition et analyse est legitime, mais ce qu'il faut echantillonner
n'est PAS l'image : a 30 fps une frame BGR 720p brute pese 2,8 Mo (83 Mo/s, 5 Go
la minute) et l'encoder en JPEG coute ~100 fois plus cher que la mesure. Le
PROFIL ANGULAIRE, lui, fait 180 octets par roue : 11 ko/s, 650 ko la minute. On
enregistre donc les profils horodates, et tout le reglable (saillance minimale,
fenetre de regression, detection de repliement) se rejoue hors ligne par
`analyze_dump()` SANS refaire tourner les roues.

Le rejeu appelle LES MEMES fonctions que le direct (`_Wheel.push`) : c'est
volontaire, deux implementations de la meme mesure finiraient par diverger et on
ne saurait plus laquelle croire.

PRECISION : LE FACTEUR LIMITANT EST L'HORODATAGE, PAS LA CADENCE. `img.stamp` est
pose quand la frame sort de la file, pas a l'exposition du capteur. A 35,8 RPM
(= 215 deg/s), 5 ms de gigue valent 1,07 deg d'erreur d'angle, la resolution du
profil affinee par parabole etant ~0,3 deg. La regression sur fenetre moyenne
cette gigue : ~1 % d'erreur de pente sur 0,4 s, tres en dedans du +-10 % exige
par le lot 4.3.

CONTRAINTE DE CADENCE (mesuree, elle justifie la place du calcul)
-----------------------------------------------------------------
Au point de fonctionnement du lot 4.3 (0,15 m/s, D=0,08 m) la roue tourne a
35,8 RPM = 0,6 tour/s. Avec UNE marque par roue, le signe se perd des que la
marque avance de plus d'un DEMI-tour entre deux images : a 29 fps cela fait
870 RPM. Verifie sur profils synthetiques (marque gaussienne, bruit 3 LSB,
gigue d'horodatage 5 ms) : ecart <= 0,5 % de 5 a 700 RPM, `alias` commence a
compter des 725 RPM (garde a 150 deg/image) et la valeur devient FAUSSE
au-dela de ~870 RPM (900 RPM lu -839). Donc : exploitable jusqu'a ~700 RPM,
soit 20x le point de fonctionnement, et tout depassement se voit dans `alias`.
En revanche l'image `latest.jpg` lue par le MCP est ecrite a 2 Hz : un calcul
fait par sondage d'images cote MCP aliaserait SILENCIEUSEMENT des 60 RPM
(magnitude ET sens faux). D'ou la mesure ICI.

PROVENANCE DE L'IMAGE : le Core dessine le HUD EN PLACE sur l'objet `frame`
porte par /camera/image, APRES spin_once(). Le node voit donc une image propre
au tour ou elle est publiee -- mais si CameraNode echoue a lire, il ne publie
PAS (le bus garde la precedente, deja annotee). On ne traite donc une image que
sur `seq` NEUF : c'est la garde qui empeche de mesurer de l'encre.

REPERE ANGULAIRE : l'index d'angle k vaut la direction (cos a, -sin a) avec
a = 2*pi*k/N, donc un angle CROISSANT = sens ANTI-HORAIRE A L'ECRAN. Le signe du
RPM publie suit cette convention (+ = anti-horaire a l'ecran) : c'est une
convention d'IMAGE, pas de robot -- la correspondance avec « avance » depend de
l'orientation de la camera (cf. la fiche memoire du banc).
"""
import os
import time
from collections import deque

import cv2
import numpy as np

from ..roslite import Node
from ..msgs import ImageMsg, WheelTachoConfig, WheelTachoState

# --- ROI par defaut, en FRACTIONS de l'image (independant de la resolution) ---
# Valeurs relevees sur le banc en 1280x720 (roues droites vues de plein travers,
# servos pan S1=88 deg / tilt S2=20 deg) : x 600..1000, y 440..690.
ROI_DEFAULT = (0.469, 0.611, 0.781, 0.958)      # (x0, y0, x1, y1) en fractions

# --- parametres Hough figes (le jeu qui a trouve EXACTEMENT les 2 roues) ------
HOUGH = dict(dp=1.2, minDist=60, param1=120, param2=40, minRadius=40, maxRadius=75)

N_ANGLES = 180                  # 2 deg par echantillon (marque a r~28 px : ~1 px)
RING_RADII = (-3.0, -1.5, 0.0, 1.5, 3.0)   # moyenne radiale : debruite le profil
# La marque est un carre de papier d'environ 13 px de cote a cette distance : une
# moyenne sur +-3 px reste SUR le papier et absorbe l'erreur de centre de Hough,
# qui autrement ferait respirer le rayon de la marque au fil de la rotation.
# Recherche de la marque A LA CALIBRATION : la marque est la PLUS GROSSE TACHE
# BLANCHE du disque de la roue (un carre de papier colle). Le seuil de blanc est
# RELATIF au maximum du disque (pas de niveau absolu : l'eclairage du banc change),
# et on ne garde que les taches assez loin du centre pour que l'angle ait un sens.
MARK_WHITE_FRAC = 0.72          # seuil = centre + frac * (max - centre) du disque
# Bande radiale admissible pour la marque, en fraction du rayon de roue. Exiger 0.55
# ecarte le moyeu et ses reflets (et une tache proche du centre donnerait de toute
# facon un angle tres bruite : bras de levier court). Le plafond est a 1.15 et non
# a 1.00 parce que les deux carres NE SONT PAS colles au meme rayon -- mesure du
# banc : 0.65 r a droite (sur la jante), 1.09 r a gauche (au bord du pneu, donc
# au-dela du cercle rendu par Hough). Un plafond a 1.00 excluait purement et
# simplement la marque de gauche, et le node suivait alors un reflet.
MARK_R_MIN_FRAC = 0.55          # plus pres du moyeu : hors sujet -> rejet
MARK_R_MAX_FRAC = 1.15          # au-dela : sol et chassis -> rejet
# Remplissage minimal de la boite englobante (surface / boite). C'est LE critere qui
# separe la marque du sol : elargir le disque a 1.15 r laisse entrer une plage de sol
# eclairee, dont la surface (198 px) approche celle du papier (292 px) -- mais c'est
# un croissant CREUX (remplissage 0.27) la ou un carre de papier est plein (0.77, ou
# 0.49 quand le bord du disque le rogne). La surface seule ne suffisait donc pas.
MARK_FILL_MIN = 0.40
# Nombre de frames agregees par la calibration. Hough est en fait STABLE par roue
# (mesure sur 14 frames : G 43,4-47,6 px, D 61,1-63,8 px), mais il oscille de +-2 px
# et le centre de +-1 px : agreger et prendre la mediane fige la geometrie une fois
# pour toutes, au lieu de la laisser respirer sous la marque.
# L'agregation est PAR ROUE, jamais entre roues : les deux roues sont physiquement
# IDENTIQUES mais pas a la meme taille A L'IMAGE -- rapport constant de 1,39 entre
# l'avant et l'arriere. C'est de la perspective (camera proche et inclinee), donc un
# fait a respecter, pas une erreur a corriger en egalisant les rayons.
CALIB_FRAMES = 12
PROM_MIN = 10.0                 # saillance minimale du pic (niveaux de gris)
FIT_WINDOW_S = 0.40             # fenetre de regression du RPM (s)
FIT_MIN_PTS = 4                 # en dessous : pas de RPM (pas de pente fiable)
ALIAS_DEG = 150.0               # |dtheta| au-dela : repliement probable -> signale
LOST_MAX = 5                    # frames sans pic avant de declarer la roue perdue
RECORD_MAX_S = 120.0            # plafond d'enregistrement (~1,3 Mo de profils)


class _Wheel:
    """Etat d'une roue suivie : geometrie figee + historique d'angle cumule.

    Separe en deux etages pour que le rejeu hors ligne partage le calcul du
    direct : `sample()` extrait le profil d'une image, `push()` fait toute la
    mesure a partir d'un profil (donc aussi a partir d'un profil ENREGISTRE).
    """

    def __init__(self, wid, cx, cy, r, r_mark, idx_y=None, idx_x=None):
        self.id = wid                   # "G"/"D" = gauche/droite DANS L'IMAGE
        self.cx, self.cy, self.r = cx, cy, r
        self.r_mark = r_mark
        self._iy, self._ix = idx_y, idx_x   # indices precalcules (3, N) du cercle
        self.theta = None               # angle courant du pic (deg, 0..360)
        self.cum = 0.0                  # angle cumule deroule (deg, signe)
        self.hist = deque()             # (t, cum) pour la regression
        self.rpm = None
        self.quality = 0.0              # saillance du pic (niveaux de gris)
        self.ok = False
        self.lost = 0
        self.alias = 0                  # nb de sauts > ALIAS_DEG (diagnostic)

    # --- etage 1 : image -> profil ------------------------------------------
    def sample(self, frame):
        """Profil angulaire (N,) : moyenne sur 3 cercles concentriques.

        Indexe DIRECTEMENT le BGR (pas de cvtColor plein cadre) et moyenne les
        canaux : seul le contraste local compte, les poids luma n'apporteraient
        rien et coutaient ~1 ms par frame.
        """
        vals = frame[self._iy, self._ix]        # (3, N) en gris, (3, N, 3) en BGR
        if vals.ndim == 3:
            return vals.mean(axis=(0, 2))
        return vals.mean(axis=0)

    # --- etage 2 : profil -> angle -> RPM (partage avec le rejeu) -----------
    def push(self, prof, t, prom_min=PROM_MIN, window_s=FIT_WINDOW_S):
        """Un echantillon : pic du profil -> angle -> angle cumule -> RPM."""
        i = int(np.argmax(prof))
        self.quality = float(prof[i] - np.median(prof))
        if self.quality < prom_min:
            # marque invisible (roue masquee, reflet perdu, ROI decalee) : on COUPE
            # l'historique plutot que d'inventer un pas -- une interpolation
            # silencieuse ferait un RPM faux, ce qui est pire qu'un trou.
            self.lost += 1
            if self.lost >= LOST_MAX:
                self.ok, self.rpm, self.theta = False, None, None
                self.hist.clear()
            return
        self.lost = 0
        self.ok = True
        theta = _refine(prof, i) * (360.0 / N_ANGLES)
        if self.theta is not None:
            d = _wrap180(theta - self.theta)
            if abs(d) > ALIAS_DEG:
                self.alias += 1          # trop rapide pour la cadence : signale
            self.cum += d
        self.theta = theta
        self.hist.append((t, self.cum))
        while self.hist and t - self.hist[0][0] > window_s:
            self.hist.popleft()
        self.rpm = _slope_rpm(self.hist)

    def asdict(self):
        return {"id": self.id, "cx": self.cx, "cy": self.cy, "r": self.r,
                "r_mark": round(self.r_mark, 1),
                "theta": None if self.theta is None else round(self.theta, 1),
                "rpm": None if self.rpm is None else round(self.rpm, 1),
                "quality": round(self.quality, 1), "ok": self.ok,
                "alias": self.alias}


# ---------------------------------------------------------------------------
# Helpers numeriques
# ---------------------------------------------------------------------------
def _wrap180(d):
    """Ramene un ecart d'angle dans ]-180, +180] (deroulage)."""
    return (d + 180.0) % 360.0 - 180.0


def _refine(prof, i):
    """Position sous-echantillon du pic par parabole sur (i-1, i, i+1), CIRCULAIRE."""
    n = len(prof)
    a, b, c = float(prof[(i - 1) % n]), float(prof[i]), float(prof[(i + 1) % n])
    den = a - 2.0 * b + c
    if den == 0.0:
        return float(i)
    return i + 0.5 * (a - c) / den


def _slope_rpm(hist):
    """Pente (deg/s) de l'angle cumule par moindres carres -> RPM signe.

    La regression (et non un ecart entre deux frames) est ce qui absorbe la gigue
    d'horodatage, qui est l'erreur dominante devant la resolution angulaire.
    """
    if len(hist) < FIT_MIN_PTS:
        return None
    t = np.fromiter((h[0] for h in hist), dtype=np.float64, count=len(hist))
    y = np.fromiter((h[1] for h in hist), dtype=np.float64, count=len(hist))
    t = t - t[0]
    var = float(((t - t.mean()) ** 2).sum())
    if var <= 1e-9:
        return None
    slope = float(((t - t.mean()) * (y - y.mean())).sum() / var)
    return slope / 6.0                   # deg/s -> tr/min (/360*60)


def _ring_index(cx, cy, radius, w, h):
    """Indices (3, N) des pixels du cercle de rayon `radius` (3 rayons voisins).

    L'index angulaire k porte la direction (cos a, -sin a), a = 2*pi*k/N : y etant
    vers le BAS dans l'image, un k croissant tourne donc ANTI-HORAIRE a l'ecran.
    """
    a = np.arange(N_ANGLES, dtype=np.float64) * (2.0 * np.pi / N_ANGLES)
    ca, sa = np.cos(a), np.sin(a)
    rs = np.array(RING_RADII, dtype=np.float64).reshape(-1, 1) + radius
    xs = np.clip(np.rint(cx + rs * ca).astype(np.int32), 0, w - 1)
    ys = np.clip(np.rint(cy - rs * sa).astype(np.int32), 0, h - 1)
    return ys, xs


def analyze_dump(path, prom_min=PROM_MIN, window_s=FIT_WINDOW_S):
    """Rejeu HORS LIGNE d'un enregistrement de profils -> series de RPM.

    Rejoue `_Wheel.push`, exactement le calcul du direct : on peut donc rebalayer
    `prom_min` et `window_s` sur une acquisition unique, sans retourner au robot.
    Rend {"<id roue>": {"t": [...], "rpm": [...], "cum": [...], "quality": [...]}}.
    """
    d = np.load(path, allow_pickle=False)
    times = d["t"]
    out = {}
    for key in (k for k in d.files if k.startswith("prof_")):
        wid = key[5:]
        wheel = _Wheel(wid, 0.0, 0.0, 0.0, 0.0)
        ts, rpms, cums, qs = [], [], [], []
        for t, prof in zip(times, d[key].astype(np.float64)):
            wheel.push(prof, float(t), prom_min=prom_min, window_s=window_s)
            ts.append(float(t))
            rpms.append(wheel.rpm)
            cums.append(wheel.cum)
            qs.append(wheel.quality)
        out[wid] = {"t": ts, "rpm": rpms, "cum": cums, "quality": qs,
                    "alias": wheel.alias}
    return out


class WheelTachoNode(Node):
    """Node tachymetre optique : /camera/image -> /tacho/state (RPM par roue).

    Inactif par defaut (`active=False`) : cout nul tant qu'on ne l'arme pas. Une
    fois arme, le premier tour calibre (Hough) puis chaque tour mesure.
    """

    def __init__(self, active=False, roi=None, telemetry=None, dump_dir=None):
        super().__init__("wheel_tacho")
        self._tel = telemetry
        self.active = bool(active)
        self.roi = tuple(roi) if roi else ROI_DEFAULT
        self._dump_dir = dump_dir or getattr(telemetry, "log_dir", None) or "."
        self.in_img = self.create_input("/camera/image", ImageMsg)
        self.in_cfg = self.create_input("/tacho/config", WheelTachoConfig)
        self.out = self.create_output("/tacho/state", WheelTachoState)
        self._wheels = []
        self._cfg_seq = -1
        self._last_seq = -1              # garde anti-encre : seq d'image traitee
        self._want_calib = False
        self._calib_acc = []             # cercles de Hough en cours d'agregation
        self._note = "desarme"
        self._seq = 0
        self._n, self._t0, self.fps = 0, time.time(), 0.0
        # --- enregistrement de profils (analyse a posteriori) ---
        self._rec_until = 0.0            # date de fin de capture (0 = inactif)
        self._rec_t = []                 # horodatages
        self._rec_p = {}                 # id roue -> liste de profils uint8
        self.last_dump = None            # chemin du dernier .npz ecrit

    # --- config (clavier/MCP) ----------------------------------------------
    def _apply_config(self, cfg: WheelTachoConfig):
        if cfg.roi is not None and tuple(cfg.roi) != self.roi:
            self.roi = tuple(cfg.roi)
            self._wheels = []             # ROI changee -> geometrie a refaire
            self._want_calib = True
            self._calib_acc = []
        if cfg.active is not None and bool(cfg.active) != self.active:
            self.active = bool(cfg.active)
            if self.active:
                self._want_calib = True
                self._calib_acc = []   # (re)calibre a chaque armement
            else:
                self._stop_record()
                self._wheels, self._note = [], "desarme"
            self._log("tacho", on=self.active)
        if cfg.calibrate:
            self._want_calib = True
            self._calib_acc = []
            self._log("tacho_calibrate")
        if cfg.record_s:
            self._start_record(float(cfg.record_s))

    def _log(self, msg, **fields):
        if self._tel is not None:
            self._tel.log("event", msg=msg, source="tacho", **fields)

    # --- enregistrement pour analyse a posteriori ---------------------------
    def _start_record(self, seconds):
        """Arme une capture de profils horodates (PAS d'images : 11 ko/s)."""
        seconds = max(0.5, min(RECORD_MAX_S, seconds))
        self._rec_t, self._rec_p = [], {w.id: [] for w in self._wheels}
        self._rec_until = time.time() + seconds
        self._log("tacho_record_start", seconds=round(seconds, 1))

    def _stop_record(self):
        """Ferme la capture et ecrit le .npz (rejouable par analyze_dump)."""
        if not self._rec_until:
            return
        self._rec_until = 0.0
        if not self._rec_t:
            self._log("tacho_record_empty")
            return
        path = os.path.join(self._dump_dir,
                            "tacho_%s.npz" % time.strftime("%Y%m%d_%H%M%S"))
        try:
            arrays = {"prof_" + wid: np.asarray(v, dtype=np.uint8)
                      for wid, v in self._rec_p.items()}
            np.savez_compressed(path, t=np.asarray(self._rec_t, dtype=np.float64),
                                **arrays)
            self.last_dump = path
            self._log("tacho_record_done", path=path, samples=len(self._rec_t))
        except Exception as exc:
            self._log("tacho_record_fail", err=str(exc))
        self._rec_t, self._rec_p = [], {}

    # --- calibration --------------------------------------------------------
    def _calibrate(self, frame):
        """Trouve les 2 roues (Hough, UNE fois) + le rayon de marque de chacune.

        Seul endroit ou l'on convertit en gris, et seulement sur la ROI : Hough et
        la recherche du rayon de marque ne tournent qu'ici.
        """
        h, w = frame.shape[:2]
        x0 = int(self.roi[0] * w)
        y0 = int(self.roi[1] * h)
        x1 = max(x0 + 8, int(self.roi[2] * w))
        y1 = max(y0 + 8, int(self.roi[3] * h))
        crop = frame[y0:y1, x0:x1]
        if crop.ndim == 3:
            crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        sub = cv2.medianBlur(crop, 5)
        circles = cv2.HoughCircles(sub, cv2.HOUGH_GRADIENT, **HOUGH)
        if circles is None or len(circles[0]) < 2:
            n = 0 if circles is None else len(circles[0])
            self._wheels = []
            self._note = "calibration : %d cercle(s) dans la ROI (2 attendus)" % n
            self._log("tacho_calib_fail", found=n, roi=list(self.roi))
            return False
        # HoughCircles rend les cercles par accumulateur DECROISSANT : les 2
        # premiers sont les plus surs. On les ordonne ensuite par x (image).
        pair = sorted(circles[0][:2].tolist(), key=lambda c: c[0])
        # AGREGATION SUR PLUSIEURS FRAMES, par roue. Un rayon sous-estime (jante au
        # lieu de pneu) fait tomber le carre de papier HORS du disque analyse, et la
        # marque suivie devient alors un reflet du moyeu : une tache IMMOBILE, donc un
        # RPM nul indiscernable d'une roue bloquee.
        self._calib_acc.append(pair)
        if len(self._calib_acc) < CALIB_FRAMES:
            self._note = "calibration : %d/%d frames" % (len(self._calib_acc), CALIB_FRAMES)
            # Il FAUT redemander la calibration et vider _wheels : process() n'appelle
            # _calibrate que si _want_calib est arme ou si _wheels est vide, et il
            # desarme le drapeau avant d'appeler. Sans ces deux lignes, une
            # RECALIBRATION (roues deja connues) s'arretait a 1/12 pour toujours, tout
            # en continuant a mesurer sur l'ancienne geometrie et a se declarer
            # calibree -- une panne muette, constatee sur le banc.
            self._wheels = []
            self._want_calib = True
            return False
        acc = np.array(self._calib_acc, dtype=np.float64)   # (frames, 2 roues, 3)
        self._calib_acc = []
        # Mediane sur les frames, composante par composante : robuste a la frame ou
        # Hough decale le cercle, et sans melanger les deux roues (cf. CALIB_FRAMES).
        best = [(float(np.median(acc[:, i, 0])), float(np.median(acc[:, i, 1])),
                 float(np.median(acc[:, i, 2]))) for i in (0, 1)]
        # Le rayon de marque se cherche sur le gris de la ROI, en coordonnees ROI ;
        # les indices d'echantillonnage, eux, sont en coordonnees IMAGE.
        self._wheels = []
        marks = {}
        for wid, (cx, cy, r) in zip(("G", "D"), best):
            r_mark, th0 = self._find_mark(crop, float(cx), float(cy), float(r))
            marks[wid] = round(th0, 1)
            gx, gy = float(cx) + x0, float(cy) + y0
            iy, ix = _ring_index(gx, gy, r_mark, w, h)
            self._wheels.append(_Wheel(wid, round(gx, 1), round(gy, 1),
                                       round(float(r), 1), r_mark, iy, ix))
        self._note = "calibree"
        self._log("tacho_calib_ok",
                  wheels=[{"id": x.id, "cx": x.cx, "cy": x.cy, "r": x.r,
                           "r_mark": round(x.r_mark, 1), "theta0": marks[x.id]}
                          for x in self._wheels])
        return True

    @staticmethod
    def _find_mark(gray, cx, cy, r_wheel):
        """(rayon, angle) de la PLUS GROSSE TACHE BLANCHE du disque de la roue.

        Le critere est la SURFACE, pas la saillance, et ce n'est pas un detail :
        mesure du banc, le reflet speculaire du moyeu est PLUS saillant que le
        papier (162 contre 155 niveaux) tout en etant beaucoup plus petit. Choisir
        par saillance faisait donc suivre le reflet -- une tache IMMOBILE, donc un
        RPM nul indiscernable d'une roue bloquee : exactement la panne que ce node
        doit rendre visible. Choisir par surface designe le carre de papier.

        Le seuil de blanc est RELATIF (centre + frac x amplitude du disque) : aucun
        niveau absolu, donc insensible a l'eclairage general -- le seuillage absolu
        avait ete essaye et noye par le texte du HUD.
        """
        h, w = gray.shape[:2]
        # Disque de la roue seulement : le sol clair et le chassis blanc sont juste
        # a cote et gagneraient tous les concours de surface.
        yy, xx = np.ogrid[0:h, 0:w]
        rr2 = (xx - cx) ** 2 + (yy - cy) ** 2
        disc = rr2 <= (MARK_R_MAX_FRAC * r_wheel) ** 2
        vals = gray[disc]
        if vals.size < 32:
            return 0.55 * r_wheel, 0.0
        lo, hi = float(np.median(vals)), float(vals.max())
        thr = lo + MARK_WHITE_FRAC * (hi - lo)
        mask = np.zeros(gray.shape[:2], dtype=np.uint8)
        mask[disc & (gray >= thr)] = 255
        # Fermeture : le carre de papier est traverse de reflets et de crans de pneu,
        # sans quoi il se fragmente et perd le concours de surface face a un reflet net.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best = None
        for k in range(1, n):
            mx, my = float(cent[k][0]), float(cent[k][1])
            rad = float(np.hypot(mx - cx, my - cy))
            if not (MARK_R_MIN_FRAC * r_wheel <= rad <= MARK_R_MAX_FRAC * r_wheel):
                continue
            area = int(stats[k, cv2.CC_STAT_AREA])
            box = (int(stats[k, cv2.CC_STAT_WIDTH]) * int(stats[k, cv2.CC_STAT_HEIGHT])) or 1
            if area / box < MARK_FILL_MIN:
                continue          # creux -> plage de sol ou d'ombre, pas un carre
            if best is None or area > best[0]:
                best = (area, rad, mx, my)
        if best is None:
            return 0.55 * r_wheel, 0.0
        _, rad, mx, my = best
        theta = float(np.degrees(np.arctan2(cy - my, mx - cx)) % 360.0)
        return rad, theta

    # --- cycle ROS ----------------------------------------------------------
    def process(self):
        cfg = self.in_cfg.get()
        if cfg is not None and cfg.seq != self._cfg_seq:
            self._cfg_seq = cfg.seq
            self._apply_config(cfg)

        if not self.active:
            return

        img = self.in_img.get()
        if img is None or img.frame is None:
            return
        # GARDE ANTI-ENCRE : seq deja traitee = image que le Core a deja annotee
        # (CameraNode ne republie pas quand la lecture echoue) -> on ne mesure pas.
        if img.seq == self._last_seq:
            return
        self._last_seq = img.seq
        frame = img.frame

        if self._want_calib or not self._wheels:
            self._want_calib = False
            if not self._calibrate(frame):
                self._publish()
                return

        t = img.stamp or time.time()
        profs = {}
        for wheel in self._wheels:
            prof = wheel.sample(frame)
            wheel.push(prof, t)
            profs[wheel.id] = prof

        # Capture : on stocke les PROFILS (180 o/roue), jamais les images.
        if self._rec_until:
            self._rec_t.append(t)
            for wid, prof in profs.items():
                self._rec_p.setdefault(wid, []).append(
                    np.clip(prof, 0, 255).astype(np.uint8))
            if time.time() >= self._rec_until:
                self._stop_record()

        self._n += 1
        now = time.time()
        if now - self._t0 >= 1.0:
            self.fps = self._n / (now - self._t0)
            self._t0, self._n = now, 0
        self._publish()

    def _publish(self):
        self._seq += 1
        rec = 0.0
        if self._rec_until:
            rec = max(0.0, round(self._rec_until - time.time(), 1))
        self.out.set(WheelTachoState(
            seq=self._seq, active=self.active, calibrated=bool(self._wheels),
            wheels=[w.asdict() for w in self._wheels],
            fps=round(self.fps, 1), roi=self.roi, note=self._note,
            recording=rec, dump=self.last_dump))

    def on_stop(self):
        self._stop_record()
        self._wheels = []

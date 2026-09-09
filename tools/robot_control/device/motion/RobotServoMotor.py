r"""RobotServoMotor - Servos camera S1 (pan) / S2 (tilt) + asservissement de suivi.

Portage de l'ancien pan_tilt.PanTilt (heritier de RobotWebCamMotorized,
Bambou4WD_python 2018) vers le style RobotXxx/camelCase. Comportement INCHANGE :
asservissement proportionnel sur l'erreur NORMALISEE (nx, ny) dans [-1, +1]
fournie par la vision (independant de la resolution), zone morte rectangulaire +
hysteresis, et lisseur (slew) a profil de vitesse anti-flou.

Cablage v4 (deja teste via l'outil MCP pwm_servo) :
  PAN  = S1 (id 1), repos 88 deg (cam de face, centre de course), bornes 17..178
  TILT = S2 (id 2), repos 42 deg (cam de face), bornes 20..70
(repos mesures au banc ; toutes bornes reglables via RobotMain --pan/tilt-min/max/home).
"""


class RobotServoMotor:
    def __init__(self, link, panId=1, tiltId=2,
                 panGain=18.0, tiltGain=10.0,
                 invertPan=False, invertTilt=False,
                 deadzone=0.08, deadHyst=0.05, maxStep=6,
                 panMin=17.0, panMax=178.0, panHome=88.0,
                 tiltMin=20.0, tiltMax=70.0, tiltHome=42.0,
                 maxVel=120.0, maxAccel=400.0, smooth=True,
                 telemetry=None):
        self.link = link
        self.tel = telemetry
        self.panId = panId
        self.tiltId = tiltId

        # Bornes et repos (deg) - reglables (butees physiques dependant du montage)
        self.panMin, self.panMax = panMin, panMax
        self.tiltMin, self.tiltMax = tiltMin, tiltMax
        self.panHome, self.tiltHome = panHome, tiltHome
        self.angleH = panHome         # pan (position REELLE envoyee au servo)
        self.angleV = tiltHome        # tilt (repos tiltHome)

        # Asservissement
        self.panGain = panGain        # deg de correction pour une erreur nx=1
        self.tiltGain = tiltGain
        self.sgnPan = -1.0 if invertPan else 1.0
        self.sgnTilt = -1.0 if invertTilt else 1.0
        # Zone morte = demi-cote de la SURFACE centrale (rectangle) : dans cette
        # boite, on ne bouge pas (cible = surface, pas un point -> evite le hunting).
        self.deadzone = deadzone
        # Hysteresis : une fois stabilise DANS la boite, il faut sortir de la boite
        # elargie (deadzone + deadHyst) pour re-enclencher -> supprime le chatter de bord.
        self.deadHyst = deadHyst
        self.maxStep = float(maxStep)     # deg max de correction visee par detection

        # --- lissage du mouvement (profil de vitesse : anti-flou de bouge) --------
        # La detection ne bouge plus le servo directement : elle fixe une CIBLE.
        # Un lisseur (slew) rapproche la position REELLE de la cible a chaque image,
        # borne par une vitesse ET une acceleration max -> demarrage progressif,
        # vitesse de croisiere, freinage a l'approche (positions intermediaires dont
        # l'espacement suit la vitesse). Reduit le flou qui fait perdre le suivi.
        self.smooth = smooth
        self.maxVel = float(maxVel)       # deg/s : vitesse angulaire max du servo
        self.maxAccel = float(maxAccel)   # deg/s^2 : montee/descente en vitesse

        self._lastPan = None
        self._lastTilt = None
        self._settled = {"pan": False, "tilt": False}  # etat hysteresis par axe
        # cible (deg) et vitesse courante (deg/s) par axe, pour le lisseur
        self._target = {"pan": panHome, "tilt": tiltHome}
        self._vel = {"pan": 0.0, "tilt": 0.0}
        # indicateur de fluidite : plus GRAND pas entre deux positions calculees
        # (deg/image). Un pic eleve = un "coup de butoir". Peak-hold a decroissance
        # (~0.3 s) pour rester lisible en direct ; step brut du dernier cycle aussi.
        self._stepPeak = {"pan": 0.0, "tilt": 0.0}
        self._stepLast = {"pan": 0.0, "tilt": 0.0}

    # --- envoi (uniquement si l'angle a change) -----------------------------
    def _pushPan(self):
        """Envoie l'angle pan si change. Retourne True si une trame est partie."""
        a = int(round(self.angleH))
        if a != self._lastPan:
            self.link.sendServo(self.panId, a)
            self._lastPan = a
            return True
        return False

    def _pushTilt(self):
        a = int(round(self.angleV))
        if a != self._lastTilt:
            self.link.sendServo(self.tiltId, a)
            self._lastTilt = a
            return True
        return False

    def _clampH(self):
        self.angleH = max(self.panMin, min(self.panMax, self.angleH))

    def _clampV(self):
        self.angleV = max(self.tiltMin, min(self.tiltMax, self.angleV))

    def _syncTarget(self):
        """Cible = position reelle, vitesse nulle : le lisseur cesse de pousser.

        Appele apres tout mouvement MANUEL (nudge/center) ou init pour que le
        profil de vitesse reparte de l'etat courant sans a-coup ni rattrapage.
        """
        self._target["pan"] = self.angleH
        self._target["tilt"] = self.angleV
        self._vel["pan"] = 0.0
        self._vel["tilt"] = 0.0

    # --- commandes -----------------------------------------------------------
    def center(self):
        self.angleH, self.angleV = self.panHome, self.tiltHome
        self._pushPan()
        self._pushTilt()
        self._syncTarget()

    def apply(self):
        """(Re)positionne les 2 servos a l'etat courant (init)."""
        self._clampH()
        self._clampV()
        self._pushPan()
        self._pushTilt()
        self._syncTarget()

    def nudgePan(self, delta):
        self.angleH += delta
        self._clampH()
        self._pushPan()
        self._target["pan"] = self.angleH
        self._vel["pan"] = 0.0

    def nudgeTilt(self, delta):
        self.angleV += delta
        self._clampV()
        self._pushTilt()
        self._target["tilt"] = self.angleV
        self._vel["tilt"] = 0.0

    def returnHome(self):
        """Vise le repos (panHome/tiltHome) en laissant le lisseur y glisser.

        Utilise quand la prediction abandonne (coast expire, visage non retrouve) :
        on ne SAUTE pas au centre, on pose la cible = repos et slew() l'atteint en
        douceur (meme profil de vitesse que le suivi). Idempotent : peut etre
        appele a chaque image sans a-coup (ne touche pas la vitesse courante). En
        mode non lisse, applique directement le repos (coherent avec track()).
        """
        self._settled["pan"] = self._settled["tilt"] = False
        self._target["pan"] = self.panHome
        self._target["tilt"] = self.tiltHome
        if not self.smooth:
            self.angleH, self.angleV = self.panHome, self.tiltHome
            self._clampH(); self._clampV()
            self._pushPan(); self._pushTilt()
            self._vel["pan"] = self._vel["tilt"] = 0.0

    def track(self, nx, ny):
        """Fixe la CIBLE pan/tilt pour amener le visage dans la SURFACE centrale.

        nx > 0 : visage a DROITE de l'image ; ny > 0 : visage en BAS.
        Ne bouge PAS le servo directement : pose une cible que le lisseur (slew)
        rejoint en douceur. La cible n'est pas le point central mais un RECTANGLE
        central (demi-cote = deadzone) : tant que le visage y est, la cible = la
        position courante (arret). Anti-oscillation (inchange) :
          - erreur ATTENUEE : on ne vise que la part d'erreur au-dela du rectangle,
            donc la correction tend vers 0 au bord (pas de depassement du centre) ;
          - HYSTERESIS : stabilise dans le rectangle, il faut ressortir de la boite
            elargie (deadzone + deadHyst) pour re-enclencher.
        En mode non lisse (smooth=False), applique le pas immediatement (ancien
        comportement) pour comparaison.
        """
        if nx is None or ny is None:
            return
        self._trackAxis("pan", nx)
        self._trackAxis("tilt", ny)

    def _trackAxis(self, axis, e):
        if axis == "pan":
            gain, sgn = self.panGain, self.sgnPan
            before = self.angleH
        else:
            gain, sgn = self.tiltGain, self.sgnTilt
            before = self.angleV
        dz = self.deadzone
        # seuil d'engagement : boite elargie si deja stabilise (hysteresis)
        thresh = dz + self.deadHyst if self._settled[axis] else dz
        if abs(e) <= thresh:
            self._settled[axis] = True          # visage dans la surface centrale
            self._target[axis] = before         # cible = position courante -> arret
            if self.tel:
                self.tel.log("track", axis=axis, err=round(e, 4),
                             skipped="deadzone", settled=True)
            return
        self._settled[axis] = False
        # erreur utile = part au-dela du rectangle (correction -> 0 au bord)
        eff = e - (dz if e > 0 else -dz)
        raw = sgn * gain * eff
        # deplacement vise borne (evite qu'une detection bruitee jette la cible loin)
        step = max(-self.maxStep, min(self.maxStep, raw))
        lo, hi = (self.panMin, self.panMax) if axis == "pan" else (self.tiltMin, self.tiltMax)
        target = max(lo, min(hi, before + step))
        self._target[axis] = target
        if not self.smooth:
            # ancien comportement : saut immediat (pour comparer avec/sans lissage)
            if axis == "pan":
                self.angleH = target; self._clampH(); self._pushPan()
            else:
                self.angleV = target; self._clampV(); self._pushTilt()
            self._vel[axis] = 0.0
        if self.tel:
            self.tel.log("track", axis=axis, err=round(e, 4), eff=round(eff, 4),
                         gain=gain, step_raw=round(raw, 3), step=round(step, 3),
                         before=round(before, 1), target=round(target, 1),
                         clamped=abs(raw) > self.maxStep)

    def slew(self, dt):
        """Rapproche la position REELLE de la cible avec profil de vitesse.

        Appele a chaque image (P1, ~30 Hz). Pour chaque axe :
          - vitesse desiree = vers la cible, bornee par maxVel ET par la vitesse
            de freinage sqrt(2*a*distance) qui permet de s'arreter PILE sur la cible
            (decele a l'approche -> pas de depassement) ;
          - la vitesse reelle suit la desiree a +-maxAccel*dt pres (demarrage/arret
            progressifs -> pas de saut brutal qui floute l'image) ;
          - position += vitesse*dt, bornee ; trame servo envoyee si l'angle entier
            a change. L'espacement des positions intermediaires suit donc la vitesse.
        Sans effet si smooth=False. Retourne True si une trame est partie.
        """
        if not self.smooth or dt <= 0.0:
            return False
        sent = False
        # decroissance du peak-hold : ~0.3 s de memoire quel que soit le fps
        decay = 0.5 ** (dt / 0.3)
        for axis in ("pan", "tilt"):
            self._stepPeak[axis] *= decay
            cur0 = self.angleH if axis == "pan" else self.angleV
            cur = cur0
            d = self._target[axis] - cur
            v = self._vel[axis]
            if abs(d) < 0.05 and abs(v) < 1.0:
                self._vel[axis] = 0.0
                self._stepLast[axis] = 0.0
                continue
            direction = 1.0 if d > 0 else -1.0
            v_brake = (2.0 * self.maxAccel * abs(d)) ** 0.5   # freinage pour stop sur cible
            v_des = direction * min(self.maxVel, v_brake)
            dv = v_des - v
            dv = max(-self.maxAccel * dt, min(self.maxAccel * dt, dv))
            v += dv
            cur += v * dt
            # depassement de la cible -> on la cale et on annule la vitesse
            if (self._target[axis] - cur) * direction < 0:
                cur = self._target[axis]
                v = 0.0
            if axis == "pan":
                self.angleH = cur
                self._clampH()
                if self.angleH in (self.panMin, self.panMax):
                    v = 0.0
                if self._pushPan():
                    sent = True
                applied = abs(self.angleH - cur0)
            else:
                self.angleV = cur
                self._clampV()
                if self.angleV in (self.tiltMin, self.tiltMax):
                    v = 0.0
                if self._pushTilt():
                    sent = True
                applied = abs(self.angleV - cur0)
            self._vel[axis] = v
            # distance reellement parcourue cette image = pas entre 2 points calcules
            self._stepLast[axis] = applied
            if applied > self._stepPeak[axis]:
                self._stepPeak[axis] = applied
        return sent

    def motionStats(self):
        """Indicateur de fluidite du mouvement.

        step_peak = plus grand pas (deg/image) recent entre deux positions
        calculees, par axe (peak-hold ~0.3 s). Un pic eleve = un "coup de butoir".
        vel = vitesse courante (deg/s). step_max = pire des deux axes.
        """
        return {
            "step_peak_pan": self._stepPeak["pan"],
            "step_peak_tilt": self._stepPeak["tilt"],
            "step_max": max(self._stepPeak["pan"], self._stepPeak["tilt"]),
            "vel_pan": self._vel["pan"],
            "vel_tilt": self._vel["tilt"],
        }

r"""RobotMotorDrive - Commandes moteurs haut niveau (chassis 4 roues Bamboo v4).

Portage de l'ancien motion.Motion (style Bambou4WD_python : RobotXxx, camelCase,
setters fluides). Comportement INCHANGE : traduit une consigne (throttle
avant/arriere, turn rotation) en 4 PWM signes envoyes via RobotComSerial.sendMotor.
La table de signes marche-avant [+1,-1,-1,+1] provient de t_motor_drive_all
(ros_mcp_server.py) : c'est le vecteur "avancer" du chassis.

/!\ A VERIFIER ROUES SURELEVEES puis ajuster les signes si un moteur tourne a
l'envers. Notes materiel connues : M3 sans marche arriere, M2 encodeur HS
(sans effet ici : teleop en PWM boucle ouverte, pas d'odometrie requise).
"""

# Vecteur marche avant : signe applique a chaque moteur pour "avancer".
FORWARD_SIGN = (+1, -1, -1, +1)

# Vecteur rotation (tourner sur place, sens horaire vu de dessus) : cote gauche
# vs cote droit. Hypothese M1/M2 = un cote, M3/M4 = l'autre -> a confirmer et
# ajuster au banc. Une rotation = avancer un cote, reculer l'autre.
TURN_SIGN = (+1, +1, -1, -1)


class RobotMotorDrive:
    """Melange throttle/turn -> 4 PWM signes, avec borne de securite."""

    def __init__(self, link, maxPwm=30):
        self.link = link
        self.maxPwm = max(1, min(100, int(maxPwm)))
        self.speedLevel = 3           # 0..9, echelonne l'amplitude du throttle

    def setSpeed(self, level):
        self.speedLevel = max(0, min(9, int(level)))
        return self

    def _amp(self):
        """Amplitude PWM courante (%) selon le niveau de vitesse (0..9)."""
        return int(round(self.maxPwm * self.speedLevel / 9.0))

    def _mix(self, throttle, turn):
        """throttle, turn dans [-1, +1] -> tuple (m1..m4) PWM signes bornes."""
        amp = self._amp()
        vals = []
        for i in range(4):
            v = FORWARD_SIGN[i] * throttle + TURN_SIGN[i] * turn
            v = max(-1.0, min(1.0, v))            # sature avant mise a l'echelle
            vals.append(int(round(amp * v)))
        return tuple(vals)

    def drive(self, throttle, turn):
        """Envoie une consigne melangee (throttle avant+, turn horaire+)."""
        return self.link.sendMotor(*self._mix(throttle, turn))

    # --- raccourcis teleop --------------------------------------------------
    def forward(self):
        return self.drive(+1.0, 0.0)

    def backward(self):
        return self.drive(-1.0, 0.0)

    def rotateLeft(self):
        return self.drive(0.0, -1.0)

    def rotateRight(self):
        return self.drive(0.0, +1.0)

    def stop(self):
        return self.link.stop()

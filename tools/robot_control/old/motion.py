r"""motion - Commandes moteurs haut niveau pour le Bamboo v4 (chassis 4 roues).

Traduit une consigne (throttle avant/arriere, turn rotation) en 4 PWM signes
envoyes via FUNC_MOTOR. La table de signes marche-avant [+1,-1,-1,+1] provient
de t_motor_drive_all (ros_mcp_server.py) : c'est le vecteur "avancer" du chassis.

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


class Motion:
    """Melange throttle/turn -> 4 PWM signes, avec borne de securite."""

    def __init__(self, link, max_pwm=30):
        self.link = link
        self.max_pwm = max(1, min(100, int(max_pwm)))
        self.speed_level = 3          # 0..9, echelonne l'amplitude du throttle

    def set_speed(self, level):
        self.speed_level = max(0, min(9, int(level)))
        return self

    def _amp(self):
        """Amplitude PWM courante (%) selon le niveau de vitesse (0..9)."""
        return int(round(self.max_pwm * self.speed_level / 9.0))

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
        return self.link.send_motor(*self._mix(throttle, turn))

    # --- raccourcis teleop --------------------------------------------------
    def forward(self):
        return self.drive(+1.0, 0.0)

    def backward(self):
        return self.drive(-1.0, 0.0)

    def rotate_left(self):
        return self.drive(0.0, -1.0)

    def rotate_right(self):
        return self.drive(0.0, +1.0)

    def stop(self):
        return self.link.stop()

r"""RobotMotorDrive - Consigne de deplacement haut niveau (chassis 4 roues Bamboo v4).

Modele « etat combine type ROS » : l'objet ne pilote plus des PWM bruts, il PORTE
un etat `cmd_vel` (Twist) memorise -- `linear.x` (m/s, avant +) et `angular.z`
(rad/s, anti-horaire +) -- que l'application republie en continu (~10 Hz) via
RobotComSerial.sendCmdVel (FUNC_MOTION 0x12). La carte STM32 fait la kinematics
differentielle embarquee + le PID par roue : on ne calcule donc AUCUN melange de
signes ni de mapping roue cote hote (tout est gere par Fourwheel_Ctrl firmware).

Les fleches AJUSTENT cet etat (nudgeLinear/nudgeAngular), Espace/coupure le remet
a zero (reset/stop). On peut avancer ET tourner en meme temps -> arcs.

Prerequis carte : type chassis CAR_FOURWHEEL (set_car_type 4) pour router sur la
kinematics 4 roues, et geometrie roue reglee (set_wheel_geom). M2 (encodeur HS)
est asservi en recopie sur sa roue de meme ligne (M4, recopie inversee) cote
firmware (sentinelle PID).
"""

# Plafonds de securite (doivent rester <= aux bornes de sendCmdVel : 1 m/s, 2 rad/s).
MAX_LIN = 1.0      # m/s   (plafond firmware ~1000 mm/s)
MAX_ANG = 2.0      # rad/s (borne sendCmdVel a +-2000 mrad/s)


class RobotMotorDrive:
    """Porteur d'etat cmd_vel (linear.x / angular.z) republie par l'application."""

    def __init__(self, link, maxPwm=30):
        self.link = link
        # maxPwm garde pour compat d'appel ; sans effet en boucle fermee (PID carte).
        self.maxPwm = max(1, min(100, int(maxPwm)))
        self.speedLevel = 3           # 0..9 : echelonne le PAS d'increment des fleches
        self.lin = 0.0                # linear.x courant (m/s)
        self.ang = 0.0                # angular.z courant (rad/s)

    def setSpeed(self, level):
        self.speedLevel = max(0, min(9, int(level)))
        return self

    # --- pas d'increment (croit avec le niveau de vitesse) ------------------
    def _stepLin(self):
        return MAX_LIN * (self.speedLevel + 1) / 20.0     # 0.05 .. 0.5 m/s par appui

    def _stepAng(self):
        return MAX_ANG * (self.speedLevel + 1) / 20.0     # 0.1 .. 1.0 rad/s par appui

    # --- ajustement de l'etat cmd_vel ---------------------------------------
    def nudgeLinear(self, sign):
        """Incremente linear.x (+1 avant / -1 arriere), borne a +-MAX_LIN."""
        self.lin = max(-MAX_LIN, min(MAX_LIN, self.lin + sign * self._stepLin()))
        return self

    def nudgeAngular(self, sign):
        """Incremente angular.z (+1 anti-horaire / -1 horaire), borne a +-MAX_ANG."""
        self.ang = max(-MAX_ANG, min(MAX_ANG, self.ang + sign * self._stepAng()))
        return self

    def reset(self):
        """Remet l'etat cmd_vel a zero (sans emettre : le republieur enverra 0)."""
        self.lin = 0.0
        self.ang = 0.0
        return self

    def isMoving(self):
        return self.lin != 0.0 or self.ang != 0.0

    # --- emission -----------------------------------------------------------
    def publish(self):
        """Republie l'etat cmd_vel courant (a appeler ~10 Hz tant que moteurs armes)."""
        return self.link.sendCmdVel(self.lin, self.ang)

    def stop(self):
        """Arret franc : etat a zero + cmd_vel(0,0) envoye plusieurs fois (Motion_Stop
        cote carte remet g_start_ctrl=0). Fiable meme en pleine boucle fermee."""
        self.reset()
        ok = False
        for _ in range(3):
            ok = self.link.sendCmdVel(0.0, 0.0) or ok
        return ok

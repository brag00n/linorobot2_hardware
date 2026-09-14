r"""RobotMotorDrive - Consigne de deplacement haut niveau (chassis 4 roues Bamboo v4).

Modele « etat combine type ROS » : l'objet ne pilote plus des PWM bruts, il PORTE
un etat `cmd_vel` (Twist) memorise -- `linear.x` (m/s, avant +) et `angular.z`
(rad/s, anti-horaire +) -- que l'application republie en continu (~10 Hz) via
RobotComSerial.sendCmdVel (FUNC_MOTION 0x12). La carte STM32 fait la kinematics
differentielle embarquee + le PID par roue : on ne calcule donc AUCUN melange de
signes ni de mapping roue cote hote (tout est gere par Fourwheel_Ctrl firmware).

Modele d'entree « tenue » : les fleches ne s'accumulent plus. L'application sonde
l'etat PHYSIQUE des fleches a chaque tour de boucle et appelle holdVelocity(fwd, turn)
-> l'etat cmd_vel vaut la vitesse PROGRAMMEE (niveau 0..9) tant qu'une fleche est
maintenue, et retombe a zero des le relachement. On peut avancer ET tourner en meme
temps (fwd et turn simultanes -> arc) ; les opposes s'annulent (fwd = haut - bas,
turn = gauche - droite).

Prerequis carte : type chassis CAR_FOURWHEEL (set_car_type 4) pour router sur la
kinematics 4 roues, et geometrie roue reglee (set_wheel_geom). M2 (encodeur HS)
est asservi en recopie sur sa roue de meme ligne (M4, recopie inversee) cote
firmware (sentinelle PID).
"""

# Plafonds de securite (doivent rester <= aux bornes de sendCmdVel : 1 m/s, 2 rad/s).
MAX_LIN = 1.0      # m/s   (plafond firmware ~1000 mm/s)
MAX_ANG = 2.0      # rad/s (borne sendCmdVel a +-2000 mrad/s)

# Plafonds teleop (vitesse a niveau 9) : <= MAX_LIN / MAX_ANG. Le niveau 0..9 echelonne
# une FRACTION de ces plafonds : niveau 0 = 10 % (mini non nul), niveau 9 = 100 %.
TELE_LIN = 0.5     # m/s a niveau 9
TELE_ANG = 1.5     # rad/s a niveau 9


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

    # --- vitesse programmee (fraction du plafond, croit avec le niveau) -----
    def _frac(self):
        return (self.speedLevel + 1) / 10.0               # 0.1 .. 1.0

    # --- pilotage par TENUE (presser = bouger / relacher = stop) ------------
    def holdVelocity(self, fwd_sign, turn_sign):
        """Fixe l'etat cmd_vel a la vitesse PROGRAMMEE selon les fleches maintenues.
        fwd_sign : +1 (haut/avant), -1 (bas/arriere), 0 (aucune / opposees annulees).
        turn_sign: +1 (gauche/anti-horaire), -1 (droite/horaire), 0 (aucune / annulees).
        Rotation a plat = fwd_sign 0 + turn_sign != 0 ; arc = les deux non nuls.
        Un relachement (signes 0) remet l'etat a zero -> le republieur emet cmd_vel(0,0)."""
        frac = self._frac()
        self.lin = max(-1, min(1, fwd_sign)) * TELE_LIN * frac
        self.ang = max(-1, min(1, turn_sign)) * TELE_ANG * frac
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

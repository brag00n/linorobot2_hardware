r"""ServoNode - node actionneur servos pan/tilt : consomme /servo/cmd, publie /servo/state.

Enveloppe la couche device.motion.RobotServoMotor de robot_control (reutilisee PAR
IMPORT : asservissement, zone morte/hysteresis, lisseur vitesse/acceleration
anti-oscillation -- ALGORITHMES INCHANGES). Le node decouple l'asservissement du
suivi (fidele ROS2) : le TrackingNode n'ordonne plus le servo directement, il
publie une cible sur /servo/cmd ; le Core y publie aussi le pilotage clavier.

Chaque ServoCmd est un message DISCRET : applique une seule fois (compare par
identite a la derniere commande appliquee), puis conserve sans effet sur le bus
profondeur 1 -- ce qui evite de re-appliquer un pas relatif (nudge) reste latche.
Le lisseur slew(dt), lui, tourne a CHAQUE tour (comme l'ancien main appelait
pt.slew() a chaque image), independamment de l'arrivee d'une commande.

Sous ROS2 : subscriber /servo/cmd + timer de slew + publisher /servo/state.
"""
import time

from robot_control.device.motion.RobotServoMotor import RobotServoMotor

from ..roslite import Node
from ..msgs import ServoCmd, ServoState


class ServoNode(Node):
    """Asservit les servos pan/tilt vers la cible recue et publie leur etat."""

    def __init__(self, args, link, telemetry=None):
        super().__init__("servo")
        self.servo = RobotServoMotor(
            link, panGain=args.pan_gain, tiltGain=args.tilt_gain,
            invertPan=args.invert_pan, invertTilt=args.invert_tilt,
            deadzone=args.deadzone, deadHyst=args.dead_hyst, maxStep=args.max_step,
            panMin=args.pan_min, panMax=args.pan_max, panHome=args.pan_home,
            tiltMin=args.tilt_min, tiltMax=args.tilt_max, tiltHome=args.tilt_home,
            maxVel=args.max_vel, maxAccel=args.max_accel, smooth=not args.no_smooth,
            telemetry=telemetry)
        self._last_cmd = None              # derniere commande appliquee (compare par identite)
        self._slew_t = 0.0                 # horloge du lisseur (dt reel)
        self._cmd_in = self.create_input("/servo/cmd", ServoCmd)
        self._state_out = self.create_output("/servo/state", ServoState)

    def on_start(self):
        """Recentre la camera au demarrage (pan 88 / tilt 42, cam de face)."""
        self.servo.center()
        self._slew_t = time.time()

    def process(self):
        """Applique une eventuelle commande neuve, lisse le mouvement, publie l'etat."""
        cmd = self._cmd_in.get()
        if cmd is not None and cmd is not self._last_cmd:
            self._apply(cmd)
            self._last_cmd = cmd

        # lisseur : rapproche le servo de la cible (dt borne, anti-flou). No-op si a la cible.
        now = time.time()
        dt = now - self._slew_t
        self._slew_t = now
        self.servo.slew(min(dt, 0.1))      # borne dt (evite un saut apres un lag)

        self._state_out.set(ServoState(
            angleH=self.servo.angleH, angleV=self.servo.angleV,
            deadzone=self.servo.deadzone, motion=self.servo.motionStats()))

    def _apply(self, cmd):
        """Dispatch d'une commande discrete (cible suivi ou pilotage manuel)."""
        if cmd.kind == "track" and cmd.nx is not None:
            self.servo.track(cmd.nx, cmd.ny)     # cible = mesure, anticipation ou coast
        elif cmd.kind == "home":
            self.servo.returnHome()              # coast expire : retour doux au centre
        elif cmd.kind == "center":
            self.servo.center()
        elif cmd.kind == "nudge_pan":
            self.servo.nudgePan(cmd.delta)
        elif cmd.kind == "nudge_tilt":
            self.servo.nudgeTilt(cmd.delta)

r"""RobotNavigation - Subsystem de navigation autonome (STUB, increment futur).

Squelette seulement : ce module marquera le 2e module metier (apres tracking).
Il COMPOSERA, comme RobotWebCamMotorized, les couches transverses :
  - device.sensor.RobotSensorWebCam   perception de l'environnement (webcam ;
                                      a terme aussi la camera UnitV2 / capteurs) ;
  - interaction (perception)          detection d'obstacles / de cibles ;
  - device.motion.RobotMotorDrive     deplacement du chassis 4 roues ;
  - communication.RobotComSerial      telemetrie STM32 (IMU/yaw, vitesses).

Aucune logique de navigation n'est implementee pour l'instant (voir le plan :
la logique reelle est hors perimetre de cet increment). L'API ci-dessous fixe
seulement la forme attendue pour brancher le subsystem dans RobotMain plus tard.
"""


class RobotNavigation:
    """Navigation autonome (non implementee). Squelette pour increment futur."""

    def __init__(self, drive, sensor=None, link=None, telemetry=None):
        self._drive = drive            # device.motion.RobotMotorDrive (chassis)
        self._sensor = sensor          # device.sensor.RobotSensorWebCam (perception)
        self._link = link              # communication.RobotComSerial (telemetrie)
        self.tel = telemetry
        self._running = False

    def start(self):
        """Demarrera la navigation autonome (non implemente)."""
        raise NotImplementedError(
            "RobotNavigation : navigation autonome non encore implementee "
            "(stub ; increment futur).")

    def step(self):
        """Un pas de la boucle de navigation (non implemente)."""
        raise NotImplementedError("RobotNavigation.step : increment futur.")

    def stop(self):
        """Arrete la navigation et le chassis (sans danger meme en stub)."""
        self._running = False
        if self._drive is not None:
            try:
                self._drive.stop()
            except Exception:
                pass

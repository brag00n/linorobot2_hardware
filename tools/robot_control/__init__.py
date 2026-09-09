r"""robot_control - Tooling PC multi-cartes du Bamboo v4 (STM32 + camera USB).

Prototypage de fonctions qui combinent PLUSIEURS cartes (suivi d'objet camera +
servos, navigation autonome). Les tests UNITAIRES d'une carte donnee vivent dans
firmware/<carte>/tools/check.py ; ici on ASSEMBLE ces briques.

Architecture en couches + modules metier (portage fidele de Bambou4WD_python, 2018 :
classes RobotXxx, methodes camelCase, setters fluides) :

  lib/            briques transverses sans dependance materielle
    Telemetry.py        journal structure (logs/ pour analyse externe / MCP)
    KalmanPredictor.py  filtre de Kalman -> prediction de trajectoire (nx, ny)
  communication/  liaison bas niveau avec une carte
    RobotComSerial.py   serie STM32 (protocole Yahboom), thread lecteur
  device/         pilotes d'organes (au-dessus de communication)
    sensor/RobotSensorWebCam.py   capture camera USB (pipeline P1) + flip/rotate
    motion/RobotServoMotor.py     servos pan/tilt S1/S2 + asservissement
    motion/RobotMotorDrive.py     4 moteurs (mix throttle/turn)
  interaction/    perception (au-dessus de device/sensor)
    FaceDetection.py    detecteurs Haar/DNN/YuNet + tracker Vit (detect-then-track)
  modules/        fonctions metier (subsystems qui COMPOSENT les couches)
    tracking/RobotWebCamMotorized.py  suivi de visage : camera + servos + prediction
    navigation/RobotNavigation.py     navigation autonome (a venir)
  RobotMain.py    orchestrateur : teleop clavier + suivi (cablage general)

Lancement (depuis tools/) :
  .venv/Scripts/python.exe -m robot_control.RobotMain --no-motion

Ancienne version (a plat) figee comme reference, toujours executable :
  .venv/Scripts/python.exe -m robot_control.old --no-motion
"""

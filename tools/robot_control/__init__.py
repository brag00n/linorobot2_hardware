"""robot_control - Outil de controle du Bamboo v4 (STM32), portage de l'ancien
Bambou4WD_python (2018) vers l'architecture v4.

Coeur : teleop clavier + suivi de visage via servos pan/tilt.

Modules :
  board_link : liaison serie COM4 (protocole Yahboom), thread lecteur, envoi trames
  motion     : commandes moteurs haut niveau (avant/arriere/rotation/stop)
  pan_tilt   : servos camera S1 (pan) / S2 (tilt) + asservissement P sur (nx,ny)
  vision     : detection de visage (pipeline P2 basse-res, reutilise face_detect.py)
  main       : boucle d'affichage P1 + clavier, cablage general, watchdog STOP

Lancement : depuis firmware/usbcam_bamboo/tools/
  ../.venv/Scripts/python.exe -m robot_control.main --no-motion
"""

"""robot_control.old - ANCIENNE version (a plat) figee comme reference.

Conservee pendant la construction de la nouvelle archi en couches
(robot_control/{lib,communication,device,interaction,modules}). Toujours
executable a l'identique pour comparer le comportement :
  .venv/Scripts/python.exe -m robot_control.old --no-motion

Portage de l'ancien Bambou4WD_python (2018) vers l'architecture v4.
Coeur : teleop clavier + suivi de visage via servos pan/tilt.

Modules :
  board_link : liaison serie COM4 (protocole Yahboom), thread lecteur, envoi trames
  motion     : commandes moteurs haut niveau (avant/arriere/rotation/stop)
  pan_tilt   : servos camera S1 (pan) / S2 (tilt) + asservissement P sur (nx,ny)
  vision     : detection de visage (pipeline P2 basse-res, reutilise face_detect.py)
  main       : boucle d'affichage P1 + clavier, cablage general, watchdog STOP

Lancement : depuis tools/
  .venv/Scripts/python.exe -m robot_control.old --no-motion
"""

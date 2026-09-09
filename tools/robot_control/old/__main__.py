r"""Point d'entree de l'ANCIENNE version (figee) : `python -m robot_control.old`.

Cette version a plat (board_link/motion/pan_tilt/vision/main) est conservee comme
reference pendant la construction de la nouvelle archi en couches. Elle reste
executable a l'identique pour comparer le comportement :

  .venv/Scripts/python.exe -m robot_control.old --no-motion --invert-tilt
"""
from .main import main

if __name__ == "__main__":
    main()

# tools/ — outils PC multi-cartes (linorobot2_hardware)

Outils Python cote **PC** (hote), partages par les differentes cartes du projet
(Bamboo v4 STM32, caméra USB, etc.). Deplacé ici depuis `firmware/usbcam_bamboo/`
car il ne concerne pas une seule carte.

## Contenu

| element | role |
|---|---|
| `robot_control/` | app de teleop clavier + suivi de visage pan/tilt (COM4, protocole STM32 v4). Pipeline capture/affichage (P1) + detection asynchrone (P2), prediction de trajectoire Kalman (coast + anticipation). |
| `rc_mcp_server.py` | serveur MCP (stdio) d'**analyse** en lecture seule : lit la telemetrie ecrite par l'app (`robot_control/logs/`). Coexiste avec l'app. |
| `face_detect.py` | helpers detection visage (cascade Haar + flip), reutilises par `robot_control`. |
| `show_usb_cam.py` | visualisation simple du flux camera USB. |
| `haarcascade_frontalface_default.xml` | cascade Haar livree. |

## Installation

Depuis ce dossier (`tools/`) :

```powershell
# Windows
.\install.ps1
```
```bash
# Linux / macOS / Git-Bash
./install.sh
```

L'installer cree un venv **`.venv/` dans ce dossier** et installe
`requirements.txt` (OpenCV, NumPy, pyserial). Options : `-Recreate` / `RECREATE=1`
pour repartir de zero, `-Python <chemin>` / `PYTHON=<chemin>` pour choisir
l'interpreteur de base (Python 3.10+ requis).

## Lancement

```powershell
# vision + servos seuls (sans moteurs)
.\.venv\Scripts\python.exe -m robot_control.main --no-motion
# avec moteurs — ROUES SURELEVEES pour les premiers essais
.\.venv\Scripts\python.exe -m robot_control.main
```

> ⚠️ **Couper le serveur MCP `bambou-board` avant** de lancer l'app : il tient le
> port serie (COM4) et l'app en a besoin en exclusif.

Raccourcis clavier (dont `F` suivi, `M` detecteur, `T` tracker, `P` mode
prediction) : voir l'en-tete de [`robot_control/main.py`](robot_control/main.py).

## Dependances externes (chemins relatifs)

Le tooling reference deux emplacements **hors de `tools/`** :

- **Modeles vision** (YuNet / DNN res10 / VitTrack) :
  `../firmware/usbcam_bamboo/Bambou4WD_python/src/resources/Other/face_detection_model/`
  — partages avec le code legacy `RobotObject.py`, donc laissés sur place
  ([vision.py](robot_control/vision.py), `_DNN_DIR`).
- **Protocole STM32** (`ros_monitor.py`) :
  `../firmware/stm32_bamboo/tools/` — importe par
  [board_link.py](robot_control/board_link.py) (`_STM32_TOOLS`).

Si tu deplaces à nouveau ce dossier, ajuste ces deux chemins relatifs.

## MCP

Le serveur `robot-control` est declaré dans le `.mcp.json` racine ; il pointe vers
`tools/.venv/Scripts/python.exe` et `tools/rc_mcp_server.py`. Après un
deplacement du dossier ou une recreation du venv, verifier ces chemins.

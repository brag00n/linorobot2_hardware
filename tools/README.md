# tools/ — outils PC multi-cartes (linorobot2_hardware)

Outils Python cote **PC** (hote), partages par les differentes cartes du projet
(Bamboo v4 STM32, caméra USB, etc.). Deplacé ici depuis `firmware/usbcam_bamboo/`
car il ne concerne pas une seule carte.

## Deux niveaux d'outillage

- **`firmware/<carte>/tools/`** — tests **mono-carte** : valident les fonctions
  d'UNE carte. Chacune expose un `check.py` (voir plus bas).
- **`tools/robot_control/`** (ce dossier) — prototypes **multi-cartes** : combinent
  des fonctions de PLUSIEURS cartes (suivi de visage = camera USB + servos STM32 ;
  navigation autonome a venir).

## Contenu

| element | role |
|---|---|
| `robot_control/` | app de teleop clavier + suivi de visage pan/tilt (COM4, protocole STM32 v4). **Reorganisee en couches transverses + modules metier** (voir ci-dessous). Pipeline capture/affichage (P1) + detection asynchrone (P2), prediction de trajectoire Kalman (coast + anticipation). |
| `robot_control/old/` | ancienne version **figee** (reference), toujours lancable : `-m robot_control.old`. |
| `robot_controlv3/` | **banc d'essai « nodes ROS2 »** : meme fonction que `robot_control`, refondue en NODES simules (framework `roslite` : Node/Port/Executor sequentiel). Chaque traitement (Camera/Tracking/Servo/Board) est isole derriere une interface standard testable sans queue ni environnement ROS ; les **algorithmes sont reutilises PAR IMPORT** depuis `robot_control` (zero reecriture). Lancable : `-m robot_controlv3.RobotMain` (memes flags). Voir plus bas. |
| `robot_control/mcp/` | **deux serveurs MCP** (stdio) : `analysis` (lecture seule des logs, coexiste avec l'app) et `action` (pilotage carte + config suivi, relaye par l'app-passerelle via socket ; repli COM4 direct). Voir la section MCP. |
| `rc_mcp_server.py`, `../firmware/stm32_bamboo/tools/ros_mcp_server.py` | anciens serveurs MCP (analyse + action), **remplaces** par `robot_control/mcp/` ; laisses en reference, retires du `.mcp.json`. |
| `face_detect.py` | demo autonome detection visage (cascade Haar + flip). |
| `show_usb_cam.py` | visualisation simple du flux camera USB. |
| `haarcascade_frontalface_default.xml` | cascade Haar livree. |

### Architecture de `robot_control/` (couches + modules, style 2018 fidele)

Couches transverses partagees, puis modules metier qui les composent :

```
robot_control/
  RobotMain.py                          orchestrateur teleop + suivi (entry point)
  lib/         Telemetry, KalmanPredictor          utilitaires transverses
  communication/ RobotComSerial                    liaison serie STM32 (Yahboom v4)
  device/
    sensor/    RobotSensorWebCam                    capture camera USB (+ flip/rotate)
    motion/    RobotMotorDrive, RobotServoMotor     4 roues / servos pan-tilt
  interaction/ FaceDetection                        perception (haar/dnn/yunet + tracker)
  modules/
    tracking/  RobotWebCamMotorized (subsystem)     suivi visage = sensor+detection+servo+Kalman
    navigation/ RobotNavigation                     STUB : navigation autonome (increment futur)
```

Les classes suivent la convention 2018 (`RobotXxx`, methodes camelCase, setters
fluides) ; **les algorithmes sont identiques** a `robot_control/old/` (seuls le
nommage et le decoupage changent).

### Banc d'essai « nodes ROS2 » (`robot_controlv3/`)

Prepare la migration ROS2 : le Core n'execute plus le traitement, il **orchestre
des nodes** via un micro-framework en process (`roslite`) qui mime rclpy.

```
robot_controlv3/
  RobotMain.py          Core : hote executeur + HMI/clavier/MCP/log/affichage
  msgs.py               types de messages (@dataclass = futurs .msg)
  roslite/              Node + Port (file profondeur 1) + Executor (bus, spin_once)
  nodes/                CameraNode, TrackingNode, ServoNode, BoardNode
```

Modele de transport : un `Port` = donnee unitaire d'une file ROS de **profondeur 1**.
L'`Executor` appelle les nodes dans l'ordre de la pipeline ; pour chacun il fait
`set` des entrees (entree de file) -> `process()` (callback) -> `get` des sorties
(sortie de file). Le corps de `process()` ne touche que ses Ports : c'est la
frontiere ou ROS2 glissera un vrai topic/thread sans rien changer aux algorithmes.

Les nodes **composent par import** les classes de couche de `robot_control`
(`RobotSensorWebCam`, `RobotWebCamMotorized`, `RobotServoMotor`, `RobotComSerial`) :
detection, Kalman, slew servo, protocole serie **inchanges**. Topics :
`/camera/image`, `/tracking/config`, `/tracking/result`, `/tracking/metrics`,
`/servo/cmd`, `/servo/state`, `/board/telemetry`.

```powershell
.\.venv\Scripts\python.exe -m robot_controlv3.RobotMain --no-motion --index 1 --flip h
```

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
.\.venv\Scripts\python.exe -m robot_control.RobotMain --no-motion
# avec moteurs — ROUES SURELEVEES pour les premiers essais
.\.venv\Scripts\python.exe -m robot_control.RobotMain
# sans IHM (pilotage MCP) : --headless (garde le suivi), --board-only (pur COM4)
.\.venv\Scripts\python.exe -m robot_control.RobotMain --headless
.\.venv\Scripts\python.exe -m robot_control.RobotMain --board-only
```

> **L'app est l'unique proprietaire de COM4.** Le serveur MCP `robot-action` lui
> relaie ses commandes carte par socket tant qu'elle tourne (aucun conflit de port,
> plus besoin de couper le MCP). Quand l'app est arretee, `robot-action` ouvre COM4
> lui-meme (repli direct, ouverture/fermeture par commande) — obligatoire pour le
> flash/bootloader, qui redemarre le MCU.

Raccourcis clavier (dont `F` suivi, `M` detecteur, `T` tracker, `P` mode
prediction) : voir l'en-tete de [`robot_control/RobotMain.py`](robot_control/RobotMain.py).

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

Tout le MCP est regroupe dans le sous-paquet **`robot_control/mcp/`** (pair de
`communication/`, `device/`, `modules/`, `lib/`), en **deux serveurs a role net**
declares dans le `.mcp.json` racine (venv `tools/.venv`, `PYTHONPATH=…\tools`,
`python -m robot_control.mcp.<srv>`) :

| serveur | role | ouvre COM4 ? |
|---|---|---|
| `robot-analysis` | **lecture seule** : `status`, `tail`, `board_tx/rx`, `detection`, `tracking`, `analyze`, `capture`, `mark`. Lit `logs/` ; coexiste avec l'app. | non |
| `robot-action` | **action** : config suivi (`set_detector`/`set_track_mode`/`set_predict_mode`/`set_tracking`), carte (`motor_drive*`, `pwm_servo`, PID, geometrie roue, `car_type`, `metrics`, `encoders`, `calibrate_*`), maintenance (`enter_bootloader`, `flash_firmware`). | via l'app (relais) ; repli direct si app off |

Transport (`mcp/gateway.py`) : **socket loopback** `127.0.0.1:8787`
(`BAMBOU_MCP_PORT`), zero fichier d'echange. L'app ecoute (`CommandServer`, drainé
sur le thread principal a chaque tour de boucle), `robot-action` est client. Le
serveur **tente de se connecter** : connexion OK → app vivante → relais ; connexion
refusee → app arretee → `robot-action` ouvre COM4 lui-meme (`RobotComSerial`,
`BAMBOU_PORT`/`BAUD`/`CPR`). L'executeur de commandes carte `handle_command()` est
**partage** entre les deux chemins (zero duplication). `enter_bootloader`/
`flash_firmware` refusent tant que l'app tient le port (le bootloader redemarre le MCU).

Anciens serveurs (`rc_mcp_server.py`, `firmware/stm32_bamboo/tools/ros_mcp_server.py`)
retires du `.mcp.json`, laisses en reference. Après un deplacement du dossier ou une
recreation du venv, verifier les chemins du `.mcp.json`.

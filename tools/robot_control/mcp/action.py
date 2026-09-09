#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""action.py - Serveur MCP (stdio) d'ACTION sur la carte STM32 du Bamboo v4.

Regroupe tout l'ancien serveur bambou-board (pilotage moteurs/servos, PID,
geometrie roue, type de chassis, calibration, bootloader/flash) + la bascule de
config du suivi (detecteur/tracker/prediction/armement), avec un modele de
transport a auto-selection :

  - L'APP EST LANCEE (socket 127.0.0.1:PORT joignable) : elle est l'unique
    proprietaire de COM4. On lui RELAIE la commande ; elle l'execute sur son
    thread principal (acces serie serialise) et renvoie le texte. C'est le cas
    nominal, et le SEUL possible pour les commandes de config (elles pilotent le
    suivi vision, qui n'existe que dans l'app).
  - L'APP EST ARRETEE (connexion refusee) : on ouvre COM4 en direct via
    RobotComSerial (la MEME passerelle protocole que l'app) et on execute le MEME
    handle_command -> zero duplication de logique.

  - MAINTENANCE (enter_bootloader / flash_firmware) : COM4 direct UNIQUEMENT, app
    OBLIGATOIREMENT arretee (le passage en bootloader redemarre le MCU et le
    flasheur exige le port en exclusif). Refuse proprement si l'app tient le port.

Variables d'environnement :
  BAMBOU_MCP_PORT  adresse du socket de l'app (defaut 127.0.0.1:8787)
  BAMBOU_PORT      port serie pour le repli direct (defaut COM4)
  BAMBOU_BAUD      debit (defaut 115200)
  BAMBOU_CPR       tics/tour pour les conversions (defaut 1320)

Lancement (via .mcp.json) :  python -m robot_control.mcp.action
Test manuel :                python -m robot_control.mcp.action selftest
"""
import os
import subprocess
import sys
import time

from . import _rpc
from . import gateway
from ..communication.RobotComSerial import RobotComSerial

SERVER_NAME = "robot-action"

_LOG = _rpc.make_log("rc-action")

# tools/robot_control/mcp/action.py -> repo (quatre niveaux au-dessus).
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
STM32_ROOT = os.path.join(_REPO, "firmware", "stm32_bamboo")
STM32_TOOLS = os.path.join(STM32_ROOT, "tools")

PORT = os.environ.get("BAMBOU_PORT", "COM4")
BAUD = int(os.environ.get("BAMBOU_BAUD", "115200"))
CPR = float(os.environ.get("BAMBOU_CPR", "1320"))

DETECTORS = ("haar", "dnn", "yunet")


def _addr():
    return gateway.parse_addr(os.environ.get("BAMBOU_MCP_PORT"))


# ---------------------------------------------------------------------------
# Repli COM4 direct : ouvre la MEME passerelle, execute le MEME handle_command
# ---------------------------------------------------------------------------
def _direct_exec(cmd, args):
    """Ouvre COM4 le temps d'une commande, l'execute, referme (rend le port a
    l'app des qu'elle demarre). handle_command n'utilise pas `motion` -> None."""
    link = RobotComSerial(PORT, BAUD, cpr=CPR)
    try:
        time.sleep(0.4)                       # laisse le thread lecteur ouvrir + capter
        return gateway.handle_command(cmd, args, link, None, link.cpr or CPR)
    finally:
        link.close()


# ---------------------------------------------------------------------------
# Aiguillage relais-app / repli-direct
# ---------------------------------------------------------------------------
def _board(cmd, args):
    """Commande CARTE : relais si l'app tourne, sinon COM4 direct."""
    host, port = _addr()
    ok, payload, alive = gateway.send_command(cmd, args, host, port)
    if alive:
        return ("[via app] " if ok else "[via app : erreur] ") + str(payload)
    return "[COM4 direct] " + _direct_exec(cmd, args)


def _config(cmd, args):
    """Commande de CONFIG suivi : n'a de sens que si l'app tourne (relais seul)."""
    host, port = _addr()
    ok, payload, alive = gateway.send_command(cmd, args, host, port)
    if not alive:
        return ("L'app n'est pas lancee : la configuration du suivi "
                "(detecteur / tracker / prediction / armement) n'a d'effet que sur "
                "l'app en cours d'execution. Lancez robot_control (ou robot_controlv3) "
                "puis reessayez.")
    return ("Config appliquee par l'app : " if ok else "Echec config : ") + str(payload)


# ---------------------------------------------------------------------------
# Outils de CONFIG (relais app uniquement)
# ---------------------------------------------------------------------------
def t_set_detector(args):
    det = (args.get("detector") or "").strip().lower()
    if det not in DETECTORS:
        return "detecteur invalide '%s'. Attendu : %s." % (det, ", ".join(DETECTORS))
    return _config("set_detector", {"detector": det})


def t_set_track_mode(args):
    return _config("set_track_mode", {"track_mode": args.get("track_mode")})


def t_set_predict_mode(args):
    return _config("set_predict_mode", {"predict_mode": args.get("predict_mode")})


def t_set_tracking(args):
    return _config("set_tracking", {"on": bool(args.get("on"))})


# ---------------------------------------------------------------------------
# Outils CARTE (relais app, sinon COM4 direct) -- garde-fous dans handle_command
# ---------------------------------------------------------------------------
def _board_tool(cmd):
    def handler(args):
        return _board(cmd, args)
    return handler


# ---------------------------------------------------------------------------
# Outils de MAINTENANCE (COM4 direct uniquement, app arretee obligatoire)
# ---------------------------------------------------------------------------
def t_enter_bootloader(args):
    host, port = _addr()
    if gateway.ping(host, port):
        return ("L'app tient COM4 (socket %s:%s joignable) : arretez-la d'abord. "
                "Le passage en bootloader redemarre le MCU et exige le port en "
                "exclusif." % (host, port))
    link = RobotComSerial(PORT, BAUD, cpr=CPR)
    time.sleep(0.4)
    if not link.connected:
        link.close()
        return ("COM4 (%s) introuvable ou occupe : impossible d'envoyer la commande "
                "bootloader." % PORT)
    sent = link.enterBootloader()             # envoie 0xA3 x3 puis FERME le port
    if not sent:
        return "Echec d'envoi de la commande bootloader (write KO)."
    return ("Commande bootloader (FUNC 0xA3) envoyee, port COM4 libere.\n"
            "La carte a du redemarrer dans son bootloader ROM.\n\n"
            "Etape suivante : flash_firmware (ou `pio run -t upload` dans un terminal).\n"
            "Si l'upload dit COM4 occupe : le firmware actuel n'a pas encore le handler "
            "0xA3 (l'installer une premiere fois via BOOT0+RESET).")


def t_flash_firmware(args):
    host, port = _addr()
    if gateway.ping(host, port):
        return ("L'app tient COM4 (socket %s:%s joignable) : arretez-la avant de "
                "flasher (iap_flash.py a besoin du port en exclusif)." % (host, port))
    flasher = os.path.join(STM32_TOOLS, "iap_flash.py")
    binpath = args.get("bin") or os.path.join(
        STM32_ROOT, ".pio", "build", "genericSTM32F103RC", "firmware.bin")
    if not os.path.isfile(flasher):
        return "outil introuvable : %s" % flasher
    if not os.path.isfile(binpath):
        return ("image introuvable : %s\nCompile d'abord l'application avec `pio run`."
                % binpath)
    cmd = [sys.executable, flasher, "--port", PORT, "--baud", str(BAUD), "--bin", binpath]
    if args.get("no_enter"):
        cmd.append("--no-enter")              # deja dans le bootloader
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        out = (proc.stdout or "") + (proc.stderr or "")
        ok = proc.returncode == 0
    except subprocess.TimeoutExpired:
        out, ok = "iap_flash.py : timeout (180 s).", False
    except Exception as e:
        out, ok = "echec lancement iap_flash.py : %s" % e, False
    return ("Flash IAP OK." if ok else "Flash IAP ECHEC.") + "\n\n" + out.strip()


# ---------------------------------------------------------------------------
# Declaration MCP
# ---------------------------------------------------------------------------
TOOLS = [
    # --- config du suivi (app requise) ---
    {"name": "set_detector",
     "description": "Change le detecteur de visage de l'app A CHAUD (relais socket). "
                    "haar=leger mais decroche de profil ; dnn=res10 robuste au profil "
                    "(defaut) ; yunet=le plus robuste (modele onnx requis). N'a d'effet "
                    "que si l'app tourne.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "detector": {"type": "string", "enum": list(DETECTORS),
                                      "description": "haar | dnn | yunet"}},
                     "required": ["detector"]}},
    {"name": "set_track_mode",
     "description": "Change le tracker de l'app A CHAUD (none | mil | vit). Relais "
                    "socket ; n'a d'effet que si l'app tourne.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "track_mode": {"type": "string",
                                        "description": "none | mil | vit"}},
                     "required": ["track_mode"]}},
    {"name": "set_predict_mode",
     "description": "Change le mode de prediction de trajectoire de l'app A CHAUD "
                    "(off | anticip | coast). Relais socket ; app requise.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "predict_mode": {"type": "string",
                                          "description": "off | anticip | coast"}},
                     "required": ["predict_mode"]}},
    {"name": "set_tracking",
     "description": "Arme (on=true) ou desarme (on=false) le suivi de visage de l'app "
                    "(equivaut a la touche F). Relais socket ; app requise.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "on": {"type": "boolean",
                                "description": "true = armer le suivi, false = desarmer"}},
                     "required": ["on"]}},

    # --- pilotage carte (relais app, sinon COM4 direct) ---
    {"name": "motor_drive",
     "description": "Pilote UN moteur (1..4) a faible PWM (borne +-50 %) pendant une "
                    "courte duree (borne 2 s) et mesure le delta de comptage encodeur. "
                    "ROUES SURELEVEES obligatoire. STOP envoye a la fin.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "motor": {"type": "integer",
                                   "description": "moteur a piloter : 1, 2, 3 ou 4"},
                         "pwm": {"type": "integer",
                                 "description": "PWM en % signe (-50..50)"},
                         "seconds": {"type": "number",
                                     "description": "duree en s (0.1..2, defaut 1)"}},
                     "required": ["motor"]}},
    {"name": "motor_drive_all",
     "description": "Pilote les 4 moteurs SIMULTANEMENT avec controle independant de "
                    "vitesse et sens (PWM signe par moteur, borne +-40 %). Fournir "
                    "m1,m2,m3,m4 pour un controle explicite, ou pwm (+reverse) pour le "
                    "raccourci marche avant M1+ M2- M3- M4+. Duree <=5 s. ROUES "
                    "SURELEVEES obligatoire. STOP garanti.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "m1": {"type": "integer", "description": "PWM signe M1 (-40..40)"},
                         "m2": {"type": "integer", "description": "PWM signe M2 (-40..40)"},
                         "m3": {"type": "integer", "description": "PWM signe M3 (-40..40)"},
                         "m4": {"type": "integer", "description": "PWM signe M4 (-40..40)"},
                         "pwm": {"type": "integer",
                                 "description": "raccourci marche avant (0..40, defaut 15)"},
                         "seconds": {"type": "number",
                                     "description": "duree en s (0.1..5, defaut 3)"},
                         "reverse": {"type": "boolean",
                                     "description": "raccourci : true = marche arriere"}}}},
    {"name": "pwm_servo",
     "description": "Positionne un servo PWM (SG90) en angle absolu. S1=id 1, S2=id 2, "
                    "S3=id 3, S4=id 4. Angle borne 0..180 deg. Option sweep=true : "
                    "balayage de test 90->0->180->90.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "id": {"type": "integer", "description": "servo 1..4"},
                         "angle": {"type": "integer",
                                   "description": "angle vise en degres (0..180, defaut 90)"},
                         "sweep": {"type": "boolean",
                                   "description": "true = balayage de test"}},
                     "required": ["id"]}},
    {"name": "metrics",
     "description": "Dernieres metriques decodees : vitesse & batterie, attitude (yaw), "
                    "comptage encodeur et vitesse moteurs (tics/s et tr/min).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "encoders",
     "description": "Comptage encodeur cumulatif par moteur (M1..M4) et vitesse "
                    "instantanee (tics/s, tr/min).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "car_type",
     "description": "Interroge la carte sur son type de chassis (FUNC_REQUEST_DATA "
                    "0x50 -> 0x15) et en deduit les tics/tour (cpr).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "set_car_type",
     "description": "Ecrit le type de chassis (FUNC_CAR_TYPE 0x15). save=true (defaut) "
                    "persiste en flash ; save=false = RAM. Types : 1 MECANUM, 2 "
                    "MECANUM_MAX, 3 MECANUM_MINI, 4 FOURWHEEL, 5 ACKERMAN, 6 SUNRISE. "
                    "Relit pour confirmer.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "car_type": {"type": "integer", "description": "1..6 (4 = FOURWHEEL)"},
                         "save": {"type": "boolean",
                                  "description": "true (defaut) = flash ; false = RAM"}},
                     "required": ["car_type"]}},
    {"name": "get_wheel_geom",
     "description": "Lit la geometrie roue runtime (REQUEST_DATA 0x16) : cpr "
                    "(tics/tour), circonference (mm), diametre et APB (mm).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "set_wheel_geom",
     "description": "Regle la geometrie roue (FUNC_SET_WHEEL_GEOM 0x16). cpr, circ_mm, "
                    "apb_mm chacun optionnel (absent = garde la valeur courante). "
                    "save=true (defaut) = flash ; save=false = RAM. Relit pour confirmer.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "cpr": {"type": "number", "description": "tics par tour (entier)"},
                         "circ_mm": {"type": "number",
                                     "description": "circonference roue en mm (<=6553.5)"},
                         "apb_mm": {"type": "number",
                                    "description": "demi-somme voie+empattement en mm"},
                         "save": {"type": "boolean",
                                  "description": "true (defaut) = flash ; false = RAM"}}}},
    {"name": "set_motor_pid",
     "description": "Regle le PID moteur (FUNC_SET_MOTOR_PID 0x13), PID UNIQUE partage "
                    "par les 4 moteurs. save=false (DEFAUT) = RAM (tuning) ; save=true "
                    "= flash. Relit pour confirmer.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "kp": {"type": "number", "description": "gain proportionnel"},
                         "ki": {"type": "number", "description": "gain integral"},
                         "kd": {"type": "number", "description": "gain derive"},
                         "save": {"type": "boolean",
                                  "description": "false (defaut) = RAM ; true = flash"}},
                     "required": ["kp", "ki", "kd"]}},
    {"name": "set_yaw_pid",
     "description": "Regle le PID de cap/yaw (FUNC_SET_YAW_PID 0x14). save=false "
                    "(DEFAUT) = RAM ; save=true = flash. Relit pour confirmer.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "kp": {"type": "number", "description": "gain proportionnel"},
                         "ki": {"type": "number", "description": "gain integral"},
                         "kd": {"type": "number", "description": "gain derive"},
                         "save": {"type": "boolean",
                                  "description": "false (defaut) = RAM ; true = flash"}},
                     "required": ["kp", "ki", "kd"]}},
    {"name": "get_pid",
     "description": "Lit les PID courants : moteur (partage, index 1) et yaw (index 5).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "calibrate_baseline",
     "description": "Fixe le comptage encodeur courant comme zero de reference, avant "
                    "de tourner une roue a la main.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "calibrate_read",
     "description": "Lit la variation de comptage depuis calibrate_baseline et en "
                    "deduit les tics/tour par moteur (delta / turns) et le sens.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "turns": {"type": "number",
                                   "description": "nb de tours effectues (defaut 1)"}}}},

    # --- maintenance (app arretee obligatoire, COM4 direct) ---
    {"name": "enter_bootloader",
     "description": "Fait sauter la carte dans son bootloader ROM (flash sans "
                    "BOOT0+RESET) et libere COM4. App OBLIGATOIREMENT arretee. "
                    "Necessite le handler FUNC_ENTER_BOOTLOADER (0xA3).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "flash_firmware",
     "description": "Met a jour l'application par UART (IAP) sans BOOT0 ni RESET : "
                    "lance iap_flash.py (0xA3 -> bootloader, ERASE/WRITE/VERIFY/GO). "
                    "App OBLIGATOIREMENT arretee. Compiler avant avec `pio run`.",
     "inputSchema": {"type": "object",
                     "properties": {
                         "bin": {"type": "string",
                                 "description": "chemin de l'image (defaut : "
                                                ".pio/build/genericSTM32F103RC/firmware.bin)"},
                         "no_enter": {"type": "boolean",
                                      "description": "true si la carte est deja dans le bootloader"}}}},
]

_BOARD_TOOLS = ("motor_drive", "motor_drive_all", "pwm_servo", "metrics", "encoders",
                "car_type", "set_car_type", "get_wheel_geom", "set_wheel_geom",
                "set_motor_pid", "set_yaw_pid", "get_pid",
                "calibrate_baseline", "calibrate_read")

HANDLERS = {
    "set_detector": t_set_detector,
    "set_track_mode": t_set_track_mode,
    "set_predict_mode": t_set_predict_mode,
    "set_tracking": t_set_tracking,
    "enter_bootloader": t_enter_bootloader,
    "flash_firmware": t_flash_firmware,
}
for _c in _BOARD_TOOLS:
    HANDLERS[_c] = _board_tool(_c)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        host, port = _addr()
        alive = gateway.ping(host, port)
        _LOG("selftest : app socket %s:%s -> %s"
             % (host, port, "JOIGNABLE (relais)" if alive else "absente (repli COM4)"))
        _LOG("status carte : " + _board("metrics", {}).replace("\n", " | "))
        return
    host, port = _addr()
    _LOG("demarrage (app=%s:%s, repli %s@%s, cpr=%g)" % (host, port, PORT, BAUD, CPR))
    _rpc.serve(SERVER_NAME, TOOLS, HANDLERS)


if __name__ == "__main__":
    main()

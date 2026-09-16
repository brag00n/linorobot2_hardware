r"""robot_profile - profils robot (fichiers JSON) + resolution CLI.

Un PROFIL par robot (robots/<name>.json) declare les cartes attendues : la (ou les)
carte(s) de CONTROLE (kind stm32/esp32, VID:PID, port, baud, cpr) et la carte
CAPTEURS (grovepi). Cela permet de basculer proprement entre robots :
  --robot bamboo4WD_V4_YBStm32   (defaut : STM32 YahBoom, comportement historique)
  --robot bamboo4WD_V4_WSEsp32   (ESP32 WaveShare parlant le meme protocole binaire)
  --robot bamboo4WD_V4_Teensy    (Teensy parlant le meme protocole binaire, sans magneto)

`control` peut etre une LISTE (profil « banc » STM32+ESP32+Teensy branches simultanement) :
setup() (robot_controlv3) instancie alors un node par carte et le HUD empile toutes
les cartes presentes.

Resolution (resolve) : lit le profil du robot choisi puis applique les SURCHARGES
CLI par carte (--port/--baud pour la STM32, --esp32-port pour l'ESP32, --teensy-port
pour le Teensy, --grovepi-port pour les capteurs ; --no-esp32/--no-teensy/--no-grovepi
desactivent). Les valeurs
« legacy » args.port/baud/grovepi_port sont repositionnees sur la carte de controle
PRINCIPALE pour les chemins existants (bannieres, telemetrie, gateway MCP).
"""
import json
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
ROBOTS_DIR = os.path.join(_HERE, "robots")
DEFAULT_ROBOT = "bamboo4WD_V4_YBStm32"


def available():
    """Noms des profils disponibles (fichiers robots/*.json), tries."""
    try:
        return sorted(f[:-5] for f in os.listdir(ROBOTS_DIR) if f.endswith(".json"))
    except OSError:
        return []


def load(name):
    """Charge et retourne le profil <name> (dict). Leve si absent/illisible."""
    path = os.path.join(ROBOTS_DIR, f"{name}.json")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            "profil robot introuvable : %s (disponibles : %s)"
            % (path, ", ".join(available()) or "aucun"))
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def resolve(args):
    """Resout le profil args.robot en cartes concretes + surcharges CLI.

    Ajoute sur `args` : robot (nom), robot_label, controls (liste de dicts
    {kind, port, baud, cpr, vid_pid, enabled}), sensors (dict ou None). Repositionne
    aussi args.port/baud/grovepi_port sur la carte de controle principale/les capteurs.
    Retourne `args`.
    """
    name = getattr(args, "robot", None) or DEFAULT_ROBOT
    prof = load(name)
    args.robot = name
    args.robot_label = prof.get("label", name)

    # --- cartes de controle (dict unique ou liste pour un banc) ---
    ctrl_raw = prof.get("control")
    if isinstance(ctrl_raw, list):
        ctrl_list = ctrl_raw
    elif ctrl_raw:
        ctrl_list = [ctrl_raw]
    else:
        ctrl_list = []

    controls = []
    for c in ctrl_list:
        kind = c.get("kind")
        entry = {
            "kind": kind,
            "port": c.get("port"),
            "baud": c.get("baud", 115200),
            "cpr": c.get("cpr"),
            "vid_pid": c.get("vid_pid"),
            # protocole de fil : "yahboom" (defaut, trames binaires) ou "mavlink".
            # L'ancien COM reste toujours disponible via ce champ (surcharge --protocol).
            "protocol": c.get("protocol", "yahboom"),
            "enabled": True,
        }
        if kind == "stm32":
            if getattr(args, "port", None):
                entry["port"] = args.port
            if getattr(args, "baud", None):
                entry["baud"] = args.baud
        elif kind == "esp32":
            if getattr(args, "esp32_port", None):
                entry["port"] = args.esp32_port
            if getattr(args, "no_esp32", False):
                entry["enabled"] = False
        elif kind == "teensy":
            if getattr(args, "teensy_port", None):
                entry["port"] = args.teensy_port
            if getattr(args, "no_teensy", False):
                entry["enabled"] = False
        # Surcharge CLI globale du protocole de fil (--protocol yahboom|mavlink),
        # appliquee a TOUTES les cartes ; sinon la valeur du profil par carte.
        if getattr(args, "protocol", None):
            entry["protocol"] = args.protocol
        controls.append(entry)
    args.controls = controls

    # --- carte capteurs (grovepi) ---
    s = prof.get("sensors")
    sensors = None
    if s:
        sensors = {
            "kind": s.get("kind"),
            "port": s.get("port"),
            "baud": s.get("baud", 115200),
            "vid_pid": s.get("vid_pid"),
            "enabled": not getattr(args, "no_grovepi", False),
        }
        if getattr(args, "grovepi_port", None):
            sensors["port"] = args.grovepi_port
    args.sensors = sensors

    # --- valeurs legacy (bannieres/telemetrie/gateway) = carte de controle principale ---
    primary = next((c for c in controls if c["enabled"]),
                   controls[0] if controls else None)
    if primary:
        args.port = primary["port"]
        args.baud = primary["baud"]
        args.protocol = primary["protocol"]   # seam legacy (robot_control mono-carte)
    if sensors:
        args.grovepi_port = sensors["port"]
    return args

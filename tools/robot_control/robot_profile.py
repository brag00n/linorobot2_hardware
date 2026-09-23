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

La GEOMETRIE n'appartient PAS a ces profils : elle vient de la source unique du depot ROS
frere, `linorobot2/bamboo_base/config/robots/<robot>.yaml` (cf. CANONICAL_DIR), exposee par
resolve() dans `args.geometry` -- avec les trois grandeurs qu'exige setWheelGeom (cpr,
circonference et APB en millimetres bruts, unites du protocole MAVLink). Sans le depot
frere, on retombe sur le seul `cpr` que portent les profils, et `source` le dit.

Resolution (resolve) : lit le profil du robot choisi puis applique les SURCHARGES
CLI par carte (--port/--baud pour la STM32, --esp32-port pour l'ESP32, --teensy-port
pour le Teensy, --grovepi-port pour les capteurs ; --no-esp32/--no-teensy/--no-grovepi
desactivent). Les valeurs
« legacy » args.port/baud/grovepi_port sont repositionnees sur la carte de controle
PRINCIPALE pour les chemins existants (bannieres, telemetrie, gateway MCP).
"""
import json
import math
import os
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
ROBOTS_DIR = os.path.join(_HERE, "robots")
DEFAULT_ROBOT = "bamboo4WD_V4_YBStm32"

# Les cartes d'un banc persistent leur port en parallele (threads lecteurs) ->
# read-modify-write du meme fichier serialise.
_PERSIST_LOCK = threading.Lock()


# Source unique de verite PHYSIQUE : elle vit dans le depot ROS frere, clone a cote de
# celui-ci (bamboo_base/config/robots/<robot>.yaml). Les profils JSON ci-dessous ne
# portent que des faits d'HOTE (port COM, baud, VID:PID) ; la geometrie vient de la.
CANONICAL_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "..", "..", "linorobot2", "bamboo_base", "config", "robots"))

# Cles physiques lues dans le YAML canonique (toutes scalaires, une par ligne).
_GEOM_KEYS = ("car_type", "counts_per_rev", "wheel_diameter_m", "wheel_separation_m",
              "motor_max_rpm", "motor_operating_voltage", "motor_power_max_voltage",
              "pid_kp", "pid_ki", "pid_kd")


def _read_canonical(robot_name):
    """Lit les cles physiques du YAML canonique du robot. {} si indisponible.

    PyYAML n'est pas une dependance du tooling (cf. requirements.txt) : on l'utilise s'il
    est la, sinon on rabat sur un balayage des scalaires `cle: valeur`, suffisant car le
    fichier est le notre et n'a qu'un niveau d'imbrication. Toute defaillance (depot frere
    absent, fichier illisible) est silencieuse : l'appelant retombe sur le profil JSON.
    """
    path = os.path.join(CANONICAL_DIR, f"{robot_name}.yaml")
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return {}
    try:
        import yaml  # optionnel
        params = yaml.safe_load(text)["/**"]["ros__parameters"]
        return {k: params[k] for k in _GEOM_KEYS if k in params}
    except ImportError:
        pass
    except Exception:
        return {}
    out = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, _, val = line.partition(":")
        key, val = key.strip(), val.strip()
        if key in _GEOM_KEYS and val:
            try:
                out[key] = float(val)
            except ValueError:
                pass
    return out


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


def persist_control_port(robot_name, kind, port):
    """Reecrit le `port` de la carte de controle <kind> dans robots/<robot_name>.json.

    Appele quand le scan d'auto-decouverte a trouve la carte sur un COM different
    de celui configure (le port a change) : le prochain lancement retrouve alors la
    carte instantanement sur son port prefere, sans repasser par le scan. Ne touche
    QUE le champ `port` de l'entree <kind> ; best-effort (echec silencieux si profil
    absent/illisible). Retourne True si le fichier a ete modifie.
    """
    if not robot_name:
        return False
    path = os.path.join(ROBOTS_DIR, f"{robot_name}.json")
    with _PERSIST_LOCK:
        try:
            with open(path, encoding="utf-8") as f:
                prof = json.load(f)
        except (OSError, ValueError):
            return False
        ctrl = prof.get("control")
        entries = ctrl if isinstance(ctrl, list) else ([ctrl] if ctrl else [])
        changed = False
        for c in entries:
            if isinstance(c, dict) and c.get("kind") == kind and c.get("port") != port:
                c["port"] = port
                changed = True
        if not changed:
            return False
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(prof, f, indent=2, ensure_ascii=False)
                f.write("\n")
        except OSError:
            return False
        return True


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
    # La cle `name` en tete de profil n'etait jamais lue. Elle sert d'identifiant du robot
    # (et de nom du fichier canonique du depot frere) ; un ecart avec le nom de fichier est
    # une erreur de redaction du profil, signalee plutot que subie.
    args.robot_id = prof.get("name") or name
    if args.robot_id != name:
        print("[robot_profile] AVERTISSEMENT : profil %s.json declare name=%s "
              "(le nom de fichier fait foi pour --robot)" % (name, args.robot_id))

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
            # protocole de fil de la carte capteurs : "yahboom" (defaut) ou "mavlink",
            # comme les cartes de controle. Sans cette cle, la GrovePi retombait
            # toujours en yahboom (RobotMain.get("protocol","yahboom")) et ne decodait
            # pas un flux MAVLink -> grove_fail malgre le profil en mavlink.
            "protocol": s.get("protocol", "yahboom"),
            "enabled": not getattr(args, "no_grovepi", False),
        }
        if getattr(args, "grovepi_port", None):
            sensors["port"] = args.grovepi_port
        # Surcharge CLI globale (--protocol), appliquee AVANT que args.protocol ne soit
        # ecrase par le protocole de la carte de controle principale (seam legacy plus bas).
        if getattr(args, "protocol", None):
            sensors["protocol"] = args.protocol
    args.sensors = sensors

    # --- manette de jeu (gamepad, teleop portable pygame) ---
    g = prof.get("gamepad")
    gamepad = None
    if g:
        gamepad = {
            "index": g.get("index", 0),          # numero de manette SDL (0 = premiere)
            "deadzone": g.get("deadzone", 0.12),  # zone morte des sticks (fraction)
            "expo": g.get("expo", 0.35),          # courbe expo (finesse au centre)
            # La cle `enabled` du profil etait ecrasee sans condition : une manette
            # desactivee dans le JSON etait activee quand meme. Le CLI ne peut plus que
            # DESACTIVER (--no-gamepad), jamais ressusciter ce que le profil a coupe.
            "enabled": bool(g.get("enabled", True)) and not getattr(args, "no_gamepad", False),
        }
        if getattr(args, "gamepad_index", None) is not None:
            gamepad["index"] = args.gamepad_index
        if getattr(args, "gamepad_deadzone", None) is not None:
            gamepad["deadzone"] = args.gamepad_deadzone
    args.gamepad = gamepad

    # --- geometrie physique (source unique du depot frere, repli sur le profil) ---
    # setWheelGeom() exige les TROIS grandeurs (cpr, circonference, APB) ; le profil JSON
    # n'a jamais porte que `cpr`. On prefere donc le YAML canonique, et on ne retombe sur
    # le profil que s'il est introuvable. circ_mm / apb_mm sont deja dans l'unite du
    # protocole MAVLink (millimetres bruts, idx 16 et 17).
    canon = _read_canonical(args.robot_id)
    geometry = {"source": "canonical" if canon else "profile"}
    if canon:
        geometry.update({k: canon[k] for k in canon})
        geometry["cpr"] = canon.get("counts_per_rev")
    else:
        primary_cpr = next((c.get("cpr") for c in ctrl_list if c.get("cpr")), None)
        geometry["cpr"] = float(primary_cpr) if primary_cpr else None
    d, sep = geometry.get("wheel_diameter_m"), geometry.get("wheel_separation_m")
    geometry["circ_mm"] = round(math.pi * d * 1000.0, 1) if d else None
    geometry["apb_mm"] = round(sep / 2.0 * 1000.0, 1) if sep else None
    args.geometry = geometry

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

#!/usr/bin/env bash
# Installe l'environnement Python du tooling robot_control (venv + dependances).
# Cree .venv DANS ce dossier tools/ et y installe requirements.txt (RUNTIME de l'app :
# vision + teleop + MCP). Idempotent. Requiert Python 3.10+.
#
# Avec --dev (ou DEV=1) : provisionne EN PLUS un venv separe .venv-dev avec le toolchain
# de developpement (requirements-dev.txt : pymavlink/mavgen pour regenerer le dialecte
# MAVLink + PlatformIO `pio` pour compiler/flasher les firmwares). Ce toolchain est isole
# du runtime pour ne pas perturber ses versions epinglees ni alourdir le deploiement
# robot/RPi -> reproductible depuis zero.
#
# Usage :
#   ./install.sh                 # runtime seul (deploiement robot/RPi)
#   ./install.sh --dev           # + toolchain de dev (regen MAVLink + PlatformIO)
#   PYTHON=/usr/bin/python3.13 ./install.sh
#   RECREATE=1 ./install.sh --dev   # recree le(s) venv de zero
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"
REQ="$HERE/requirements.txt"
VENVDEV="$HERE/.venv-dev"
REQDEV="$HERE/requirements-dev.txt"

# --dev en argument ou DEV=1 en env
DEV="${DEV:-0}"
for a in "$@"; do case "$a" in --dev) DEV=1 ;; esac; done

# interpreteur de base : $PYTHON, sinon python3, sinon python
PYBASE="${PYTHON:-}"
if [ -z "$PYBASE" ]; then
    if command -v python3 >/dev/null 2>&1; then PYBASE="$(command -v python3)"
    elif command -v python >/dev/null 2>&1; then PYBASE="$(command -v python)"
    else echo "Aucun Python trouve. Installe Python 3.10+." >&2; exit 1; fi
fi

# Garde-fou de version : les pins runtime (numpy 2.2.x / opencv) exigent Python 3.10+.
if ! "$PYBASE" -c "import sys; sys.exit(0 if sys.version_info[:2] >= (3,10) else 1)"; then
    echo "Python 3.10+ requis (detecte : $("$PYBASE" -c 'import sys;print(".".join(map(str,sys.version_info[:3])))')). Definir PYTHON=/chemin/python3.10+." >&2
    exit 1
fi

echo "== Installation du tooling robot_control =="
echo "   dossier : $HERE"

if [ "${RECREATE:-0}" = "1" ]; then
    for v in "$VENV" "$VENVDEV"; do
        [ -d "$v" ] && { echo "-- suppression de $v (RECREATE=1)"; rm -rf "$v"; }
    done
fi

# venvpath <dir> -> chemin python (Windows Scripts/ ou POSIX bin/), cree le venv au besoin
venvpy() {
    local dir="$1"
    if   [ -x "$dir/bin/python" ]; then echo "$dir/bin/python"
    elif [ -x "$dir/Scripts/python.exe" ]; then echo "$dir/Scripts/python.exe"
    else
        "$PYBASE" -m venv "$dir" >&2
        if [ -x "$dir/bin/python" ]; then echo "$dir/bin/python"; else echo "$dir/Scripts/python.exe"; fi
    fi
}

# --- venv runtime ---
[ -x "$VENV/bin/python" ] || [ -x "$VENV/Scripts/python.exe" ] || echo "-- creation du venv runtime avec : $PYBASE"
VENVPY="$(venvpy "$VENV")"
echo "-- mise a jour de pip"
"$VENVPY" -m pip install --upgrade pip --quiet
echo "-- installation des dependances runtime (requirements.txt)"
"$VENVPY" -m pip install -r "$REQ"
echo "-- verification des imports"
"$VENVPY" -c "import cv2, numpy, serial; print('  OK  cv2', cv2.__version__, '| numpy', numpy.__version__, '| pyserial', serial.__version__)"

# Le dialecte MAVLink genere doit s'importer et fonctionner SANS pymavlink (stdlib seule).
echo "-- verification du dialecte MAVLink genere (runtime, sans pymavlink)"
"$VENVPY" -c "import sys; sys.path.insert(0, r'$HERE/robot_control'); from communication.mav import bamboo as b; m=b.MAVLink(file=None, srcSystem=255, srcComponent=1); buf=m.bamboo_cmd_vel_encode(0.1,0.0,0.2).pack(m); assert m.parse_buffer(buf)[0].get_type()=='BAMBOO_CMD_VEL'; print('  OK  dialecte bamboo (crc:', b.x25crc.__name__, ')')"

# --- toolchain de developpement (venv separe .venv-dev) ---
if [ "$DEV" = "1" ]; then
    echo ""
    echo "== Toolchain de developpement (.venv-dev) =="
    [ -x "$VENVDEV/bin/python" ] || [ -x "$VENVDEV/Scripts/python.exe" ] || echo "-- creation du venv dev avec : $PYBASE"
    DEVPY="$(venvpy "$VENVDEV")"
    "$DEVPY" -m pip install --upgrade pip --quiet
    echo "-- installation du toolchain dev (requirements-dev.txt : pymavlink + platformio)"
    "$DEVPY" -m pip install -r "$REQDEV"
    echo "-- verification du toolchain dev"
    "$DEVPY" -c "import pymavlink; print('  OK  pymavlink', pymavlink.__version__)"
    if [ -x "$VENVDEV/bin/pio" ]; then "$VENVDEV/bin/pio" --version
    else "$VENVDEV/Scripts/pio.exe" --version; fi
fi

echo ""
echo "Termine. Lancer l'outil depuis ce dossier :"
echo "  ./.venv/bin/python -m robot_control.main --no-motion   # vision + servos"
echo "  ./.venv/bin/python -m robot_control.main               # + moteurs (roues surelevees !)"
echo "  (couper le MCP bambou-board avant : il tient le port serie)"
if [ "$DEV" = "1" ]; then
    echo ""
    echo "Toolchain dev (.venv-dev) :"
    echo "  bash firmware/_common/protocol/mavlink/regen.sh         # regenerer le dialecte MAVLink"
    echo "  ./.venv-dev/bin/pio run -d ../firmware/stm32_bamboo     # compiler un firmware"
else
    echo ""
    echo "Pour le poste de dev complet (regen MAVLink + build firmware) : ./install.sh --dev"
fi

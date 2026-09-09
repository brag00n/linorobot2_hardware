#!/usr/bin/env bash
# Installe l'environnement Python du tooling robot_control (venv + dependances).
# Cree .venv DANS ce dossier tools/ et y installe requirements.txt. Idempotent.
# Requiert Python 3.10+. Usage :
#   ./install.sh                 # detection auto de python3
#   PYTHON=/usr/bin/python3.13 ./install.sh
#   RECREATE=1 ./install.sh      # recree le venv de zero
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"
REQ="$HERE/requirements.txt"

# interpreteur de base : $PYTHON, sinon python3, sinon python
PYBASE="${PYTHON:-}"
if [ -z "$PYBASE" ]; then
    if command -v python3 >/dev/null 2>&1; then PYBASE="$(command -v python3)"
    elif command -v python >/dev/null 2>&1; then PYBASE="$(command -v python)"
    else echo "Aucun Python trouve. Installe Python 3.10+." >&2; exit 1; fi
fi

echo "== Installation du tooling robot_control =="
echo "   dossier : $HERE"

if [ "${RECREATE:-0}" = "1" ] && [ -d "$VENV" ]; then
    echo "-- suppression du venv existant (RECREATE=1)"
    rm -rf "$VENV"
fi

# le venv Windows met python sous Scripts/, POSIX sous bin/
if [ -x "$VENV/bin/python" ]; then VENVPY="$VENV/bin/python"
elif [ -x "$VENV/Scripts/python.exe" ]; then VENVPY="$VENV/Scripts/python.exe"
else
    echo "-- creation du venv avec : $PYBASE"
    "$PYBASE" -m venv "$VENV"
    if [ -x "$VENV/bin/python" ]; then VENVPY="$VENV/bin/python"; else VENVPY="$VENV/Scripts/python.exe"; fi
fi

echo "-- mise a jour de pip"
"$VENVPY" -m pip install --upgrade pip --quiet
echo "-- installation des dependances (requirements.txt)"
"$VENVPY" -m pip install -r "$REQ"
echo "-- verification des imports"
"$VENVPY" -c "import cv2, numpy, serial; print('  OK  cv2', cv2.__version__, '| numpy', numpy.__version__, '| pyserial', serial.__version__)"

echo ""
echo "Termine. Lancer l'outil depuis ce dossier :"
echo "  ./.venv/bin/python -m robot_control.main --no-motion   # vision + servos"
echo "  ./.venv/bin/python -m robot_control.main               # + moteurs (roues surelevees !)"
echo "  (couper le MCP bambou-board avant : il tient le port serie)"

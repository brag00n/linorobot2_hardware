#!/usr/bin/env bash
# Regenere le code MAVLink du dialecte Bamboo depuis la source UNIQUE bamboo.xml :
#   - C (header-only, wire 2.0) -> firmware/_common/protocol/mavlink/generated/c/**  (vendore,
#     checke dans git, inclus par les 3 firmwares via -I ; builds reproductibles sans mavgen)
#   - Python -> tools/robot_control/communication/mav/bamboo.py  (module dialecte de l'hote,
#     autonome : n'importe QUE la stdlib a l'execution, pymavlink requis seulement ici)
#
# mavgen resout <include> RELATIVEMENT au dossier du XML source. On stage donc bamboo.xml et
# la fermeture des includes standard (common -> standard -> minimal, fournis par pymavlink)
# cote a cote dans un dossier temporaire avant la generation.
#
# Prerequis : venv tools/.venv-dev avec pymavlink installe (cf. install.sh --dev).
# Usage : bash firmware/_common/protocol/mavlink/regen.sh   (depuis n'importe ou ; chemins resolus)
set -euo pipefail

# Seed de hash fixe : mavgen derive MAVLINK_PRIMARY_XML_HASH du hash() Python des chaines,
# sale par processus par defaut -> sortie C non reproductible d'un run a l'autre. On le fige
# pour que regen produise un diff vide (le hash ne sert qu'a l'introspection mavlink_get_info,
# ni au framing ni au CRC_EXTRA).
export PYTHONHASHSEED=0

# --- resolution des chemins (racine depot = firmware/_common/protocol/mavlink -> remonte de 4) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
# mavgen (pymavlink) vit dans le venv de DEV (tools/.venv-dev, cf. install.sh --dev) ; on
# retombe sur tools/.venv si pymavlink y a ete installe a la main. venv Windows (Scripts)
# ou POSIX (bin) selon la plateforme.
PY=""; MAVGEN=""
for V in .venv-dev .venv; do
  if   [ -f "$REPO_ROOT/tools/$V/Scripts/mavgen.py" ]; then PY="$REPO_ROOT/tools/$V/Scripts/python.exe"; MAVGEN="$REPO_ROOT/tools/$V/Scripts/mavgen.py"; break
  elif [ -f "$REPO_ROOT/tools/$V/bin/mavgen.py" ];     then PY="$REPO_ROOT/tools/$V/bin/python";         MAVGEN="$REPO_ROOT/tools/$V/bin/mavgen.py";     break; fi
done
[ -n "$MAVGEN" ] || { echo "mavgen introuvable : installer le toolchain de dev (install.sh --dev)" >&2; exit 1; }

SRC_XML="$SCRIPT_DIR/bamboo.xml"
OUT_C="$SCRIPT_DIR/generated/c"
OUT_PY="$REPO_ROOT/tools/robot_control/communication/mav/bamboo.py"

[ -f "$SRC_XML" ] || { echo "source introuvable : $SRC_XML" >&2; exit 1; }

# --- dossier des definitions standard fournies par pymavlink ---
DEFS="$("$PY" -c "import pymavlink,os; print(os.path.join(os.path.dirname(pymavlink.__file__),'message_definitions','v1.0'))")"
[ -d "$DEFS" ] || { echo "definitions pymavlink introuvables : $DEFS" >&2; exit 1; }

# --- staging de la fermeture des includes cote a cote ---
STAGE="$(mktemp -d "${TMPDIR:-/tmp}/bamboo_mavgen_XXXXXX")"
cleanup() { rm -rf "$STAGE"; }
trap cleanup EXIT

cp "$SRC_XML" "$STAGE/bamboo.xml"
for x in common.xml standard.xml minimal.xml; do cp "$DEFS/$x" "$STAGE/$x"; done
STAGED_XML="$STAGE/bamboo.xml"

# --- generation C (header-only, wire 2.0) ---
rm -rf "$OUT_C"; mkdir -p "$OUT_C"
echo "[regen] C     -> $OUT_C"
"$PY" "$MAVGEN" --lang=C --wire-protocol=2.0 --output="$OUT_C" "$STAGED_XML"

# --- generation Python (module dialecte hote) ---
mkdir -p "$(dirname "$OUT_PY")"
echo "[regen] Python -> $OUT_PY"
"$PY" "$MAVGEN" --lang=Python --wire-protocol=2.0 --output="$OUT_PY" "$STAGED_XML"

echo "[regen] OK"

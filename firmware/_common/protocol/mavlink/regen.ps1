<#
  Regenere le code MAVLink du dialecte Bamboo depuis la source UNIQUE bamboo.xml :
    - C (header-only, wire 2.0) -> firmware/_common/protocol/mavlink/generated/c/**  (vendore,
      checke dans git, inclus par les 3 firmwares via -I ; builds reproductibles sans mavgen)
    - Python -> tools/robot_control/communication/mav/bamboo.py  (module dialecte de l'hote,
      autonome : n'importe QUE la stdlib a l'execution, pymavlink requis seulement ici)

  mavgen resout <include> RELATIVEMENT au dossier du XML source. On stage donc bamboo.xml et
  la fermeture des includes standard (common -> standard -> minimal, fournis par pymavlink)
  cote a cote dans un dossier temporaire avant de lancer la generation.

  Prerequis : venv tools/.venv-dev avec pymavlink installe (cf. install.ps1 -Dev).
  Usage : pwsh firmware/_common/protocol/mavlink/regen.ps1   (depuis n'importe ou ; chemins resolus)
#>
$ErrorActionPreference = 'Stop'

# Seed de hash fixe : mavgen derive MAVLINK_PRIMARY_XML_HASH du hash() Python des chaines,
# sale par processus par defaut -> sortie C non reproductible d'un run a l'autre. On le fige
# pour que regen produise un diff vide (le hash ne sert qu'a l'introspection mavlink_get_info,
# ni au framing ni au CRC_EXTRA).
$env:PYTHONHASHSEED = '0'

# --- resolution des chemins (racine depot = firmware/_common/protocol/mavlink -> remonte de 4) ---
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot  = (Resolve-Path (Join-Path $ScriptDir '..\..\..\..')).Path
# mavgen (pymavlink) vit dans le venv de DEV (tools/.venv-dev, cf. install.ps1 -Dev) ; on
# retombe sur tools/.venv si pymavlink y a ete installe a la main.
$Py     = $null; $Mavgen = $null
foreach ($v in 'tools\.venv-dev','tools\.venv') {
    $cand = Join-Path $RepoRoot "$v\Scripts\mavgen.py"
    if (Test-Path $cand) { $Mavgen = $cand; $Py = Join-Path $RepoRoot "$v\Scripts\python.exe"; break }
}
if ($null -eq $Mavgen) { throw "mavgen introuvable : installer le toolchain de dev (install.ps1 -Dev)" }
$SrcXml    = Join-Path $ScriptDir 'bamboo.xml'
$OutC      = Join-Path $ScriptDir 'generated\c'
$OutPy     = Join-Path $RepoRoot 'tools\robot_control\communication\mav\bamboo.py'

if (-not (Test-Path $SrcXml)) { throw "source introuvable : $SrcXml" }

# --- dossier des definitions standard fournies par pymavlink ---
$Defs = & $Py -c "import pymavlink,os; print(os.path.join(os.path.dirname(pymavlink.__file__),'message_definitions','v1.0'))"
if (-not (Test-Path $Defs)) { throw "definitions pymavlink introuvables : $Defs" }

# --- staging de la fermeture des includes cote a cote ---
$Stage = Join-Path ([System.IO.Path]::GetTempPath()) ("bamboo_mavgen_" + [System.Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $Stage | Out-Null
try {
    Copy-Item $SrcXml (Join-Path $Stage 'bamboo.xml')
    foreach ($x in 'common.xml','standard.xml','minimal.xml') {
        Copy-Item (Join-Path $Defs $x) (Join-Path $Stage $x)
    }
    $StagedXml = Join-Path $Stage 'bamboo.xml'

    # --- generation C (header-only, wire 2.0) ---
    if (Test-Path $OutC) { Remove-Item -Recurse -Force $OutC }
    New-Item -ItemType Directory -Path $OutC | Out-Null
    Write-Host "[regen] C     -> $OutC"
    & $Py $Mavgen --lang=C --wire-protocol=2.0 --output=$OutC $StagedXml
    if ($LASTEXITCODE -ne 0) { throw "mavgen C a echoue (code $LASTEXITCODE)" }

    # --- generation Python (module dialecte hote) ---
    New-Item -ItemType Directory -Path (Split-Path -Parent $OutPy) -Force | Out-Null
    Write-Host "[regen] Python -> $OutPy"
    & $Py $Mavgen --lang=Python --wire-protocol=2.0 --output=$OutPy $StagedXml
    if ($LASTEXITCODE -ne 0) { throw "mavgen Python a echoue (code $LASTEXITCODE)" }
}
finally {
    Remove-Item -Recurse -Force $Stage -ErrorAction SilentlyContinue
}
Write-Host "[regen] OK"

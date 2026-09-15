<#
.SYNOPSIS
  Installe l'environnement Python du tooling robot_control (venv + dependances).

.DESCRIPTION
  Cree un environnement virtuel .venv DANS ce dossier tools/ et y installe les
  paquets de requirements.txt (RUNTIME de l'app : vision + teleop + MCP). Idempotent :
  relance sans risque (met a jour les paquets). Ne touche ni a COM4 ni a la carte.

  Avec -Dev : provisionne EN PLUS un venv separe .venv-dev avec le toolchain de
  developpement (requirements-dev.txt : pymavlink/mavgen pour regenerer le dialecte
  MAVLink + PlatformIO `pio` pour compiler/flasher les firmwares). Ce toolchain est
  isole du runtime pour ne pas perturber ses versions epinglees ni alourdir le
  deploiement robot/RPi -> reproductible depuis zero (runtime : install.ps1 ; poste de
  dev complet : install.ps1 -Dev).

.PARAMETER Python
  Interpreteur Python de base a utiliser (defaut : detection automatique via
  'py -3' puis 'python'). Requiert Python 3.10+.

.PARAMETER Recreate
  Supprime et recree le(s) venv de zero (utile si casse).

.PARAMETER Dev
  Provisionne aussi le venv de developpement .venv-dev (regen MAVLink + PlatformIO).

.EXAMPLE
  .\install.ps1
  .\install.ps1 -Dev
  .\install.ps1 -Python "C:\Python313\python.exe" -Recreate -Dev
#>
[CmdletBinding()]
param(
    [string]$Python = "",
    [switch]$Recreate,
    [switch]$Dev
)
$ErrorActionPreference = "Stop"
$Here    = $PSScriptRoot
$Venv    = Join-Path $Here ".venv"
$Req     = Join-Path $Here "requirements.txt"
$VenvDev = Join-Path $Here ".venv-dev"
$ReqDev  = Join-Path $Here "requirements-dev.txt"

function Resolve-BasePython {
    param([string]$Explicit)
    if ($Explicit) {
        if (-not (Test-Path $Explicit)) { throw "Python introuvable : $Explicit" }
        return $Explicit
    }
    # 'py -3' est le lanceur recommande sous Windows ; sinon 'python' du PATH.
    $py = Get-Command py -ErrorAction SilentlyContinue
    if ($py) { return @($py.Source, "-3") }
    $p = Get-Command python -ErrorAction SilentlyContinue
    if ($p) { return @($p.Source) }
    throw "Aucun Python trouve (ni 'py -3' ni 'python'). Installe Python 3.10+."
}

Write-Host "== Installation du tooling robot_control ==" -ForegroundColor Cyan
Write-Host "   dossier : $Here"

$base = Resolve-BasePython -Explicit $Python

# Garde-fou de version : les pins runtime (numpy 2.2.x / opencv) exigent Python 3.10+.
& $base[0] $base[1..($base.Length-1)] -c "import sys; raise SystemExit(0 if sys.version_info[:2] >= (3,10) else 1)"
if ($LASTEXITCODE -ne 0) {
    $v = & $base[0] $base[1..($base.Length-1)] -c "import sys;print('.'.join(map(str,sys.version_info[:3])))"
    throw "Python 3.10+ requis (detecte : $v). Installer un Python 3.10+ ou pointer -Python vers lui."
}

if ($Recreate) {
    foreach ($v in @($Venv, $VenvDev)) {
        if (Test-Path $v) { Write-Host "-- suppression de $v (Recreate)"; Remove-Item -Recurse -Force $v }
    }
}

if (-not (Test-Path (Join-Path $Venv "Scripts\python.exe"))) {
    Write-Host "-- creation du venv runtime avec : $($base -join ' ')"
    & $base[0] $base[1..($base.Length-1)] -m venv $Venv
    if ($LASTEXITCODE -ne 0) { throw "echec creation du venv" }
} else {
    Write-Host "-- venv runtime deja present : $Venv"
}

$VenvPy = Join-Path $Venv "Scripts\python.exe"
Write-Host "-- mise a jour de pip"
& $VenvPy -m pip install --upgrade pip --quiet
Write-Host "-- installation des dependances runtime (requirements.txt)"
& $VenvPy -m pip install -r $Req
if ($LASTEXITCODE -ne 0) { throw "echec pip install" }

Write-Host "-- verification des imports"
& $VenvPy -c "import cv2, numpy, serial; print('  OK  cv2', cv2.__version__, '| numpy', numpy.__version__, '| pyserial', serial.__version__)"
if ($LASTEXITCODE -ne 0) { throw "les imports ont echoue" }

# Le dialecte MAVLink genere doit s'importer et fonctionner SANS pymavlink (stdlib seule).
Write-Host "-- verification du dialecte MAVLink genere (runtime, sans pymavlink)"
$RcPath = Join-Path $Here "robot_control"
& $VenvPy -c "import sys; sys.path.insert(0, r'$RcPath'); from communication.mav import bamboo as b; m=b.MAVLink(file=None, srcSystem=255, srcComponent=1); buf=m.bamboo_cmd_vel_encode(0.1,0.0,0.2).pack(m); assert m.parse_buffer(buf)[0].get_type()=='BAMBOO_CMD_VEL'; print('  OK  dialecte bamboo (crc:', b.x25crc.__name__, ')')"
if ($LASTEXITCODE -ne 0) { throw "le dialecte MAVLink genere ne s'importe pas (relancer protocol/mavlink/regen.ps1 ?)" }

# --- toolchain de developpement (venv separe .venv-dev) ---
if ($Dev) {
    Write-Host ""
    Write-Host "== Toolchain de developpement (.venv-dev) ==" -ForegroundColor Cyan
    if (-not (Test-Path (Join-Path $VenvDev "Scripts\python.exe"))) {
        Write-Host "-- creation du venv dev avec : $($base -join ' ')"
        & $base[0] $base[1..($base.Length-1)] -m venv $VenvDev
        if ($LASTEXITCODE -ne 0) { throw "echec creation du venv dev" }
    } else {
        Write-Host "-- venv dev deja present : $VenvDev"
    }
    $DevPy = Join-Path $VenvDev "Scripts\python.exe"
    & $DevPy -m pip install --upgrade pip --quiet
    Write-Host "-- installation du toolchain dev (requirements-dev.txt : pymavlink + platformio)"
    & $DevPy -m pip install -r $ReqDev
    if ($LASTEXITCODE -ne 0) { throw "echec pip install (dev)" }
    Write-Host "-- verification du toolchain dev"
    & $DevPy -c "import pymavlink; print('  OK  pymavlink', pymavlink.__version__)"
    if ($LASTEXITCODE -ne 0) { throw "pymavlink absent du venv dev" }
    & (Join-Path $VenvDev "Scripts\pio.exe") --version
    if ($LASTEXITCODE -ne 0) { throw "PlatformIO (pio) absent du venv dev" }
}

Write-Host ""
Write-Host "Termine. Lancer l'outil depuis ce dossier :" -ForegroundColor Green
Write-Host "  .\.venv\Scripts\python.exe -m robot_control.main --no-motion   # vision + servos"
Write-Host "  .\.venv\Scripts\python.exe -m robot_control.main               # + moteurs (roues surelevees !)"
Write-Host "  (couper le MCP bambou-board avant : il tient COM4)"
if ($Dev) {
    Write-Host ""
    Write-Host "Toolchain dev (.venv-dev) :" -ForegroundColor Green
    Write-Host "  pwsh firmware\_common\protocol\mavlink\regen.ps1             # regenerer le dialecte MAVLink"
    Write-Host "  .\.venv-dev\Scripts\pio.exe run -d ..\firmware\stm32_bamboo  # compiler un firmware"
} else {
    Write-Host ""
    Write-Host "Pour le poste de dev complet (regen MAVLink + build firmware) : .\install.ps1 -Dev" -ForegroundColor DarkGray
}

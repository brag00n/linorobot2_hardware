<#
.SYNOPSIS
  Installe l'environnement Python du tooling robot_control (venv + dependances).

.DESCRIPTION
  Cree un environnement virtuel .venv DANS ce dossier tools/ et y installe les
  paquets de requirements.txt. Idempotent : relance sans risque (met a jour les
  paquets). Ne touche ni a COM4 ni a la carte.

.PARAMETER Python
  Interpreteur Python de base a utiliser (defaut : detection automatique via
  'py -3' puis 'python'). Requiert Python 3.10+.

.PARAMETER Recreate
  Supprime et recree le venv de zero (utile si casse).

.EXAMPLE
  .\install.ps1
  .\install.ps1 -Python "C:\Python313\python.exe" -Recreate
#>
[CmdletBinding()]
param(
    [string]$Python = "",
    [switch]$Recreate
)
$ErrorActionPreference = "Stop"
$Here = $PSScriptRoot
$Venv = Join-Path $Here ".venv"
$Req  = Join-Path $Here "requirements.txt"

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

if ($Recreate -and (Test-Path $Venv)) {
    Write-Host "-- suppression du venv existant (Recreate)"
    Remove-Item -Recurse -Force $Venv
}

if (-not (Test-Path (Join-Path $Venv "Scripts\python.exe"))) {
    $base = Resolve-BasePython -Explicit $Python
    Write-Host "-- creation du venv avec : $($base -join ' ')"
    & $base[0] $base[1..($base.Length-1)] -m venv $Venv
    if ($LASTEXITCODE -ne 0) { throw "echec creation du venv" }
} else {
    Write-Host "-- venv deja present : $Venv"
}

$VenvPy = Join-Path $Venv "Scripts\python.exe"
Write-Host "-- mise a jour de pip"
& $VenvPy -m pip install --upgrade pip --quiet
Write-Host "-- installation des dependances (requirements.txt)"
& $VenvPy -m pip install -r $Req
if ($LASTEXITCODE -ne 0) { throw "echec pip install" }

Write-Host "-- verification des imports"
& $VenvPy -c "import cv2, numpy, serial; print('  OK  cv2', cv2.__version__, '| numpy', numpy.__version__, '| pyserial', serial.__version__)"
if ($LASTEXITCODE -ne 0) { throw "les imports ont echoue" }

Write-Host ""
Write-Host "Termine. Lancer l'outil depuis ce dossier :" -ForegroundColor Green
Write-Host "  .\.venv\Scripts\python.exe -m robot_control.main --no-motion   # vision + servos"
Write-Host "  .\.venv\Scripts\python.exe -m robot_control.main               # + moteurs (roues surelevees !)"
Write-Host "  (couper le MCP bambou-board avant : il tient COM4)"

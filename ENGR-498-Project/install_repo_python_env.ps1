param(
  [string]$Python = "",
  [switch]$UseLockFile,
  [switch]$IncludeMatlabEngine
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

if ([string]::IsNullOrWhiteSpace($Python)) {
  $Python = "python"
}

$requirements = if ($UseLockFile) {
  Join-Path $scriptDir "requirements-repo-python-lock.txt"
} else {
  Join-Path $scriptDir "requirements-repo-python.txt"
}

if (-not (Test-Path -LiteralPath $requirements)) {
  throw "Could not find requirements file: $requirements"
}

Write-Host "Installing repo Python dependencies using: $Python"
Write-Host "Requirements file: $requirements"
& $Python -m pip install -r $requirements

if ($IncludeMatlabEngine) {
  $matlabRequirements = if ($UseLockFile) {
    Join-Path $scriptDir "requirements-repo-python-matlab-lock.txt"
  } else {
    Join-Path $scriptDir "requirements-repo-python-matlab.txt"
  }

  if (-not (Test-Path -LiteralPath $matlabRequirements)) {
    throw "Could not find MATLAB requirements file: $matlabRequirements"
  }

  Write-Host "Installing optional MATLAB Python bridge dependencies from: $matlabRequirements"
  & $Python -m pip install -r $matlabRequirements
}

Write-Host "Repo Python environment installation complete."

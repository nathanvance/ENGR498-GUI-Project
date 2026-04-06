param(
  [string]$Python = ""
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$requirements = Join-Path $scriptDir "requirements-local-inference.txt"

if (-not (Test-Path -LiteralPath $requirements)) {
  throw "Could not find requirements file: $requirements"
}

if ([string]::IsNullOrWhiteSpace($Python)) {
  $Python = "python"
}

Write-Host "Installing local inference dependencies using: $Python"
& $Python -m pip install -r $requirements

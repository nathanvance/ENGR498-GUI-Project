Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$fusionRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$pythonCmd = Get-Command python -ErrorAction SilentlyContinue
$pyLauncher = Get-Command py -ErrorAction SilentlyContinue
$port = 8765
$url = "http://localhost:$port/leaflet_viewer/index.html"
$urlObjectsExample = "http://localhost:$port/leaflet_viewer/index.html?data=../outputs/fused_objects.json"
$urlPowerlinesExample = "http://localhost:$port/leaflet_viewer/index.html?powerlines=../outputs/eng498_powerlines_overlay.json"
$urlCombinedExample = "http://localhost:$port/leaflet_viewer/index.html?data=../outputs/fused_objects.json&powerlines=../outputs/eng498_powerlines_overlay.json"

Write-Host "Serving Fusion from $fusionRoot"
Write-Host "Open this URL in your browser:"
Write-Host $url
Write-Host ""
Write-Host "Example URL with Fusion objects loaded:"
Write-Host $urlObjectsExample
Write-Host ""
Write-Host "Example URL with generated powerline overlay only:"
Write-Host $urlPowerlinesExample
Write-Host ""
Write-Host "Example URL with both layers loaded (only use when they share a common frame or GPS):"
Write-Host $urlCombinedExample
Write-Host ""
Write-Host "Press Ctrl+C to stop the server."

Push-Location $fusionRoot
try {
    if ($env:FUSION_PYTHON) {
        & $env:FUSION_PYTHON -m http.server $port
    }
    elseif ($pythonCmd) {
        & $pythonCmd.Source -m http.server $port
    }
    elseif ($pyLauncher) {
        & $pyLauncher.Source -3 -m http.server $port
    }
    else {
        throw "Could not find a Python interpreter. Set FUSION_PYTHON or install python/py on PATH."
    }
}
finally {
    Pop-Location
}

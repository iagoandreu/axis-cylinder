# Push this folder into Fusion's AddIns directory.
# AxisCylinder.py is the shell - changing it needs the add-in stopped and re-run.
# ac_core.py and ac_fit.py hot-reload, so pushing those takes effect on the next click.
$src = Split-Path -Parent $MyInvocation.MyCommand.Definition
$dst = Join-Path $env:APPDATA 'Autodesk\Autodesk Fusion 360\API\AddIns\AxisCylinder'

New-Item -ItemType Directory -Force -Path $dst | Out-Null

$cache = Join-Path $dst '__pycache__'
if (Test-Path -LiteralPath $cache) { Remove-Item -LiteralPath $cache -Recurse -Force }

foreach ($f in @('AxisCylinder.py', 'ac_core.py', 'ac_fit.py', 'AxisCylinder.manifest')) {
    Copy-Item -LiteralPath (Join-Path $src $f) -Destination $dst -Force
}
Copy-Item -LiteralPath (Join-Path $src 'resources') -Destination $dst -Recurse -Force

# the dev bridge is opt-in: the marker file is only copied if it exists here
$flag = Join-Path $src 'devbridge.on'
$dstFlag = Join-Path $dst 'devbridge.on'
if (Test-Path -LiteralPath $flag) {
    Copy-Item -LiteralPath $flag -Destination $dst -Force
} elseif (Test-Path -LiteralPath $dstFlag) {
    Remove-Item -LiteralPath $dstFlag -Force
}

Write-Host "Installed to $dst"
if (Test-Path -LiteralPath $dstFlag) { Write-Host "dev bridge: ON" } else { Write-Host "dev bridge: off" }

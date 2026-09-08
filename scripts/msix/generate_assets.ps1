# Generate MSIX assets from the supplied Narwhal master, preserving alpha.
# Run locally when the logo changes, then commit assets/. CI uses those PNGs.
# Prerequisite: Python 3 with Pillow (python -m pip install Pillow).
param(
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'

$generator = Join-Path (Split-Path $PSScriptRoot -Parent) 'generate_narwhal_branding.py'
& $Python $generator --msix-only
if ($LASTEXITCODE -ne 0) {
    throw 'Narwhal asset generation failed. Install Pillow for the selected Python.'
}

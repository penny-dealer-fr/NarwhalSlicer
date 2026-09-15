# Narwhal artwork

The user-supplied 1000 × 1000 RGBA PNG is stored without modification at
`resources/images/NarwhalSlicer.png`. Its rounded square, horn, colors, and
transparent corners are preserved. All derived square assets use uniform
Lanczos resizing, with no cropping or stretching.

The supplied `resources/Icon.icns` remains the macOS application icon. Its
original representations are preserved byte for byte, with missing 16, 32,
and 64 pixel 1x PNG representations appended. The document icons are separate
folded-page designs containing the Narwhal artwork and PROJECT, STL, or GCODE
labels.

| Use | Resource | Dimensions / representations |
| --- | --- | --- |
| General dialogs and wizard | `images/NarwhalSlicer.png` | 1000 × 1000 master; scaled by `ScalableBitmap` |
| Linux launchers / window icons | `images/NarwhalSlicer_<size>px.png` | 16, 32, 48, 64, 128, 192, 256, 512 square |
| Windows executable, installer, dialog titles / macOS dock | `images/NarwhalSlicer.ico` | 16, 24, 32, 48, 64, 128, 256; 32-bit RGBA BMP frames |
| Error dialog | `images/NarwhalSlicer_192px_grayscale.png` | 192 × 192, retained alpha |
| About light/dark | `images/NarwhalSlicer_about_{light,dark}.png` | 1680 × 375; 560 × 125 logical layout |
| Troubleshoot light/dark | `images/NarwhalSlicer_horizontal_{light,dark}.png` | 642 × 240; original 214:80 aspect ratio |
| Splash light/dark | `images/NarwhalSlicer_splash_{light,dark}.png` | 1440 × 1440; 480 × 480 logical layout |
| macOS document associations | `images/NarwhalSlicer.icns`, `stl.icns`, `gcode.icns` | 16–1024 px with standard and Retina slots |
| G-code viewer | `images/NarwhalSlicer-gcodeviewer.ico`, `NarwhalSlicer-gcodeviewer_192px.png` | Same ICO frame sizes; 192 × 192 PNG |
| Web home / welcome | `web/image/logo.png` | 308 × 308; explicit 96 / 154 px display sizes |
| Legacy portrait web slot | `web/image/logo2.png` | 678 × 812; square artwork centered without distortion |
| Windows MSIX | `scripts/msix/assets/*.png` | 44 × 44 (two forms), 50 × 50, 150 × 150 |
| Custom printer bed | `profiles/Custom/narwhalslicer_bed_texture.png` | 2048 × 2048 transparent canvas; bottom-center mark and corner guides |

The native SVG loader is NanoSVG and does not support embedded raster images.
Branding therefore uses native PNG loading, including the splash. Light and
dark wordmarks are rendered at 3× resolution with bundled HarmonyOS Sans
fonts. The PNG cache now preserves Retina backing pixels and scale metadata,
matching the SVG path without changing logical dimensions. The splash leaves the bottom 30% transparent for native version,
status, and progress rendering. About leaves the right side clear for live
version and build labels.

All app/logo references use the new resource filenames. Linux installation
still uses the existing desktop-entry icon identifiers (`OrcaSlicer.png`) so
launchers continue to resolve the new artwork. Flatpak installs a real 512 px
PNG in the 512×512 icon directory. Both macOS plist templates point to the
new document icons; the packaging plist uses `Icon.icns` for the app itself.

The Custom vendor revision is bumped to `02.04.00.04`, and all five custom
printer model definitions point to the new bed PNG. Manufacturer/service
artwork, calibration models, historical screenshots, and upstream project
identifiers are outside this app-logo pass. The Troodon texture's filename
contains OrcaSlicer but its artwork is the printer's TROODON 2 label.

## Regeneration

With Python 3 and Pillow installed:

```sh
python scripts/generate_narwhal_branding.py
```

The generator uses checked-in artwork and fonts, and supports
`--msix-only`. `scripts/msix/generate_assets.ps1` delegates to that mode so
future package asset regeneration also uses Narwhal artwork. No added build
or runtime dependency is required; generated assets are checked in.

## Verification

- Source PNG matches the attachment exactly. Every original ICNS chunk is
  retained, and the completed app icon contains 16–1024 px representations.
- PNG dimensions, source-pixel resizing, grayscale alpha, ICO frames, ICNS
  frames, splash text clearance, 37 literal native/web logo references, and
  both macOS plist icon paths were checked.
- The actual PNG-loading method and native wx bitmap conversion were exercised
  against wxWidgets at 1×, 2×, and 3×: backing dimensions, logical dimensions,
  aspect ratio, cache reuse, grayscale, transparency, centered padding, and
  missing-file handling passed.
- Local QA assets and the verification harness are in `build/branding-qa/`.

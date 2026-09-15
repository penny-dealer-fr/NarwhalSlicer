# Repository Guidelines

## Architecture and Feature Placement

Keep strength computation independent of the GUI.

- `src/libslic3r/` owns the model, configuration, slicing stages, G-code, and formats. Put simulation, loads, orientation, and modifier generation here for headless testing.
- `src/slic3r/GUI/` owns wxWidgets views, Plater interaction, gizmos, and `Jobs`. Long analysis must be cancellable and off the UI thread.
- `src/libvgcode/` renders G-code; `resources/` holds profiles and UI assets.

Use native `ModelVolumeType::PARAMETER_MODIFIER` volumes for localized reinforcement; do not patch emitted G-code.

## Building

Dependencies and the application build separately. Platform scripts supply required CMake and packaging options.

- macOS: `./build_release_macos.sh -a arm64` for everything; with dependencies built, use `./build_release_macos.sh -s -a arm64`. The current Xcode Release tree is `build/arm64`.
- Linux: `./build_linux.sh -u` once, `./build_linux.sh -dsi` initially, then `./build_linux.sh -s`.
- Windows: `build_release_vs.bat`, then `build_release_vs.bat slicer` for app-only rebuilds.

## Tests

Tests are off by default. Build and run them with `./build_release_macos.sh -s -a arm64 -T` or `./build_linux.sh -st`. On Windows, run `build_release_vs.bat tests`, then `ctest --test-dir build/tests -C Release --output-on-failure`. Rerun macOS tests with `./scripts/run_unit_tests.sh build/arm64/tests Release`.

Put Catch2 files named `test_<subsystem>.cpp` in the matching suite and register them in its `CMakeLists.txt`. Use behavioral names and PascalCase tags; follow `tests/AGENTS.md`.

## Coding Conventions

Use C++17 unless surrounding code requires C++20. Follow `.clang-format`: four spaces, no tabs, 140 columns, left-aligned pointers, and unchanged include order. Types use `PascalCase`; functions and variables use `snake_case`. Prefer RAII; avoid shared mutable state in TBB work.

Slicer settings belong in `PrintConfig`, must trigger the correct pipeline invalidation, and must serialize deliberately. Wrap visible strings with `_L`/`_u8L`.

## UI Consistency

Match neighboring panels. Reuse `GUI/Widgets`, `Label::Head_*`/`Body_*`, `StateColor`, and scalable bitmap/button classes. Size through `FromDIP()` or `em_unit()`; avoid hard-coded fonts, colors, or unscaled pixels. Check light/dark themes, HiDPI, localization expansion, and keyboard use. Top-level windows should call `SetSizerAndFit`, or `SetSizeHints` after an early `SetSizer`.

## Verification and Fork Constraints

Build the affected target and run focused tests. Slicing changes require the full suite and a representative slice; UI changes require light/dark and HiDPI checks with screenshots. Project-state changes require a `.3mf` save/reload round trip. Confirm disabled features preserve existing behavior and G-code.

Maintain OrcaSlicer project/profile compatibility and all three desktop platforms. Add migrations for format changes; bump the sibling vendor JSON version for profile edits. Localize fork changes; avoid unrelated renames or formatting. New dependencies must build reproducibly through `deps/` and be license-compatible on every platform.

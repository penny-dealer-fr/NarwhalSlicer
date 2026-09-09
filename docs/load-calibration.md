# CNC Testhook load calibration

Calibration → Load creates a one-parameter sweep from the current printer, process, and first filament. The supplied `testHook_V2 (1).stl` is bundled, unchanged, as **CNC Testhook** in the convenient-model menu.

1. Select printer/filament/process settings in Prepare. Configure support and brim for upright hooks as needed.
2. In Calibration → Load, enter material name, material type, and color. Choose the parameter and comma-separated values, or use start/end/increment to build a numeric range. Enum options list their valid serialized values beneath the form.
3. Choose repeats per value **per orientation**. Three repeats at three values produce 18 hooks: nine XY and nine Z.
4. Generate and slice the study. XY preserves the supplied flat geometry (35 × 70 × 9 mm); Z rotates it 90° about X, putting the 70 mm axis upright. Every hook gets its own plate and stable ID, for independent cooling and unambiguous results. Each plate is exported to a native `.3mf` and `.gcode` in the study folder. Completed exports get completion records; Cancel and Resume preserve completed work.
5. Use Open study folder to access the outputs, or select a result row and Open selected hook in Prepare to inspect the native project. Physical hook IDs should match the file/plate labels; the specimen geometry is not embossed or altered.
6. Record every specimen as Failed, Did not break (maximum tested load), or Not tested. Notes distinguish print failures, lost parts, and omitted tests. Load units can be N, kgf, or lbf and convert when changed. Optional mass is the actual hook mass without support/brim. Valid table edits save automatically; Save results and calculate also persists metadata.
7. Reopen saved studies to continue results later. Select one setting value, a base mechanical material, and either a new or existing calibrated material when publishing to the Load material selector.

## What the calculations mean

Results remain separated by parameter value and orientation. Only actual tensile failures contribute to the mean, minimum, sample standard deviation, and mean failure-force/mass ratio. Survived samples are censored lower bounds, and untested specimens are excluded. The table retains both.

Failure force in newtons is a property of the specimen, fixture, and print settings; it is not itself a material stress. Publishing an effective tensile calibration requires validated **peak tensile stress per applied force (MPa/N)** for the actual hook and fixture in XY and Z. These factors can come from a separately validated unit-load model. Do not substitute an arbitrary cross-sectional area: this hook carries nonuniform stress. The factors and selected results are retained with the calibration.

Publishing requires at least two failures in each orientation and no survived specimens in the selected group. The minimum measured failure force multiplied by the corresponding stress/force factor sets each effective ultimate tensile limit. Existing yield estimates are capped at that ultimate limit; they are not measured yield values. Elastic moduli, shear properties, Poisson ratio, and density retain the base material's values. Weight is used for N/g comparisons and does not infer density or elastic properties.

Calibrated material names appear as `(Material) -- CALIBRATED`. They retain the material type, color, source study, parameter, and value in provenance. Updating an existing entry preserves its key. Previously saved load-analysis projects retain their embedded material snapshot until the user explicitly selects the updated library entry.

## Scope and persistence

Supported sweeps: infill pattern/density, wall loops, top/bottom surface patterns, layer height, line width, minimum layer time, nozzle and bed temperatures, plate type, top/bottom shell counts, outer/inner wall and infill speeds, maximum fan speed, flow ratio, and infill direction. Nozzle/bed temperature sweeps update initial-layer temperatures too. Minimum layer time is a cooling target; actual layers may take longer.

Studies allow 1–50 unique setting values, 1–20 repeats per orientation, and at most 200 total hooks. Each output file is a separate plate, so the in-memory 36-plate limit does not constrain the study. The current implementation always uses one hook per plate. It preserves the baseline geometry and setting snapshot instead of packing dissimilar thermal tests together.

Data is local in `<slicer data directory>/load_calibration/`: versioned study JSON, an exact copy of the hook STL, per-hook project/G-code/completion files, and a versioned `materials.json` library. Generation runs on a cancellable background worker and leaves the current Prepare project unchanged. Failed exports do not receive completion records and are retried by Resume.

## Verification

`[LoadCalibration]` in `libslic3r_tests` covers range bounds, identity, censored observations, statistics, invalid inputs, mechanical-property preservation, native setting application, geometry orientation, printer-bed fit, and native `.3mf` round trips. The matching `fff_print_tests` test slices the real bundled STL in both orientations and checks settings and extrusion output.

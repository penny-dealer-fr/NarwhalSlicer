# Strength analysis verification status

Audit date: 2026-09-06. This is a progress record, **not a release sign-off**. The Todoist project has 21 completed items and no open items, but that administrative state does not prove all requested functionality is finished.

## Verified in this change

- macOS arm64 Release application build succeeds.
- Full core suite: 331 cases, 171,626 assertions, randomized order, passing.
- Focused strength suite: 28 cases, 113,551 assertions, passing.
- Full FFF suite: 145 cases, 3,545 assertions, randomized order, passing.
- The slicing regression creates the same native parameter-modifier mesh and settings used by the GUI. It checks 15%, 50%, and 100% selected model volume, increasing actual generated infill, replacement, removal, and restored/redo model states.
- Geometry regressions cover separate stress concentrations, low-stress gaps, cavities, thin disconnected solids, sloped faces, tetrahedra, preserve shapes, cancellation, stale profiles, zero/full targets, and threshold-to-volume conversion.
- Both standard 3MF and native Orca project 3MF round trips retain modifier type, ownership marker, density, and pattern. The Orca path also checks generated modifier volume.
- Orientation alignment is tested for pre-rotated, nonuniformly scaled, and mirrored instances, including repeated application and preservation of offsets/scales.
- Pre-check text/background color pairs have calculated contrast above 5.8:1 in both themes. This does not replace visual UI testing.

Tests are in `tests/libslic3r/test_strength_analysis.cpp` and `tests/fff_print/test_printobject.cpp`. The current Xcode test targets compile/link but automatic Catch discovery fails when macOS rejects the unsigned test executables. Ad-hoc signing the locally built `.app` test bundles and invoking their executables directly produces the passing results above. No paid solver, cloud service, reset credit, or purchased dependency was used.

## Original Todoist feature audit

| Original item | Current evidence and remaining work |
| --- | --- |
| Load tab | Native tab, study tree, viewport placement, operation panel and dialogs exist and compile. Interactive usability checks remain open. |
| Simulation Tab | Native results tab, camera controls, legends and overlays exist and compile. Interactive checks remain open. |
| Safety Factor (Load) | Directional, per-load allowable-strength calculations have core tests. Bundled defaults are generic estimates, not a sourced material-specific validation dataset. |
| Apply stresses | All six supported load/constraint types have resultant-force tests. Bearing and impact are equivalent-static approximations, not contact/dynamic analyses. UI placement checks remain open. |
| Per-Material Safety Factor calibration for more accurate results | Editable calibration and serialization are tested. Actual printed-coupon calibration remains user/material dependent. |
| Directions matter | Changing layer direction changes the calculated anisotropic response in tests. Numerical responses are not a substitute for experimental validation. |
| Maybe variable settings to find optimal settings | Candidate ranking covers infill, wall count, and layer height. Temperature, cooling, and minimum-layer-time recommendations are **not implemented**. |
| Preserve Regions | Sphere, box, cylinder, and selected-face exclusion have geometry tests; the slicer retains the original external part shape. |
| Gravity | Self-weight and density scaling have tests. Reinforcement previews intentionally withhold revised safety factor/deformation when added mass changes self-weight. |
| Material | Editable properties and generic PLA/PETG/ABS/PA/PC/TPU estimates exist. Sourced defaults and a specified printer/material calibration dataset remain open. |
| Change orientation for strength or support | Candidate scoring and native instance alignment have core coverage; the GUI action accounts for existing transforms. End-to-end applied-instance orientation still needs interactive verification. |
| Global Loads | Equivalent-static resultant and gravity interactions have core coverage. |
| Criteria | Safety factor, displacement, mass/stiffness/support/time weights exist. Solver vertex limits are not mesh-size optimization; triangle-count/mesh-size optimization is **not implemented**. |
| Estimated Mass at different strength sizes | Mass/strength comparisons and reinforced-volume sizing have core tests. Exact print mass still comes from slicing all walls, solid layers, supports, and infill. |
| Dense Region and Other region infill density and pattern | Native modifier/background settings and generated FFF infill are tested together, including replacement and removal. |
| Strength by infil pattern | Deterministic pattern comparisons exist and are tested, using disclosed heuristic factors rather than calibrated lattice FEA. |
| Safety Factor (Simulation) | Core results tested; reversed rainbow scalar display and probing require interactive visual verification. |
| Simulated Deformation | Finite core displacement results tested; magnified/deformed display requires interactive visual verification. |
| Simulated Stress | Core stress results tested; contour/legend interaction requires interactive visual verification. |
| Simulated Sheer? | Core shear results tested; signed component legends require interactive visual verification. |
| Point Probe | Native viewport hit-testing and probe readout are implemented. End-to-end click/contour agreement remains open. |

The solver remains a linear-elastic edge-network engineering approximation. It does not establish full Fusion simulation/topology-optimization equivalence. Preview safety-factor changes scale the baseline local response without re-solving redistributed load paths. See [workflow and interpretation limits](strength-analysis-workflow.md).

## Native UI gate still open

macOS denied the UI-control attempt with “osascript is not allowed assistive access.” Accessibility permission is needed before that interaction route can be used. A screenshot of the app's Prepare tab is not evidence that the Load/Simulation controls work. No alternate input route was used to bypass the denial.

The remaining interactive checklist is:

- Light/dark screenshots; standard/HiDPI scaling; narrow-window and localized-label layout; keyboard focus and Delete/Backspace behavior.
- Place, edit, drag, delete, undo, and redo supports, forces, gravity, and preserve regions. Confirm dragged positions survive pre-check and solve.
- Confirm invalid numeric input gives an orange pre-check state and corrected inputs give green. Confirm stale results cannot be applied and canceled/obsolete workers cannot overwrite current state.
- Inspect 0%, intermediate, and 100% volume masks; use the setup threshold and reachable/unreachable safety-factor targets; verify baseline-only response labels with gravity.
- Apply, replace, remove, Undo/Redo, and save/reopen the modifier through the UI. Inspect actual generated infill in Preview and confirm existing user modifiers remain untouched.
- Check instance scaling/rotation and multiple-object changes against the analysis coordinate frame; raw-mesh unit tests alone do not prove every transformed-instance workflow.

The unrelated local `resources/Icon.icns` change is intentionally outside this feature's changes until its owner confirms whether to include it. It must not be discarded just to report a clean worktree.

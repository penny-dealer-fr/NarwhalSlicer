# Strength analysis verification status

Audit date: 2026-09-06. This is a progress record, **not a release sign-off**. The Todoist project has 21 completed items and no open items, but that administrative state does not prove all requested functionality is finished.

## Follow-up: 2026-09-07

- Pre-check now recalculates readiness when the selected object or accepted setup changes, without requiring a button click. Invalid numeric text is tracked separately from the last accepted setup, so it cannot leave the button or study tree showing READY. Undo/Redo and restored setups reset that input state.
- The arm64 Release app rebuilt successfully after this change. The focused strength suite passed again: 28 cases, 113,551 assertions. These core tests exercise validation, not native widget events; interactive readiness checks remain open.
- Initial independent source review found missing default Prepare-orientation synchronization: the study loader read the raw object mesh and stored layer axis, without tracking selected-instance transforms. Existing alignment tests covered applying a recommendation, not synchronization. This prompted the follow-up below; physical scaled-instance geometry/load/modifier coordinate handling remains open.
- Subsequent implementation adds a saved, default-on Follow Prepare choice, derives the layer normal from the selected instance, and invalidates results on identity/transform changes during study synchronization. Ambiguous multi-instance selections require choosing one copy; applying a recommendation rotates only that copy. Six new tests cover plane-normal mapping, invalid transforms, and default/manual mode serialization; the standard 3MF round trip now checks both modes. Full core results: 337 cases, 171,777 assertions, passing. The app builds. Interactive synchronization and physical scaled-instance analysis remain unverified/incomplete respectively.
- The FFF reinforcement regression now runs in both orientation-follow modes. Fresh compilation and the full randomized slicing suite passed: 145 cases, 4,109 assertions. Xcode initially reused the old test object despite the changed setup header; that stale-object run was not accepted as verification of this revision.

## Verified in this change

Latest checkpoint (2026-09-07): physical instance scaling now affects mechanical distances/areas, mass, gravity, layer-normal mapping, and physical-distance stress interpolation. Raw region selections and modifier coordinates remain attached to the original object. Regression checks compare nonuniform scaling with explicitly resized geometry, test uniform force/mass similarity, and round-trip a scaled instance and its setup through 3MF.

The scaled slicing regression exposed an existing overlap-cache defect: removing the combined instance-and-volume XY translation could make a valid offset modifier appear outside its parent. The bounding-box transform now removes only shared instance placement. A second check found non-manifold voxel edge contacts; exact selected cells are now partitioned into closed box pieces for native slicing. The tests check closed edges, double-precision modifier volume, direct mesh slices, and actual generated dense infill at 15%, 50%, and 100% on unscaled, uniformly scaled, and nonuniformly scaled instances in both orientation modes.

Final tests for this checkpoint: **344 core cases / 283,143 assertions**, **145 FFF cases / 8,011 assertions**, randomized order, passing; arm64 Release app build succeeds. Each test target was built separately and ad-hoc signed before execution: a failed unsigned-discovery step in a multi-target build can cancel another target before its new executable is linked. Earlier runs with stale executables or failing diagnostics were not accepted as final verification.

Still open: native UI testing and matching the study viewport's displayed silhouette/handles to the scaled, rotated Prepare instance. The current viewport uses raw-object coordinates; deformation vectors are mapped back into that frame. The broader material-data and optimization-workflow gaps below also remain open.

- macOS arm64 Release application build succeeds.
- Full core suite: 344 cases, 283,143 assertions, randomized order, passing.
- Earlier focused strength checkpoint: 28 cases, 113,551 assertions, passing; the expanded strength cases are included in the full core run above.
- Full FFF suite: 145 cases, 8,011 assertions, randomized order, passing.
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

Separate branding/resource/build-file changes are now present in this checkout, beyond the previously noted `resources/Icon.icns` edit. They are intentionally outside this feature's changes and must not be discarded or committed as strength work just to report a clean worktree.

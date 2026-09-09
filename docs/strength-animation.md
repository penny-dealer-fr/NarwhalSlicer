# Load animation and comparison

Simulation → Animate opens mechanical and print-temperature options for a current,
successfully solved load case. The animation is computed off the UI thread with
progress and cancellation. Playback starts automatically. The model viewport and
force-region graph have explicit sizes; the dialog is bounded by its parent's
monitor and supports scrolling on smaller displays. Detailed settings are behind
Data so they cannot push the graph out of the initial viewport.

The viewer provides play/pause/replay, scrubbing, deformation, Von Mises stress and
maximum shear contours, isometric/front/top/right views, a section toggle and
shape magnification. Initial magnification makes small displacements visible and
is shown numerically. Graphs always use physical displacement in millimeters.

Each force has a selectable curve showing the maximum displacement within its
original application region, including detached cells. This is a regional maximum,
not a measurement at a single fixed node. The force axis reports the total force
actually still applied to that region. Gravity has its own curve when enabled.
The time axis shows the entire recorded history. A marker identifies the first
computed plastic yield anywhere in the discretized part. Loading is solid and
unloading is dashed, including hysteresis and nonzero displacement at zero force.

## Mechanical model

`StrengthTransient` is a layer-resolved, small-strain mechanical lattice estimate.
It is distinct from the existing linear surface-lattice study and re-solves
nonlinear equilibrium at each history increment. It does not scale the old elastic
solution to invent a plastic curve.

- Transform geometry to physical print coordinates, including instance scale and
  the study's layer normal. Slice the mesh at each physical layer center using the
  existing mesh slicer. First and subsequent layer heights are separate inputs.
- Fill those sections with finite cells at the requested in-plane resolution.
  Nearest-neighbor axial/shear bonds connect cells within and between layers.
  Empty space is omitted. Cell-centered occupancy approximates boundary geometry;
  thin features require a sufficiently fine grid. This is not a conforming solid
  finite-element mesh, and the lattice has no independent rotational degrees of
  freedom or Poisson coupling.
- Use calibrated XY/Z elastic and shear properties, effective infill density,
  normalized pattern factors, approximate wall coverage, and inter-layer bond
  scaling. Wall loops, layer heights, extrusion width and print temperatures are
  initialized from the effective process/object settings. Material and infill
  remain explicit properties of the load study. Multi-material extrusion paths,
  adaptive layers and modifier-specific toolpaths are not reconstructed.
- Integrate axial and shear plastic strain with implicit return mapping and
  isotropic hardening. Newton equilibrium uses consistent tangents and a residual
  backtracking search, including at load reversal. Commit irreversible strain only
  after equilibrium converges. Hardening ratio and failure strain are editable
  assumptions, not measured properties inferred from a generic filament name.
- Permanently break bonds above ultimate axial stress or the specified accumulated
  plastic/shear strain. With plasticity off, shear strength is a direct failure
  threshold. Recompute support connectivity and equilibrium after each cascade.
- Keep detached fragments at their last position. Remove their shares of applied
  force and gravity; do not redistribute those shares to surviving cells or
  reconnect broken bonds. Fragment inertia, free flight, collision and contact
  are outside this quasi-static model.

The axial return-map structure follows the standard one-dimensional hardening
construction described in [ETH's material-nonlinearity lab](https://ethz.ch/content/dam/ethz/special-interest/baug/ibk/structural-mechanics-dam/education/femII/presentation_02_material_nonlinearity_part_c_v1.pdf).
The implementation uses separate bond axial/shear yield laws, not a full continuum
J2/Hill constitutive model. The general distinction between trial stress, plastic
strain correction and equilibrium iteration is also documented by
[MOOSE](https://mooseframework.inl.gov/moose/source/materials/RadialReturnStressUpdate.html).

## Unloading and thermal options

Unload to zero uses the first half of the entered duration to increase force and
the second half to remove it. Disabling unloading leaves full force at the final
frame. Elastic parts return to their initial shape to numerical tolerance; yielded
parts can retain permanent deformation. Failed bonds remain broken throughout.
Recorded frames make scrubbing deterministic: moving the slider backward does not
undo the irreversible history used to compute later frames.

Optional thermal bonding estimates the previous layer's cooling and the interface
contact temperature from nozzle, bed, chamber, layer time and a reference cooling
time. Cooling time scales with layer thickness squared. A normalized contact-
temperature proxy changes inter-layer stiffness and strength relative to a reference
process. Per-layer interface temperatures and bond factors are retained in the
result. This is an explicitly approximate process-calibration model, not a transient
heat-equation solve, polymer diffusion model or prediction of residual thermal
stress. Coupons and process-specific calibration are needed for quantitative use.

Work is bounded at 2000 physical layers, the configured cell budget (3500 by
default), and 400000 cell-frame samples. Exceeding a limit fails explicitly instead
of silently merging layers. Nonconvergence and cancellation are not saved as
successful simulations. Inertia, creep, rate dependence and geometric nonlinearity
are not modeled; duration is a quasi-static playback/loading coordinate.

## Comparison and persistence

Add to compare retains an independent geometry, setup, print options and computed
history. Compare selects two different retained runs and synchronizes by elapsed-
time percentage, displaying each run's own time, force and graph ranges. The longer
run controls playback duration. Data opens the captured settings. Histories are
session-only; they do not change 3MF schema, slicing settings or emitted G-code.

## Verification (2026-09-08)

- Combined macOS arm64 Release `libslic3r_tests` and `OrcaSlicer` build succeeds.
  Test executables are ad-hoc signed before Catch discovery so discovery no longer
  fails before Xcode's later bundle-signing phase.
- CTest StrengthAnalysis label: **51/51 tests pass**. Five transient cases cover
  zero/peak/unloaded frames, elastic springback, plastic permanent set, irreversible
  detachment and force removal, layer and thermal sensitivity, work limits,
  cancellation, print-orientation stiffness, loading-only endpoints and axial
  stress recovery against force divided by section area.
- Native light-theme checks on an isolated 4 mm cube project: opened both option
  pages; computed a history; scrubbed to peak load; captured a screenshot with the
  model, deformation contour, graph and marker all visible. Saved an unloading
  run and a loading-only run; selected both in Compare and captured both model
  viewports and graphs side by side. Endpoints show zero versus full applied load.
- A subsequent sizing adjustment bounds the initial dialog to its parent's monitor.
  Dark-theme checks were explicitly deferred by the user. Broad localization and
  physical coupon validation are not claimed.

# Load animation and comparison

Simulation → Animate opens mechanical, print-temperature and sampling/infill options for a current,
successfully solved load case. The animation is computed off the UI thread with
progress and cancellation. Playback starts automatically. The model viewport and
force-region graph have explicit sizes; the dialog is bounded by its parent's
monitor and supports scrolling on smaller displays. Detailed settings are behind
Data so they cannot push the graph out of the initial viewport.

The viewer provides play/pause/replay, scrubbing, deformation, Von Mises stress and
maximum shear contours, isometric/front/top/right views, a section toggle and
shape magnification. Initial magnification makes small displacements visible and
is shown numerically. Graphs use physical displacement in millimeters or Von Mises / maximum shear stress in MPa.

Each force has a selectable curve showing the maximum displacement within its
original application region, including detached cells. This is a regional maximum,
not a measurement at a single fixed node. The force axis reports the total force
actually still applied to that region. Gravity has its own curve when enabled.
The time axis shows the entire recorded history. A marker identifies the first
computed plastic yield anywhere in the discretized part. The graph annotates its
maximum with its time, marks extrema and supports dragging to scrub both models.
Hovering samples shows time, applied force, displacement, both stresses, branch,
maxima/minima, first yield and first failure. These are sampled extrema and yield
times; increasing FPS improves temporal resolution. Loading is solid and
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
  initialized from the effective process/object settings, along with background
  infill density and supported pattern. These defaults are editable per animation;
  unsupported slicer patterns are explicitly identified and require a supported
  approximation. Material remains a property of the load study.
- Capture the current dense-region slider's generated geometry regardless of
  preview visibility or whether a Slice Modifier exists. Slice that geometry in
  the same print basis and apply its density/pattern locally when assembling the
  lattice; retain its geometry, requested percentage and resolved cell count.
  Empty space and holes are respected at the chosen cell-center resolution. Multi-material extrusion paths,
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

Work is bounded at 2000 physical layers and 20 million candidate cells. The
maximum occupied-cell budget defaults to 20000 and is editable up to 200000.
The history budget defaults to 512 MiB and is editable from 1 to 8192 MiB; this
bounds retained cell results, not total solver/factorization memory. Smaller
positive cell widths are accepted within those explicit budgets. The progress
dialog offers Cancel compute during construction and analysis. FPS is editable
from 1 to 120, with at most 100000 increments. Loading/unloading uses an even
increment count to include the exact peak; the minimum is four increments, so
short runs can have a higher effective sampling rate. Playback uses elapsed time
and the requested refresh rate. Exceeding a limit fails explicitly instead of
silently merging layers or dropping frames. Nonconvergence and cancellation are not saved as
successful simulations. Inertia, creep, rate dependence and geometric nonlinearity
are not modeled; duration is a quasi-static playback/loading coordinate.

## Comparison and persistence

Add to compare opens a naming popup with the generated name as placeholder;
blank input accepts that suggestion and Cancel leaves the result unsaved. Saving
retains independent geometry, setup, print options and computed history. Compare selects two different retained runs and synchronizes by elapsed-
time percentage, displaying each run's own time, force and graph ranges. The longer
run controls playback duration. Data opens the captured settings. Histories are
session-only; they do not change 3MF schema, slicing settings or emitted G-code.

## Verification (2026-09-08)

The initial enhancement build passed 55 StrengthAnalysis tests, including eight
Transient cases for elastic springback, permanent set, irreversible detachment,
thermal/layer sensitivity, orientation, shear traction, captured reinforcement,
finer grids, extended histories, memory budgets and cancellation.

A subsequent header-only change exposed stale callers: the root CMake file marked
project `src` headers SYSTEM, excluding them from `-MMD` dependency files. The
project include directory now uses normal includes so layout changes rebuild
callers. Final rebuild, tests and updated native UI checks are in progress.

Earlier native light-theme checks verified the model and deformation graphs,
playback, scrubbing and two-run comparison on an isolated 4 mm cube fixture.
Dark-theme checks are deferred at the user's request. Physical coupon validation
is not claimed; this is a calibrated lattice estimate, not toolpath thermal FEA.

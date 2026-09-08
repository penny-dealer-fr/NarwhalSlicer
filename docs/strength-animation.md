# Load animation and comparison

Simulation → Animate opens a duration dialog for a current, successful solved study.
Durations must be finite and between 0.1 and 86400 seconds. All active forces,
equivalent-static impact forces, and gravity increase proportionally from zero to
full load; constraints remain fixed. Playback time is not a dynamic simulation.

The viewer provides play/pause/replay, keyboard-accessible scrubbing, deformation,
Von Mises stress and maximum shear contours, isometric/front/top/right projections,
a section toggle, and a display-only deformation magnification. Graphs always show
physical displacement, independently of the magnification. Each active force has
a selectable probe at the most displaced node in its application region. Values
include the response to all simultaneous loads. A graph's force axis represents
that load's total applied force (including the equivalent-static impact factor),
not the force at an individual distributed-load node.

The estimated elastic/shear limit is the first calibrated directional yield or
shear-strength threshold reached anywhere in the part, using the same pattern
strength factor as the existing solver. It is independent of requested safety
factors and ultimate-strength settings. A graph marker identifies this threshold;
the remainder of the elastic curve is dashed. This is not a calculation of the
elastic-to-plastic constitutive transition or permanent deformation. Values beyond
the marker remain linear extrapolations. No threshold is reported when none is
reached within the selected loading range.

Add to compare retains an independent mesh, solved setup, material, orientation,
endpoint field and duration. Compare selects two distinct retained runs. The panes
scrub together by load fraction and display their own elapsed time and graph axis
ranges. Playback uses the longer run's duration so both reach full load together.
Runs remain in memory only for the current app session; they are not saved to 3MF.

## Analysis coverage

The ramp uses the existing solved directional-material and infill model. It does
not introduce slicing changes. Change the orientation/material/infill in the Load
study and solve again before capturing another run. The viewer displays these
snapshot settings alongside results.

Individual deposited layers, actual toolpaths, wall settings, layer heights,
inter-layer bonding, nozzle/bed/chamber temperatures, cooling history, creep,
inertia, strain-rate effects, damage, contact and plastic flow are not implemented.
The duration popup explicitly disables the corresponding unsupported options.
A coupled thermal/mechanical layered model and calibrated material laws remain
necessary to implement that part of the requested workflow.

## Verification (2026-09-08)

- macOS arm64 Release OrcaSlicer target builds successfully.
- StrengthAnalysis filter: 46 cases, 225131 assertions pass, including two new
  LoadRamp cases. Tests compare the ramp against a fresh proportional solve with
  gravity and impact, check zero/end frames, and verify strength-limit behavior.
- Xcode's combined build compiles and links tests but automatic discovery fails on
  the unsigned executable. Ad-hoc signing the local test bundle allows the tests
  to run directly; the app target builds separately without that discovery step.
- Native app opens to Prepare. File-dialog input timed out during fixture import;
  animation/compare interaction, light/dark screenshots, HiDPI, narrow-window and
  localization layout are **not visually verified**.

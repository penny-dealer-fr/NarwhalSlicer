# Strength-analysis workflow

Strength Analysis runs locally and does not require a cloud solver or a paid service. It provides an **engineering estimate**, not certified finite-element analysis or a guarantee that a printed part is safe.

## Set up and solve

1. Select a part in **Prepare**, then open **Load**.
2. Select faces or place region tools in the 3D view. Add fixed constraints, forces, preserve regions, and optional gravity. Edit an operation through its dialog or the selected-operation panel.
3. Choose material properties, print-layer direction, calibration, background/dense infill settings, and optimization criteria. Generic material values and pattern factors are estimates; measured properties for the actual material and printing process are preferable.
4. Run **Pre-check**. Green means the current inputs pass the input checks; orange means issues need attention. Passing does not validate physical accuracy.
5. **Solve** prepares the response and reinforcement profile in a cancellable background worker. If inputs change during calculation, its obsolete result is discarded.

## Size and apply reinforcement

- Open **Simulation** to inspect safety factor, displacement, stress, shear, and point probes. Rainbow contours use the displayed legend. Deformation magnification is a display control, not a geometry edit.
- Move the strengthened-volume slider to choose a percentage of the original solid model volume. Higher-demand interior cells are selected first, so reinforcement can cover separate stress concentrations. Preserve regions restrict what can be selected.
- The purple, translucent mask shows the **undeformed modifier geometry**. Exterior part dimensions do not change. The slicer clips that modifier to the printable model.
- Read the stress threshold, reinforced volume, approximate mass change, and local-response estimates together. **Size to target SF** searches the available percentage settings in 1% increments and explicitly reports an unreachable target.
- **Use setup stress threshold** converts the Load tab's threshold into the eligible high-stress volume, excluding preserved features, and rounds it to the nearest 1% slider setting. The metrics show the threshold actually reached by that rounded setting.
- Click **Create slicer modifier** to apply the mask as a native parameter modifier. Applying again replaces the managed modifier. It also applies the study's background infill settings to the object; other user-created modifiers remain untouched and may override settings in overlapping regions.
- Return to **Prepare/Preview**, slice the plate, and inspect the generated infill and filament statistics. The slider alone does not change the print. Actual sliced mass depends on walls, solid layers, support, and the rest of the print settings.

## Change, undo, and remove

Select a load, constraint, or preserve operation in the study tree or viewport, then use **Delete operation**. **Undo setup** and **Redo setup** handle study edits. The main application Undo/Redo handles applied model and modifier changes. **Remove dense modifier** removes only modifiers marked as managed by Strength Analysis; it does not delete similarly named user modifiers.

Saving a project as 3MF retains the setup and native modifier settings. Geometry or setup changes mark the old results stale. A new solve is required before applying another result.

## Interpretation limits

The solver is a small-deformation, linear-elastic edge-network approximation with directional material properties. The interactive reinforcement response scales the existing local response; it does not re-solve redistributed load paths, nonlinear failure, fatigue, impact dynamics, buckling, or the exact printed lattice. The stress-ranked interior field is interpolated from the solved mesh, not a Fusion topology-optimization result.

Added material changes self-weight. With gravity enabled and added mass, the preview therefore does **not** report an updated safety factor or deformation; contours and probes retain the baseline response. Validate the reinforced design with appropriate physical testing or a suitable independent analysis. Input pre-checks, an estimated safety-factor target, and a successful slice do not replace that validation.

The interaction follows Fusion's separation of setup, solve, results, and design iteration. Autodesk's [target-mass workflow](https://help.autodesk.com/cloudhelp/ENU/Fusion-Simulate/files/SIM-SO-TARGET-MASS-TSK.htm) and [safety-factor result guide](https://help.autodesk.com/cloudhelp/ENU/Fusion-Simulate/files/GUID-8D0B2980-6B54-4006-A89E-53C0C2E202B4.htm) are useful references; their solver capabilities should not be inferred for this estimate model.

# Strength-analysis workflow

Strength Analysis runs locally and does not require a cloud solver or a paid service. It provides an **engineering estimate**, not certified finite-element analysis or a guarantee that a printed part is safe.

## Workspace toolbars

The Load toolbar groups commands into **Setup**, **Loads**, **Study**, **Print**, and **Tools**. Click a section name to access every command, including tools that do not fit in the current window. Use its **Pin to toolbar** submenu (also available by right-clicking a shortcut) to choose visible tools. Pins are remembered separately for Load and Simulation and do not alter the project. Pinned tools receive priority when space is limited. Additional tools fill spare section space unless explicitly unpinned. Resizing redistributes the available width across sections. Shortcuts show only icons: hovering shows the tool name immediately and adds an explanation after about two seconds.

Material, orientation, optimized settings, and dense-region actions are available in **Print**. **Study** also provides the same dense-region preview. Detailed numeric editors remain available below the Load workspace. Pre-check, Solve, and Cancel solve are in **Study**.

Simulation groups tools into **Results**, **Display**, **Inspect**, **Print**, and **Tools**. Checked menu entries indicate the current result, view, and display options. **Inspect → Point probe** focuses the viewport; click the visible model to inspect a vertex. **Inspect → Section** toggles an uncapped surface cut at the midpoint of the object's Z extent, retaining the lower half. It interpolates surface colors along clipped triangles; it does not calculate interior section stresses. The section hides the full-model reference wireframe, dense overlay, and extrema markers; turn it off to restore those overlays. The legend retains the full solved field range.

## Set up and solve

1. Select one part instance in **Prepare**, then open **Load**. When there are multiple copies, select the specific copy to study.
2. Select faces or place region tools in the 3D view. Add fixed constraints, forces, preserve regions, and optional gravity. Edit an operation through its dialog or the selected-operation panel.
3. Choose material properties, calibration, background/dense infill settings, and optimization criteria. **Follow selected instance orientation in Prepare** is enabled by default; turn it off to enter a hypothetical layer-normal axis manually. **Apply best orientation** rotates only the selected copy and returns to following Prepare. Generic material values and pattern factors are estimates; measured properties for the actual material and printing process are preferable.
4. **Pre-check** updates automatically. Green means the current inputs pass the input checks; yellow means setup is incomplete or not yet checked, and red means validation failed. Click it for details. Passing does not validate physical accuracy.
5. **Solve** prepares the response and reinforcement profile in a cancellable background worker. If inputs change during calculation, its obsolete result is discarded.

Force and gravity vectors use physical, object-aligned axes, not the viewport's global XYZ gizmo. In the gravity dialog, **Use Prepare -Z gravity** converts downward gravity from the current Prepare orientation without scaling its acceleration. It edits the dialog only until you confirm. The saved vector stays attached to the object; use the preset again if you rotate the part and want gravity to remain globally downward. This explicit direction choice follows the distinction in Autodesk's [gravity guide](https://help.autodesk.com/cloudhelp/ENU/Fusion-Simulate/files/SIM-GRAVITY-CONCEPT.htm).

## Size and apply reinforcement

- Open **Simulation** to inspect safety factor, displacement, stress, shear, and point probes. Rainbow contours use the displayed legend. Deformation magnification is a display control, not a geometry edit.
- Move the strengthened-volume slider to choose a percentage of the original solid model volume. Higher-demand interior cells are selected first, so reinforcement can cover separate stress concentrations. Preserve regions restrict what can be selected.
- The purple, translucent mask shows the **undeformed modifier geometry**. Exterior part dimensions do not change. The slicer clips that modifier to the printable model.
- Read the stress threshold, reinforced volume, approximate mass change, and local-response estimates together. **Print → Target SF** opens a small popup for entering the target; **Size region** searches the available percentage settings in 1% increments and explicitly reports an unreachable target.
- **Use setup stress threshold** converts the Load tab's threshold into the eligible high-stress volume, excluding preserved features, and rounds it to the nearest 1% slider setting. The metrics show the threshold actually reached by that rounded setting.
- Click **Create Slice Modifier** to apply the mask as a native parameter modifier. Applying again replaces the managed modifier. **Remove Slice Modifier** is a separate Print command, enabled whenever a managed modifier exists, regardless of preview percentage or stale results. At 0%, creation is disabled. Native modifiers and background infill settings belong to the object, so they affect all its copies; the response and mass readout describe the selected instance. Other user-created modifiers remain untouched and may override settings in overlapping regions.
- Return to **Prepare/Preview**, slice the plate, and inspect the generated infill and filament statistics. The slider alone does not change the print. Actual sliced mass depends on walls, solid layers, support, and the rest of the print settings.

## Change, undo, and remove

Select a load, constraint, or preserve operation in the study tree or viewport, then use **Delete operation**, Delete, or Backspace. **Undo setup** and **Redo setup** handle study edits; keyboard shortcuts in the Load Setup workspace are Cmd/Ctrl+Z for undo and Cmd/Ctrl+Shift+Z or Cmd/Ctrl+Y for redo. Text fields retain their normal text-editing shortcuts. The main application Undo/Redo handles applied model and modifier changes. **Remove dense modifier** removes only modifiers marked as managed by Strength Analysis; it does not delete similarly named user modifiers.

Saving a project as 3MF retains the setup, orientation-follow choice, and native modifier settings. Geometry, selected-instance transforms, or setup changes mark the old results stale when the study synchronizes on activation or before an action. A new solve is required before applying another result.

## Interpretation limits

The solver is a small-deformation, linear-elastic edge-network approximation with directional material properties. The interactive reinforcement response scales the existing local response; it does not re-solve redistributed load paths, nonlinear failure, fatigue, impact dynamics, buckling, or the exact printed lattice. The stress-ranked interior field is interpolated from the solved mesh, not a Fusion topology-optimization result.

Prepare instance scaling is incorporated into mechanical distances, areas, volume, gravity mass, and the reinforcement volume/mass readout. Loads and selected regions remain attached to raw-object coordinates; their vector components and reported displacement use physical, object-aligned axes. The generated modifier stays in raw coordinates so the slicer applies the instance transform exactly once. Both study viewports display that instance's rotation, scale, and reflection, with transformed region outlines and handles. The camera fits the part independently of its bed translation. Stale results retain the transform from their solve until replaced. Native visual and placement-handle verification remains open.

Added material changes self-weight. With gravity enabled and added mass, the preview therefore does **not** report an updated safety factor or deformation; contours and probes retain the baseline response. Validate the reinforced design with appropriate physical testing or a suitable independent analysis. Input pre-checks, an estimated safety-factor target, and a successful slice do not replace that validation.

The interaction follows Fusion's separation of setup, solve, results, and design iteration. Autodesk's [target-mass workflow](https://help.autodesk.com/cloudhelp/ENU/Fusion-Simulate/files/SIM-SO-TARGET-MASS-TSK.htm) and [safety-factor result guide](https://help.autodesk.com/cloudhelp/ENU/Fusion-Simulate/files/GUID-8D0B2980-6B54-4006-A89E-53C0C2E202B4.htm) are useful references; their solver capabilities should not be inferred for this estimate model.

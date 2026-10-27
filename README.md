# Axis Cylinder

A Fusion 360 add-in with one command, **Cylinder from Axis**: a solid cylinder
placed by a direction and an origin that are both read off the model rather than
typed in.

Built for screw and pin trajectories on patient-specific plates and guides, where
the direction that matters is *perpendicular to this bit of bone* or *down this
bore*, and there is often no analytic normal or axis to read off it.

## Direction first, then the origin

You choose **how the direction is found** and pick the geometry for it. The
origin then **defaults from that same geometry**, and the dialog shows where it
landed. Picking something in **Origin (optional)** overrides the default; clear
it to go back.

| Direction from | you select | direction | default origin |
|---|---|---|---|
| **Perpendicular to a surface (fitted plane)** | one or more faces | normal of one total-least-squares plane through them | their centre of gravity, dropped onto the surface |
| **Along an edge or axis** | a straight or circular edge, construction axis, sketch line, cylindrical/conical face | read off the entity | the midpoint of the axis: edge or sketch-line midpoint, circle centre, middle of the face along its axis |
| **Axis of a cylindrical face** | one cylindrical face | its analytic axis, exactly | the midpoint of the axis over the face |
| **Axis of a cylinder fitted to a surface** | one or more faces, cylindrical or not | axis of a least-squares cylinder through them | the midpoint of the fitted axis over the surface |

In the two cylinder modes the **diameter and length default to the surface's
own** when you pick it — written into the fields once, so you can still type a
pin longer than its bore. Switching to any axis mode also ticks **Half each way
from the origin**, since the origin is now the middle of the axis; untick it if
you want the cylinder to start there.

A construction axis built from an edge or a round face takes that entity's
midpoint. Any other construction axis is infinite and has no midpoint, and the
dialog says so: pick an origin.

### Every fit reports itself

A direction is only as meaningful as the primitive it came from, so both fits
print their residuals in the dialog:

```
plane fit      rms 0.0412 mm, worst 0.1883 mm, over 2104 points
cylinder fit   rms 0.0031 mm, worst 0.0120 mm, 353 deg of arc, over 1880 points
```

and warn in bold when they should not be trusted: a plane that the surface stands
more than 2% of its own width off; a cylinder the surface departs from by more
than 5% of its radius, or one fitted to less than 45 degrees of arc, where the
radius and the axis are barely constrained.

Several faces are fitted **together**, as one patch, rather than fitted
separately and averaged — averaging would weight a sliver face as heavily as the
one actually carrying the surface.

The cylinder fit is Fit Primitives' `fit_cylinder`, unchanged: seeded from the
uncentred moment of the surface normals (every normal of a cylinder is
perpendicular to its axis), then Levenberg-Marquardt on the true geometric
residual. Past 4000 mesh vertices it fits a subsample; the extent along the axis
always uses every vertex, so the midpoint and length are not shortened.

### Origin override

| you select | origin |
|---|---|
| a vertex, sketch point or construction point | that point |
| a face | its **centre of gravity** — `BRepFace.centroid`, computed by Fusion from the real surface, not from a tessellation |

On a curved face the centre of gravity generally does **not** lie on the face, so
**Put a surface origin on the surface** (on by default) drops it onto the face
with the surface evaluator and reports how far it moved. For a screw starting at
the bone you want the point on the bone; for an axis down the middle of a bore you
do not, so it is a checkbox rather than a rule.

## Length

A length you type, running from the origin along the direction, with **Half each
way from the origin** to centre it instead and **Flip the direction** to reverse
it. Nothing is trimmed to other geometry — boolean the result yourself.

The live preview is the real body, rebuilt on every input change.

## Why it is fitted to vertices, and weighted by area

Both points are inherited from [Fit Primitives](https://github.com/iagoandreu/fit-primitives),
whose plane fit this is:

- **Vertices, not triangle centroids.** Centroids of a tessellated surface lie
  *inside* it; vertices sit on it.
- **Weighted by a third of each adjacent triangle's area**, the standard lumped-area
  approximation to a surface integral — so a face tessellated finely in one corner
  does not drag the plane towards that corner.

## Install

1. Clone or copy this folder anywhere.
2. Run `powershell -File install.ps1`. It copies the add-in into
   `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\AxisCylinder`.
3. In Fusion: **Utilities → ADD-INS → Scripts and Add-Ins → Add-Ins** →
   `Axis Cylinder` → tick **Run on Startup** → **Run**.
4. The button appears in **Solid → Create** and **Mesh → Modify**.

No dependencies. It runs on Fusion's bundled Python, stdlib only.

## How the body is built

`TemporaryBRepManager.createCylinderOrCone`, added through a `BaseFeature`. That
works identically in a parametric and a direct design. A sketch-and-extrude would
be parametrically editable, but positioning it needs a construction plane placed
by an arbitrary `Plane`, and `ConstructionPlaneInput.setByPlane` is documented as
direct-modelling only — so it would work in exactly the designs where it matters
least.

## Repository layout

```
AxisCylinder.py   add-in shell: lifecycle, toolbar, events, dev bridge
ac_core.py        all the working code; re-imported on every command
ac_fit.py         the plane and cylinder fits; stdlib, no adsk
install.ps1       push into Fusion's AddIns folder
resources/        toolbar icons
tools/            dev-bridge client, icon generator
tests/            offline tests, no Fusion required
```

`ac_core.py` and `ac_fit.py` **hot-reload** — re-imported on every command
(`ac_core` reloads `ac_fit` itself; the shell only reloads `ac_core`), so pushing
them takes effect on the next click; only `AxisCylinder.py` needs a
Stop/Run. Because of that reload, nothing may keep state in a module global
between events.

## Tests

```
py -3.10 tests/test_geometry.py
```

`tests/stub` fakes `adsk`, so it runs without Fusion. It covers the plane fit
against planes whose answer is known by hand, the cylinder fit through a
tessellated tube and a 40 degree strip, and the cylinder placement maths (length,
symmetric, flip, skew axes).

What it deliberately does **not** cover is reading an origin or a direction off a
real selection: those are Fusion object types and the stub answers yes to every
`cast`, so a test against it would prove nothing. Those go through the dev bridge
in a live Fusion. `tools/livetest.py` runs the add-in's own self-test — all four
direction modes through the real dialog logic, against a box, a cylinder and a
sphere whose answers are known in advance. It opens a **new** document and closes
it unsaved; the active document is never touched.

```
py -3.10 tools/livetest.py
py -3.10 tools/ac.py ping
py -3.10 tools/ac.py fit  '{"body": "Body1", "face": 0}'
py -3.10 tools/ac.py make '{"body": "Body1", "face": 0, "diameter_mm": 4, "length_mm": 20}'
```

Off by default — create a `devbridge.on` file next to the add-in to enable it.
Nothing in normal operation depends on it.

## Licence

MIT — see [LICENSE](LICENSE).

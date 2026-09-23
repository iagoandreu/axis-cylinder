# Axis Cylinder

A Fusion 360 add-in with one command, **Cylinder from Axis**: a solid cylinder of
the diameter and length you type, placed by an origin and a direction that are
both read off the model rather than typed in.

Built for screw and pin trajectories on patient-specific plates and guides, where
the direction that matters is *perpendicular to this bit of bone* and there is no
analytic normal to read off it.

## The two things it decides for you

### Origin

| you select | origin |
|---|---|
| a vertex, sketch point or construction point | that point |
| a face | its **centre of gravity** — `BRepFace.centroid`, computed by Fusion from the real surface, not from a tessellation |

On a curved face the centre of gravity generally does **not** lie on the face, so
**Put a surface origin on the surface** (on by default) drops it onto the face
with the surface evaluator and reports how far it moved. For a screw starting at
the bone you want the point on the bone; for an axis down the middle of a bore you
do not, so it is a checkbox rather than a rule.

### Direction

**Along an edge, axis or cylinder** reads a direction that already exists: a
straight edge, a circular edge's normal, a construction axis, a sketch line, the
axis of a cylindrical or conical face, the normal of a planar face or construction
plane.

**Perpendicular to a surface (fitted plane)** fits one total-least-squares plane
through the faces you select and runs the cylinder along its normal. This is the
mode that earns the add-in: a lofted, scanned or offset face has no normal to
read, and the best-fit plane through it is the only well-defined thing to be
perpendicular to. Leave the surface selection empty and it uses the face you
picked as the origin, so the common case is one click.

Several faces are fitted **together**, as one patch, rather than fitted separately
and averaged — averaging would weight a sliver face as heavily as the one actually
carrying the surface.

The fit is reported in the dialog:

```
plane fit      rms 0.0412 mm, worst 0.1883 mm, over 2104 points
```

and if the surface stands more than 2% of its own width off that plane, the dialog
says so in bold. A normal is only as meaningful as the plane it came from; a plane
fitted to something strongly curved is a number the tool can produce and a
direction you should not trust.

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
ac_fit.py         the total-least-squares plane fit; stdlib, no adsk
install.ps1       push into Fusion's AddIns folder
resources/        toolbar icons
tools/            dev-bridge client, icon generator
tests/            offline tests, no Fusion required
```

`ac_core.py` and `ac_fit.py` **hot-reload** — re-imported on every command, so
pushing them takes effect on the next click; only `AxisCylinder.py` needs a
Stop/Run. Because of that reload, nothing may keep state in a module global
between events.

## Tests

```
py -3.10 tests/test_geometry.py
```

`tests/stub` fakes `adsk`, so it runs without Fusion. It covers the plane fit
against planes whose answer is known by hand, and the cylinder placement maths
(length, symmetric, flip, skew axes).

What it deliberately does **not** cover is reading an origin or a direction off a
real selection: those are Fusion object types and the stub answers yes to every
`cast`, so a test against it would prove nothing. Those go through the dev bridge
in a live Fusion:

```
py -3.10 tools/ac.py ping
py -3.10 tools/ac.py fit  '{"body": "Body1", "face": 0}'
py -3.10 tools/ac.py make '{"body": "Body1", "face": 0, "diameter_mm": 4, "length_mm": 20}'
```

Off by default — create a `devbridge.on` file next to the add-in to enable it.
Nothing in normal operation depends on it.

## Licence

MIT — see [LICENSE](LICENSE).

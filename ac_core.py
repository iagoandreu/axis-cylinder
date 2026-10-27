"""Axis Cylinder - all the working code. RE-IMPORTED ON EVERY COMMAND EVENT.

Because of that reload, nothing may keep state in a module global between events:
anything the dialog decides has to be readable back off the inputs themselves.

A cylinder needs three things - which way it points, where it sits, and how big
it is - and the first two are the ones worth getting right. They are chosen in
that order: DIRECTION FIRST, and the origin defaults from whatever gave the
direction. Picking an origin is an optional override, not a step.

DIRECTION, four ways
    fit      a total-least-squares plane through the faces you select, and the
             cylinder runs along its normal. A scan-derived or lofted face has no
             analytic normal to read, and the plane through it is the only
             well-defined thing to be perpendicular to. Origin: the centre of
             gravity of those faces.
    entity   read off something that already carries one - a straight edge, a
             construction axis, a sketch line, a circular edge, the axis of a
             cylinder or cone. Origin: the midpoint of that axis.
    cylface  the analytic axis of a cylindrical face. Origin: the midpoint of the
             axis over the face; diameter and length default to the face's own.
    cylfit   the axis of a least-squares cylinder through the faces you select,
             cylindrical or not (a scanned bore, a lofted shaft). Same defaults.

    Every fit reports its own rms and worst deviation, because a direction is only
    as meaningful as the primitive it came from. A plane fitted to something
    strongly curved, or a cylinder to a 20 degree sliver, is a number the tool can
    produce and a direction you should not trust, and the only honest thing to do
    is show how far off the surface actually is.

ORIGIN OVERRIDE
    A point is a point. A face gives its CENTROID, which Fusion computes exactly
    (BRepFace.centroid) rather than from a tessellation. On a curved face that
    centroid is the centre of gravity of the surface and generally does not lie ON
    it, so there is an option - on by default - to drop it onto the face with the
    surface evaluator. For a screw trajectory starting at the bone you want the
    point on the bone; for an axis through the middle of a bore you do not.

The body is built with TemporaryBRepManager and added through a BaseFeature, which
works the same way in a parametric and a direct design. A sketch-and-extrude would
be parametrically editable, but positioning it needs a construction plane, and
ConstructionPlaneInput.setByPlane is direct-modelling only - so it would work in
exactly the designs where it matters least.
"""

import importlib
import math
import os
import traceback

import adsk.core
import adsk.fusion

import ac_fit

# the shell reloads this module on every event, but a plain import would keep the
# ac_fit cached from the first one - reload it too, or edits to it never land
importlib.reload(ac_fit)

ADDIN_DIR = os.path.dirname(os.path.realpath(__file__))
RES_DIR = os.path.join(ADDIN_DIR, 'resources', 'AxisCylinder')
LOG_PATH = os.path.join(ADDIN_DIR, 'AxisCylinder.log')
SETTINGS = os.path.join(ADDIN_DIR, 'last_settings.json')

CMD_ID = 'AxisCylinderCreate'
CMD_NAME = 'Cylinder from Axis'

CM = 10.0          # internal cm -> mm
MM = 0.1           # mm -> internal cm

VERSION = '1.1.0'

# key, label - appended only, the saved dirMode is an index into this
DIRECTION_MODES = [
    ('fit', 'Perpendicular to a surface (fitted plane)'),
    ('entity', 'Along an edge or axis'),
    ('cylface', 'Axis of a cylindrical face'),
    ('cylfit', 'Axis of a cylinder fitted to a surface'),
]

# the selection input that carries each mode's direction
DIRECTION_INPUT = {'fit': 'selPlane', 'entity': 'selAxis',
                   'cylface': 'selCylFace', 'cylfit': 'selCylFit'}

# modes whose default origin is the middle of an axis, so the natural cylinder
# runs half each way from it
AXIS_MODES = ('entity', 'cylface', 'cylfit')

# the cylinder fit is pure Python in the dialog's event loop; past this many
# vertices it is subsampled for the fit (not for the extent along the axis)
MAX_FIT_POINTS = 4000

DEFAULTS = {
    'diameter': 4.0,          # mm
    'length': 20.0,           # mm
    'symmetric': False,
    'flip': False,
    'onSurface': True,
    'dirMode': 0,
}


def log(msg):
    try:
        with open(LOG_PATH, 'a', encoding='utf-8') as fh:
            fh.write('%s\n' % msg)
    except Exception:
        pass


def short_error(tb):
    for line in reversed(tb.strip().splitlines()):
        line = line.strip()
        if line and not line.startswith(('File "', 'Traceback', '^', '~')):
            return line
    return tb.strip().splitlines()[-1]


def load_settings():
    import json
    s = dict(DEFAULTS)
    try:
        with open(SETTINGS, encoding='utf-8') as fh:
            s.update(json.load(fh))
    except Exception:
        pass
    return s


def save_settings(s):
    import json
    try:
        with open(SETTINGS, 'w', encoding='utf-8') as fh:
            json.dump(s, fh, indent=2)
    except Exception:
        pass


# --------------------------------------------------------------------------- #
# reading an origin and a direction off whatever was selected
# --------------------------------------------------------------------------- #

def _v(p):
    return (p.x, p.y, p.z)


def _pt(t):
    return adsk.core.Point3D.create(t[0], t[1], t[2])


def _vec(t):
    return adsk.core.Vector3D.create(t[0], t[1], t[2])


def selected(sel_input):
    return [sel_input.selection(i).entity for i in range(sel_input.selectionCount)]


def face_mesh(face):
    """A triangle mesh for one face, whether or not Fusion has drawn it yet.

    The display mesh is what both Fit Primitives and Mesh to Part fit through, and
    for the same reason: it exists for every face, analytic or not, and it is the
    only representation a lofted or imported surface shares with a plane. But it
    only exists once the face has been DISPLAYED - on a body built a moment ago
    inside a BaseFeature, bestMesh raises "InternalValidationError : count > 0".

    So fall back to calculating one. The tolerances are deliberately loose and
    tied to the face's own size: TriangleMeshCalculator with a tight tolerance and
    no side-length or aspect limit has taken this Fusion down before, and a plane
    fit does not need a fine mesh - it needs points spread over the face.
    """
    mm = face.meshManager
    try:
        meshes = mm.displayMeshes
        if meshes.count > 0:
            return meshes.bestMesh
    except Exception:
        pass
    bb = face.boundingBox
    diag = bb.minPoint.distanceTo(bb.maxPoint) or 1.0
    calc = mm.createMeshCalculator()
    calc.surfaceTolerance = max(diag / 500.0, 1.0e-4)
    calc.maxSideLength = diag / 8.0
    calc.maxAspectRatio = 10.0
    return calc.calculate()


def face_samples(face):
    """Vertices of a face's triangle mesh, with lumped-area weights."""
    tm = face_mesh(face)
    coords = tm.nodeCoordinatesAsDouble
    idx = list(tm.nodeIndices)
    pts = [(coords[3 * i], coords[3 * i + 1], coords[3 * i + 2])
           for i in range(len(coords) // 3)]
    tris = [(idx[t], idx[t + 1], idx[t + 2]) for t in range(0, len(idx), 3)]
    return pts, tris


def surface_samples(faces):
    """Every face's mesh merged into one vertex list, triangles re-indexed."""
    pts = []
    tris = []
    for f in faces:
        p, t = face_samples(f)
        base = len(pts)
        pts.extend(p)
        tris.extend((a + base, b + base, c + base) for a, b, c in t)
    return pts, tris


def fitted_plane(faces):
    """One total-least-squares plane through every face given.

    Several faces are fitted together rather than one at a time and averaged: the
    patch is the thing being approximated, and averaging normals would weight a
    sliver face as heavily as the one carrying the surface.
    """
    pts, tris = surface_samples(faces)
    if len(pts) < 3:
        raise ac_fit.FitError('the selected surface has no usable mesh')
    w = ac_fit.vertex_weights(pts, tris)

    # seed the sign from the face's own outward normal so the cylinder grows the
    # way the surface faces, rather than whichever way the eigenvector came out
    hint = None
    try:
        f0 = faces[0]
        ok, param = f0.evaluator.getParameterAtPoint(f0.pointOnFace)
        if ok:
            ok2, nrm = f0.evaluator.getNormalAtParameter(param)
            if ok2:
                hint = _v(nrm)
                if f0.isParamReversed:
                    hint = ac_fit.scale(hint, -1.0)
    except Exception:
        pass
    return ac_fit.fit_plane(pts, w, hint_normal=hint)


def _axis_extent(pts, axis_point, axis):
    """Where the points start and end along an axis, as a cylinder dict's tail."""
    ts = [ac_fit.dot(ac_fit.sub(p, axis_point), axis) for p in pts]
    t0, t1 = min(ts), max(ts)
    return {'start': ac_fit.add(axis_point, ac_fit.scale(axis, t0)),
            'end': ac_fit.add(axis_point, ac_fit.scale(axis, t1)),
            'length': t1 - t0}


def fitted_cylinder(faces):
    """A least-squares cylinder through every face given, fitted together.

    The fit may run on a subsample; the extent along the axis never does, or a
    stride could step over the rim and shorten the cylinder.
    """
    pts, tris = surface_samples(faces)
    if len(pts) < 5:
        raise ac_fit.FitError('the selected surface has no usable mesh')
    w = ac_fit.vertex_weights(pts, tris)
    nrm = ac_fit.vertex_normals(pts, tris)
    step = -(-len(pts) // MAX_FIT_POINTS)
    try:
        fit = ac_fit.fit_cylinder(pts[::step], normals=nrm[::step],
                                  weights=w[::step])
    except ac_fit.FitError as exc:
        if 'collinear' in str(exc):
            raise ac_fit.FitError('that surface is flat - there is no cylinder '
                                  'to fit to it')
        raise
    fit.update(_axis_extent(pts, fit['axis_point'], fit['axis']))
    fit['points'] = len(pts[::step])
    return fit


def cylinder_of_face(face):
    """The analytic cylinder a cylindrical face lies on, clipped to the face."""
    cyl = adsk.core.Cylinder.cast(face.geometry)
    if cyl is None:
        raise ac_fit.FitError('that face is not cylindrical - use "%s" for it'
                              % DIRECTION_MODES[3][1])
    o = _v(cyl.origin)
    d = ac_fit.unit(_v(cyl.axis))
    pts, _t = face_samples(face)
    out = {'axis_point': o, 'axis': d, 'radius': cyl.radius}
    out.update(_axis_extent(pts, o, d))
    return out


def _midpoint(a, b):
    return ac_fit.scale(ac_fit.add(a, b), 0.5)


def axis_midpoint(ent):
    """(point, description): the default origin for a direction read off `ent`."""
    edge = adsk.fusion.BRepEdge.cast(ent)
    if edge:
        g = edge.geometry
        if adsk.core.Line3D.cast(g):
            return (_midpoint(_v(edge.startVertex.geometry),
                              _v(edge.endVertex.geometry)),
                    'the midpoint of the edge')
        circ = adsk.core.Circle3D.cast(g)
        if circ:
            return _v(circ.center), 'the centre of the circular edge'
        arc = adsk.core.Arc3D.cast(g)
        if arc:
            return _v(arc.center), 'the centre of the arc'
        raise ac_fit.FitError('that edge is not straight and not circular')

    face = adsk.fusion.BRepFace.cast(ent)
    if face:
        g = face.geometry
        surf = adsk.core.Cylinder.cast(g) or adsk.core.Cone.cast(g)
        if surf:
            pts, _t = face_samples(face)
            ext = _axis_extent(pts, _v(surf.origin), ac_fit.unit(_v(surf.axis)))
            return (_midpoint(ext['start'], ext['end']),
                    'the midpoint of the axis over the face')
        return origin_from_entity(face, True)

    ax = adsk.fusion.ConstructionAxis.cast(ent)
    if ax:
        # an axis built from an edge or a round face has a real extent to take the
        # middle of; any other construction axis is infinite
        try:
            defn = ax.definition
            by_edge = adsk.fusion.ConstructionAxisEdgeDefinition.cast(defn)
            if by_edge:
                return axis_midpoint(by_edge.edgeEntity)
            by_face = adsk.fusion.ConstructionAxisCircularFaceDefinition.cast(defn)
            if by_face:
                return axis_midpoint(by_face.circularFace)
        except Exception:
            log(traceback.format_exc())
        return (_v(ax.geometry.origin),
                'a point on the construction axis - it is infinite, so there is '
                'no midpoint; pick an origin if this is not the one')

    sl = adsk.fusion.SketchLine.cast(ent)
    if sl:
        g = sl.worldGeometry
        return (_midpoint(_v(g.startPoint), _v(g.endPoint)),
                'the midpoint of the sketch line')

    cp = adsk.fusion.ConstructionPlane.cast(ent)
    if cp:
        return _v(cp.geometry.origin), 'the construction plane origin'

    raise ac_fit.FitError('nothing in that selection carries a direction')


def _drop_onto(face, c):
    """The point of `face` nearest-by-parameter to c, or None."""
    ok, param = face.evaluator.getParameterAtPoint(_pt(c))
    if ok:
        ok2, p = face.evaluator.getPointAtParameter(param)
        if ok2:
            return _v(p)
    return None


def surface_centre(faces, on_surface=True):
    """(point, description): the area-weighted centre of gravity of the faces."""
    if len(faces) == 1:
        return origin_from_entity(faces[0], on_surface)
    tot = 0.0
    acc = (0.0, 0.0, 0.0)
    for f in faces:
        a = f.area
        acc = ac_fit.add(acc, ac_fit.scale(_v(f.centroid), a))
        tot += a
    if tot <= 0.0:
        raise ac_fit.FitError('the selected faces have no area')
    c = ac_fit.scale(acc, 1.0 / tot)
    what = 'the centre of gravity of the %d surfaces' % len(faces)
    if on_surface:
        drops = [p for p in (_drop_onto(f, c) for f in faces) if p]
        if drops:
            p = min(drops, key=lambda q: ac_fit.length(ac_fit.sub(q, c)))
            moved = ac_fit.length(ac_fit.sub(p, c)) * CM
            if moved > 1.0e-6:
                what += ', dropped %.3f mm onto the nearest' % moved
            c = p
    return c, what


def direction_from_entity(ent):
    """(direction, description) from anything that carries one."""
    edge = adsk.fusion.BRepEdge.cast(ent)
    if edge:
        g = edge.geometry
        line = adsk.core.Line3D.cast(g)
        if line:
            d = line.startPoint.vectorTo(line.endPoint)
            return ac_fit.unit(_v(d)), 'the straight edge'
        circ = adsk.core.Circle3D.cast(g)
        if circ:
            return ac_fit.unit(_v(circ.normal)), 'the circular edge normal'
        arc = adsk.core.Arc3D.cast(g)
        if arc:
            return ac_fit.unit(_v(arc.normal)), 'the arc normal'
        raise ac_fit.FitError('that edge is not straight and not circular')

    face = adsk.fusion.BRepFace.cast(ent)
    if face:
        g = face.geometry
        cyl = adsk.core.Cylinder.cast(g)
        if cyl:
            return ac_fit.unit(_v(cyl.axis)), 'the cylindrical face axis'
        cone = adsk.core.Cone.cast(g)
        if cone:
            return ac_fit.unit(_v(cone.axis)), 'the conical face axis'
        pl = adsk.core.Plane.cast(g)
        if pl:
            return ac_fit.unit(_v(pl.normal)), 'the planar face normal'
        raise ac_fit.FitError('that face is neither planar, cylindrical nor '
                              'conical - use one of the fitted modes for it')

    ax = adsk.fusion.ConstructionAxis.cast(ent)
    if ax:
        return ac_fit.unit(_v(ax.geometry.direction)), 'the construction axis'

    sl = adsk.fusion.SketchLine.cast(ent)
    if sl:
        g = sl.worldGeometry
        return ac_fit.unit(_v(g.startPoint.vectorTo(g.endPoint))), 'the sketch line'

    cp = adsk.fusion.ConstructionPlane.cast(ent)
    if cp:
        return ac_fit.unit(_v(cp.geometry.normal)), 'the construction plane normal'

    raise ac_fit.FitError('nothing in that selection carries a direction')


def origin_from_entity(ent, on_surface=True):
    """(point, description). A face gives its centroid; a point gives itself."""
    face = adsk.fusion.BRepFace.cast(ent)
    if face:
        c = face.centroid
        if c is None:
            raise ac_fit.FitError('Fusion did not return a centroid for that face')
        what = 'the centre of gravity of the surface'
        if on_surface:
            ok, param = face.evaluator.getParameterAtPoint(c)
            if ok:
                ok2, p = face.evaluator.getPointAtParameter(param)
                if ok2 and not p.isEqualTo(c):
                    moved = c.distanceTo(p) * CM
                    if moved > 1.0e-6:
                        what = ('the centre of gravity of the surface, dropped '
                                '%.3f mm onto it' % moved)
                    c = p
        return _v(c), what

    v = adsk.fusion.BRepVertex.cast(ent)
    if v:
        return _v(v.geometry), 'the vertex'

    sp = adsk.fusion.SketchPoint.cast(ent)
    if sp:
        return _v(sp.worldGeometry), 'the sketch point'

    cp = adsk.fusion.ConstructionPoint.cast(ent)
    if cp:
        return _v(cp.geometry), 'the construction point'

    raise ac_fit.FitError('that selection is neither a point nor a face')


# --------------------------------------------------------------------------- #
# the cylinder itself
# --------------------------------------------------------------------------- #

def cylinder_ends(origin, direction, length_cm, symmetric, flip):
    """The two end points, in internal centimetres."""
    d = ac_fit.unit(direction)
    if flip:
        d = ac_fit.scale(d, -1.0)
    if symmetric:
        half = 0.5 * length_cm
        return (ac_fit.add(origin, ac_fit.scale(d, -half)),
                ac_fit.add(origin, ac_fit.scale(d, half)))
    return origin, ac_fit.add(origin, ac_fit.scale(d, length_cm))


def _add_body(comp, tmp, name):
    """Put a temporary BRepBody into the document. Parametric or direct alike.

    In a parametric design a body built from geometry rather than from features
    has to go inside a BaseFeature, or bRepBodies.add refuses it.
    """
    parametric = (comp.parentDesign.designType
                  == adsk.fusion.DesignTypes.ParametricDesignType)
    if parametric:
        base = comp.features.baseFeatures.add()
        base.startEdit()
        try:
            body = comp.bRepBodies.add(tmp, base)
        finally:
            base.finishEdit()
    else:
        body = comp.bRepBodies.add(tmp)
    try:
        body.name = name
    except Exception:
        pass
    return body


def build_cylinder(comp, p0, p1, radius_cm, name):
    """A solid cylinder body, from the two end points and the radius."""
    tmp = adsk.fusion.TemporaryBRepManager.get().createCylinderOrCone(
        _pt(p0), radius_cm, _pt(p1), radius_cm)
    if tmp is None:
        raise ac_fit.FitError('Fusion would not build a cylinder from those ends '
                              '- check the length and diameter are not zero')
    return _add_body(comp, tmp, name)


def resolve(inputs):
    """Everything the command needs, or a FitError saying what is missing.

    Returns a dict; the dialog renders it and execute builds from it, so the
    preview and the result can never disagree about what is being made.
    """
    dia_cm = inputs.itemById('diameter').value
    len_cm = inputs.itemById('length').value
    symmetric = inputs.itemById('symmetric').value
    flip = inputs.itemById('flip').value
    on_surface = inputs.itemById('onSurface').value
    mode = current_mode(inputs)

    # direction first; the origin defaults from whatever gave it
    plane = None
    cyl = None
    if mode == 'fit':
        faces = selected_faces(inputs, 'selPlane')
        if not faces:
            raise ac_fit.FitError('select the surface to be perpendicular to')
        plane = fitted_plane(faces)
        direction = plane['normal']
        dir_what = 'the normal of a plane fitted to %d face(s)' % len(faces)
        origin, origin_what = surface_centre(faces, on_surface)
        anchor = faces[0]
    elif mode == 'entity':
        ents = selected(inputs.itemById('selAxis'))
        if not ents:
            raise ac_fit.FitError('select the edge or axis to run along')
        direction, dir_what = direction_from_entity(ents[0])
        origin, origin_what = axis_midpoint(ents[0])
        anchor = ents[0]
    else:
        cyl, faces = surface_cylinder(inputs, mode)
        if cyl is None:
            raise ac_fit.FitError('select the cylindrical face'
                                  if mode == 'cylface' else
                                  'select the surface to fit a cylinder to')
        direction = cyl['axis']
        dir_what = ('the axis of the cylindrical face' if mode == 'cylface' else
                    'the axis of a cylinder fitted to %d face(s)' % len(faces))
        origin = _midpoint(cyl['start'], cyl['end'])
        origin_what = 'the midpoint of the axis over the surface'
        anchor = faces[0]

    org_sel = selected(inputs.itemById('selOrigin'))
    if org_sel:
        origin, origin_what = origin_from_entity(org_sel[0], on_surface)

    if dia_cm <= 0.0:
        raise ac_fit.FitError('the diameter must be greater than zero')
    if len_cm <= 0.0:
        raise ac_fit.FitError('the length must be greater than zero')

    p0, p1 = cylinder_ends(origin, direction, len_cm, symmetric, flip)
    return {'origin': origin, 'origin_what': origin_what,
            'origin_picked': bool(org_sel),
            'direction': direction, 'direction_what': dir_what,
            'plane': plane, 'cylinder': cyl, 'p0': p0, 'p1': p1,
            'radius_cm': 0.5 * dia_cm, 'diameter_mm': dia_cm * CM,
            'length_mm': len_cm * CM, 'symmetric': symmetric, 'flip': flip,
            'entity': anchor}


def current_mode(inputs):
    return DIRECTION_MODES[inputs.itemById('dirMode').selectedItem.index][0]


def selected_faces(inputs, sel_id):
    return [f for f in (adsk.fusion.BRepFace.cast(e)
                        for e in selected(inputs.itemById(sel_id))) if f]


def surface_cylinder(inputs, mode):
    """(cylinder dict, faces) for the two cylinder modes, (None, []) if unpicked."""
    if mode == 'cylface':
        faces = selected_faces(inputs, 'selCylFace')
        return (cylinder_of_face(faces[0]) if faces else None), faces
    faces = selected_faces(inputs, 'selCylFit')
    return (fitted_cylinder(faces) if faces else None), faces


def body_name(r):
    return 'Cylinder %.2f x %.2f mm' % (r['diameter_mm'], r['length_mm'])


# --------------------------------------------------------------------------- #
# dialog
# --------------------------------------------------------------------------- #

def _esc(t):
    return (str(t).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))


def _rows(pairs):
    out = ['<table cellpadding="0" cellspacing="0">']
    for k, v in pairs:
        out.append('<tr><td width="150"><b>%s</b></td><td>%s</td></tr>'
                   % (_esc(k), v if k == '' else _esc(v)))
    out.append('</table>')
    return ''.join(out)


def on_created(app, cmd):
    s = load_settings()
    cmd.isRepeatable = True
    cmd.okButtonText = 'Create'
    inputs = cmd.commandInputs

    mode_i = int(s.get('dirMode', 0))
    if not 0 <= mode_i < len(DIRECTION_MODES):
        mode_i = 0
    dm = inputs.addDropDownCommandInput('dirMode', 'Direction from',
                                        adsk.core.DropDownStyles
                                        .TextListDropDownStyle)
    for i, (_k, lab) in enumerate(DIRECTION_MODES):
        dm.listItems.add(lab, i == mode_i)

    sp = inputs.addSelectionInput('selPlane', 'Surface',
                                  'Surfaces to fit one plane through; the '
                                  'cylinder runs along its normal')
    sp.addSelectionFilter('Faces')
    sp.setSelectionLimits(0, 0)

    sa = inputs.addSelectionInput('selAxis', 'Edge or axis',
                                  'A straight or circular edge, construction '
                                  'axis, sketch line, or a cylindrical face')
    for f in ('LinearEdges', 'CircularEdges', 'ConstructionLines', 'SketchLines',
              'Faces', 'ConstructionPlanes'):
        sa.addSelectionFilter(f)
    sa.setSelectionLimits(0, 1)

    sc = inputs.addSelectionInput('selCylFace', 'Cylindrical face',
                                  'A cylindrical face; its axis is used exactly')
    sc.addSelectionFilter('CylindricalFaces')
    sc.setSelectionLimits(0, 1)

    sf = inputs.addSelectionInput('selCylFit', 'Surface',
                                  'Surfaces to fit one cylinder through - they '
                                  'need not be cylindrical')
    sf.addSelectionFilter('Faces')
    sf.setSelectionLimits(0, 0)

    so = inputs.addSelectionInput('selOrigin', 'Origin (optional)',
                                  'Leave empty for the default from the '
                                  'direction; or a point, or a surface to take '
                                  'the centre of gravity of')
    for f in ('Vertices', 'SketchPoints', 'ConstructionPoints', 'Faces'):
        so.addSelectionFilter(f)
    so.setSelectionLimits(0, 1)

    inputs.addValueInput('diameter', 'Diameter', 'mm',
                         adsk.core.ValueInput.createByReal(
                             float(s['diameter']) * MM))
    inputs.addValueInput('length', 'Length', 'mm',
                         adsk.core.ValueInput.createByReal(
                             float(s['length']) * MM))
    inputs.addBoolValueInput('symmetric', 'Half each way from the origin', True,
                             '', bool(s['symmetric']))
    inputs.addBoolValueInput('flip', 'Flip the direction', True, '',
                             bool(s['flip']))
    inputs.addBoolValueInput('onSurface', 'Put a surface origin on the surface',
                             True, '', bool(s['onSurface']))

    info = inputs.addTextBoxCommandInput('info', '', '', 10, True)
    info.isFullWidth = True
    refresh(inputs)
    focus_direction(inputs)


def focus_direction(inputs):
    """Put the cursor on the current mode's direction selection, not the origin."""
    try:
        inputs.itemById(DIRECTION_INPUT[current_mode(inputs)]).hasFocus = True
    except Exception:
        log(traceback.format_exc())


def refresh(inputs):
    mode = current_mode(inputs)
    for m, sel_id in DIRECTION_INPUT.items():
        inputs.itemById(sel_id).isVisible = m == mode
    org = selected(inputs.itemById('selOrigin'))
    inputs.itemById('onSurface').isVisible = (
        adsk.fusion.BRepFace.cast(org[0]) is not None if org else mode == 'fit')
    inputs.itemById('info').formattedText = describe(inputs)


def take_surface_size(inputs):
    """Default the diameter and length to the surface's own, in the cylinder modes.

    Written into the inputs once, when the surface changes, so they stay editable:
    a pin longer than its bore is the usual case, not the exception.
    """
    mode = current_mode(inputs)
    if mode not in ('cylface', 'cylfit'):
        return
    try:
        cyl, _faces = surface_cylinder(inputs, mode)
    except ac_fit.FitError:
        return
    if cyl is None:
        return
    inputs.itemById('diameter').value = 2.0 * cyl['radius']
    if cyl['length'] > 1.0e-6:
        inputs.itemById('length').value = cyl['length']


def describe(inputs):
    try:
        r = resolve(inputs)
    except ac_fit.FitError as exc:
        return _rows([('', _esc(str(exc)))])
    except Exception:
        log(traceback.format_exc())
        return _rows([('', _esc(short_error(traceback.format_exc())))])

    d = r['direction']
    if r['flip']:
        d = ac_fit.scale(d, -1.0)
    pairs = [
        ('direction', '%.4f, %.4f, %.4f &mdash; %s'
         % (d[0], d[1], d[2], r['direction_what'])),
        ('origin (picked)' if r['origin_picked'] else 'origin (default)',
         '%.3f, %.3f, %.3f mm &mdash; %s'
         % (r['origin'][0] * CM, r['origin'][1] * CM, r['origin'][2] * CM,
            r['origin_what'])),
        ('cylinder', '&#8960; %.2f &times; %.2f mm, %s'
         % (r['diameter_mm'], r['length_mm'],
            'half each way' if r['symmetric'] else 'all one way')),
    ]
    c = r['cylinder']
    if c:
        pairs.append(('surface', '&#8960; %.3f mm, %.3f mm along the axis'
                      % (2.0 * c['radius'] * CM, c['length'] * CM)))
        if 'rms' in c:
            pairs.append(('cylinder fit', 'rms %.4f mm, worst %.4f mm, %.0f&deg; '
                          'of arc, over %d points'
                          % (c['rms'] * CM, c['max_dev'] * CM, c['sweep_deg'],
                             c['points'])))
            if c['sweep_deg'] < 45.0:
                pairs.append(('', '<b>only %.0f&deg; of the circumference is '
                                  'there</b> - the radius and the axis are '
                                  'weakly pinned down by so little arc'
                              % c['sweep_deg']))
            if c['max_dev'] > 0.05 * c['radius']:
                pairs.append(('', '<b>the surface is up to %.1f %% of the radius '
                                  'off that cylinder</b> - it is not really a '
                                  'cylinder, so check the axis is the one you '
                                  'meant' % (100.0 * c['max_dev'] / c['radius'])))
    p = r['plane']
    if p:
        pairs.append(('plane fit', 'rms %.4f mm, worst %.4f mm, over %d points'
                      % (p['rms'] * CM, p['max_dev'] * CM, p['points'])))
        span = max(p['u_max'] - p['u_min'], p['v_max'] - p['v_min']) * CM
        if span > 0 and p['max_dev'] * CM > 0.02 * span:
            pairs.append(('', '<b>the surface is %.1f %% of its own width away '
                              'from that plane</b> - the normal is only as '
                              'meaningful as the fit, so check this is the '
                              'direction you meant'
                          % (100.0 * p['max_dev'] / max(p['u_max'] - p['u_min'],
                                                        p['v_max'] - p['v_min']))))
    return _rows(pairs)


def on_input_changed(app, inputs, changed_id):
    if changed_id == 'dirMode':
        if current_mode(inputs) in AXIS_MODES:
            # the default origin is now the middle of an axis
            inputs.itemById('symmetric').value = True
        focus_direction(inputs)
    if changed_id in ('dirMode', 'selCylFace', 'selCylFit'):
        take_surface_size(inputs)
    refresh(inputs)


def on_validate(inputs):
    try:
        resolve(inputs)
        return True
    except Exception:
        return False


def settings_from(inputs):
    return {
        'diameter': inputs.itemById('diameter').value * CM,
        'length': inputs.itemById('length').value * CM,
        'symmetric': inputs.itemById('symmetric').value,
        'flip': inputs.itemById('flip').value,
        'onSurface': inputs.itemById('onSurface').value,
        'dirMode': inputs.itemById('dirMode').selectedItem.index,
    }


def target_component(app, entity):
    """Build into the component the selection belongs to, not always the root."""
    try:
        des = adsk.fusion.Design.cast(app.activeProduct)
        ent = entity
        for cast in (adsk.fusion.BRepFace, adsk.fusion.BRepEdge,
                     adsk.fusion.BRepVertex):
            c = cast.cast(ent)
            if c:
                return c.body.parentComponent
        for cast in (adsk.fusion.SketchPoint, adsk.fusion.SketchLine):
            c = cast.cast(ent)
            if c:
                return c.parentSketch.parentComponent
        for cast in (adsk.fusion.ConstructionPoint, adsk.fusion.ConstructionAxis,
                     adsk.fusion.ConstructionPlane):
            c = cast.cast(ent)
            if c:
                return c.component
        return des.rootComponent
    except Exception:
        return adsk.fusion.Design.cast(app.activeProduct).rootComponent


def on_preview(app, inputs, args):
    """The real thing, built and left for Fusion to throw away. isValidResult stays
    False so execute builds it again properly rather than adopting the preview."""
    try:
        r = resolve(inputs)
    except Exception:
        return
    comp = target_component(app, r['entity'])
    build_cylinder(comp, r['p0'], r['p1'], r['radius_cm'], body_name(r))


def on_execute(app, ui, inputs):
    r = resolve(inputs)
    save_settings(settings_from(inputs))
    comp = target_component(app, r['entity'])
    body = build_cylinder(comp, r['p0'], r['p1'], r['radius_cm'], body_name(r))
    log('%s  %s  origin %.4f %.4f %.4f  dir %.4f %.4f %.4f'
        % (VERSION, body.name, r['origin'][0], r['origin'][1], r['origin'][2],
           r['direction'][0], r['direction'][1], r['direction'][2]))
    return body


def on_destroy(app):
    pass


# --------------------------------------------------------------------------- #
# toolbar
# --------------------------------------------------------------------------- #

TOOLTIP = ('Create a solid cylinder from a direction and an origin.\n\n'
           'The direction comes first: the normal of a plane fitted to a surface, '
           'an edge or axis, the axis of a cylindrical face, or the axis of a '
           'cylinder fitted to any surface. The origin defaults from it - the '
           'centre of the surface, or the midpoint of the axis - and can be '
           'overridden with a point or a face. Every fit reports its own rms and '
           'worst deviation.')

PANELS = ['SolidCreatePanel', 'ParaMeshModifyPanel']


def install(app, ui, created_handler_factory):
    old = ui.commandDefinitions.itemById(CMD_ID)
    if old:
        old.deleteMe()
    folder = RES_DIR if os.path.isdir(RES_DIR) else ''
    cmd_def = ui.commandDefinitions.addButtonDefinition(
        CMD_ID, CMD_NAME, TOOLTIP, folder)
    cmd_def.commandCreated.add(created_handler_factory())
    placed = []
    for pid in PANELS:
        panel = ui.allToolbarPanels.itemById(pid)
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)
            placed.append(pid)
    return placed


def uninstall(ui):
    for pid in PANELS:
        panel = ui.allToolbarPanels.itemById(pid)
        if panel:
            c = panel.controls.itemById(CMD_ID)
            if c:
                c.deleteMe()
    d = ui.commandDefinitions.itemById(CMD_ID)
    if d:
        d.deleteMe()


# --------------------------------------------------------------------------- #
# dev bridge
# --------------------------------------------------------------------------- #

def dev_ping(app, job):
    des = adsk.fusion.Design.cast(app.activeProduct)
    return {'version': VERSION, 'document': app.activeDocument.name,
            'design_type': ('parametric'
                            if des.designType
                            == adsk.fusion.DesignTypes.ParametricDesignType
                            else 'direct'),
            'bodies': des.rootComponent.bRepBodies.count}


def _find_face(app, job):
    des = adsk.fusion.Design.cast(app.activeProduct)
    name = job.get('body')
    for b in des.rootComponent.bRepBodies:
        if name in (None, b.name):
            i = int(job.get('face', 0))
            if i < b.faces.count:
                return b.faces.item(i)
    raise ac_fit.FitError('no such body/face')


def dev_fit(app, job):
    """Fit a plane to a face and report it, without making anything."""
    f = _find_face(app, job)
    p = fitted_plane([f])
    c, what = origin_from_entity(f, bool(job.get('on_surface', True)))
    return {'normal': p['normal'], 'origin_mm': [v * CM for v in c],
            'origin_what': what,
            'rms_mm': p['rms'] * CM, 'max_dev_mm': p['max_dev'] * CM,
            'points': p['points'], 'face_area_mm2': f.area * CM * CM}


def dev_make(app, job):
    """Build a cylinder from a face, the way the dialog would."""
    f = _find_face(app, job)
    p = fitted_plane([f])
    origin, _w = origin_from_entity(f, bool(job.get('on_surface', True)))
    dia = float(job.get('diameter_mm', 4.0)) * MM
    ln = float(job.get('length_mm', 20.0)) * MM
    p0, p1 = cylinder_ends(origin, p['normal'], ln,
                           bool(job.get('symmetric', False)),
                           bool(job.get('flip', False)))
    comp = f.body.parentComponent
    body = build_cylinder(comp, p0, p1, 0.5 * dia,
                          'Cylinder %.2f x %.2f mm' % (dia * CM, ln * CM))
    return {'body': body.name, 'solid': body.isSolid,
            'volume_mm3': body.volume * 1000.0,
            'expected_mm3': math.pi * (0.5 * dia * CM) ** 2 * (ln * CM),
            'faces': body.faces.count,
            'p0_mm': [v * CM for v in p0], 'p1_mm': [v * CM for v in p1]}


def vclose3(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _fake_inputs(mode, ents, origin=None, dia_cm=0.4, len_cm=2.0):
    """A stand-in for the dialog's CommandInputs, so resolve() and
    take_surface_size() run on real entities without a dialog open."""
    from types import SimpleNamespace as NS

    def sel(items):
        return NS(selectionCount=len(items),
                  selection=lambda i: NS(entity=items[i]))

    d = {'diameter': NS(value=dia_cm), 'length': NS(value=len_cm),
         'symmetric': NS(value=True), 'flip': NS(value=False),
         'onSurface': NS(value=True),
         'dirMode': NS(selectedItem=NS(
             index=[k for k, _l in DIRECTION_MODES].index(mode))),
         'selOrigin': sel([origin] if origin else [])}
    for m, sel_id in DIRECTION_INPUT.items():
        d[sel_id] = sel(ents if m == mode else [])
    return NS(itemById=d.__getitem__)


def dev_selftest(app, job):
    """End to end inside a live Fusion, against answers known in advance.

    Runs in a NEW document that is closed unsaved afterwards (left open with
    "keep"), never in the active one: in a parametric design body.deleteMe()
    inside a BaseFeature fails silently, so "build and delete again" left the
    scratch bodies in real work - it did, once, in a working plate design.

    A box gives a planar face, whose fitted plane must come back exact and whose
    centroid must be the face centre; a cylinder gives the two cylinder modes an
    axis, radius and midpoint known in advance; a sphere gives a curved face that
    must come back with a large deviation - that is the path that warns you the
    direction is not trustworthy, and it is worth proving it fires.
    """
    prev = app.activeDocument
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        return _selftest(adsk.fusion.Design.cast(
            doc.products.itemByProductType('DesignProductType')))
    finally:
        if not job.get('keep'):
            doc.close(False)
            if prev:
                prev.activate()


def _selftest(des):
    comp = des.rootComponent
    tbm = adsk.fusion.TemporaryBRepManager.get()
    results = []

    def note(name, ok, detail=''):
        results.append({'check': name, 'ok': bool(ok), 'detail': str(detail)})

    try:
        # ---- a 4 x 6 x 10 cm box, axis-aligned ---------------------------- #
        centre = adsk.core.Point3D.create(0.0, 0.0, 0.0)
        obb = adsk.core.OrientedBoundingBox3D.create(
            centre, adsk.core.Vector3D.create(1.0, 0.0, 0.0),
            adsk.core.Vector3D.create(0.0, 1.0, 0.0), 4.0, 6.0, 10.0)
        box = _add_body(comp, tbm.createBox(obb), 'ac_selftest_box')

        top = None
        for f in box.faces:
            pl = adsk.core.Plane.cast(f.geometry)
            if pl and abs(pl.normal.z) > 0.999:
                if top is None or f.centroid.z > top.centroid.z:
                    top = f
        note('the box has a planar top face', top is not None)

        if top is not None:
            fit = fitted_plane([top])
            note('a planar face fits exactly',
                 fit['rms'] * CM < 1e-6 and fit['max_dev'] * CM < 1e-6,
                 'rms %.3e mm, worst %.3e mm' % (fit['rms'] * CM,
                                                 fit['max_dev'] * CM))
            note('and its normal is the face normal',
                 abs(abs(fit['normal'][2]) - 1.0) < 1e-9, fit['normal'])

            org, what = origin_from_entity(top, True)
            note('a planar face gives its centre',
                 abs(org[0]) < 1e-6 and abs(org[1]) < 1e-6
                 and abs(org[2] - 5.0) < 1e-6,
                 '%s at %.4f %.4f %.4f cm' % (what, org[0], org[1], org[2]))

            d, what = direction_from_entity(top)
            note('a planar face also reads as a direction',
                 abs(abs(d[2]) - 1.0) < 1e-9, what)

            # ---- the cylinder, against pi r^2 L ------------------------- #
            dia_mm, len_mm = 4.0, 20.0
            p0, p1 = cylinder_ends(org, fit['normal'], len_mm * MM, False, False)
            cyl = build_cylinder(comp, p0, p1, 0.5 * dia_mm * MM, 'ac_selftest_cyl')
            want = math.pi * (0.5 * dia_mm) ** 2 * len_mm
            got = cyl.volume * 1000.0
            note('the cylinder volume is pi r^2 L',
                 abs(got - want) / want < 1e-4,
                 '%.4f vs %.4f mm3' % (got, want))
            note('and it is a solid', cyl.isSolid)
            bb = cyl.boundingBox
            note('it starts at the origin and runs one length',
                 abs((bb.maxPoint.z - bb.minPoint.z) * CM - len_mm) < 1e-6,
                 '%.4f mm tall' % ((bb.maxPoint.z - bb.minPoint.z) * CM))
            note('and it sits on the face, not through it',
                 abs(bb.minPoint.z * CM - org[2] * CM) < 1e-6,
                 'base at %.4f mm' % (bb.minPoint.z * CM))

            # ---- the dialog path, direction first, on the real face ------- #
            r = resolve(_fake_inputs('fit', [top]))
            note('fit mode defaults the origin to the face centre, unpicked',
                 vclose3(r['origin'], (0.0, 0.0, 5.0)) and not r['origin_picked'],
                 r['origin_what'])
            vtx = top.vertices.item(0)
            r = resolve(_fake_inputs('fit', [top], origin=vtx))
            note('and a picked origin overrides it',
                 vclose3(r['origin'], _v(vtx.geometry)) and r['origin_picked'],
                 r['origin_what'])

        # ---- a straight edge: the origin defaults to its midpoint ---------- #
        edge = next(e for e in box.edges if adsk.core.Line3D.cast(e.geometry))
        r = resolve(_fake_inputs('entity', [edge]))
        # every edge midpoint of a centred 4 x 6 x 10 box has exactly one zero
        # coordinate and the other two among the half extents 2, 3, 5
        a = sorted(abs(x) for x in r['origin'])
        note('an edge defaults the origin to its midpoint',
             a[0] < 1e-9 and any(vclose3(a[1:], pair)
                                 for pair in ((2, 3), (2, 5), (3, 5))),
             '%s at %s cm' % (r['origin_what'], r['origin']))

        # ---- a cylinder r = 1 cm from z = -1 to z = 2 at (20, 10) ---------- #
        cb = _add_body(comp, tbm.createCylinderOrCone(
            adsk.core.Point3D.create(20.0, 10.0, -1.0), 1.0,
            adsk.core.Point3D.create(20.0, 10.0, 2.0), 1.0), 'ac_selftest_bore')
        side = next((f for f in cb.faces if adsk.core.Cylinder.cast(f.geometry)),
                    None)
        note('the test cylinder has a cylindrical face', side is not None)
        if side is not None:
            mid = (20.0, 10.0, 0.5)
            for mode in ('cylface', 'cylfit'):
                r = resolve(_fake_inputs(mode, [side]))
                c = r['cylinder']
                note('%s: the axis is the cylinder axis' % mode,
                     abs(abs(r['direction'][2]) - 1.0) < 1e-9, r['direction'])
                note('%s: the origin defaults to the midpoint of the axis' % mode,
                     vclose3(r['origin'], mid, 1e-6),
                     '%s at %s cm' % (r['origin_what'], r['origin']))
                note('%s: radius 10 mm and 30 mm along the axis' % mode,
                     abs(c['radius'] - 1.0) < 1e-6 and abs(c['length'] - 3.0) < 1e-6,
                     'r %.7f mm, %.7f mm long%s'
                     % (c['radius'] * CM, c['length'] * CM,
                        ', rms %.2e mm, %.0f deg' % (c['rms'] * CM, c['sweep_deg'])
                        if 'rms' in c else ''))
                fake = _fake_inputs(mode, [side])
                take_surface_size(fake)
                note('%s: diameter and length default to the surface' % mode,
                     abs(fake.itemById('diameter').value - 2.0) < 1e-6
                     and abs(fake.itemById('length').value - 3.0) < 1e-6)
            r = resolve(_fake_inputs('entity', [side]))
            note('entity mode on the face also takes the axis midpoint',
                 vclose3(r['origin'], mid, 1e-6), r['origin'])
            rim = next(e for e in cb.edges if adsk.core.Circle3D.cast(e.geometry))
            o, what = axis_midpoint(rim)
            note('a circular edge defaults to its centre',
                 abs(o[0] - 20.0) < 1e-9 and abs(o[1] - 10.0) < 1e-9
                 and min(abs(o[2] + 1.0), abs(o[2] - 2.0)) < 1e-9, o)

        if top is not None:
            try:
                fit = fitted_cylinder([top])
                note('a flat face does not come back as a confident cylinder',
                     fit['radius'] > 100.0 or fit['sweep_deg'] < 45.0,
                     'r %.1f mm, %.1f deg' % (fit['radius'] * CM, fit['sweep_deg']))
            except ac_fit.FitError as exc:
                note('a flat face does not come back as a confident cylinder',
                     True, 'refused: %s' % exc)

        # ---- a curved face must report a large rms, not a quiet one ------- #
        sph = _add_body(
            comp, tbm.createSphere(adsk.core.Point3D.create(30.0, 0.0, 0.0), 2.0),
            'ac_selftest_sphere')
        if sph.faces.count:
            fit = fitted_plane([sph.faces.item(0)])
            note('a curved face reports a large deviation rather than a quiet one',
                 fit['max_dev'] * CM > 1.0,
                 'worst %.3f mm over %d points' % (fit['max_dev'] * CM,
                                                   fit['points']))
            fit = fitted_cylinder([sph.faces.item(0)])
            note('and a sphere trips the not-a-cylinder warning',
                 fit['max_dev'] > 0.05 * fit['radius'],
                 'worst %.3f mm on r %.3f mm' % (fit['max_dev'] * CM,
                                                 fit['radius'] * CM))
    except Exception:
        note('the self-test ran to the end', False,
             short_error(traceback.format_exc()))
        log(traceback.format_exc())

    return {'passed': sum(1 for r in results if r['ok']),
            'failed': sum(1 for r in results if not r['ok']),
            'checks': results}


def dev_timeline(app, job):
    """The last few timeline items, read-only - to see what a run left behind."""
    des = adsk.fusion.Design.cast(app.activeProduct)
    tl = des.timeline
    n = int(job.get('last', 15))
    out = []
    for i in range(max(0, tl.count - n), tl.count):
        it = tl.item(i)
        ent = it.entity
        row = {'index': i, 'name': it.name,
               'type': ent.objectType if ent else None,
               'suppressed': it.isSuppressed}
        bf = adsk.fusion.BaseFeature.cast(ent)
        if bf:
            row['bodies'] = [b.name for b in bf.bodies]
        out.append(row)

    def box(b):
        bb = b.boundingBox
        return [round(v * CM, 3) for v in (_v(bb.minPoint) + _v(bb.maxPoint))]
    return {'count': tl.count, 'marker': tl.markerPosition, 'items': out,
            'bodies': [[b.name, box(b), round(b.volume * 1000.0, 3)]
                       for b in des.rootComponent.bRepBodies
                       if b.name.startswith(('Body', 'ac_'))]}


def dev_drop_features(app, job):
    """Delete the named BaseFeatures, and only if they are the timeline's tail -
    for undoing what a test run left, never for reaching into real work."""
    des = adsk.fusion.Design.cast(app.activeProduct)
    tl = des.timeline
    names = list(job['names'])
    tail = [tl.item(i) for i in range(tl.count - len(names), tl.count)]
    if sorted(it.name for it in tail) != sorted(names):
        raise ac_fit.FitError('those are not the last %d timeline items: %s'
                              % (len(names), [it.name for it in tail]))
    for it in reversed(tail):
        if not adsk.fusion.BaseFeature.cast(it.entity):
            raise ac_fit.FitError('%s is not a BaseFeature' % it.name)
    for it in reversed(tail):
        it.entity.deleteMe()
    return {'count': tl.count}


DEV_COMMANDS = {
    'drop_features': dev_drop_features,
    'timeline': dev_timeline,
    'ping': dev_ping,
    'fit': dev_fit,
    'make': dev_make,
    'selftest': dev_selftest,
}

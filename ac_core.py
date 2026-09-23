"""Axis Cylinder - all the working code. RE-IMPORTED ON EVERY COMMAND EVENT.

Because of that reload, nothing may keep state in a module global between events:
anything the dialog decides has to be readable back off the inputs themselves.

A cylinder needs three things - where it starts, which way it points, and how big
it is - and the first two are the ones worth getting right:

ORIGIN
    A point is a point. A face gives its CENTROID, which Fusion computes exactly
    (BRepFace.centroid) rather than from a tessellation. On a curved face that
    centroid is the centre of gravity of the surface and generally does not lie ON
    it, so there is an option - on by default - to drop it onto the face with the
    surface evaluator. For a screw trajectory starting at the bone you want the
    point on the bone; for an axis through the middle of a bore you do not.

DIRECTION
    Either read off an entity that already carries one (a straight edge, a
    construction axis, a sketch line, the axis of a cylinder or cone, the normal
    of a planar face), or FITTED: a total-least-squares plane through the faces
    you select, and the cylinder runs along its normal. Fitting is the point of
    the second mode - a scan-derived or lofted face has no analytic normal to read,
    and the plane through it is the only well-defined thing to be perpendicular to.

    The fit reports its own rms and worst deviation, because a normal is only as
    meaningful as the plane it came from. A plane fitted to something strongly
    curved is a number the tool can produce and a direction you should not trust,
    and the only honest thing to do is show how far off the surface actually is.

The body is built with TemporaryBRepManager and added through a BaseFeature, which
works the same way in a parametric and a direct design. A sketch-and-extrude would
be parametrically editable, but positioning it needs a construction plane, and
ConstructionPlaneInput.setByPlane is direct-modelling only - so it would work in
exactly the designs where it matters least.
"""

import math
import os
import traceback

import adsk.core
import adsk.fusion

import ac_fit

ADDIN_DIR = os.path.dirname(os.path.realpath(__file__))
RES_DIR = os.path.join(ADDIN_DIR, 'resources', 'AxisCylinder')
LOG_PATH = os.path.join(ADDIN_DIR, 'AxisCylinder.log')
SETTINGS = os.path.join(ADDIN_DIR, 'last_settings.json')

CMD_ID = 'AxisCylinderCreate'
CMD_NAME = 'Cylinder from Axis'

CM = 10.0          # internal cm -> mm
MM = 0.1           # mm -> internal cm

VERSION = '1.0.0'

# key, label
DIRECTION_MODES = [
    ('fit', 'Perpendicular to a surface (fitted plane)'),
    ('entity', 'Along an edge, axis or cylinder'),
]

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


def face_samples(face):
    """Display-mesh vertices of one face, with lumped-area weights.

    The display mesh is what both Fit Primitives and Mesh to Part fit through, and
    for the same reason: it exists for every face, analytic or not, and it is the
    only representation a lofted or imported surface shares with a plane.
    """
    tm = face.meshManager.displayMeshes.bestMesh
    coords = tm.nodeCoordinatesAsDouble
    idx = list(tm.nodeIndices)
    pts = [(coords[3 * i], coords[3 * i + 1], coords[3 * i + 2])
           for i in range(len(coords) // 3)]
    tris = [(idx[t], idx[t + 1], idx[t + 2]) for t in range(0, len(idx), 3)]
    return pts, tris


def fitted_plane(faces):
    """One total-least-squares plane through every face given.

    Several faces are fitted together rather than one at a time and averaged: the
    patch is the thing being approximated, and averaging normals would weight a
    sliver face as heavily as the one carrying the surface.
    """
    pts = []
    tris = []
    for f in faces:
        p, t = face_samples(f)
        base = len(pts)
        pts.extend(p)
        tris.extend((a + base, b + base, c + base) for a, b, c in t)
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
                              'conical - use the fitted-plane mode for it')

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
    mode = DIRECTION_MODES[inputs.itemById('dirMode').selectedItem.index][0]

    org_sel = selected(inputs.itemById('selOrigin'))
    if not org_sel:
        raise ac_fit.FitError('select the origin: a point, or a surface to take '
                              'the centre of gravity of')
    origin, origin_what = origin_from_entity(org_sel[0], on_surface)

    plane = None
    if mode == 'fit':
        faces = [f for f in
                 (adsk.fusion.BRepFace.cast(e)
                  for e in selected(inputs.itemById('selPlane'))) if f]
        if not faces:
            # the common case: the origin surface is also the one to be
            # perpendicular to, so do not make it be selected twice
            f0 = adsk.fusion.BRepFace.cast(org_sel[0])
            if f0 is None:
                raise ac_fit.FitError('select the surface to fit a plane to, or '
                                      'pick a surface as the origin')
            faces = [f0]
        plane = fitted_plane(faces)
        direction = plane['normal']
        dir_what = ('the normal of a plane fitted to %d face(s)' % len(faces))
    else:
        ents = selected(inputs.itemById('selAxis'))
        if not ents:
            raise ac_fit.FitError('select the edge, axis or cylinder to run along')
        direction, dir_what = direction_from_entity(ents[0])

    if dia_cm <= 0.0:
        raise ac_fit.FitError('the diameter must be greater than zero')
    if len_cm <= 0.0:
        raise ac_fit.FitError('the length must be greater than zero')

    p0, p1 = cylinder_ends(origin, direction, len_cm, symmetric, flip)
    return {'origin': origin, 'origin_what': origin_what,
            'direction': direction, 'direction_what': dir_what,
            'plane': plane, 'p0': p0, 'p1': p1,
            'radius_cm': 0.5 * dia_cm, 'diameter_mm': dia_cm * CM,
            'length_mm': len_cm * CM, 'symmetric': symmetric, 'flip': flip,
            'entity': org_sel[0]}


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

    so = inputs.addSelectionInput('selOrigin', 'Origin',
                                  'A point, or a surface to take the centre of '
                                  'gravity of')
    for f in ('Vertices', 'SketchPoints', 'ConstructionPoints', 'Faces'):
        so.addSelectionFilter(f)
    so.setSelectionLimits(0, 1)

    dm = inputs.addDropDownCommandInput('dirMode', 'Direction from',
                                        adsk.core.DropDownStyles
                                        .TextListDropDownStyle)
    for i, (_k, lab) in enumerate(DIRECTION_MODES):
        dm.listItems.add(lab, i == int(s.get('dirMode', 0)))

    sp = inputs.addSelectionInput('selPlane', 'Surface to fit',
                                  'Surfaces to fit one plane through; leave empty '
                                  'to use the origin surface')
    sp.addSelectionFilter('Faces')
    sp.setSelectionLimits(0, 0)

    sa = inputs.addSelectionInput('selAxis', 'Edge or axis',
                                  'A straight edge, construction axis, sketch '
                                  'line, or a cylindrical face')
    for f in ('LinearEdges', 'CircularEdges', 'ConstructionLines', 'SketchLines',
              'Faces', 'ConstructionPlanes'):
        sa.addSelectionFilter(f)
    sa.setSelectionLimits(0, 1)

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

    info = inputs.addTextBoxCommandInput('info', '', '', 9, True)
    info.isFullWidth = True
    refresh(inputs)


def refresh(inputs):
    mode = DIRECTION_MODES[inputs.itemById('dirMode').selectedItem.index][0]
    inputs.itemById('selPlane').isVisible = mode == 'fit'
    inputs.itemById('selAxis').isVisible = mode == 'entity'
    org = selected(inputs.itemById('selOrigin'))
    inputs.itemById('onSurface').isVisible = bool(
        org and adsk.fusion.BRepFace.cast(org[0]))
    inputs.itemById('info').formattedText = describe(inputs)


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
        ('origin', '%.3f, %.3f, %.3f mm &mdash; %s'
         % (r['origin'][0] * CM, r['origin'][1] * CM, r['origin'][2] * CM,
            r['origin_what'])),
        ('direction', '%.4f, %.4f, %.4f &mdash; %s'
         % (d[0], d[1], d[2], r['direction_what'])),
        ('cylinder', '&#8960; %.2f &times; %.2f mm, %s'
         % (r['diameter_mm'], r['length_mm'],
            'half each way' if r['symmetric'] else 'all one way')),
    ]
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
        for cast in (adsk.fusion.BRepFace, adsk.fusion.BRepVertex):
            c = cast.cast(ent)
            if c:
                return c.body.parentComponent
        sp = adsk.fusion.SketchPoint.cast(ent)
        if sp:
            return sp.parentSketch.parentComponent
        cp = adsk.fusion.ConstructionPoint.cast(ent)
        if cp:
            return cp.component
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

TOOLTIP = ('Create a solid cylinder from an origin and a direction.\n\n'
           'The origin is a point, or the centre of gravity of a surface. The '
           'direction is read off an edge or axis, or taken as the normal of a '
           'plane fitted to a surface you select - which is what to use when the '
           'surface has no analytic normal to read. The fit reports its own rms '
           'and worst deviation.')

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


def dev_selftest(app, job):
    """End to end inside a live Fusion, against answers known in advance.

    A box and a sphere are built as scratch bodies in the active document and
    deleted again. The box gives a planar face, whose fitted plane must come back
    exact and whose centroid must be the face centre; the sphere gives a curved
    one, which must come back with a large rms - that is the path that warns you
    the normal is not trustworthy, and it is worth proving it fires.
    """
    des = adsk.fusion.Design.cast(app.activeProduct)
    comp = des.rootComponent
    tbm = adsk.fusion.TemporaryBRepManager.get()
    results = []
    made = []

    def note(name, ok, detail=''):
        results.append({'check': name, 'ok': bool(ok), 'detail': str(detail)})

    try:
        # ---- a 4 x 6 x 10 cm box, axis-aligned ---------------------------- #
        centre = adsk.core.Point3D.create(0.0, 0.0, 0.0)
        obb = adsk.core.OrientedBoundingBox3D.create(
            centre, adsk.core.Vector3D.create(1.0, 0.0, 0.0),
            adsk.core.Vector3D.create(0.0, 1.0, 0.0), 4.0, 6.0, 10.0)
        box = _add_body(comp, tbm.createBox(obb), 'ac_selftest_box')
        made.append(box)

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
            made.append(cyl)
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

        # ---- a curved face must report a large rms, not a quiet one ------- #
        sph = _add_body(
            comp, tbm.createSphere(adsk.core.Point3D.create(30.0, 0.0, 0.0), 2.0),
            'ac_selftest_sphere')
        made.append(sph)
        if sph.faces.count:
            fit = fitted_plane([sph.faces.item(0)])
            note('a curved face reports a large deviation rather than a quiet one',
                 fit['max_dev'] * CM > 1.0,
                 'worst %.3f mm over %d points' % (fit['max_dev'] * CM,
                                                   fit['points']))
    finally:
        if not job.get('keep'):
            for b in made:
                try:
                    b.deleteMe()
                except Exception:
                    pass

    return {'passed': sum(1 for r in results if r['ok']),
            'failed': sum(1 for r in results if not r['ok']),
            'checks': results}


DEV_COMMANDS = {
    'ping': dev_ping,
    'fit': dev_fit,
    'make': dev_make,
    'selftest': dev_selftest,
}

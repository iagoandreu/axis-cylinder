"""The plane fit and the cylinder fit, and nothing else.

NO adsk IMPORTS, NO numpy - pure stdlib, so the geometry can be proved right with
`py -3.10 tests/test_fit.py` and no CAD in the loop. That matters here for the
same reason it mattered in Fit Primitives, which this is lifted from: a wrong
normal looks perfectly plausible on screen.

Units are whatever you feed in. ac_core feeds Fusion internal centimetres.

FITTED TO VERTICES, NOT TRIANGLE CENTROIDS
    Centroids of a tessellated surface lie inside it; vertices sit on it. They are
    weighted by a third of the area of each adjacent triangle, the standard lumped
    -area approximation to a surface integral - so a face tessellated finely in one
    corner does not drag the plane towards that corner.

THE CYLINDER FIT IS FIT PRIMITIVES' fp_fit.fit_cylinder, UNCHANGED
    Seeded from the smallest eigenvector of the UNCENTRED normal moment
    sum(w n n^T) - every normal of a cylinder is perpendicular to its axis - then
    refined by Levenberg-Marquardt on the geometric residual (distance to axis - r).
    fp_fit's docstring has the conditioning argument for the uncentred moment; the
    short version is that the centred one returns a random axis on a narrow strip.
"""


import math

__all__ = ['fit_plane', 'fit_cylinder', 'cylinder_residuals', 'jacobi_eigen',
           'vertex_weights', 'vertex_normals', 'FitError',
           'unit', 'cross', 'dot', 'sub', 'add', 'scale', 'length', 'perp_frame']


class FitError(Exception):
    """The patch cannot support the primitive asked of it."""


# --------------------------------------------------------------------------- #
# small vector helpers - tuples, not classes; this runs in a mouse-move loop
# --------------------------------------------------------------------------- #

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def length(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def unit(a):
    n = length(a)
    if n < 1e-300:
        raise FitError('zero-length direction')
    return (a[0] / n, a[1] / n, a[2] / n)


def perp_frame(d):
    """Two unit vectors completing a right-handed frame with d."""
    ref = (0.0, 0.0, 1.0) if abs(d[2]) < 0.9 else (1.0, 0.0, 0.0)
    e1 = unit(cross(d, ref))
    return e1, cross(d, e1)


# --------------------------------------------------------------------------- #
# symmetric 3x3 eigen - cyclic Jacobi
# --------------------------------------------------------------------------- #

def jacobi_eigen(m, sweeps=60):
    """Eigen-decomposition of a symmetric 3x3.

    Returns (values, vectors) sorted ASCENDING by value, vectors[i] being the unit
    eigenvector for values[i]. Ascending because every use here wants the smallest.
    """
    a = [list(row) for row in m]
    v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for _ in range(sweeps):
        off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2])
        if off < 1e-18:
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(a[p][q]) < 1e-300:
                continue
            theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
            t = (1.0 if theta >= 0.0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            for k in range(3):
                akp, akq = a[k][p], a[k][q]
                a[k][p] = c * akp - s * akq
                a[k][q] = s * akp + c * akq
            for k in range(3):
                apk, aqk = a[p][k], a[q][k]
                a[p][k] = c * apk - s * aqk
                a[q][k] = s * apk + c * aqk
            for k in range(3):
                vkp, vkq = v[k][p], v[k][q]
                v[k][p] = c * vkp - s * vkq
                v[k][q] = s * vkp + c * vkq
    pairs = sorted(((a[i][i], (v[0][i], v[1][i], v[2][i])) for i in range(3)),
                   key=lambda pr: pr[0])
    return [p[0] for p in pairs], [p[1] for p in pairs]


def _moment(vectors, weights, centre=None):
    """Weighted 3x3 second-moment matrix, centred on `centre` if given."""
    m = [[0.0] * 3 for _ in range(3)]
    cx, cy, cz = centre if centre else (0.0, 0.0, 0.0)
    for vec, w in zip(vectors, weights):
        x, y, z = vec[0] - cx, vec[1] - cy, vec[2] - cz
        m[0][0] += w * x * x
        m[0][1] += w * x * y
        m[0][2] += w * x * z
        m[1][1] += w * y * y
        m[1][2] += w * y * z
        m[2][2] += w * z * z
    m[1][0] = m[0][1]
    m[2][0] = m[0][2]
    m[2][1] = m[1][2]
    return m


def _centroid(points, weights):
    tw = sum(weights)
    if tw <= 0.0:
        raise FitError('total weight is zero')
    cx = sum(p[0] * w for p, w in zip(points, weights)) / tw
    cy = sum(p[1] * w for p, w in zip(points, weights)) / tw
    cz = sum(p[2] * w for p, w in zip(points, weights)) / tw
    return (cx, cy, cz)


def vertex_weights(vertices, triangles):
    """Lumped area weight per vertex: a third of each adjacent triangle's area.

    vertices  : list of (x, y, z)
    triangles : list of (i, j, k) indices into vertices
    """
    w = [0.0] * len(vertices)
    for i, j, k in triangles:
        a = 0.5 * length(cross(sub(vertices[j], vertices[i]),
                               sub(vertices[k], vertices[i])))
        third = a / 3.0
        w[i] += third
        w[j] += third
        w[k] += third
    # A vertex with no area still carries information about where the surface is.
    total = sum(w)
    if total <= 0.0:
        return [1.0] * len(vertices)
    floor = total / (len(w) * 1e6)
    return [x if x > floor else floor for x in w]


def vertex_normals(vertices, triangles):
    """Area-weighted unit normal per vertex; (0, 0, 0) where there is no area.

    The cylinder seed only needs each normal's line, not its sign, so a mesh
    whose winding flips between faces does no harm as long as it is consistent
    within one - which is what a per-face tessellation gives.
    """
    acc = [(0.0, 0.0, 0.0)] * len(vertices)
    for i, j, k in triangles:
        n = cross(sub(vertices[j], vertices[i]), sub(vertices[k], vertices[i]))
        acc[i] = add(acc[i], n)
        acc[j] = add(acc[j], n)
        acc[k] = add(acc[k], n)
    out = []
    for n in acc:
        ln = length(n)
        out.append(scale(n, 1.0 / ln) if ln > 1e-300 else (0.0, 0.0, 0.0))
    return out


# --------------------------------------------------------------------------- #
# plane
# --------------------------------------------------------------------------- #

def fit_plane(points, weights=None, hint_normal=None):
    """Total-least-squares plane. Minimises the perpendicular distance, not the
    vertical one, which is the only sensible choice when the patch is steep.

    Returns origin, normal, u/v in-plane axes, the half extents along them, and the
    residuals. `hint_normal` only flips the sign so the normal points sensibly
    outward; it never moves the plane.
    """
    n = len(points)
    if n < 3:
        raise FitError('a plane needs at least 3 points, got %d' % n)
    w = list(weights) if weights else [1.0] * n
    origin = _centroid(points, w)
    vals, vecs = jacobi_eigen(_moment(points, w, origin))
    normal = vecs[0]                       # smallest spread = across the plane
    # Collinear means the second spread vanishes RELATIVE to the first. An absolute
    # threshold cannot work: these are squared model units, so it would reject a
    # small plane and accept a big line.
    if vals[2] <= 0.0 or vals[1] <= vals[2] * 1e-10:
        raise FitError('the painted points are collinear - no plane is defined')
    if hint_normal and dot(normal, hint_normal) < 0.0:
        normal = scale(normal, -1.0)
    u, v = vecs[2], vecs[1]                # widest in-plane direction first
    if dot(cross(u, v), normal) < 0.0:
        v = scale(v, -1.0)

    devs = [dot(sub(p, origin), normal) for p in points]
    tw = sum(w)
    rms = math.sqrt(sum(wi * d * d for wi, d in zip(w, devs)) / tw)
    us = [dot(sub(p, origin), u) for p in points]
    vs = [dot(sub(p, origin), v) for p in points]
    return {
        'kind': 'plane',
        'origin': origin, 'normal': normal, 'u': u, 'v': v,
        'u_min': min(us), 'u_max': max(us), 'v_min': min(vs), 'v_max': max(vs),
        'rms': rms, 'max_dev': max(abs(d) for d in devs),
        'signed_min': min(devs), 'signed_max': max(devs), 'points': n,
    }


# --------------------------------------------------------------------------- #
# cylinder
# --------------------------------------------------------------------------- #

def fit_circle_2d(pts):
    """Algebraic (Kasa) circle fit - the seed for the real one. Returns (cx, cy, r)."""
    n = len(pts)
    if n < 3:
        raise FitError('a circle needs at least 3 points')
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for px, py in pts:
        u, v = px - mx, py - my
        suu += u * u
        suv += u * v
        svv += v * v
        suuu += u * u * u
        svvv += v * v * v
        suvv += u * v * v
        svuu += v * u * u
    det = suu * svv - suv * suv
    if abs(det) < 1e-300:
        raise FitError('the painted points are collinear - no circle is defined')
    b1 = 0.5 * (suuu + suvv)
    b2 = 0.5 * (svvv + svuu)
    uc = (b1 * svv - b2 * suv) / det
    vc = (b2 * suu - b1 * suv) / det
    r = math.sqrt(max(0.0, uc * uc + vc * vc + (suu + svv) / n))
    return (uc + mx, vc + my, r)


def _solve(a, b):
    """Gaussian elimination with partial pivoting. a is n x n (mutated), b length n."""
    n = len(b)
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-300:
            return None
        m[col], m[piv] = m[piv], m[col]
        inv = 1.0 / m[col][col]
        for r in range(col + 1, n):
            f = m[r][col] * inv
            if f:
                for c in range(col, n + 1):
                    m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        s = m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))
        x[r] = s / m[r][r]
    return x


def _cyl_residuals(points, weights, c, d, r):
    """Weighted RMS and max of (distance from axis - r)."""
    worst = 0.0
    acc = 0.0
    tw = 0.0
    for p, w in zip(points, weights):
        v = sub(p, c)
        t = dot(v, d)
        rho = length(sub(v, scale(d, t)))
        e = rho - r
        acc += w * e * e
        tw += w
        if abs(e) > worst:
            worst = abs(e)
    return math.sqrt(acc / tw), worst


def fit_cylinder(points, normals=None, weights=None, max_iter=80, tol=1e-12):
    """Geometric least-squares cylinder: minimises sum w*(dist_to_axis - r)^2.

    Seeded from the normals (see the module docstring), then refined by
    Levenberg-Marquardt over five parameters - two for the axis direction, two for
    the axis position across it, one for the radius. Position along the axis is not
    a parameter; it is not observable and leaving it in makes the system singular.

    `normals` may be omitted, in which case the seed falls back to the longest
    principal direction of the points. That is a much weaker seed and only works on
    a patch that is long compared with its radius.
    """
    n = len(points)
    if n < 5:
        raise FitError('a cylinder needs at least 5 points, got %d' % n)
    w = list(weights) if weights else [1.0] * n

    if normals:
        # M d = 0 exactly for a cylinder: every normal is perpendicular to the axis.
        _, vecs = jacobi_eigen(_moment(normals, w))
        d = unit(vecs[0])
    else:
        centre = _centroid(points, w)
        _, vecs = jacobi_eigen(_moment(points, w, centre))
        d = unit(vecs[2])

    centre = _centroid(points, w)
    e1, e2 = perp_frame(d)
    cx, cy, r = fit_circle_2d([(dot(sub(p, centre), e1), dot(sub(p, centre), e2))
                               for p in points])
    c = add(centre, add(scale(e1, cx), scale(e2, cy)))
    if not (r > 0.0) or not (r < float('inf')):
        raise FitError('the seed radius is not finite - is the patch flat?')

    lam = 1e-6
    rms, _ = _cyl_residuals(points, w, c, d, r)
    for _ in range(max_iter):
        e1, e2 = perp_frame(d)
        jtj = [[0.0] * 5 for _ in range(5)]
        jtf = [0.0] * 5
        for p, wi in zip(points, w):
            v = sub(p, c)
            t = dot(v, d)
            perp = sub(v, scale(d, t))
            rho = length(perp)
            if rho < 1e-300:
                continue
            nr = scale(perp, 1.0 / rho)          # unit radial, perpendicular to d
            f = rho - r
            # d(rho)/d(axis rotation) = -t * (nr . e_i)  because nr . d == 0
            j = (-dot(nr, e1), -dot(nr, e2),
                 -t * dot(nr, e1), -t * dot(nr, e2),
                 -1.0)
            for a_ in range(5):
                jtf[a_] += wi * j[a_] * f
                for b_ in range(a_, 5):
                    jtj[a_][b_] += wi * j[a_] * j[b_]
        for a_ in range(5):
            for b_ in range(a_):
                jtj[a_][b_] = jtj[b_][a_]

        improved = False
        for _try in range(12):
            damped = [list(row) for row in jtj]
            for k in range(5):
                damped[k][k] *= (1.0 + lam)
                damped[k][k] += lam * 1e-12
            step = _solve(damped, [-x for x in jtf])
            if step is None:
                lam *= 10.0
                continue
            c_try = add(c, add(scale(e1, step[0]), scale(e2, step[1])))
            try:
                d_try = unit(add(d, add(scale(e1, step[2]), scale(e2, step[3]))))
            except FitError:
                lam *= 10.0
                continue
            r_try = r + step[4]
            if r_try <= 0.0:
                lam *= 10.0
                continue
            rms_try, _ = _cyl_residuals(points, w, c_try, d_try, r_try)
            if rms_try <= rms:
                delta = rms - rms_try
                c, d, r, rms = c_try, d_try, r_try, rms_try
                lam = max(lam * 0.3, 1e-12)
                improved = delta > tol * max(1.0, rms)
                break
            lam *= 10.0
        if not improved:
            break

    # Canonical output: axis point closest to the patch, and the extent along it.
    centre = _centroid(points, w)
    ts = [dot(sub(p, c), d) for p in points]
    c = add(c, scale(d, dot(sub(centre, c), d)))
    ts = [dot(sub(p, c), d) for p in points]
    rms, worst = _cyl_residuals(points, w, c, d, r)
    return {
        'kind': 'cylinder',
        'axis_point': c, 'axis': d, 'radius': r,
        't_min': min(ts), 't_max': max(ts), 'length': max(ts) - min(ts),
        'start': add(c, scale(d, min(ts))), 'end': add(c, scale(d, max(ts))),
        'rms': rms, 'max_dev': worst, 'points': n,
        'sweep_deg': _sweep_degrees(points, c, d, r),
    }


def cylinder_residuals(points, weights, axis_point, axis, radius):
    """How far the points sit from a cylinder you name, rather than the fitted one.

    Used to answer "if I round this bore to 12.00 mm, how much does the real surface
    depart from it" - the question that actually matters when a fitted number is
    about to be replaced by a nominal one.
    """
    w = list(weights) if weights else [1.0] * len(points)
    return _cyl_residuals(points, w, axis_point, unit(axis), radius)


def _sweep_degrees(points, c, d, r):
    """How much of the circumference the patch actually covers.

    A fit over a 20 degree sliver is numerically fine and physically meaningless -
    the radius is then almost unconstrained. The caller warns on this rather than
    pretending the number is trustworthy.
    """
    e1, e2 = perp_frame(d)
    angs = []
    for p in points:
        v = sub(p, c)
        a = math.atan2(dot(v, e2), dot(v, e1))
        angs.append(a)
    angs.sort()
    if len(angs) < 2:
        return 0.0
    gaps = [angs[i + 1] - angs[i] for i in range(len(angs) - 1)]
    gaps.append(angs[0] + 2.0 * math.pi - angs[-1])
    return math.degrees(2.0 * math.pi - max(gaps))

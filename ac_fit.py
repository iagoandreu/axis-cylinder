"""The plane fit, and nothing else.

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
"""


import math

__all__ = ['fit_plane', 'jacobi_eigen', 'vertex_weights', 'FitError',
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

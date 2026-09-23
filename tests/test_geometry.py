"""Offline tests: the plane fit and the cylinder placement, with no Fusion.

    py -3.10 tests/test_geometry.py

adsk is stubbed (tests/stub), so ac_core imports outside Fusion. Everything here
has an answer worked out in advance rather than recorded from a previous run.

What is NOT covered here is reading an origin or a direction off a real selection:
those are Fusion object types and the stub answers yes to every cast, so testing
them against it would prove nothing. They are covered by tools/livetest.py, which
runs inside a live Fusion through the dev bridge.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'stub'))
sys.path.insert(0, os.path.dirname(HERE))

import ac_fit                                               # noqa: E402
import ac_core as C                                         # noqa: E402

FAILED = []


def check(name, cond, detail=''):
    print('  %s %s%s' % ('ok  ' if cond else 'FAIL', name,
                         '' if cond else '   %s' % (detail,)))
    if not cond:
        FAILED.append(name)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def vclose(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


# --------------------------------------------------------------------------- #
print('the plane fit')
# --------------------------------------------------------------------------- #

# a plane tilted so no axis is special, sampled on a grid
N = (1.0 / math.sqrt(14.0), 2.0 / math.sqrt(14.0), 3.0 / math.sqrt(14.0))
ORG = (0.3, -0.7, 1.1)
u = ac_fit.unit(ac_fit.cross(N, (0.0, 0.0, 1.0)))
v = ac_fit.cross(N, u)
pts = [ac_fit.add(ORG, ac_fit.add(ac_fit.scale(u, 0.4 * i - 1.0),
                                  ac_fit.scale(v, 0.3 * j - 0.9)))
       for i in range(6) for j in range(7)]

r = ac_fit.fit_plane(pts)
check('an exact plane is recovered to machine precision',
      close(abs(ac_fit.dot(r['normal'], N)), 1.0, 1e-12),
      abs(ac_fit.dot(r['normal'], N)))
check('and its rms is zero', r['rms'] < 1e-12, r['rms'])
check('the origin lands on the plane',
      abs(ac_fit.dot(ac_fit.sub(r['origin'], ORG), N)) < 1e-12)

# the hint only flips the sign; it never moves the plane
r2 = ac_fit.fit_plane(pts, hint_normal=ac_fit.scale(N, -1.0))
check('a hint flips the normal without moving the plane',
      close(ac_fit.dot(r2['normal'], N), -1.0, 1e-12)
      and vclose(r2['origin'], r['origin'], 1e-12))

# noise perpendicular to the plane must show up in rms, and only there
noisy = [ac_fit.add(p, ac_fit.scale(N, 0.01 * (1 if k % 2 else -1)))
         for k, p in enumerate(pts)]
rn = ac_fit.fit_plane(noisy)
# just under 0.01, never over: a least-squares plane absorbs whatever part of the
# noise it can represent as a tilt, and an alternating sign on a 6x7 grid leaves a
# little of that. Asserting exact equality would be asserting that the fit does
# not fit.
check('perpendicular noise is reported as rms, not swallowed',
      0.0095 < rn['rms'] <= 0.01, rn['rms'])
check('and the normal survives it',
      abs(abs(ac_fit.dot(rn['normal'], N)) - 1.0) < 1e-6)

# collinear points define no plane and must say so rather than return one
try:
    ac_fit.fit_plane([(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3)])
    check('collinear points are refused', False, 'no error raised')
except ac_fit.FitError:
    check('collinear points are refused', True)

# area weighting: a corner tessellated finely must not drag the plane
sq = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)]
tris = [(0, 1, 2), (0, 2, 3)]
w = ac_fit.vertex_weights(sq, tris)
check('lumped-area weights sum to the total area', close(sum(w), 1.0, 1e-12),
      sum(w))
check('the two diagonal corners carry twice the weight of the others',
      close(w[0], 2.0 * w[1], 1e-12) and close(w[2], 2.0 * w[1], 1e-12),
      w)

# --------------------------------------------------------------------------- #
print('cylinder placement')
# --------------------------------------------------------------------------- #

O = (1.0, 2.0, 3.0)
D = (0.0, 0.0, 2.0)                      # deliberately not unit length

p0, p1 = C.cylinder_ends(O, D, 5.0, symmetric=False, flip=False)
check('one-way starts at the origin', vclose(p0, O))
check('and ends one length along the direction', vclose(p1, (1.0, 2.0, 8.0)))
check('a non-unit direction does not change the length',
      close(ac_fit.length(ac_fit.sub(p1, p0)), 5.0, 1e-12))

p0, p1 = C.cylinder_ends(O, D, 5.0, symmetric=True, flip=False)
check('symmetric puts half each way', vclose(p0, (1.0, 2.0, 0.5))
      and vclose(p1, (1.0, 2.0, 5.5)))
check('and the origin is the midpoint',
      vclose(ac_fit.scale(ac_fit.add(p0, p1), 0.5), O))

p0, p1 = C.cylinder_ends(O, D, 5.0, symmetric=False, flip=True)
check('flip reverses it about the origin', vclose(p0, O)
      and vclose(p1, (1.0, 2.0, -2.0)))

p0f, p1f = C.cylinder_ends(O, D, 5.0, symmetric=True, flip=True)
p0s, p1s = C.cylinder_ends(O, D, 5.0, symmetric=True, flip=False)
check('flipping a symmetric cylinder only swaps its ends',
      vclose(p0f, p1s) and vclose(p1f, p0s))

# the length is along the axis whatever the axis is
DD = ac_fit.unit((1.0, -2.0, 0.5))
p0, p1 = C.cylinder_ends(O, DD, 7.25, symmetric=True, flip=False)
check('length holds on a skew axis',
      close(ac_fit.length(ac_fit.sub(p1, p0)), 7.25, 1e-12))
check('and the axis direction is preserved',
      close(ac_fit.dot(ac_fit.unit(ac_fit.sub(p1, p0)), DD), 1.0, 1e-12))

# --------------------------------------------------------------------------- #
print('units and naming')
# --------------------------------------------------------------------------- #

check('CM converts internal centimetres to millimetres', close(C.CM, 10.0))
check('MM converts millimetres to internal centimetres', close(C.MM, 0.1))
check('a round trip through both is the identity',
      close(3.7 * C.MM * C.CM, 3.7, 1e-12))
check('the body name carries the size in millimetres',
      C.body_name({'diameter_mm': 4.0, 'length_mm': 20.0})
      == 'Cylinder 4.00 x 20.00 mm',
      C.body_name({'diameter_mm': 4.0, 'length_mm': 20.0}))

check('both direction modes are declared',
      [k for k, _l in C.DIRECTION_MODES] == ['fit', 'entity'])

print()
if FAILED:
    print('%d FAILED: %s' % (len(FAILED), ', '.join(FAILED)))
    raise SystemExit(1)
print('all tests passed')

"""Generate the toolbar icon set.

A cylinder standing on a surface with its axis drawn through it - the axis in the
same blue Fit Primitives uses for a fitted axis, so the two add-ins read as
related. Rendered at 8x and downsampled so it survives 16 px.

    py -3.10 tools/make_icons.py
"""
import os

from PIL import Image, ImageDraw

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'resources', 'AxisCylinder')

BODY_FILL = (206, 213, 221, 255)
BODY_EDGE = (92, 104, 118, 255)
SURFACE = (168, 178, 190, 255)
AXIS = (44, 122, 168, 255)
ORIGIN = (255, 138, 0, 255)


def _ellipse(d, cx, cy, rx, ry, fill, outline, w):
    d.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=fill, outline=outline,
              width=w)


def icon(size):
    S = size * 8
    img = Image.new('RGBA', (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    w = max(1, int(S * 0.024))

    cx = S * 0.5
    rx, ry = S * 0.22, S * 0.085
    top, bot = S * 0.26, S * 0.70

    # the surface the cylinder stands on, seen edge-on and slightly curved
    d.arc([S * 0.04, S * 0.52, S * 0.96, S * 1.06], 200, 340,
          fill=SURFACE, width=int(w * 1.6))

    # the cylinder
    d.polygon([(cx - rx, top), (cx + rx, top), (cx + rx, bot), (cx - rx, bot)],
              fill=BODY_FILL)
    _ellipse(d, cx, bot, rx, ry, BODY_FILL, BODY_EDGE, w)
    d.line([(cx - rx, top), (cx - rx, bot)], fill=BODY_EDGE, width=w)
    d.line([(cx + rx, top), (cx + rx, bot)], fill=BODY_EDGE, width=w)
    _ellipse(d, cx, top, rx, ry, BODY_FILL, BODY_EDGE, w)

    # the axis, running past both ends
    d.line([(cx, S * 0.10), (cx, S * 0.90)], fill=AXIS, width=w)

    # the origin, on the surface
    r = S * 0.045
    d.ellipse([cx - r, bot - r, cx + r, bot + r], fill=ORIGIN)

    return img.resize((size, size), Image.LANCZOS)


def main():
    os.makedirs(OUT, exist_ok=True)
    for s in (16, 32, 64):
        icon(s).save(os.path.join(OUT, '%dx%d.png' % (s, s)))
    print('wrote 16/32/64 px icons to %s' % OUT)


if __name__ == '__main__':
    main()

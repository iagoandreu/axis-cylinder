"""Run the add-in's self-test inside a live Fusion and print what it found.

    py -3.10 tools/livetest.py          # in a new document, closed unsaved afterwards
    py -3.10 tools/livetest.py keep     # leave that document open, to look at

Needs the dev bridge on: a file called devbridge.on next to the installed add-in,
then Stop and Run it once. It opens a NEW document, runs there and closes it
unsaved (keep leaves it open); the active document is never touched.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ac as client                                          # noqa: E402


def main():
    keep = len(sys.argv) > 1 and sys.argv[1] == 'keep'
    if not client.alive():
        print('The dev bridge is not answering.')
        print('  - is Fusion running with Axis Cylinder started?')
        print('  - is there a devbridge.on file next to the installed add-in?')
        print('    %s' % os.path.join(client.ADDIN, 'devbridge.on'))
        return 2

    ping = client.send('ping')
    if not ping.get('ok'):
        print('ping failed: %s' % ping.get('error'))
        return 2
    r = ping['result']
    print('Fusion %s, document "%s", %s design'
          % (r['version'], r['document'], r['design_type']))
    print()

    res = client.send('selftest', {'keep': keep})
    if not res.get('ok'):
        print('selftest failed:\n%s' % res.get('error'))
        return 1
    out = res['result']
    for c in out['checks']:
        print('  %s %s%s' % ('ok  ' if c['ok'] else 'FAIL', c['check'],
                             '   %s' % c['detail'] if c['detail'] else ''))
    print()
    print('%d passed, %d failed' % (out['passed'], out['failed']))
    return 1 if out['failed'] else 0


if __name__ == '__main__':
    raise SystemExit(main())

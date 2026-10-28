"""Regenerates the four curve legs' heading pieces (Zenith draft and AutoPoses) from the starter's
pieces, so Pedro follows them without a jump: Pedro reads piece ends as a fraction of the path's
length, and hands a tangent piece its own 0..1 t (so it would read the wrong part of the curve).
Each tangent span becomes linear pieces between the true tangent at path-length steps; each linear
piece starts and ends on its neighbours' headings. Also: drops backOffFlower, adds the park."""
import json, math, re

def bez(P, t):
    pts = P[:]
    while len(pts) > 1:
        pts = [((1 - t) * a[0] + t * b[0], (1 - t) * a[1] + t * b[1]) for a, b in zip(pts, pts[1:])]
    return pts[0]

def tangent_t(P, t):
    n = len(P) - 1
    Q = [(n * (b[0] - a[0]), n * (b[1] - a[1])) for a, b in zip(P, P[1:])]
    d = bez(Q, t)
    return math.atan2(d[1], d[0])

def t_of(P, c, N=4000):
    pts = [bez(P, i / N) for i in range(N + 1)]
    L = [0.0]
    for a, b in zip(pts, pts[1:]): L.append(L[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    target = c * L[-1]
    for i in range(N):
        if L[i + 1] >= target:
            f = (target - L[i]) / (L[i + 1] - L[i]) if L[i + 1] > L[i] else 0
            return (i + f) / N
    return 1.0

def near(a, ref):
    """a, unwrapped to within pi of ref."""
    while a - ref > math.pi: a -= 2 * math.pi
    while a - ref < -math.pi: a += 2 * math.pi
    return a

STEP = 0.1
def build(P, start_h, spec):
    """spec: list of (endC, mode, extra). Returns [(startC, endC, mode, from, to)]."""
    out = []
    cur_c, cur_h = 0.0, start_h
    for k, (end_c, mode, extra) in enumerate(spec):
        if mode in ('tangent', 'tangentReversed'):
            off = math.pi if mode == 'tangentReversed' else 0
            cs = [cur_c]
            while cs[-1] + STEP < end_c - 0.02: cs.append(round(cs[-1] + STEP, 4))
            cs.append(end_c)
            for a, b in zip(cs, cs[1:]):
                hb = near(tangent_t(P, t_of(P, b)) + off, cur_h)
                out.append((a, b, 'linear', cur_h, hb))
                cur_h = hb
        elif mode == 'linear':
            # ends on the next piece's start: a tangent there, a constant, or its own end if last
            nxt = spec[k + 1] if k + 1 < len(spec) else None
            if extra is not None: hb = extra
            elif nxt[1] == 'constant': hb = nxt[2]
            else:
                off = math.pi if nxt[1] == 'tangentReversed' else 0
                hb = tangent_t(P, t_of(P, end_c)) + off
            if extra is None: hb = near(hb, cur_h)
            out.append((cur_c, end_c, 'linear', cur_h, hb))
            cur_h = hb
        else:  # constant
            out.append((cur_c, end_c, 'constant', extra, None))
            cur_h = extra
        cur_c = end_c
    return out

R = lambda x: round(x, 4)
PI = math.pi
# Each leg's heading plan: the waypoint it starts from (the step before it ends there) and its
# pieces, as (end, mode, heading). The curve itself is read from the draft below, so a tuned control
# point or waypoint gets headings for the route as it now is.
PLAN = {
    'toOtherSide': ('scoreSouth',
                    [(0.2038, 'linear', None), (0.8679, 'tangent', None), (1.0, 'linear', PI / 2)]),
    'toScoreNorth': ('flowerNorthPickup',
                     # The starter backs off with tangentReversed; from our pickup, 6.4 in north of
                     # its first control point, that turns at once and swings a corner into the
                     # flower's foot. So: straight back facing the flower for the starter's first
                     # piece, then its turn on through west to facing south.
                     [(0.2041, 'constant', PI / 2), (0.3742, 'linear', PI), (0.5442, 'linear', 3 * PI / 2),
                      (1.0, 'constant', -PI / 2)]),
    'toGarden': ('scoreNorth', [(0.2186, 'linear', None), (1.0, 'tangent', None)]),
    'driveBack': ('offGardenLane',
                  [(0.1612, 'linear', None), (0.707, 'tangentReversed', None), (0.8242, 'linear', None),
                   (1.0, 'constant', -PI / 2)]),
}

def all_steps(steps):
    for st in steps:
        yield st
        yield from all_steps(st.get('steps', []))

waypoints = json.load(open('autos/waypoints.json', encoding='utf-8'))['waypoints']
draft = {st['id']: st for st in all_steps(json.load(open('autos/two-tip.auto.json', encoding='utf-8'))['steps'])}
legs = {}
for name, (start, spec) in PLAN.items():
    seg = draft[name]['segments'][0]
    assert seg['kind'] == 'bezier' and seg['from'] == 'current' and len(draft[name]['segments']) == 1, name
    w0 = waypoints[start]
    pts = [(w0['xIn'], w0['yIn'])] + [(c['xIn'], c['yIn']) for c in seg['control']]
    to = seg['to']
    end = waypoints[to['ref']] if 'ref' in to else to
    pts.append((end['xIn'], end['yIn']))
    legs[name] = (pts, w0['headingRad'], spec)
pieces = {}
for name, (P, h0, spec) in legs.items():
    pieces[name] = build(P, h0, spec)
for n, ps in pieces.items():
    for p in ps:
        if p[2] == 'linear': assert abs(p[4] - p[3]) < PI - 0.05, (n, p)

# ---- the draft ----
dp = 'autos/two-tip.auto.json'
a = json.load(open(dp, encoding='utf-8'))
NOTE = (" Pedro reads a piece's end as a fraction of the path's length and hands a tangent piece its own"
        " 0..1 t, so the starter's tangent spans are written as linear pieces between the curve's true"
        " direction every tenth of the path (tools/zenith_headings.py); each piece starts on the heading the last"
        " one ended on, so the robot never spins on the spot.")
def ranges(ps):
    out = []
    for s, e, mode, f, t in ps:
        h = {'mode': mode}
        if mode == 'linear': h.update(fromRad=R(f), toRad=R(t))
        else: h['headingRad'] = R(f)
        out.append({'startT': R(s), 'endT': R(e), 'heading': h})
    return out
def fix(steps):
    keep = []
    for s in steps:
        if s['id'] == 'backOffFlower': continue
        if s['id'] in pieces:
            s['heading'] = {'mode': 'piecewise', 'ranges': ranges(pieces[s['id']])}
            if NOTE not in s.get('notes', ''): s['notes'] = s.get('notes', '') + NOTE
        if s['id'] == 'toScoreNorth':
            s['notes'] = ("The starter's leg: backs straight off flower1 still facing it for the first fifth"
                          " (turning at once swings a corner into the flower's foot), then turns round through"
                          " west and comes in to the scoring spot north of the hive facing south, the launcher"
                          " toward the up cell." + NOTE)
        if 'steps' in s: s['steps'] = fix(s['steps'])
        keep.append(s)
    return keep
a['steps'] = fix(a['steps'])
if not any(s['id'] == 'park' for s in a['steps']):
    a['steps'].append({
        'id': 'park', 'kind': 'path',
        'segments': [{'kind': 'line', 'from': 'current', 'to': {'ref': 'park'}}],
        'heading': {'mode': 'linear', 'fromRad': -1.5708, 'toRad': -3.1416},
        'notes': "Parks after the last volley: west to the park spot, partly in RED's loading zone, turning"
                 " from facing the hive to facing west (clockwise, a quarter turn) on the way."})
open(dp, 'w', encoding='utf-8', newline='\n').write(json.dumps(a, indent=2, ensure_ascii=False) + '\n')

wp = 'autos/waypoints.json'
w = json.load(open(wp, encoding='utf-8'))
w['waypoints'].setdefault('park', {
    'xIn': -56, 'yIn': 36, 'headingRad': 3.1416,
    'provenance': "SET BY HAND: planning choice (the earlier 2-tip draft's park), RED side (BLUE is the point"
                  " mirror); partly inside RED's loading zone (x -72..-61, y 24..48), an open spot to finish in,"
                  " clear of flower1 and the hive frame. NEEDS MEASUREMENT on the real field."})
open(wp, 'w', encoding='utf-8', newline='\n').write(json.dumps(w, indent=2, ensure_ascii=False) + '\n')

# ---- AutoPoses ----
jp = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AutoPoses.java'
s = open(jp, encoding='utf-8').read()
const = {'toOtherSide': 'TO_OTHER_SIDE_HEADING', 'toScoreNorth': 'TO_SCORE_NORTH_HEADING',
         'toGarden': 'TO_GARDEN_HEADING', 'driveBack': 'DRIVE_BACK_HEADING'}
for n, c in const.items():
    items = []
    for st, e, mode, f, t in pieces[n]:
        items.append('HeadingPiece.linear(%s, %s, %s)' % (R(e), R(f), R(t)) if mode == 'linear'
                     else 'HeadingPiece.constant(%s, %s)' % (R(e), R(f)))
    lines, line = [], '            '
    for i, it in enumerate(items):
        piece = it + (',' if i < len(items) - 1 else '};')
        if len(line) + len(piece) + 1 > 110:
            lines.append(line.rstrip()); line = '            '
        line += piece + ' '
    lines.append(line.rstrip())
    body = '\n'.join(lines)
    s, k = re.subn(r'    public static final HeadingPiece\[\] %s = \{\n.*?\};' % c,
                   lambda m: '    public static final HeadingPiece[] %s = {\n%s' % (c, body), s, flags=re.S)
    assert k == 1, c
if 'Pedro reads' not in s:
    s = s.replace('    // ---- headings along the curves, piece by piece (radians, RED), as in the draft ----',
                  '    // ---- headings along the curves, piece by piece (radians, RED), as in the draft ----\n'
                  '    // Written by tools/zenith_headings.py: Pedro reads a piece\'s end as a fraction of the\n'
                  '    // path\'s length and hands a tangent piece its own 0..1 t, so the starter\'s tangent spans are\n'
                  '    // linear pieces between the curve\'s true direction, each starting where the last ended.\n'
                  '    // AutoPosesTest checks there is no jump.')
if 'PARK' not in s:
    s = s.replace('    public static final Pose OFF_GARDEN_LANE = new Pose(-50.0, -50.0, Math.PI);',
                  '    public static final Pose OFF_GARDEN_LANE = new Pose(-50.0, -50.0, Math.PI);\n'
                  '    public static final Pose PARK = new Pose(-56.0, 36.0, Math.PI);')
open(jp, 'w', encoding='utf-8', newline='').write(s)
for n, ps in pieces.items(): print(n, len(ps), 'pieces')

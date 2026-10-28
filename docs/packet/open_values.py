"""Lists every value still labelled PLACEHOLDER / NEEDS MEASUREMENT / SET FROM SIM / CALIBRATED FROM SIM /
APPROX in the robot code, robot.json and waypoints.json. Run from the repo root."""
import os, re, json
root = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
pat = re.compile(r'PLACEHOLDER|NEEDS MEASUREMENT|SET FROM SIM|CALIBRATED FROM SIM|APPROX')
rows = []
for dp, _, fs in os.walk(root):
    if 'generated' in dp or 'procedures' in dp:
        continue
    for f in fs:
        if not f.endswith('.java') or f == 'TwoTipGenerated.java':
            continue
        p = os.path.join(dp, f).replace(os.sep, '/')
        above = ''  # the comment block right above this line, where a label is often written
        for i, l in enumerate(open(p, encoding='utf-8'), 1):
            t = l.strip()
            if t.startswith(('//', '/*', '*')):
                above += ' ' + t
                continue
            # A value is set by an assignment, a .set(...), or a continuation line of a longer call
            # (pedro/Constants' .put(...) gains): any code line that carries a number.
            assigns = ('=' in t and not t.startswith(('if', 'for', 'while', 'return'))) or '.set(' in t                 or bool(re.search(r'\d', t.split('//')[0]))
            if assigns and (pat.search(t) or pat.search(above)):
                label = '' if pat.search(t) else '   [' + pat.search(above).group(0) + ', in the comment above]'
                rows.append((p.replace(root + '/', ''), i, t[:160] + label))
            above = ''
for r in rows:
    print('%s:%d  %s' % r)
print(len(rows), 'java lines')
for fn in ('autos/robot.json', 'autos/waypoints.json'):
    d = json.load(open(fn, encoding='utf-8'))

    def walk(o, path):
        if isinstance(o, dict):
            pv = o.get('provenance', '')
            if isinstance(pv, str) and pat.search(pv):
                val = {k: v for k, v in o.items() if k != 'provenance' and not isinstance(v, (dict, list))}
                print(fn, path, val, '|', pv[:90])
            for k, v in o.items():
                if k != 'provenance':
                    walk(v, path + '.' + k)
        elif isinstance(o, list):
            for i, v in enumerate(o):
                walk(v, path + '[%d]' % i)
    walk(d, '')

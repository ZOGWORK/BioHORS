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
        for i, l in enumerate(open(p, encoding='utf-8'), 1):
            if pat.search(l) and ('=' in l or '.set(' in l):
                rows.append((p.replace(root + '/', ''), i, l.strip()[:160]))
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

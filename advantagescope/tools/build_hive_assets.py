"""Builds AdvantageScope assets that show the hives tipping, from AdvantageScope's own 2026-2027
field model (downloaded by AdvantageScope into its autoAssets folder; not ours to commit, so the
outputs go straight into its userAssets folder):

    Field3d_BIOBUZZTippingHive/  "2026-2027 Field (tipping hive)": the field with both hives removed
    Robot_HiveRed/, Robot_HiveBlue/  each hive alone, as a robot model posed about its own axle

A sim logs Field/HiveModel (BiobuzzMatch): a Pose3d at the hive's CAD axle, turned by the hive's
angle about the axle (our +X, AdvantageScope's +Y). Draw it as a Robot with the "Hive (Red)" or
"Hive (Blue)" model on the tipping-hive field.

Usage (Python with numpy):  python build_hive_assets.py
Re-run after AdvantageScope updates its field model. Needs AdvantageScope closed or restarted after.

Frames. The field GLB is y-up, metres; turned +90 deg about x it is our field frame (x toward BLUE,
y away from the audience, z up), which is what StagedPollen and AdvantageScopeFrameTest check.
AdvantageScope's logged frame ("Center/Rotated") is ours turned +90 deg: (x, y, z) -> (-y, x, z).
So a GLB point (gx, gy, gz) is (gz, gx, gy) in the logged frame. Each hive model is that, minus the
axle point, turned back by the hive's kickoff angle, so the logged pose (axle, kickoff angle) draws
it exactly where the field CAD has it.
"""
import json
import math
import os
import shutil
import struct

import numpy as np

SRC = os.path.join(os.environ['APPDATA'], 'AdvantageScope', 'autoAssets', 'Field3d_20262027FTCFieldV1')
DST = os.path.join(os.environ['APPDATA'], 'AdvantageScope', 'userAssets')
IN_TO_M = 0.0254
# MEASURED FROM CAD (this field model): the hive axle's two F608ZZ bearings are centred on y = 0,
# z = 43.95 in; the pivot x is the field file's +/-12.75 in (SPEC).
AXLE_Z_IN = 43.95
PIVOT_X_IN = 12.75
TILT_RAD = math.radians(30.0)  # SPEC (manual: the bar rests at 30 deg), BiobuzzHive.TILT_RAD
HIVES = {'Red': 'am-5853-Red Hive', 'Blue': 'am-5853-Blue Hive'}


def read_glb(path):
    with open(path, 'rb') as f:
        data = f.read()
    js_len = struct.unpack_from('<I', data, 12)[0]
    gltf = json.loads(data[20:20 + js_len])
    bin_len = struct.unpack_from('<I', data, 20 + js_len)[0]
    binary = data[20 + js_len + 8:20 + js_len + 8 + bin_len]
    return gltf, binary


def write_glb(path, gltf, binary):
    js = json.dumps(gltf, separators=(',', ':')).encode()
    js += b' ' * ((4 - len(js) % 4) % 4)
    bn = binary + b'\0' * ((4 - len(binary) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(bn)
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, total))
        f.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
        f.write(struct.pack('<II', len(bn), 0x004E4942) + bn)


def local(nd):
    if 'matrix' in nd:
        return np.array(nd['matrix']).reshape(4, 4).T
    t = nd.get('translation', [0, 0, 0])
    x, y, z, w = nd.get('rotation', [0, 0, 0, 1])
    s = nd.get('scale', [1, 1, 1])
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    m = np.eye(4)
    m[:3, :3] = r * np.array(s)
    m[:3, 3] = t
    return m


def kickoff_angle(gltf, hive, name):
    """The hive's angle at kickoff, about our +X: +TILT with its north cell up, -TILT with its south."""
    cells = [k for k in gltf['nodes'][hive]['children'] if 'Cell' in gltf['nodes'][k].get('name', '')]
    assert len(cells) == 2, name + ': expected two cells'
    ys, zs = [], []
    for k in cells:
        m = local(gltf['nodes'][hive]) @ local(gltf['nodes'][k])
        pts = []

        def walk(i, mm):
            nd = gltf['nodes'][i]
            mm = mm @ local(nd)
            if 'mesh' in nd:
                for pr in gltf['meshes'][nd['mesh']]['primitives']:
                    a = gltf['accessors'][pr['attributes']['POSITION']]
                    for c in [(u, v, w) for u in (a['min'][0], a['max'][0]) for v in (a['min'][1], a['max'][1])
                              for w in (a['min'][2], a['max'][2])]:
                        pts.append((mm @ np.array([*c, 1]))[:3])
            for kk in nd.get('children', []):
                walk(kk, mm)

        walk(k, local(gltf['nodes'][hive]))
        p = np.array(pts)
        ys.append(-p[:, 2].mean() / IN_TO_M)   # our y = -gz
        zs.append(p[:, 1].mean() / IN_TO_M)    # our z = gy
    north_up = zs[int(np.argmax(ys))] > zs[int(np.argmin(ys))]
    return TILT_RAD if north_up else -TILT_RAD


def main():
    gltf, binary = read_glb(os.path.join(SRC, 'model.glb'))
    nodes = gltf['nodes']
    by_name = {nd.get('name', ''): i for i, nd in enumerate(nodes)}
    root = gltf['scenes'][0]['nodes'][0]
    assert not any(k in nodes[root] for k in ('matrix', 'translation', 'rotation', 'scale')), 'field root moved'
    hive_nodes = {}
    for alliance, prefix in HIVES.items():
        found = [i for n, i in by_name.items() if n.startswith(prefix)]
        assert len(found) == 1 and found[0] in nodes[root]['children'], prefix + ' not found under the field root'
        hive_nodes[alliance] = found[0]

    # The field without its hives.
    field_dir = os.path.join(DST, 'Field3d_BIOBUZZTippingHive')
    os.makedirs(field_dir, exist_ok=True)
    g = json.loads(json.dumps(gltf))
    g['nodes'][root]['children'] = [k for k in g['nodes'][root]['children'] if k not in hive_nodes.values()]
    write_glb(os.path.join(field_dir, 'model.glb'), g, binary)
    with open(os.path.join(SRC, 'config.json'), encoding='utf-8') as f:
        cfg = json.load(f)
    cfg['name'] = '2026-2027 Field (tipping hive)'
    cfg.pop('locales', None)
    with open(os.path.join(field_dir, 'config.json'), 'w', encoding='utf-8') as f:
        json.dump(cfg, f, indent=2)
    for fn in os.listdir(SRC):
        if fn.startswith('model_') and fn.endswith('.glb'):
            shutil.copyfile(os.path.join(SRC, fn), os.path.join(field_dir, fn))

    # Each hive alone, about its axle.
    perm = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)   # GLB (gx, gy, gz) -> logged (gz, gx, gy)
    for alliance, hive in hive_nodes.items():
        x_in = -PIVOT_X_IN if alliance == 'Red' else PIVOT_X_IN
        pivot = np.array([0.0, x_in, AXLE_Z_IN]) * IN_TO_M                # logged frame: (-y, x, z), y = 0
        a0 = kickoff_angle(gltf, hive, alliance)
        c, s = math.cos(-a0), math.sin(-a0)
        ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        m = np.eye(4)
        m[:3, :3] = ry @ perm
        m[:3, 3] = -ry @ pivot
        g = json.loads(json.dumps(gltf))
        g['nodes'][root]['children'] = [k for k in g['nodes'][root]['children'] if k != hive]
        g['nodes'].append({'name': 'Hive' + alliance + 'Axle', 'matrix': list(m.T.flatten()), 'children': [hive]})
        g['scenes'] = [{'nodes': [len(g['nodes']) - 1]}]
        g['scene'] = 0
        out = os.path.join(DST, 'Robot_Hive' + alliance)
        os.makedirs(out, exist_ok=True)
        write_glb(os.path.join(out, 'model.glb'), g, binary)
        with open(os.path.join(out, 'config.json'), 'w', encoding='utf-8') as f:
            json.dump({'name': 'Hive (' + alliance + ')', 'isFTC': True, 'rotations': [], 'position': [0, 0, 0],
                       'cameras': [], 'components': []}, f, indent=2)
        print('%s hive: kickoff %+.0f deg (%s cell up), axle (%.2f, 0, %.2f) in' % (
            alliance, math.degrees(a0), 'north' if a0 > 0 else 'south', x_in, AXLE_Z_IN))
    print('wrote', field_dir, 'and Robot_HiveRed, Robot_HiveBlue under', DST)


if __name__ == '__main__':
    main()

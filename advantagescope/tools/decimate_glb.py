"""Shrinks a GLB written by step_to_glb.py by vertex clustering, in place.

Usage: python decimate_glb.py <model.glb> [cell_mm]

Every vertex is snapped to a cubic grid of cell_mm; the vertices in one cell merge into their
mean, triangles that collapse are dropped, and duplicates are removed. Detail finer than a cell
(threads, fillets, tread texture) disappears, which is invisible at field scale. Reads only the
simple layout step_to_glb.py writes (one mesh, POSITION + uint32 indices per primitive).
"""
import json
import struct
import sys

import numpy as np

path = sys.argv[1]
cell_m = (float(sys.argv[2]) if len(sys.argv) > 2 else 3.0) / 1000.0

with open(path, "rb") as f:
    data = f.read()
magic, _, _ = struct.unpack_from("<III", data, 0)
assert magic == 0x46546C67
js_len, _ = struct.unpack_from("<II", data, 12)
gltf = json.loads(data[20:20 + js_len])
bin_off = 20 + js_len + 8
binary = data[bin_off:]


def read(acc_index, dtype, width):
    acc = gltf["accessors"][acc_index]
    view = gltf["bufferViews"][acc["bufferView"]]
    start = view["byteOffset"]
    arr = np.frombuffer(binary, dtype=dtype, count=acc["count"] * width, offset=start)
    return arr.reshape(-1, width) if width > 1 else arr


before = after = 0
groups = []
for prim in gltf["meshes"][0]["primitives"]:
    pos = read(prim["attributes"]["POSITION"], np.float32, 3).astype(np.float64)
    tri = read(prim["indices"], np.uint32, 1).reshape(-1, 3).astype(np.int64)
    before += len(tri)
    keys = np.floor(pos / cell_m).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    merged = np.zeros((len(uniq), 3))
    np.add.at(merged, inverse, pos)
    merged /= np.bincount(inverse, minlength=len(uniq))[:, None]
    t = inverse[tri]
    t = t[(t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 0] != t[:, 2])]
    _, first = np.unique(np.sort(t, axis=1), axis=0, return_index=True)
    t = t[np.sort(first)]
    used, t = np.unique(t, return_inverse=True)
    t = t.reshape(-1, 3)
    after += len(t)
    groups.append((prim["material"], merged[used].astype(np.float32), t.astype(np.uint32).ravel()))

bin_parts, views, accessors, prims = [], [], [], []
offset = 0


def add_view(raw, target):
    global offset
    pad = (-len(raw)) % 4
    bin_parts.append(raw + b"\0" * pad)
    views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(raw), "target": target})
    offset += len(raw) + pad
    return len(views) - 1


for material, pos, idx in groups:
    if len(idx) == 0:
        continue
    pv = add_view(pos.tobytes(), 34962)
    iv = add_view(idx.tobytes(), 34963)
    accessors.append({"bufferView": pv, "componentType": 5126, "count": len(pos), "type": "VEC3",
                      "min": pos.min(axis=0).tolist(), "max": pos.max(axis=0).tolist()})
    accessors.append({"bufferView": iv, "componentType": 5125, "count": len(idx), "type": "SCALAR"})
    prims.append({"attributes": {"POSITION": len(accessors) - 2}, "indices": len(accessors) - 1,
                  "material": material})
gltf["meshes"][0]["primitives"] = prims
gltf["accessors"], gltf["bufferViews"] = accessors, views
gltf["buffers"] = [{"byteLength": offset}]
js = json.dumps(gltf, separators=(",", ":")).encode()
js += b" " * ((-len(js)) % 4)
out = b"".join(bin_parts)
with open(path, "wb") as f:
    f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(out)))
    f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
    f.write(struct.pack("<II", len(out), 0x004E4942) + out)
print("%s: %d -> %d triangles, %.2f MB" % (path, before, after, (28 + len(js) + len(out)) / 1e6))

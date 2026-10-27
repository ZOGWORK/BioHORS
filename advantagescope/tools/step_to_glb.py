"""Builds the AdvantageScope robot asset (advantagescope/Robot_BioHORS) from the Onshape STEP.

Usage (Python with cadquery-ocp and numpy installed):
    python step_to_glb.py "<robot>.step" <out_dir> [linear_deflection_mm]

Reads the STEP with OpenCascade XCAF (gmsh hangs on this file), meshes every placed part, moves
it into the robot frame of docs/ROBOT_CAD.md, and writes:
    model.glb    everything except the two flywheel assemblies
    model_0.glb  the pollen flywheel ("Wheel W sonic and weight" under "Pollen Shooter Assembly")
    model_1.glb  the nectar flywheel (the same, under "Nectar Shooter Assembly")

Robot frame (inches, then metres in the file): forward = 1.735 - cadX, left = -cadY,
up = cadZ + 1.105, so the origin is the drive centre on the floor, the pose AdvantageScope draws.
glTF is written with that frame's axes as-is (x forward, y left, z up), so config.json needs no
rotations. Vertices are not given normals, so three.js shades flat.

The two "Pollen" balls the CAD shows held in the robot are left out: the sim draws the pollen
the robot really holds (Field/Pollen).

Parts whose bounding box is under SMALL_PART_IN on every side (screws, nuts, spacers) are dropped
to keep the files small; they are invisible at field scale.
"""
import json
import struct
import sys
import time

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType, XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence

MM_TO_IN = 1 / 25.4
IN_TO_M = 0.0254
DRIVE_CENTRE_CAD_X_IN = 1.735  # docs/ROBOT_CAD.md
FLOOR_CAD_Z_IN = -1.105        # docs/ROBOT_CAD.md
SMALL_PART_IN = 0.45
DEFAULT_RGB = (0.55, 0.57, 0.6)

src, out_dir = sys.argv[1], sys.argv[2]
deflection_mm = float(sys.argv[3]) if len(sys.argv) > 3 else 0.8

t0 = time.time()
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
reader.SetLayerMode(False)
if reader.ReadFile(src) != IFSelect_RetDone:
    sys.exit("read failed")
reader.Transfer(doc)
print("transferred in %.1f s" % (time.time() - t0), flush=True)
shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def name_of(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return "?"


def color_of(label):
    c = Quantity_Color()
    for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if XCAFDoc_ColorTool.GetColor_s(label, kind, c):
            return (c.Red(), c.Green(), c.Blue())
    return None


def to_robot_m(p):
    """CAD millimetres (n x 3) to robot-frame metres."""
    x_in, y_in, z_in = p[:, 0] * MM_TO_IN, p[:, 1] * MM_TO_IN, p[:, 2] * MM_TO_IN
    return np.stack([DRIVE_CENTRE_CAD_X_IN - x_in, -y_in, z_in - FLOOR_CAD_Z_IN], axis=1) * IN_TO_M


def target_of(path):
    joined = "/".join(path)
    if "Wheel W sonic and weight" in joined:
        if "Pollen Shooter Assembly" in joined:
            return "model_0"
        if "Nectar Shooter Assembly" in joined:
            return "model_1"
    return "model"


# target -> rgb -> list of (positions, indices)
meshes = {"model": {}, "model_0": {}, "model_1": {}}
dropped = 0
axles = {}


def add_part(shape, loc, path, rgb):
    global dropped
    if path[-1] == "Pollen":
        return
    placed = shape.Moved(loc)
    box = Bnd_Box()
    BRepBndLib.Add_s(placed, box, False)
    if box.IsVoid():
        return
    lo, hi = box.CornerMin(), box.CornerMax()
    size_in = max(hi.X() - lo.X(), hi.Y() - lo.Y(), hi.Z() - lo.Z()) * MM_TO_IN
    target = target_of(path)
    if "Steel Shaft" in path[-1] and target != "model":
        # The flywheel's spin axis: the shaft's bounding-box centre (it runs along cadY).
        axles[target] = to_robot_m(np.array([[(lo.X() + hi.X()) / 2, 0, (lo.Z() + hi.Z()) / 2]]))[0]
    if size_in < SMALL_PART_IN and target == "model":
        dropped += 1
        return
    BRepMesh_IncrementalMesh(placed, deflection_mm, False, 0.6, True)
    exp = TopExp_Explorer(placed, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        floc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, floc)
        exp.Next()
        if tri is None:
            continue
        trsf = floc.Transformation()
        pts = []
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(trsf)
            pts.append((p.X(), p.Y(), p.Z()))
        idx = np.empty((tri.NbTriangles(), 3), dtype=np.int64)
        for i in range(1, tri.NbTriangles() + 1):
            idx[i - 1] = tri.Triangle(i).Get()
        idx -= 1
        if face.Orientation() == TopAbs_REVERSED:
            idx = idx[:, [0, 2, 1]]
        key = tuple(round(v, 2) for v in rgb)
        meshes[target].setdefault(key, []).append((to_robot_m(np.array(pts)), idx))


def walk(label, loc, path, rgb):
    rgb = color_of(label) or rgb
    if shapes.IsReference_s(label):
        ref = TDF_Label()
        shapes.GetReferredShape_s(label, ref)
        walk(ref, loc.Multiplied(shapes.GetLocation_s(label)), path + [name_of(label)], rgb)
        return
    if shapes.IsAssembly_s(label):
        comps = TDF_LabelSequence()
        shapes.GetComponents_s(label, comps, False)
        for c in comps:
            walk(c, loc, path, rgb)
        return
    shape = shapes.GetShape_s(label)
    if not shape.IsNull():
        add_part(shape, loc, path + [name_of(label)], rgb or DEFAULT_RGB)


free = TDF_LabelSequence()
shapes.GetFreeShapes(free)
for lab in free:
    walk(lab, TopLoc_Location(), [name_of(lab)], None)
print("meshed in %.1f s, dropped %d small parts" % (time.time() - t0, dropped), flush=True)


def write_glb(path, groups):
    """One mesh, one primitive per colour, positions float32 and indices uint32."""
    bin_parts, views, accessors, prims, materials = [], [], [], [], []
    offset = 0
    tris = 0

    def add_view(data, target):
        nonlocal offset
        pad = (-len(data)) % 4
        bin_parts.append(data + b"\0" * pad)
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(data), "target": target})
        offset += len(data) + pad
        return len(views) - 1

    for rgb, parts in sorted(groups.items()):
        pos_list, idx_list, base = [], [], 0
        for pos, idx in parts:
            pos_list.append(pos)
            idx_list.append(idx + base)
            base += len(pos)
        pos = np.concatenate(pos_list).astype(np.float32)
        idx = np.concatenate(idx_list).astype(np.uint32).ravel()
        tris += len(idx) // 3
        pv = add_view(pos.tobytes(), 34962)
        iv = add_view(idx.tobytes(), 34963)
        accessors.append({"bufferView": pv, "componentType": 5126, "count": len(pos), "type": "VEC3",
                          "min": pos.min(axis=0).tolist(), "max": pos.max(axis=0).tolist()})
        accessors.append({"bufferView": iv, "componentType": 5125, "count": len(idx), "type": "SCALAR"})
        materials.append({"pbrMetallicRoughness": {"baseColorFactor": [*rgb, 1.0], "metallicFactor": 0.1,
                                                   "roughnessFactor": 0.7}, "doubleSided": True})
        prims.append({"attributes": {"POSITION": len(accessors) - 2}, "indices": len(accessors) - 1,
                      "material": len(materials) - 1})
    gltf = {"asset": {"version": "2.0", "generator": "BioHORS step_to_glb.py"}, "scene": 0,
            "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}], "meshes": [{"primitives": prims}],
            "materials": materials, "accessors": accessors, "bufferViews": views,
            "buffers": [{"byteLength": offset}]}
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((-len(js)) % 4)
    binary = b"".join(bin_parts)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(binary)))
        f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        f.write(struct.pack("<II", len(binary), 0x004E4942) + binary)
    return tris


# Parts are found by their CAD names: after a rename, stop before overwriting any file with an
# empty model or leaving config.json zeroed at an axle that is no longer found.
missing = [n for n, g in meshes.items() if not g] + ["%s axle" % n for n in ("model_0", "model_1") if n not in axles]
if missing:
    sys.exit("nothing found for %s; check target_of() and the shaft name against the CAD. "
             "No file written." % ", ".join(missing))
for name, groups in meshes.items():
    n = write_glb("%s/%s.glb" % (out_dir, name), groups)
    print(name, "triangles", n, "colours", len(groups))
for name, a in sorted(axles.items()):
    print(name, "axle (robot frame, m)", [round(v, 4) for v in a])
print("done in %.1f s" % (time.time() - t0))

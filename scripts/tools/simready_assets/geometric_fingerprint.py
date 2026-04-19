"""Per-part geometric fingerprint for a SimReady USD asset.

Emits compact structured ground-truth descriptors (bbox, axis, pivot, vertex
count, sibling layout) that let a classifier LLM reason about shape and
placement exactly, instead of inferring from rendered images.

The output is a single JSON-serializable dict meant to be pasted into the
classifier/vision prompt alongside the image views.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from pxr import Usd, UsdGeom, Gf


def _mesh_world_bbox(prim):
    """World-space bbox over every descendant Mesh under this prim."""
    bmin = Gf.Vec3d(1e30, 1e30, 1e30)
    bmax = Gf.Vec3d(-1e30, -1e30, -1e30)
    found = False
    n_verts = 0
    for desc in Usd.PrimRange(prim):
        if not desc.IsA(UsdGeom.Mesh):
            continue
        pts_attr = desc.GetAttribute("points")
        if not pts_attr or not pts_attr.HasValue():
            continue
        pts = pts_attr.Get()
        n_verts += len(pts)
        l2w = UsdGeom.Xformable(desc).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for p in pts:
            wp = l2w.TransformAffine(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
            bmin = Gf.Vec3d(min(bmin[0], wp[0]), min(bmin[1], wp[1]), min(bmin[2], wp[2]))
            bmax = Gf.Vec3d(max(bmax[0], wp[0]), max(bmax[1], wp[1]), max(bmax[2], wp[2]))
            found = True
    if not found:
        return None
    return bmin, bmax, n_verts


def _pivot_local(prim):
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and "pivot" in op.GetOpName() and not op.IsInverseOp():
            v = op.Get()
            if v is not None:
                return [round(float(v[0]), 4), round(float(v[1]), 4), round(float(v[2]), 4)]
    return None


AXIS_NAMES = ("X", "Y", "Z")


def _aspect(size, tol=1.6):
    """Classify a bbox's shape. Returns one of:
      disk_AXIS      — two dims close, third is clearly smaller (wheel/puck; AXIS is the axle)
      elongated_AXIS — one dim dominates the other two (arm/column; AXIS is the long direction)
      flat_AXIS      — one dim is much smaller than the other two (panel/plate; AXIS is the normal)
      blocky         — all three dims comparable
    """
    sorted_dims = sorted([(size[i], i) for i in range(3)])
    smallest, mid, largest = sorted_dims
    if largest[0] < 1e-4:
        return "point"
    ratio_large_mid = largest[0] / max(mid[0], 1e-9)
    ratio_mid_small = mid[0] / max(smallest[0], 1e-9)
    if ratio_large_mid < 1.3 and ratio_mid_small > tol:
        return f"disk_{AXIS_NAMES[smallest[1]]}"
    if ratio_large_mid > tol and ratio_mid_small > tol:
        return f"elongated_{AXIS_NAMES[largest[1]]}"
    if ratio_mid_small > 3.0:
        return f"flat_{AXIS_NAMES[smallest[1]]}"
    return "blocky"


def _thin_axis(size):
    """Axis letter of the smallest bbox dimension. Useful for wheel-axle hinting."""
    return AXIS_NAMES[min(range(3), key=lambda i: size[i])]


def _long_axis(size):
    """Axis letter of the largest bbox dimension. Useful for slider/panel hinting."""
    return AXIS_NAMES[max(range(3), key=lambda i: size[i])]


def _round_vec(v, n=4):
    return [round(float(v[0]), n), round(float(v[1]), n), round(float(v[2]), n)]


def fingerprint(stage):
    """Build the fingerprint for every Xform descendant of the default prim.

    Returns a dict with:
        stage: mpu, default_prim, world_bbox
        parts: list of per-prim records
    """
    dp = stage.GetDefaultPrim()
    if not dp:
        raise ValueError("stage has no default prim")
    mpu = UsdGeom.GetStageMetersPerUnit(stage)

    stage_bb = _mesh_world_bbox(dp)
    stage_info = {
        "meters_per_unit": mpu,
        "default_prim": dp.GetName(),
    }
    if stage_bb:
        bmin, bmax, n_verts = stage_bb
        stage_info["world_bbox"] = {
            "min": _round_vec(bmin), "max": _round_vec(bmax),
            "size": [round(float(bmax[i] - bmin[i]), 4) for i in range(3)],
            "center": [round(float((bmax[i] + bmin[i]) / 2), 4) for i in range(3)],
        }
        stage_info["total_vertex_count"] = n_verts

    # Walk the default prim subtree and emit one record per Xform.
    parts = []
    for prim in Usd.PrimRange(dp):
        if not prim.IsA(UsdGeom.Xform):
            continue
        if prim == dp:
            continue
        bb = _mesh_world_bbox(prim)
        if bb is None:
            continue
        bmin, bmax, n_verts = bb
        size = [round(float(bmax[i] - bmin[i]), 4) for i in range(3)]
        center = [round(float((bmax[i] + bmin[i]) / 2), 4) for i in range(3)]
        parent = prim.GetParent()
        parent_name = parent.GetName() if parent and parent.IsValid() else None
        parent_bb = _mesh_world_bbox(parent) if parent else None
        rec = {
            "name": prim.GetName(),
            "path": str(prim.GetPath()),
            "depth": str(prim.GetPath()).count("/") - 1,
            "parent_name": parent_name,
            "bbox_size": size,
            "bbox_center_world": center,
            "aspect": _aspect(size),
            "thin_axis": _thin_axis(size),
            "long_axis": _long_axis(size),
            "vertex_count": n_verts,
        }
        if parent_bb:
            p_bmin, p_bmax, _ = parent_bb
            p_center = [(p_bmax[i] + p_bmin[i]) / 2 for i in range(3)]
            rec["offset_from_parent_center"] = [
                round(float(center[i] - p_center[i]), 4) for i in range(3)
            ]
            # Relative size vs parent — useful for "is this a small mechanism inside a big body?"
            p_size = [p_bmax[i] - p_bmin[i] for i in range(3)]
            rec["relative_size_vs_parent"] = [
                round(float(size[i] / p_size[i]) if p_size[i] > 1e-9 else 0.0, 3)
                for i in range(3)
            ]
        pivot = _pivot_local(prim)
        if pivot is not None:
            rec["has_pivot"] = True
            rec["pivot_local"] = pivot
        # Siblings at the same Xform depth under the same parent
        if parent:
            sibs = [c.GetName() for c in parent.GetChildren()
                    if c.IsA(UsdGeom.Xform) and c.GetName() != prim.GetName()]
            if sibs:
                rec["siblings"] = sibs
        parts.append(rec)

    return {"stage": stage_info, "parts": parts}


def to_prompt_text(fp):
    """Render the fingerprint as a compact string that fits nicely in a prompt."""
    s = []
    stg = fp["stage"]
    bb = stg.get("world_bbox", {})
    s.append(f"ASSET: default_prim={stg['default_prim']} mpu={stg['meters_per_unit']}")
    if bb:
        s.append(f"  world_bbox size={bb['size']} center={bb['center']} verts={stg['total_vertex_count']}")
    s.append("")
    s.append(f"PARTS ({len(fp['parts'])}):  (thin_axis = wheel axle / flat-panel normal; long_axis = slider direction / arm)")
    for p in fp["parts"]:
        line = (f"  {p['name']:50s} depth={p['depth']} parent={p.get('parent_name','-'):30s} "
                f"size={p['bbox_size']} aspect={p['aspect']:10s} "
                f"thin={p['thin_axis']} long={p['long_axis']} verts={p['vertex_count']:>6}")
        if "offset_from_parent_center" in p:
            line += f" offset={p['offset_from_parent_center']}"
        if p.get("has_pivot"):
            line += f" pivot={p['pivot_local']}"
        s.append(line)
    return "\n".join(s)


if __name__ == "__main__":
    usd_path = sys.argv[1]
    stage = Usd.Stage.Open(usd_path)
    fp = fingerprint(stage)
    if len(sys.argv) > 2 and sys.argv[2] == "--text":
        print(to_prompt_text(fp))
    else:
        print(json.dumps(fp, indent=2))

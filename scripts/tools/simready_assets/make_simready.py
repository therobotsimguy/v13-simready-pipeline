#!/usr/bin/env python3
"""
make_simready.py  (V8)

Single script that takes a raw USD and makes it SimReady:

  Phase 1 — AUDIT:    Check the 7 SimReady criteria, report what's present vs missing.
  Phase 2 — CLASSIFY: LLM reads hierarchy, labels each part (body/movable/structural/decorative).
  Phase 3 — APPLY:    Add missing physics (rigid bodies, colliders, friction, joints, drives).

Usage:
  python make_simready.py --input asset.usd                       # audit only (dry run)
  python make_simready.py --input asset.usd --fix                 # audit + classify + fix
  python make_simready.py --input asset.usd --fix --provider openai

Fridge / trolley rules (viewport drag, masses, collision) are documented in the V8 repo:
  PRINCIPLES_FRIDGE_TROLLEY.md
"""

import argparse
import json
import os
import shutil
import sys

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf


# ═══════════════════════════════════════════════════════════════════
# CONSTANTS
# ═══════════════════════════════════════════════════════════════════

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
API_KEYS_PATH = os.path.join(SCRIPT_DIR, "..", "api_keys.json")

MAX_DECOMP_BUDGET = 5
QUALITY_VERT_THRESHOLD = 50000

FRICTION_TABLE = {
    "rubber": (0.8, 0.7),
    "steel": (0.74, 0.57),
    "metal": (0.6, 0.45),
    "chrome": (0.6, 0.45),
    "aluminium": (0.6, 0.45),
    "aluminum": (0.6, 0.45),
    "glossy": (0.6, 0.45),
    "plastic": (0.35, 0.3),
    "glass": (0.5, 0.35),
    "wood": (0.5, 0.4),
}


# ═══════════════════════════════════════════════════════════════════
# PHASE 1 — AUDIT
# ═══════════════════════════════════════════════════════════════════

def audit(stage):
    """Check all 7 SimReady criteria. Returns dict of criterion -> {pass, details}."""
    results = {}

    rigid_bodies = []
    colliders = []
    joints = []
    drives = []
    physics_materials = []
    mat_bindings = 0
    has_physics_scene = False
    has_contact_offset = False
    nested_rigid = False

    for prim in stage.Traverse():
        apis = [str(s) for s in prim.GetAppliedSchemas()]

        if "PhysicsRigidBodyAPI" in apis:
            mass_attr = prim.GetAttribute("physics:mass")
            mass = mass_attr.Get() if mass_attr and mass_attr.HasValue() else None
            kin = prim.GetAttribute("physics:kinematicEnabled")
            kin_val = kin.Get() if kin and kin.HasValue() else False
            rigid_bodies.append({
                "path": str(prim.GetPath()),
                "mass": mass,
                "kinematic": kin_val,
                "has_mass_api": "PhysicsMassAPI" in apis,
            })
            parent = prim.GetParent()
            if parent and parent.HasAPI(UsdPhysics.RigidBodyAPI):
                nested_rigid = True

        if "PhysicsCollisionAPI" in apis:
            approx = prim.GetAttribute("physics:approximation")
            approx_val = approx.Get() if approx and approx.HasValue() else "none"
            bind = UsdShade.MaterialBindingAPI(prim)
            physics_mat = bind.GetDirectBinding("physics")
            has_binding = bool(
                physics_mat and physics_mat.GetMaterialPath()
                and str(physics_mat.GetMaterialPath()) != ""
            )
            if has_binding:
                mat_bindings += 1
            colliders.append({
                "name": prim.GetName(),
                "approx": approx_val,
                "has_physics_mat_binding": has_binding,
            })

        if prim.IsA(UsdPhysics.Joint):
            lp0 = prim.GetAttribute("physics:localPos0")
            lp1 = prim.GetAttribute("physics:localPos1")
            lp0_val = lp0.Get() if lp0 and lp0.HasValue() else None
            lp1_val = lp1.Get() if lp1 and lp1.HasValue() else None
            lp0_zero = lp0_val is not None and all(abs(float(v)) < 1e-6 for v in lp0_val)
            lp1_zero = lp1_val is not None and all(abs(float(v)) < 1e-6 for v in lp1_val)
            joints.append({
                "name": prim.GetName(),
                "type": prim.GetTypeName(),
                "both_anchors_zero": lp0_zero and lp1_zero,
            })

        for api in apis:
            if "PhysicsDriveAPI" in api:
                drives.append({"joint": prim.GetName(), "api": api})

        if "PhysicsMaterialAPI" in apis:
            sf = prim.GetAttribute("physics:staticFriction")
            physics_materials.append({
                "name": prim.GetName(),
                "sf": sf.Get() if sf and sf.HasValue() else None,
            })

        if prim.IsA(UsdPhysics.Scene):
            has_physics_scene = True

        co = prim.GetAttribute("physxCollision:contactOffset")
        if co and co.HasValue():
            has_contact_offset = True

    # C1: Rigid Bodies
    c1_pass = len(rigid_bodies) > 0 and all(rb["has_mass_api"] for rb in rigid_bodies)
    c1_detail = f"{len(rigid_bodies)} rigid bodies"
    if rigid_bodies and not all(rb["has_mass_api"] for rb in rigid_bodies):
        c1_detail += " (some missing MassAPI)"
    if not rigid_bodies:
        c1_detail = "0 found, need at least 1"
    if nested_rigid:
        c1_pass = False
        c1_detail += " — NESTED rigid body detected"
    results["C1 Rigid Bodies"] = {"pass": c1_pass, "detail": c1_detail}

    # C2: Collision Shapes (global + per-rigid-body coverage)
    c2_pass = len(colliders) > 0 and all(c["approx"] != "none" for c in colliders)
    approx_counts = {}
    for c in colliders:
        approx_counts[c["approx"]] = approx_counts.get(c["approx"], 0) + 1
    c2_detail = f"{len(colliders)} colliders"
    if approx_counts:
        c2_detail += f" ({approx_counts})"
    if not colliders:
        c2_detail = "0 colliders"
    # Per-rigid-body coverage: every rigid body must have ≥1 descendant collider
    bodies_without_colliders = []
    for rb in rigid_bodies:
        rb_prim = stage.GetPrimAtPath(rb["path"])
        if not rb_prim:
            continue
        has_col = False
        for desc in Usd.PrimRange(rb_prim):
            if desc.HasAPI(UsdPhysics.CollisionAPI):
                has_col = True
                break
        if not has_col:
            bodies_without_colliders.append(rb["path"])
    if bodies_without_colliders:
        c2_pass = False
        c2_detail += f" — {len(bodies_without_colliders)} rigid body(s) have NO colliders: {bodies_without_colliders}"
    results["C2 Collision Shapes"] = {"pass": c2_pass, "detail": c2_detail}

    # C3: Friction Materials + GripMaterial on handles (F29, F31)
    c3_pass = len(colliders) > 0 and mat_bindings == len(colliders)
    c3_detail = f"{mat_bindings}/{len(colliders)} colliders have material:binding:physics"
    if not colliders:
        c3_detail = "no colliders to bind"
    # F29/F31: Check that handle meshes exist and have GripMaterial
    handle_keywords = ("handle", "knob", "grip", "pull", "lever")
    handle_meshes = []
    handles_with_grip = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and any(kw in prim.GetName().lower() for kw in handle_keywords):
            handle_meshes.append(prim.GetName())
            bind = UsdShade.MaterialBindingAPI(prim)
            physics_mat = bind.GetDirectBinding("physics")
            if physics_mat and physics_mat.GetMaterialPath():
                mat_path = str(physics_mat.GetMaterialPath())
                if "grip" in mat_path.lower():
                    handles_with_grip += 1
    if handle_meshes and handles_with_grip == 0:
        c3_detail += f" — WARNING: {len(handle_meshes)} handle(s) found but none bound to GripMaterial (F31)"
    results["C3 Friction"] = {"pass": c3_pass, "detail": c3_detail}

    # C4: Flat Hierarchy
    dp = stage.GetDefaultPrim()
    dp_path = dp.GetPath() if dp else Sdf.Path("/")
    movable_nested = []
    for rb in rigid_bodies:
        rb_path = Sdf.Path(rb["path"])
        parent = rb_path.GetParentPath()
        if parent != dp_path:
            parent_prim = stage.GetPrimAtPath(parent)
            if parent_prim and parent_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                movable_nested.append(rb["path"])
    if len(rigid_bodies) <= 1:
        c4_pass = True
        c4_detail = "single body — hierarchy N/A"
        c4_na = True
    else:
        c4_pass = len(movable_nested) == 0
        c4_detail = (
            "all movable parts are siblings"
            if c4_pass
            else f"{len(movable_nested)} movable parts nested under another rigid body"
        )
        c4_na = False
    results["C4 Flat Hierarchy"] = {"pass": c4_pass, "detail": c4_detail, "na": c4_na if len(rigid_bodies) <= 1 else False}

    # C5: Joints (existence + anchor validity)
    has_movables = len(rigid_bodies) > 1
    if has_movables:
        enough_joints = len(joints) >= len(rigid_bodies) - 1
        zero_anchor_joints = [j for j in joints if j.get("both_anchors_zero", False)]
        anchors_ok = len(zero_anchor_joints) == 0
        c5_pass = enough_joints and anchors_ok
        c5_detail = f"{len(joints)} joints for {len(rigid_bodies) - 1} movable parts"
        if not enough_joints:
            c5_detail += " — need more joints"
        if not anchors_ok:
            c5_pass = False
            c5_detail += f" — {len(zero_anchor_joints)} joints have ZERO anchors (localPos0=localPos1=(0,0,0))"
    else:
        c5_pass = True
        c5_detail = "no movable parts — joints N/A"
    results["C5 Joints"] = {"pass": c5_pass, "detail": c5_detail, "na": not has_movables}

    # C6: Joint Drives + stiffness/damping validation (F18, F32)
    if joints:
        c6_pass = len(drives) >= len(joints)
        c6_detail = f"{len(drives)} drives for {len(joints)} joints"
        # F18: Check stiffness=0 on all drives (non-zero jams doors)
        # F32: Check damping>0 on all drives (zero causes oscillation)
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint):
                for attr in prim.GetAttributes():
                    aname = attr.GetName()
                    if "stiffness" in aname.lower() and "drive" in aname.lower():
                        val = attr.Get()
                        if val is not None and float(val) > 0:
                            c6_detail += f" — WARNING: {prim.GetName()} has stiffness={val} (F18: should be 0)"
                    if "damping" in aname.lower() and "drive" in aname.lower():
                        val = attr.Get()
                        if val is not None and float(val) <= 0:
                            c6_detail += f" — WARNING: {prim.GetName()} has damping={val} (F32: should be >0)"
    else:
        c6_pass = True
        c6_detail = "no joints — drives N/A"
    results["C6 Joint Drives"] = {"pass": c6_pass, "detail": c6_detail, "na": not joints}

    # C7: Clean Asset (no scene, no contactOffset, meters)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    c7_issues = []
    if has_physics_scene:
        c7_issues.append("PhysicsScene found")
    if has_contact_offset:
        c7_issues.append("contactOffset found")
    if abs(mpu - 1.0) > 0.01:
        unit_name = "centimeters" if abs(mpu - 0.01) < 0.001 else f"mpu={mpu}"
        c7_issues.append(f"stage in {unit_name}, not meters")
    c7_pass = len(c7_issues) == 0
    c7_detail = "clean (meters, no scene)" if c7_pass else "; ".join(c7_issues)
    results["C7 Clean Asset"] = {"pass": c7_pass, "detail": c7_detail}

    return results


def print_audit(results, label="AUDIT"):
    """Print a formatted scorecard."""
    print(f"\n  {label}:")
    total = 0
    passed = 0
    for name, info in results.items():
        is_na = info.get("na", False)
        if is_na:
            status = "N/A "
        elif info["pass"]:
            status = "PASS"
            total += 1
            passed += 1
        else:
            status = "FAIL"
            total += 1
        print(f"    {status}  {name}: {info['detail']}")
    if total > 0:
        print(f"    SCORE: {passed}/{total}")


# ═══════════════════════════════════════════════════════════════════
# PHASE 2 — CLASSIFY (hierarchy reader + LLM)
# ═══════════════════════════════════════════════════════════════════

def read_hierarchy(stage):
    """Read the USD hierarchy into a structured dict for the LLM."""
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        raise ValueError("USD has no default prim")

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    hierarchy = {
        "default_prim": default_prim.GetName(),
        "meters_per_unit": mpu,
        "children": [],
    }

    def describe_prim(prim, depth=0):
        info = {
            "name": prim.GetName(),
            "type": prim.GetTypeName(),
            "path": str(prim.GetPath()),
            "depth": depth,
            "children": [],
        }
        if prim.GetTypeName() == "Mesh":
            pts = prim.GetAttribute("points")
            info["vertex_count"] = len(pts.Get()) if pts and pts.HasValue() else 0

        if prim.GetTypeName() == "Xform":
            xf = UsdGeom.Xformable(prim)
            ops = xf.GetOrderedXformOps()
            info["xform_ops"] = [op.GetOpName() for op in ops]
            bbox = _quick_bbox(prim, mpu)
            if bbox:
                info["bbox_meters"] = bbox
            mesh_children = []
            for child in prim.GetAllChildren():
                if child.GetTypeName() == "Mesh":
                    pts = child.GetAttribute("points")
                    nv = len(pts.Get()) if pts and pts.HasValue() else 0
                    mesh_children.append({"name": child.GetName(), "vertices": nv})
            if mesh_children:
                info["meshes"] = mesh_children

        for child in prim.GetChildren():
            info["children"].append(describe_prim(child, depth + 1))
        return info

    for child in default_prim.GetChildren():
        hierarchy["children"].append(describe_prim(child, depth=1))
    return hierarchy


def _quick_bbox(prim, mpu):
    bmin = [1e30, 1e30, 1e30]
    bmax = [-1e30, -1e30, -1e30]
    found = False
    for child in prim.GetAllChildren():
        if child.GetTypeName() != "Mesh":
            continue
        pts = child.GetAttribute("points")
        if not pts or not pts.HasValue():
            continue
        for pt in pts.Get():
            for i in range(3):
                v = float(pt[i]) * mpu
                bmin[i] = min(bmin[i], v)
                bmax[i] = max(bmax[i], v)
            found = True
    if not found:
        return None
    dims = [round(bmax[i] - bmin[i], 4) for i in range(3)]
    return {"width_m": dims[0], "depth_m": dims[1], "height_m": dims[2]}


def hierarchy_to_text(hierarchy):
    """Convert hierarchy dict to readable text for LLM prompt."""
    lines = []
    lines.append(f"USD Asset: default_prim = {hierarchy['default_prim']}")
    lines.append(f"Meters per unit: {hierarchy['meters_per_unit']}")
    lines.append("")

    def fmt(info, indent=0):
        prefix = "  " * indent
        typ = info["type"]
        name = info["name"]
        if typ == "Xform":
            line = f"{prefix}[Xform] {name}"
            if "bbox_meters" in info:
                b = info["bbox_meters"]
                line += f"  (bbox: {b['width_m']:.3f} x {b['depth_m']:.3f} x {b['height_m']:.3f} m)"
            if "xform_ops" in info and info["xform_ops"]:
                ops = ", ".join(info["xform_ops"])
                line += f"  ops=[{ops}]"
            lines.append(line)
            if "meshes" in info:
                for m in info["meshes"]:
                    lines.append(f"{prefix}  [Mesh] {m['name']}  ({m['vertices']} verts)")
        elif typ == "Mesh":
            lines.append(f"{prefix}[Mesh] {name}  ({info.get('vertex_count', '?')} verts)")
        elif typ == "Scope":
            lines.append(f"{prefix}[Scope] {name}")
        else:
            lines.append(f"{prefix}[{typ}] {name}")
        for child in info.get("children", []):
            fmt(child, indent + 1)

    for child in hierarchy["children"]:
        fmt(child)
    return "\n".join(lines)


SYSTEM_PROMPT = """You are a SimReady asset classifier for robotic simulation.

Given a USD hierarchy, classify each part so physics can be applied.

## Rules

1. Identify the BODY — the main structural Xform (largest, most meshes/vertices).

2. For each Xform child of the body (or default prim), classify:
   - Door/lid/flap (hinged): "movable:revolute" + axis (Z=vertical hinge, X=horizontal)
   - Drawer/slider: "movable:prismatic" + axis (Y=depth, X=lateral)
   - Wheel/caster: "movable:continuous" + axis (axle direction)
   - Shelf/divider/interior: "structural"
   - Bolts/clips/LEDs/logos: "decorative"

3. Use name AND geometry (bbox, mesh count, xform ops) to decide.

4. Parts nested INSIDE a movable (shelves/racks/bins inside a door) are STRUCTURAL —
   they move with their parent, not independently. Only DIRECT children of the body
   should be classified as movable. Never classify a grandchild of the body as movable.

5. Output ONLY valid JSON, no markdown fences, no explanation.

## Output format

{
  "body": "<body Xform name>",
  "parts": {
    "<part_name>": {"class": "movable:revolute", "axis": "Z"},
    "<part_name>": {"class": "movable:prismatic", "axis": "Y"},
    "<part_name>": {"class": "movable:continuous", "axis": "Y"},
    "<part_name>": {"class": "structural"},
    "<part_name>": {"class": "decorative"}
  }
}
"""


def _load_api_config(provider):
    if not os.path.isfile(API_KEYS_PATH):
        return None, None
    with open(API_KEYS_PATH) as f:
        keys = json.load(f)
    if provider in keys:
        cfg = keys[provider]
        return cfg.get("api_key"), cfg.get("model")
    return None, None


def classify_with_openai(hierarchy_text, model=None):
    try:
        from openai import OpenAI
    except ImportError:
        print("ERROR: pip install openai")
        sys.exit(1)
    file_key, file_model = _load_api_config("openai")
    api_key = os.environ.get("OPENAI_API_KEY") or file_key
    model = model or file_model or "gpt-4o"
    if not api_key:
        print("ERROR: Set OPENAI_API_KEY or add to scripts/tools/api_keys.json")
        sys.exit(1)
    client = OpenAI(api_key=api_key)
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = client.chat.completions.create(
                model=model,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": f"Classify this USD hierarchy:\n\n{hierarchy_text}"},
                ],
                temperature=0.0,
            )
            text = response.choices[0].message.content.strip()
            if text.startswith("```"):
                text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
            result = json.loads(text)
            if "body" in result and "parts" in result:
                return result
            print(f"  Retry {attempt + 1}/{max_retries}: missing 'body' or 'parts' in response")
        except (json.JSONDecodeError, IndexError, KeyError) as e:
            print(f"  Retry {attempt + 1}/{max_retries}: {type(e).__name__}: {e}")
    raise ValueError(f"LLM classification failed after {max_retries} retries (F04)")


def classify_with_anthropic(hierarchy_text, model=None):
    try:
        import anthropic
    except ImportError:
        print("ERROR: pip install anthropic")
        sys.exit(1)
    file_key, file_model = _load_api_config("anthropic")
    api_key = os.environ.get("ANTHROPIC_API_KEY") or file_key
    model = model or file_model or "claude-sonnet-4-20250514"
    if not api_key:
        print("ERROR: Set ANTHROPIC_API_KEY or add to scripts/tools/api_keys.json")
        sys.exit(1)
    client = anthropic.Anthropic(api_key=api_key)
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = client.messages.create(
                model=model,
                max_tokens=4096,
                system=SYSTEM_PROMPT,
                messages=[
                    {"role": "user", "content": f"Classify this USD hierarchy:\n\n{hierarchy_text}"},
                ],
                temperature=0.0,
            )
            text = response.content[0].text.strip()
            if text.startswith("```"):
                text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
            result = json.loads(text)
            if "body" in result and "parts" in result:
                return result
            print(f"  Retry {attempt + 1}/{max_retries}: missing 'body' or 'parts' in response")
        except (json.JSONDecodeError, IndexError, KeyError) as e:
            print(f"  Retry {attempt + 1}/{max_retries}: {type(e).__name__}: {e}")
    raise ValueError(f"LLM classification failed after {max_retries} retries (F04)")


def classify_parts(stage, provider="anthropic", model=None):
    """Read USD hierarchy and classify parts via LLM. Returns classification dict."""
    hierarchy = read_hierarchy(stage)
    hierarchy_text = hierarchy_to_text(hierarchy)

    print(f"\n  LLM CLASSIFICATION ({provider}):")
    print(f"  Sending {len(hierarchy_text)} chars of hierarchy...")

    if provider == "openai":
        result = classify_with_openai(hierarchy_text, model=model)
    else:
        result = classify_with_anthropic(hierarchy_text, model=model)

    # Validate
    if "body" not in result or "parts" not in result:
        raise ValueError(f"LLM returned invalid classification: {result}")

    print(f"    body: {result['body']}")
    for name, spec in result["parts"].items():
        cls = spec.get("class", "?")
        axis = spec.get("axis", "")
        axis_str = f" axis={axis}" if axis else ""
        print(f"    {name:40s} -> {cls}{axis_str}")

    return result


# ═══════════════════════════════════════════════════════════════════
# PHASE 3 — APPLY (geometry helpers + physics applicators)
# ═══════════════════════════════════════════════════════════════════

# --- Geometry ---

def get_joint_anchor_world(stage, path):
    """World-space anchor point for a joint on this Xform.
    Uses pivot xformOp if present (transformed by L2W), otherwise Xform world origin.
    """
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return Gf.Vec3d(0, 0, 0)
    xf = UsdGeom.Xformable(prim)
    pivot_local = None
    for op in xf.GetOrderedXformOps():
        opname = op.GetOpName()
        if "pivot" in opname and "invert" not in opname:
            v = op.Get()
            pivot_local = Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))
            break
    l2w = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    if pivot_local is not None:
        return l2w.TransformAffine(pivot_local)
    return Gf.Vec3d(float(l2w[3][0]), float(l2w[3][1]), float(l2w[3][2]))


def world_point_to_local(stage, body_path, world_pt):
    """Transform a world point into a body's local frame."""
    prim = stage.GetPrimAtPath(body_path)
    if not prim:
        return Gf.Vec3f(0, 0, 0)
    xf = UsdGeom.Xformable(prim)
    l2w = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    w2l = l2w.GetInverse()
    lp = w2l.TransformAffine(world_pt)
    return Gf.Vec3f(float(lp[0]), float(lp[1]), float(lp[2]))


def mesh_world_bbox(stage, xform_path):
    """Compute world bbox from mesh vertices under an Xform (recursive)."""
    prim = stage.GetPrimAtPath(xform_path)
    if not prim:
        return None
    bmin = Gf.Vec3d(1e30, 1e30, 1e30)
    bmax = Gf.Vec3d(-1e30, -1e30, -1e30)
    found = False
    for child in _get_all_descendant_meshes(prim):
        pts = child.GetAttribute("points")
        if not pts or not pts.HasValue():
            continue
        mesh_xf = UsdGeom.Xformable(child)
        l2w = mesh_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for pt in pts.Get():
            wp = l2w.TransformAffine(Gf.Vec3d(float(pt[0]), float(pt[1]), float(pt[2])))
            bmin = Gf.Vec3d(min(bmin[0], wp[0]), min(bmin[1], wp[1]), min(bmin[2], wp[2]))
            bmax = Gf.Vec3d(max(bmax[0], wp[0]), max(bmax[1], wp[1]), max(bmax[2], wp[2]))
            found = True
    if not found:
        return None
    return bmin, bmax


# Keywords for rail/mechanism meshes that inflate drawer bbox beyond actual travel
_DRAWER_RAIL_KEYWORDS = ("mechanism", "frame", "rail", "track", "slide", "runner", "guide")


def mesh_world_bbox_excluding(stage, xform_path, exclude_keywords):
    """Like mesh_world_bbox but skip meshes whose names contain any exclude keyword."""
    prim = stage.GetPrimAtPath(xform_path)
    if not prim:
        return None
    bmin = Gf.Vec3d(1e30, 1e30, 1e30)
    bmax = Gf.Vec3d(-1e30, -1e30, -1e30)
    found = False
    for child in _get_all_descendant_meshes(prim):
        if any(kw in child.GetName().lower() for kw in exclude_keywords):
            continue
        pts = child.GetAttribute("points")
        if not pts or not pts.HasValue():
            continue
        mesh_xf = UsdGeom.Xformable(child)
        l2w = mesh_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for pt in pts.Get():
            wp = l2w.TransformAffine(Gf.Vec3d(float(pt[0]), float(pt[1]), float(pt[2])))
            bmin = Gf.Vec3d(min(bmin[0], wp[0]), min(bmin[1], wp[1]), min(bmin[2], wp[2]))
            bmax = Gf.Vec3d(max(bmax[0], wp[0]), max(bmax[1], wp[1]), max(bmax[2], wp[2]))
            found = True
    if not found:
        return None
    return bmin, bmax


def detect_hinge_edge(stage, door_path, anchor_world=None):
    """Detect which vertical edge is the hinge. Returns 'min_x' or 'max_x'.

    anchor_world should be passed explicitly when calling after reparent
    (pivot xformOps are cleared during reparent, so re-reading them gives wrong results).
    """
    if anchor_world is None:
        anchor_world = get_joint_anchor_world(stage, door_path)
    bbox = mesh_world_bbox(stage, door_path)
    if not bbox:
        return "min_x"
    bmin, bmax = bbox
    dist_to_min = abs(anchor_world[0] - bmin[0])
    dist_to_max = abs(anchor_world[0] - bmax[0])
    return "min_x" if dist_to_min < dist_to_max else "max_x"


def _mesh_vert_count(prim):
    pts = prim.GetAttribute("points")
    if pts and pts.HasValue():
        return len(pts.Get())
    return 0


def _get_all_descendant_meshes(prim):
    """Recursively collect all Mesh prims under a prim, including those under child Xforms."""
    meshes = []
    for child in prim.GetChildren():
        if child.GetTypeName() == "Mesh":
            meshes.append(child)
        elif child.GetTypeName() == "Xform":
            meshes.extend(_get_all_descendant_meshes(child))
    return meshes


MASS_CLAMPS = {
    # Fridge door Xforms (mesh bbox) often estimate 40–90kg; caps B–F SimReady outputs. Shift+drag is tuned via revolute drive damping, not mass cap.
    "revolute": (2.0, 100.0),
    "prismatic": (0.5, 5.0),
    "continuous": (0.05, 1.0),
    "fixed": (0.1, 10.0),
}


def estimate_mass(bbox, mpu=1.0, density=500.0):
    """Estimate mass from bbox volume (fallback when mesh volume unavailable)."""
    if not bbox:
        return 1.0
    bmin, bmax = bbox
    w = abs(bmax[0] - bmin[0]) * mpu
    d = abs(bmax[1] - bmin[1]) * mpu
    h = abs(bmax[2] - bmin[2]) * mpu
    vol = w * d * h
    return max(0.1, round(vol * density, 2))


def estimate_mass_from_mesh(stage, xform_path, density=500.0):
    """Estimate mass from actual mesh volume × density (more accurate than bbox).

    Uses the divergence theorem on triangle meshes. Falls back to bbox if
    mesh volume computation fails or returns zero.
    """
    prim = stage.GetPrimAtPath(xform_path)
    if not prim:
        return None
    total_volume = 0.0
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    for mesh_prim in _get_all_descendant_meshes(prim):
        pts_attr = mesh_prim.GetAttribute("points")
        idx_attr = mesh_prim.GetAttribute("faceVertexIndices")
        cnt_attr = mesh_prim.GetAttribute("faceVertexCounts")
        if not all(a and a.HasValue() for a in [pts_attr, idx_attr, cnt_attr]):
            continue
        pts = pts_attr.Get()
        indices = idx_attr.Get()
        counts = cnt_attr.Get()
        if not pts or not indices or not counts:
            continue
        # Scale vertices to meters
        verts = [(float(p[0]) * mpu, float(p[1]) * mpu, float(p[2]) * mpu) for p in pts]
        # Compute volume using divergence theorem
        vol = 0.0
        idx_offset = 0
        for fc in counts:
            if fc < 3:
                idx_offset += fc
                continue
            # Triangulate: fan from first vertex
            i0 = int(indices[idx_offset])
            for t in range(1, fc - 1):
                i1 = int(indices[idx_offset + t])
                i2 = int(indices[idx_offset + t + 1])
                v0, v1, v2 = verts[i0], verts[i1], verts[i2]
                # Signed volume contribution: v0 · (v1 × v2) / 6
                cross = (
                    v1[1] * v2[2] - v1[2] * v2[1],
                    v1[2] * v2[0] - v1[0] * v2[2],
                    v1[0] * v2[1] - v1[1] * v2[0],
                )
                vol += v0[0] * cross[0] + v0[1] * cross[1] + v0[2] * cross[2]
            idx_offset += fc
        total_volume += abs(vol) / 6.0
    if total_volume < 1e-10:
        return None
    mass = total_volume * density
    return max(0.01, round(mass, 4))


# --- Strip existing physics ---

def strip_existing_physics(stage):
    """Remove all existing physics APIs, joints, and physics materials for a clean slate."""
    prims_to_remove = []
    n_props = 0
    n_joints = 0
    n_mats = 0

    physics_schemas = [
        "PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
        "PhysicsMeshCollisionAPI", "PhysicsMassAPI",
        "PhysicsArticulationRootAPI",
    ]

    for prim in stage.Traverse():
        prim_type = prim.GetTypeName()

        if "Joint" in prim_type:
            prims_to_remove.append(prim.GetPath())
            n_joints += 1
            continue

        # C7: host app owns PhysicsScene — remove embedded scenes (audit flags them; previously only props were stripped)
        if prim.IsA(UsdPhysics.Scene):
            prims_to_remove.append(prim.GetPath())
            continue

        if prim.GetName() in ("GripMaterial", "DefaultPhysMaterial") and prim_type == "Material":
            prims_to_remove.append(prim.GetPath())
            n_mats += 1
            continue

        if prim.GetName() == "joints" and prim_type == "Scope":
            prims_to_remove.append(prim.GetPath())
            continue

        props_to_remove = []
        for prop in prim.GetAuthoredProperties():
            n = prop.GetName()
            if n.startswith("physics:") or n.startswith("physx"):
                props_to_remove.append(n)
            if n == "material:binding:physics":
                props_to_remove.append(n)
        for n in props_to_remove:
            prim.RemoveProperty(n)
            n_props += 1

        prim_spec = stage.GetRootLayer().GetPrimAtPath(prim.GetPath())
        if prim_spec:
            schemas_info = prim_spec.GetInfo("apiSchemas")
            if schemas_info and hasattr(schemas_info, "prependedItems"):
                current = list(schemas_info.prependedItems)
                filtered = [s for s in current if not any(ps in s for ps in physics_schemas)]
                if len(filtered) < len(current):
                    if filtered:
                        new_list = Sdf.TokenListOp()
                        new_list.prependedItems = filtered
                        prim_spec.SetInfo("apiSchemas", new_list)
                    else:
                        prim_spec.ClearInfo("apiSchemas")

    if prims_to_remove:
        edit = Sdf.BatchNamespaceEdit()
        for path in prims_to_remove:
            edit.Add(path, Sdf.Path.emptyPath)
        stage.GetRootLayer().Apply(edit)

    return n_joints, n_props, n_mats


# --- Collision ---

# Movable direct-child meshes matching these substrings get no collider — they overlap the cabinet
# cavity / frame and jam revolute doors in viewport drag (Refrigerator_A vs B: extra clips/bolts/logo/locker
# hulls). Keep outer panel (*body*) and *handle* for manipulation. Matches simready-collision “skip bolts, clips, rubber”.
_MOVABLE_COLLISION_SKIP_SUBSTR = (
    "interior",
    "clips",
    "bolt",
    "logo",
    "rubber",
    "lockerbox",
    "lockercilinder",
    "lockerbase",
    "refresher",
    "mechanism",
    "frame",
)


def _filter_movable_collision_meshes(mesh_prims):
    kept = [m for m in mesh_prims
            if not any(s in m.GetName().lower() for s in _MOVABLE_COLLISION_SKIP_SUBSTR)]
    return kept if kept else list(mesh_prims)


def apply_collision_q1(stage, xform_path, is_body=False):
    """Apply CollisionAPI: decomp on large concave body meshes, hull on small parts.

    For body: recurse into all descendant meshes.
    For movable parts: direct child meshes only — interior sub-Xform meshes
    (door shelves, rack bins) would clip with body internals when closed.
    """
    prim = stage.GetPrimAtPath(xform_path)
    if not prim:
        return 0, 0

    if is_body:
        meshes = [(m, _mesh_vert_count(m)) for m in _get_all_descendant_meshes(prim)]
    else:
        raw = [m for m in prim.GetChildren() if m.GetTypeName() == "Mesh"]
        raw = _filter_movable_collision_meshes(raw)
        meshes = [(m, _mesh_vert_count(m)) for m in raw]
        # Fallback: if no direct Mesh children (deeply nested Xform→Xform→Mesh),
        # search recursively. Common on small tools (scissors, forceps).
        if not meshes:
            raw = list(_get_all_descendant_meshes(prim))
            raw = _filter_movable_collision_meshes(raw)
            meshes = [(m, _mesh_vert_count(m)) for m in raw]
    if not meshes:
        return 0, 0

    meshes.sort(key=lambda x: x[1], reverse=True)
    n_col = 0
    n_decomp = 0
    for mesh_prim, npts in meshes:
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
        mc = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        use_decomp = is_body and npts > 2000
        if use_decomp and n_decomp < MAX_DECOMP_BUDGET:
            mc.CreateApproximationAttr("convexDecomposition")
            n_decomp += 1
            # Always set quality params on body decomposition — default
            # decomposition produces bloated hulls around thin concave
            # geometry (e.g., trolley rails, fridge frames). The vertex
            # threshold only gates additional quality; body always gets it.
            if is_body or npts > QUALITY_VERT_THRESHOLD:
                mesh_prim.CreateAttribute(
                    "physxConvexDecompositionCollision:maxConvexHulls",
                    Sdf.ValueTypeNames.Int).Set(128)
                mesh_prim.CreateAttribute(
                    "physxConvexDecompositionCollision:voxelResolution",
                    Sdf.ValueTypeNames.Int).Set(500000)
                mesh_prim.CreateAttribute(
                    "physxConvexDecompositionCollision:errorPercentage",
                    Sdf.ValueTypeNames.Float).Set(1.0)
        else:
            mc.CreateApproximationAttr("convexHull")
        n_col += 1
    return n_col, n_decomp


def apply_collision_wheels(stage, xform_path):
    """All wheel meshes get convexDecomposition (hull creates blobs)."""
    prim = stage.GetPrimAtPath(xform_path)
    if not prim:
        return 0
    n = 0
    for desc in _get_all_descendant_meshes(prim):
        UsdPhysics.CollisionAPI.Apply(desc)
        mc = UsdPhysics.MeshCollisionAPI.Apply(desc)
        mc.CreateApproximationAttr("convexDecomposition")
        n += 1
    return n


# --- Friction ---

def _guess_friction(material_name):
    """Guess friction coefficients from material name using the reference table."""
    name_lower = material_name.lower()
    for keyword, (sf, df) in FRICTION_TABLE.items():
        if keyword in name_lower:
            return sf, df
    return 0.5, 0.4


def wire_friction(stage, dp_path, handle_mesh_paths):
    """Create GripMaterial, bind friction on all collision meshes."""
    grip_path = Sdf.Path(f"{dp_path}/GripMaterial")
    grip_prim = stage.GetPrimAtPath(grip_path)
    if not grip_prim.IsValid():
        grip_mat = UsdShade.Material.Define(stage, grip_path)
        phys_api = UsdPhysics.MaterialAPI.Apply(grip_mat.GetPrim())
        phys_api.CreateStaticFrictionAttr(1.0)
        phys_api.CreateDynamicFrictionAttr(0.9)
        phys_api.CreateRestitutionAttr(0.0)

    handle_paths_set = set(str(p) for p in handle_mesh_paths)
    n_grip = 0
    n_body = 0

    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue

        binding_api = UsdShade.MaterialBindingAPI.Apply(prim)

        if str(prim.GetPath()) in handle_paths_set:
            binding_api.Bind(
                UsdShade.Material(stage.GetPrimAtPath(grip_path)),
                UsdShade.Tokens.weakerThanDescendants,
                "physics")
            n_grip += 1
        else:
            existing = binding_api.GetDirectBinding()
            if existing.GetMaterial().GetPrim().IsValid():
                mat_prim = existing.GetMaterial().GetPrim()
                if not mat_prim.HasAPI(UsdPhysics.MaterialAPI):
                    UsdPhysics.MaterialAPI.Apply(mat_prim)
                    sf, df = _guess_friction(mat_prim.GetName())
                    mat_prim.CreateAttribute("physics:staticFriction",
                                             Sdf.ValueTypeNames.Float).Set(sf)
                    mat_prim.CreateAttribute("physics:dynamicFriction",
                                             Sdf.ValueTypeNames.Float).Set(df)
                    mat_prim.CreateAttribute("physics:restitution",
                                             Sdf.ValueTypeNames.Float).Set(0.01)
                binding_api.Bind(
                    UsdShade.Material(mat_prim),
                    UsdShade.Tokens.weakerThanDescendants,
                    "physics")
            else:
                default_path = Sdf.Path(f"{dp_path}/DefaultPhysMaterial")
                default_prim = stage.GetPrimAtPath(default_path)
                if not default_prim.IsValid():
                    default_mat = UsdShade.Material.Define(stage, default_path)
                    phys_api = UsdPhysics.MaterialAPI.Apply(default_mat.GetPrim())
                    phys_api.CreateStaticFrictionAttr(0.5)
                    phys_api.CreateDynamicFrictionAttr(0.4)
                    phys_api.CreateRestitutionAttr(0.1)
                binding_api.Bind(
                    UsdShade.Material(stage.GetPrimAtPath(default_path)),
                    UsdShade.Tokens.weakerThanDescendants,
                    "physics")
            n_body += 1

    return n_grip, n_body


# --- Physics applicators ---

def apply_rigid_body(stage, path, kinematic=False, dynamic_body=False):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return
    UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    if dynamic_body:
        # V13: lowered from 100/200 — too sluggish for trolley pushing.
        # 10/20 provides stability without resisting Franka-level forces.
        prim.CreateAttribute("physics:linearDamping", Sdf.ValueTypeNames.Float).Set(10.0)
        prim.CreateAttribute("physics:angularDamping", Sdf.ValueTypeNames.Float).Set(20.0)


def apply_mass(stage, path, mass_kg):
    prim = stage.GetPrimAtPath(path)
    if prim:
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass_kg)


# --- Joints ---

def make_revolute_joint(stage, joint_path, body0, body1, local_pos0, local_pos1,
                        axis="Z", hinge_edge="min_x", lower_deg=-120, upper_deg=120):
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateAxisAttr(axis)
    if hinge_edge == "min_x":
        joint.CreateLowerLimitAttr(float(lower_deg))
        joint.CreateUpperLimitAttr(0.0)
    else:
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(float(upper_deg))
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(local_pos0)
    joint.CreateLocalPos1Attr(local_pos1)
    drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(joint_path), "angular")
    # Low damping so Isaac viewport shift+drag can rotate hinged parts (skill: ~2 Nm·s/rad for doors)
    drive.CreateDampingAttr(2.0)
    # Always stiffness 0: a positional spring to 0° (old dynamic_body branch) locks doors closed and blocks drag/gripper.
    drive.CreateStiffnessAttr(0.0)


def make_prismatic_joint(stage, joint_path, body0, body1, local_pos0, local_pos1,
                         axis="Y", lower_m=0.0, upper_m=0.4):
    joint = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(lower_m)
    joint.CreateUpperLimitAttr(upper_m)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(local_pos0)
    joint.CreateLocalPos1Attr(local_pos1)
    drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(joint_path), "linear")
    drive.CreateDampingAttr(5.0)
    drive.CreateStiffnessAttr(0.0)


def make_continuous_joint(stage, joint_path, body0, body1, local_pos0, local_pos1,
                          axis="X"):
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateAxisAttr(axis)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(local_pos0)
    joint.CreateLocalPos1Attr(local_pos1)
    joint.CreateLowerLimitAttr(-9999.0)
    joint.CreateUpperLimitAttr(9999.0)
    drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(joint_path), "angular")
    drive.CreateDampingAttr(2.0)
    drive.CreateStiffnessAttr(0.0)


def make_fixed_joint(stage, joint_path, body0, body1, local_pos0, local_pos1):
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(local_pos0)
    joint.CreateLocalPos1Attr(local_pos1)


# --- Reparent ---

def reparent_prims(stage, prim_paths, new_parent_path):
    """Move prims to be children of new_parent_path.

    Processes deepest paths first in separate batch edits to avoid
    parent-child conflicts (moving a parent invalidates children's source paths).
    """
    layer = stage.GetRootLayer()

    by_depth = {}
    for path in prim_paths:
        depth = len(path.GetPrefixes())
        by_depth.setdefault(depth, []).append(path)

    all_moved = {}
    for depth in sorted(by_depth.keys(), reverse=True):
        edit = Sdf.BatchNamespaceEdit()
        batch = {}
        for old_path in by_depth[depth]:
            new_path = new_parent_path.AppendChild(old_path.name)
            if old_path == new_path:
                continue
            edit.Add(old_path, new_path)
            batch[str(old_path)] = str(new_path)
        if batch and not layer.Apply(edit):
            print(f"  WARNING: SdfBatchNamespaceEdit failed at depth {depth}")
            continue
        all_moved.update(batch)
    return all_moved


def reparent_prims_preserve_world_xform(stage, prim_paths, new_parent_path):
    """Reparent preserving world pose via local = inv(parent_world) * world."""
    world_mats = {}
    for old_path in prim_paths:
        prim = stage.GetPrimAtPath(old_path)
        if not prim or not prim.IsValid():
            continue
        xf = UsdGeom.Xformable(prim)
        world_mats[str(old_path)] = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    moved = reparent_prims(stage, prim_paths, new_parent_path)

    for old_s, new_s in moved.items():
        prim = stage.GetPrimAtPath(Sdf.Path(new_s))
        if not prim or not prim.IsValid():
            continue
        wmat = world_mats.get(old_s)
        if wmat is None:
            continue
        parent = prim.GetParent()
        pxf = UsdGeom.Xformable(parent)
        pw = pxf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local_mat = pw.GetInverse() * wmat

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        top = xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
        top.Set(local_mat)

    return moved


# --- Wheel structural splitting ---

WHEEL_STRUCTURAL_KEYWORDS = ("fixer", "bolt", "body", "mount", "stopper")

def split_wheel_structural_parts(stage, movables, body_path):
    """Move structural child meshes (fixer/body/bolts) from wheel Xforms to body.

    Caster wheels contain rotating parts (tire, disc, detail) and structural
    parts (fixer, body, bolts = the bracket/fork). Structural parts must stay
    with the body; if they rotate with the wheel, brackets detach under force.
    """
    all_moved = {}
    for name, info in movables.items():
        if info["joint"] != "continuous":
            continue
        wheel_prim = stage.GetPrimAtPath(info["path"])
        if not wheel_prim:
            continue
        structural_paths = []
        for child in wheel_prim.GetAllChildren():
            if child.GetTypeName() != "Mesh":
                continue
            if any(kw in child.GetName().lower() for kw in WHEEL_STRUCTURAL_KEYWORDS):
                structural_paths.append(child.GetPath())
        if structural_paths:
            moved = reparent_prims_preserve_world_xform(stage, structural_paths, body_path)
            all_moved.update(moved)
    return all_moved


# --- Handle detection ---

def find_handle_meshes(stage, movable_paths):
    """Find Mesh prims that are handles/knobs under movable Xforms (recursive)."""
    handle_paths = []
    handle_keywords = ("handle", "knob", "grip", "pull", "lever")

    for path in movable_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        for mesh in _get_all_descendant_meshes(prim):
            if any(kw in mesh.GetName().lower() for kw in handle_keywords):
                handle_paths.append(mesh.GetPath())

    return handle_paths


# ═══════════════════════════════════════════════════════════════════
# MAIN — orchestrate all three phases
# ═══════════════════════════════════════════════════════════════════

def resolve_body_xform(stage, default_prim, body_name):
    """Find the body Xform by name."""
    dp_path = default_prim.GetPath()
    candidate = dp_path.AppendChild(body_name)
    if stage.GetPrimAtPath(candidate).IsValid():
        return candidate
    for child in default_prim.GetChildren():
        if child.GetTypeName() == "Xform":
            return child.GetPath()
    return dp_path


def resolve_movable_parts(stage, body_path, dp_path, classification):
    """Resolve classified movable parts to prim paths and joint info."""
    movables = {}
    for name, spec in classification["parts"].items():
        cls = spec.get("class", "")
        if not cls.startswith("movable:"):
            continue

        joint_type = cls.split(":")[1]
        axis = spec.get("axis", "Z" if joint_type == "revolute" else "Y")

        path = body_path.AppendChild(name)
        if not stage.GetPrimAtPath(path).IsValid():
            path = dp_path.AppendChild(name)
        if not stage.GetPrimAtPath(path).IsValid():
            for prim in stage.Traverse():
                if prim.GetName() == name and prim.GetTypeName() == "Xform":
                    path = prim.GetPath()
                    break
        if not stage.GetPrimAtPath(path).IsValid():
            print(f"  WARNING: Part '{name}' not found in USD, skipping")
            continue

        movables[name] = {"path": path, "joint": joint_type, "axis": axis}
    return movables


def normalize_to_meters(stage):
    """Convert stage from any unit (cm, mm, etc.) to meters.

    Scales all mesh vertices and translation xformOps by metersPerUnit,
    then sets metersPerUnit to 1.0. This ensures the output USD works
    in any simulator without needing external scale factors.
    """
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    if abs(mpu - 1.0) < 0.001:
        return False

    print(f"\n  NORMALIZE: stage is in {'centimeters' if abs(mpu-0.01)<0.001 else f'units (mpu={mpu})'}, converting to meters")

    for prim in stage.Traverse():
        if prim.GetTypeName() == "Mesh":
            pts_attr = prim.GetAttribute("points")
            if pts_attr and pts_attr.HasValue():
                pts = pts_attr.Get()
                scaled = [Gf.Vec3f(float(p[0])*mpu, float(p[1])*mpu, float(p[2])*mpu) for p in pts]
                pts_attr.Set(scaled)
            ext_attr = prim.GetAttribute("extent")
            if ext_attr and ext_attr.HasValue():
                ext = ext_attr.Get()
                ext_attr.Set([
                    Gf.Vec3f(float(ext[0][0])*mpu, float(ext[0][1])*mpu, float(ext[0][2])*mpu),
                    Gf.Vec3f(float(ext[1][0])*mpu, float(ext[1][1])*mpu, float(ext[1][2])*mpu),
                ])

        if prim.GetTypeName() in ("Xform", "Mesh"):
            xf = UsdGeom.Xformable(prim)
            for op in xf.GetOrderedXformOps():
                if op.IsInverseOp():
                    continue
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    v = op.Get()
                    if v is not None:
                        op.Set(Gf.Vec3d(float(v[0])*mpu, float(v[1])*mpu, float(v[2])*mpu)
                               if isinstance(v, Gf.Vec3d) else
                               Gf.Vec3f(float(v[0])*mpu, float(v[1])*mpu, float(v[2])*mpu))
                elif op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                    m = op.Get()
                    if m is not None:
                        scaled = Gf.Matrix4d(m)
                        scaled.SetRow3(3, Gf.Vec3d(m[3][0]*mpu, m[3][1]*mpu, m[3][2]*mpu))
                        op.Set(scaled)

    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    n_meshes = sum(1 for p in stage.Traverse() if p.GetTypeName() == "Mesh")
    print(f"    Scaled {n_meshes} meshes + xform translations by {mpu}")
    print(f"    metersPerUnit set to 1.0")
    return True


def apply_physics(stage, classification, output_usd, dynamic_body=False,
                  gemini_mass=None, gemini_density=None, gemini_articulation=None):
    """Phase 3: Apply all missing physics based on classification."""
    default_prim = stage.GetDefaultPrim()
    dp_path = default_prim.GetPath()

    normalize_to_meters(stage)
    mpu = 1.0

    # Strip existing physics
    n_j, n_a, n_m = strip_existing_physics(stage)
    if n_j + n_a + n_m > 0:
        print(f"\n  STRIPPED: {n_j} joints, {n_a} physics attrs, {n_m} materials")

    body_path = resolve_body_xform(stage, default_prim, classification["body"])
    movables = resolve_movable_parts(stage, body_path, dp_path, classification)

    # Guard: skip movables nested inside other movables (physically wrong —
    # they'd get jointed to body instead of their parent movable)
    movable_path_strs = {str(info["path"]) for info in movables.values()}
    nested = [name for name, info in movables.items()
              if any(str(info["path"]).startswith(mp + "/")
                     for mp in movable_path_strs - {str(info["path"])})]
    if nested:
        print(f"\n  NESTED MOVABLES (treating as structural — move with parent):")
        for name in nested:
            print(f"    {name}")
            del movables[name]

    print(f"\n  Body: {body_path}")
    print(f"  Movable parts: {len(movables)}")
    for name, info in movables.items():
        print(f"    {name}: {info['joint']} (axis={info['axis']})")

    # Save joint anchors BEFORE reparent — reparenting clears pivot xformOps
    saved_anchors = {}
    body_bbox = mesh_world_bbox(stage, body_path)
    for name, info in movables.items():
        saved_anchors[name] = get_joint_anchor_world(stage, info["path"])
        anchor = saved_anchors[name]
        # Fallback: if anchor is at origin (no pivot xformOp found), compute from
        # body bbox edge. For prismatic joints: anchor at the body face nearest the
        # movable part (the slide start point). For revolute: use movable bbox edge.
        is_zero = all(abs(float(v)) < 1e-6 for v in anchor)
        if is_zero and body_bbox:
            part_bbox = mesh_world_bbox(stage, info["path"])
            if part_bbox:
                axis = info.get("axis", "Y")
                axis_idx = {"X": 0, "Y": 1, "Z": 2}.get(axis, 1)
                jtype = info.get("joint", "prismatic")
                if jtype == "prismatic":
                    # For prismatic: anchor at body edge nearest the part's
                    # jaw/face (the max extent on slide axis). This is where
                    # the slide starts (q=0 position).
                    part_jaw = part_bbox[1][axis_idx]
                    dist_to_min = abs(part_jaw - body_bbox[0][axis_idx])
                    dist_to_max = abs(part_jaw - body_bbox[1][axis_idx])
                    edge = body_bbox[1][axis_idx] if dist_to_max < dist_to_min else body_bbox[0][axis_idx]
                else:
                    # For revolute: anchor at body edge nearest the part center
                    part_center = (part_bbox[0][axis_idx] + part_bbox[1][axis_idx]) / 2
                    body_center = (body_bbox[0][axis_idx] + body_bbox[1][axis_idx]) / 2
                    edge = body_bbox[0][axis_idx] if part_center < body_center else body_bbox[1][axis_idx]
                fallback = list(anchor)
                fallback[axis_idx] = edge
                # Center on other axes
                for i in range(3):
                    if i != axis_idx:
                        fallback[i] = (part_bbox[0][i] + part_bbox[1][i]) / 2
                saved_anchors[name] = Gf.Vec3d(*fallback)
                print(f"    anchor {name}: ({fallback[0]:.4f}, {fallback[1]:.4f}, {fallback[2]:.4f}) (fallback from body edge)")
            else:
                print(f"    anchor {name}: ({anchor[0]:.4f}, {anchor[1]:.4f}, {anchor[2]:.4f}) (zero — no fallback)")
        else:
            print(f"    anchor {name}: ({anchor[0]:.4f}, {anchor[1]:.4f}, {anchor[2]:.4f})")

    # --- C4: Flatten hierarchy ---
    paths_to_move = []
    for info in movables.values():
        if info["path"].GetParentPath() != dp_path:
            paths_to_move.append(info["path"])
    if paths_to_move:
        print(f"\n  REPARENT: {len(paths_to_move)} movable parts -> siblings of body")
        moved = reparent_prims_preserve_world_xform(stage, paths_to_move, dp_path)
        for old, new in moved.items():
            print(f"    {old} -> {new}")
        for name in movables:
            old_p = str(movables[name]["path"])
            if old_p in moved:
                movables[name]["path"] = Sdf.Path(moved[old_p])

    # --- Wheel structural split (fixer/body/bolts -> body) ---
    wheel_moved = split_wheel_structural_parts(stage, movables, body_path)
    if wheel_moved:
        print(f"\n  WHEEL SPLIT: {len(wheel_moved)} structural meshes -> body")
        for old, new in wheel_moved.items():
            print(f"    {old} -> {new}")
        for name, info in movables.items():
            if info["joint"] == "continuous":
                bbox = mesh_world_bbox(stage, info["path"])
                if bbox:
                    tire_center = Gf.Vec3d(
                        (bbox[0][0] + bbox[1][0]) / 2,
                        (bbox[0][1] + bbox[1][1]) / 2,
                        (bbox[0][2] + bbox[1][2]) / 2)
                    saved_anchors[name] = tire_center
                    size_x = abs(bbox[1][0] - bbox[0][0])
                    size_y = abs(bbox[1][1] - bbox[0][1])
                    detected_axis = "Y" if size_y < size_x else "X"
                    if detected_axis != info["axis"]:
                        print(f"    axis override {name}: {info['axis']} -> {detected_axis} (tire X={size_x:.4f} Y={size_y:.4f})")
                        info["axis"] = detected_axis
                    print(f"    anchor {name} (tire center): ({tire_center[0]:.4f}, {tire_center[1]:.4f}, {tire_center[2]:.4f})")

    # --- C1: Rigid Bodies + Mass ---
    print(f"\n  RIGID BODIES:")
    # Graspable props: if no movable parts and object is small (<3kg estimated),
    # make body dynamic so the robot can pick it up. Large furniture stays kinematic.
    has_movables = len(movables) > 0
    if not has_movables and not dynamic_body:
        est_mass = gemini_mass  # Use Gemini mass if available
        if not est_mass:
            est_mass = estimate_mass_from_mesh(stage, body_path, density=500)
        if not est_mass:
            body_bbox_check = mesh_world_bbox(stage, body_path)
            est_mass = estimate_mass(body_bbox_check, mpu, density=500) if body_bbox_check else 999
        if est_mass < 3.0:
            dynamic_body = True
            print(f"    (small object {est_mass:.2f}kg, no joints — auto-dynamic for grasping)")
    body_kinematic = not dynamic_body
    apply_rigid_body(stage, body_path, kinematic=body_kinematic, dynamic_body=dynamic_body)
    body_bbox = mesh_world_bbox(stage, body_path)

    # Mass estimation: Gemini total → skill-based part masses → body gets remainder.
    # V13: Use skill-recommended mass ranges for known part types (wheels, doors,
    # drawers) instead of volume ratio. Volume ratio gives wheels too much mass
    # because wheel meshes (fixer+bolts+body+disc+tire) are disproportionately large.
    use_density = gemini_density if gemini_density else (80.0 if dynamic_body else 600.0)

    # Skill-recommended mass per joint type (from simready-joint-params)
    SKILL_MASS = {
        "continuous": 0.5,    # cart/caster wheel: 0.2-1.0kg, use 0.5
        "revolute":   5.0,    # door: 2-15kg, use 5.0 as default
        "prismatic":  2.0,    # drawer: 0.5-5kg, use 2.0 as default
        "fixed":      1.0,
    }

    if gemini_mass:
        # Step 1: Assign skill-recommended mass to each part
        part_masses = {}
        total_parts_mass = 0
        for name, info in movables.items():
            skill_mass = SKILL_MASS.get(info["joint"], 1.0)
            part_masses[name] = skill_mass
            total_parts_mass += skill_mass

        # Step 2: If parts would take more than 80% of total, scale them down
        max_parts_fraction = 0.4  # parts get at most 40% of total mass
        if total_parts_mass > gemini_mass * max_parts_fraction:
            scale = (gemini_mass * max_parts_fraction) / total_parts_mass
            for name in part_masses:
                part_masses[name] *= scale
            total_parts_mass = sum(part_masses.values())

        # Step 3: Body gets the remainder
        body_mass = gemini_mass - total_parts_mass
        body_mass = max(1.0, body_mass)  # body always at least 1kg
        mass_method = "gemini+skill"
    else:
        body_mass_mesh = estimate_mass_from_mesh(stage, body_path, density=use_density)
        body_mass_bbox = estimate_mass(body_bbox, mpu, density=use_density)
        if body_mass_mesh:
            body_mass = body_mass_mesh
            mass_method = "mesh_vol"
        else:
            body_mass = body_mass_bbox
            mass_method = "bbox"
    if dynamic_body and mass_method not in ("gemini+skill", "gemini"):
        body_mass = max(5.0, min(100.0, body_mass))
    apply_mass(stage, body_path, body_mass)
    body_mode = "dynamic" if dynamic_body else "kinematic"
    print(f"    body: {body_mode}, mass={body_mass:.1f}kg ({mass_method})")

    # Per-part mass
    part_density = gemini_density if gemini_density else 500.0
    for name, info in movables.items():
        path = info["path"]
        apply_rigid_body(stage, path)
        bbox = mesh_world_bbox(stage, path)

        if gemini_mass:
            mass = part_masses.get(name, 1.0)
            m_method = "gemini+skill"
        else:
            mass_mesh = estimate_mass_from_mesh(stage, path, density=part_density)
            mass_bbox = estimate_mass(bbox, mpu, density=part_density)
            mass = mass_mesh if mass_mesh else mass_bbox
            m_method = "mesh_vol" if mass_mesh else "bbox"

        if m_method != "gemini+skill":
            clamp = MASS_CLAMPS.get(info["joint"], (0.1, 50.0))
            mass = max(clamp[0], min(clamp[1], mass))
        apply_mass(stage, path, mass)
        print(f"    {name}: dynamic, mass={mass:.2f}kg ({m_method})")

    # --- C2: Collision Shapes ---
    print(f"\n  COLLIDERS:")
    n_body_col, n_body_decomp = apply_collision_q1(stage, body_path, is_body=True)
    total_decomp = n_body_decomp
    print(f"    body: {n_body_col} colliders ({n_body_decomp} decomp)")

    for name, info in movables.items():
        is_wheel = info["joint"] == "continuous"
        if is_wheel:
            n_col = apply_collision_wheels(stage, info["path"])
            n_d = n_col
        else:
            n_col, n_d = apply_collision_q1(stage, info["path"], is_body=False)
        total_decomp += n_d
        print(f"    {name}: {n_col} colliders ({n_d} decomp)")

    if total_decomp > MAX_DECOMP_BUDGET:
        print(f"    WARNING: {total_decomp} decomp exceeds budget of {MAX_DECOMP_BUDGET}")

    # --- C5: Joints ---
    print(f"\n  JOINTS:")
    joints_scope = Sdf.Path(f"{dp_path}/joints")
    if not stage.GetPrimAtPath(joints_scope).IsValid():
        UsdGeom.Scope.Define(stage, joints_scope)

    for name, info in movables.items():
        path = info["path"]
        jtype = info["joint"]
        axis = info["axis"]
        joint_path = joints_scope.AppendChild(f"{name}_joint")

        anchor = saved_anchors[name]
        lp0 = world_point_to_local(stage, body_path, anchor)
        lp0_f = Gf.Vec3f(float(lp0[0]), float(lp0[1]), float(lp0[2]))
        lp1 = world_point_to_local(stage, path, anchor)
        lp1_f = Gf.Vec3f(float(lp1[0]), float(lp1[1]), float(lp1[2]))

        if jtype == "revolute":
            hinge = detect_hinge_edge(stage, path, anchor_world=anchor)
            make_revolute_joint(stage, joint_path, body_path, path,
                                lp0_f, lp1_f, axis=axis, hinge_edge=hinge)
            print(f"    RevoluteJoint  {name}  axis={axis} hinge={hinge}")
        elif jtype == "prismatic":
            bbox = mesh_world_bbox(stage, path)
            axis_idx = {"X": 0, "Y": 1, "Z": 2}[axis]
            part_depth = abs(bbox[1][axis_idx] - bbox[0][axis_idx]) if bbox else 0.4
            body_depth = abs(body_bbox[1][axis_idx] - body_bbox[0][axis_idx]) if body_bbox else part_depth
            # Use the overlap region between part and body on the slide axis.
            # For overlapping parts (caliper blade over ruler), the useful travel
            # is how far the part can slide before exiting the body.
            if bbox and body_bbox:
                overlap_min = max(bbox[0][axis_idx], body_bbox[0][axis_idx])
                overlap_max = min(bbox[1][axis_idx], body_bbox[1][axis_idx])
                overlap = max(0, overlap_max - overlap_min)
                if overlap > 0 and overlap < part_depth * 0.95:
                    # Part overlaps body partially (caliper, sliding tool) —
                    # use full overlap as travel (not 85%), since the useful
                    # range IS the overlap region
                    depth = overlap
                else:
                    depth = min(part_depth, body_depth)
            else:
                depth = min(part_depth, body_depth)
            # If drawer has rail mechanism meshes, limit travel to maintain
            # rail-track overlap (rail must not fully exit the body track).
            has_rail = False
            drawer_prim = stage.GetPrimAtPath(path)
            if drawer_prim:
                for child in Usd.PrimRange(drawer_prim):
                    if child.IsA(UsdGeom.Mesh) and any(
                            kw in child.GetName().lower() for kw in _DRAWER_RAIL_KEYWORDS):
                        has_rail = True
                        break
            is_overlap_travel = (bbox and body_bbox and overlap > 0 and overlap < part_depth * 0.95)
            if has_rail:
                travel = depth * 0.45   # ~45% of total depth keeps rail overlapped
                print(f"    (rail detected — limiting travel to {travel:.3f}m for overlap)")
            elif is_overlap_travel:
                travel = depth  # overlap IS the full useful range, no 85% reduction
                print(f"    (overlap-based travel: {travel:.3f}m = full ruler/slide range)")
            else:
                travel = depth * 0.85
            # Detect slider vs drawer: a slider (caliper, measuring tool)
            # spans nearly the FULL body length on the slide axis (>70%).
            # A drawer is much shorter than the body. Sliders need
            # bidirectional limits; drawers need one-directional.
            is_slider = False
            if bbox and body_bbox:
                part_extent = abs(bbox[1][axis_idx] - bbox[0][axis_idx])
                body_extent = abs(body_bbox[1][axis_idx] - body_bbox[0][axis_idx])
                if body_extent > 0:
                    span_ratio = part_extent / body_extent
                    if span_ratio > 0.9:
                        is_slider = True

            if is_slider:
                # V13: Slider travels full body length, both directions.
                # The driving part slides along the body (ruler). It can go
                # left until its outer end (part_hi) reaches body_lo, and
                # right until its inner end (part_lo) reaches body_hi.
                if bbox and body_bbox:
                    body_lo = body_bbox[0][axis_idx]
                    body_hi = body_bbox[1][axis_idx]
                    part_lo = bbox[0][axis_idx]  # inner end (jaw)
                    part_hi = bbox[1][axis_idx]  # outer end

                    # Slide right: inner end (part_lo) reaches body far end
                    upper_m = body_hi - part_lo
                    # Slide left: outer end (part_hi) reaches body near end
                    lower_m = body_lo - part_hi

                    body_len = body_hi - body_lo
                    travel = upper_m - lower_m
                    print(f"    (slider — full ruler travel both ways: [{lower_m:.3f}, {upper_m:.3f}]m = {travel*100:.0f}cm, body={body_len*100:.0f}cm)")
                else:
                    body_len = abs(body_bbox[1][axis_idx] - body_bbox[0][axis_idx]) if body_bbox else depth
                    lower_m = -body_len * 0.45
                    upper_m = body_len * 0.45
                    print(f"    (slider — geometry fallback: [{lower_m:.3f}, {upper_m:.3f}]m)")
            elif bbox and body_bbox:
                # Drawer: one direction, face toward body exterior
                body_center_ax = (body_bbox[0][axis_idx] + body_bbox[1][axis_idx]) / 2
                drawer_center_ax = (bbox[0][axis_idx] + bbox[1][axis_idx]) / 2
                if drawer_center_ax < body_center_ax:
                    lower_m, upper_m = -travel, 0.0
                else:
                    lower_m, upper_m = 0.0, travel
            else:
                lower_m, upper_m = 0.0, travel
            make_prismatic_joint(stage, joint_path, body_path, path,
                                 lp0_f, lp1_f, axis=axis,
                                 lower_m=lower_m, upper_m=upper_m)
            print(f"    PrismaticJoint {name}  axis={axis} travel=[{lower_m:.3f}, {upper_m:.3f}]m")
        elif jtype == "continuous":
            make_continuous_joint(stage, joint_path, body_path, path,
                                  lp0_f, lp1_f, axis=axis)
            print(f"    ContinuousJoint {name}  axis={axis}")
        elif jtype == "fixed":
            make_fixed_joint(stage, joint_path, body_path, path, lp0_f, lp1_f)
            print(f"    FixedJoint      {name}")

    # --- C3 + C6: Friction ---
    print(f"\n  FRICTION:")
    movable_paths = [info["path"] for info in movables.values()]
    handle_meshes = find_handle_meshes(stage, movable_paths)
    n_grip, n_body_fric = wire_friction(stage, dp_path, handle_meshes)
    print(f"    GripMaterial on {n_grip} handle meshes")
    print(f"    Physics material binding on {n_body_fric} body meshes")

    # --- V13: ArticulationRootAPI on default prim ---
    # Matches Lightwheel + Palatial approach: placed on the common ancestor
    # Xform (default prim) that contains all rigid bodies. Enables:
    # - shift+drag in Isaac Sim viewport
    # - ArticulationCfg drive targets for RL
    # - reduced-coordinate solver (more stable)
    print(f"\n  ARTICULATION:")
    dp_spec = stage.GetRootLayer().GetPrimAtPath(dp_path)
    schemas = dp_spec.GetInfo("apiSchemas")
    items = list(schemas.prependedItems) if schemas and hasattr(schemas, "prependedItems") else []
    if "PhysicsArticulationRootAPI" not in items:
        items.append("PhysicsArticulationRootAPI")
        new_list = Sdf.TokenListOp()
        new_list.prependedItems = items
        dp_spec.SetInfo("apiSchemas", new_list)
    print(f"    ArticulationRootAPI on '{default_prim.GetName()}' (default prim)")

    # --- Save ---
    stage.GetRootLayer().Save()
    print(f"\n  SAVED: {output_usd}")


def run(input_usd, fix=False, provider="anthropic", model=None, output_dir=None,
        classify_json=None, dynamic_body=False, object_json=None):
    """Main entry point: audit, optionally classify + fix."""
    # Load Gemini object understanding if provided
    gemini_mass = None
    gemini_density = None
    gemini_articulation = {}  # part_name → {range_meters, limits_bidirectional}
    if object_json and os.path.exists(object_json):
        with open(object_json) as f:
            obj_data = json.load(f)
        gemini_mass = obj_data.get("estimated_mass_kg")
        gemini_density = obj_data.get("material_density_kg_m3")
        if gemini_mass:
            print(f"  Gemini mass: {gemini_mass}kg, density: {gemini_density} kg/m³")
        # V13: Extract articulation ranges from Gemini (range_meters per part)
        for ap in obj_data.get("movable_parts", []):
            pname = ap.get("name", "")
            rm = ap.get("range_meters")
            if pname and rm and rm > 0:
                gemini_articulation[pname] = {
                    "range_meters": rm,
                    "limits_bidirectional": ap.get("limits_bidirectional", False),
                }
        if gemini_articulation:
            print(f"  Gemini articulation: {len(gemini_articulation)} parts with range data")
    print(f"\n{'='*60}")
    print(f"  make_simready (V8)")
    print(f"{'='*60}")
    print(f"  Input: {input_usd}")
    print(f"  Mode:  {'AUDIT + FIX' if fix else 'AUDIT ONLY'}")

    stage = Usd.Stage.Open(input_usd)

    # Phase 1: Audit
    results = audit(stage)
    print_audit(results, label="AUDIT (current state)")

    all_pass = all(r["pass"] for r in results.values())
    if all_pass:
        print(f"\n  Asset is already SimReady. Nothing to do.")
        return input_usd

    if not fix:
        print(f"\n  Run with --fix to apply missing physics.")
        return None

    # Phase 2: Classify
    if classify_json:
        with open(classify_json) as f:
            classification = json.load(f)
        print(f"\n  CLASSIFICATION (from file):")
        print(f"    body: {classification['body']}")
        for name, spec in classification.get("parts", {}).items():
            cls = spec.get("class", "?")
            axis = spec.get("axis", "")
            axis_str = f" axis={axis}" if axis else ""
            print(f"    {name:40s} -> {cls}{axis_str}")
    else:
        classification = classify_parts(stage, provider=provider, model=model)

    # Phase 3: Apply
    out_dir = output_dir or os.path.join(os.path.dirname(input_usd), "simready_out")
    os.makedirs(out_dir, exist_ok=True)
    basename = os.path.splitext(os.path.basename(input_usd))[0]
    # Single output name for entire SimReady fleet (fridges B–F): always {name}_physics.usd
    output_usd = os.path.join(out_dir, f"{basename}_physics.usd")
    shutil.copy2(input_usd, output_usd)

    src_dir = os.path.dirname(input_usd)
    tex_src = os.path.join(src_dir, "Textures")
    tex_dst = os.path.join(out_dir, "Textures")
    if os.path.isdir(tex_src) and not os.path.isdir(tex_dst):
        shutil.copytree(tex_src, tex_dst)
        print(f"  Copied Textures/")

    out_stage = Usd.Stage.Open(output_usd)
    apply_physics(out_stage, classification, output_usd, dynamic_body=dynamic_body,
                  gemini_mass=gemini_mass, gemini_density=gemini_density,
                  gemini_articulation=gemini_articulation)

    # Re-audit
    final_stage = Usd.Stage.Open(output_usd)
    final_results = audit(final_stage)
    print_audit(final_results, label="AUDIT (after fix)")

    # Summary
    n_rigid = sum(1 for p in final_stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    n_col = sum(1 for p in final_stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
    n_joints = sum(1 for p in final_stage.Traverse() if "Joint" in p.GetTypeName())
    print(f"\n  SUMMARY: {n_rigid} rigid bodies, {n_col} colliders, {n_joints} joints")

    # Ready-to-run commands
    abs_output = os.path.abspath(output_usd)
    print(f"\n  Run commands:")
    print(f"    # Franka teleop")
    print(f"    ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent_cinematic.py \\")
    print(f"      --asset {abs_output} --device cpu")
    print(f"\n{'='*60}")

    return output_usd


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="SimReady V8: audit + classify + fix")
    ap.add_argument("--input", required=True, help="Input USD file")
    ap.add_argument("--fix", action="store_true", help="Apply missing physics (default: audit only)")
    ap.add_argument("--output-dir", default=None, help="Output directory (default: simready_out/ next to input)")
    ap.add_argument("--provider", default="anthropic", choices=["openai", "anthropic"],
                    help="LLM provider for classification (default: anthropic)")
    ap.add_argument("--model", default=None, help="LLM model override")
    ap.add_argument("--classify-json", default=None,
                    help="Pre-made classification JSON (skips LLM call)")
    ap.add_argument("--object-json", default=None,
                    help="Object understanding JSON from Gemini (mass, material, density)")
    ap.add_argument("--dynamic", action="store_true",
                    help="Dynamic main body (e.g. trolley drag tests). Same *_physics.usd path. Not the fridge B–F recipe — omit for refrigerators.")
    args = ap.parse_args()

    input_path = os.path.abspath(args.input)
    if not os.path.isfile(input_path):
        print(f"ERROR: USD not found: {input_path}")
        sys.exit(1)

    run(input_path, fix=args.fix, provider=args.provider,
        model=args.model, output_dir=args.output_dir,
        classify_json=args.classify_json, dynamic_body=args.dynamic,
        object_json=args.object_json)

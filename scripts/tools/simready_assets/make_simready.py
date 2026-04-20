#!/usr/bin/env python3
"""
make_simready.py  (V13 — forked from V11, originally authored in V8 era)

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

def audit(stage, classification=None):
    """Check all 7 SimReady criteria. Returns dict of criterion -> {pass, details}.

    When `classification` is provided, C5 also verifies every classified
    movable part produced a joint — catches serial-chain collapse where the
    classifier declared N links but only 1 joint survived (e.g. boom arm
    before the declared-parent fix).
    """
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
    art_root_paths = []

    for prim in stage.Traverse():
        apis = [str(s) for s in prim.GetAppliedSchemas()]

        if "PhysicsArticulationRootAPI" in apis:
            art_root_paths.append(str(prim.GetPath()))

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
            # F47: flag zero-thickness collision meshes. Qhull fails on
            # coplanar points, PhysX broadphase gets NaN, asset disappears
            # at sim start. Fix in apply_collision_q1 / apply_collision_wheels:
            # skip meshes where any bbox axis < 1e-6.
            is_degenerate = prim.IsA(UsdGeom.Mesh) and _is_degenerate_mesh(prim)
            colliders.append({
                "name": prim.GetName(),
                "approx": approx_val,
                "has_physics_mat_binding": has_binding,
                "degenerate": is_degenerate,
            })

        if prim.IsA(UsdPhysics.Joint):
            lp0 = prim.GetAttribute("physics:localPos0")
            lp1 = prim.GetAttribute("physics:localPos1")
            lp0_val = lp0.Get() if lp0 and lp0.HasValue() else None
            lp1_val = lp1.Get() if lp1 and lp1.HasValue() else None
            lp0_zero = lp0_val is not None and all(abs(float(v)) < 1e-6 for v in lp0_val)
            lp1_zero = lp1_val is not None and all(abs(float(v)) < 1e-6 for v in lp1_val)

            # Check joint anchor consistency: localPos0 (in body0's frame)
            # and localPos1 (in body1's frame) should map to the SAME world
            # point. If they don't, the joint springs at physics init and
            # parts detach (symptom: wheels flying off a trolley).
            anchor_miss_m = None
            try:
                body0_rel = prim.GetRelationship("physics:body0")
                body1_rel = prim.GetRelationship("physics:body1")
                t0 = body0_rel.GetTargets() if body0_rel else []
                t1 = body1_rel.GetTargets() if body1_rel else []
                if t0 and t1 and lp0_val is not None and lp1_val is not None:
                    body0_prim = stage.GetPrimAtPath(t0[0])
                    body1_prim = stage.GetPrimAtPath(t1[0])
                    if body0_prim and body1_prim:
                        xf0 = UsdGeom.Xformable(body0_prim).ComputeLocalToWorldTransform(0)
                        xf1 = UsdGeom.Xformable(body1_prim).ComputeLocalToWorldTransform(0)
                        w0 = xf0.Transform(Gf.Vec3d(*[float(x) for x in lp0_val]))
                        w1 = xf1.Transform(Gf.Vec3d(*[float(x) for x in lp1_val]))
                        anchor_miss_m = float((w0 - w1).GetLength())
            except Exception:
                pass

            # Continuous = revolute with effectively unbounded limits (wheels/casters).
            # V13 creates these with ±9999 limits in make_continuous_joint.
            is_continuous = False
            if prim.GetTypeName() == "PhysicsRevoluteJoint":
                lo = prim.GetAttribute("physics:lowerLimit")
                hi = prim.GetAttribute("physics:upperLimit")
                lo_v = lo.Get() if lo and lo.HasValue() else None
                hi_v = hi.Get() if hi and hi.HasValue() else None
                if lo_v is not None and hi_v is not None and abs(float(hi_v) - float(lo_v)) > 1000:
                    is_continuous = True
            # Prismatic lower/upper + axis for drawer-direction audit.
            pris_lo = pris_hi = None
            axis_str = None
            if prim.GetTypeName() == "PhysicsPrismaticJoint":
                lo = prim.GetAttribute("physics:lowerLimit")
                hi = prim.GetAttribute("physics:upperLimit")
                ax = prim.GetAttribute("physics:axis")
                pris_lo = float(lo.Get()) if lo and lo.HasValue() else None
                pris_hi = float(hi.Get()) if hi and hi.HasValue() else None
                axis_str = ax.Get() if ax and ax.HasValue() else None
            # F49: a world-anchor FixedJoint has body0 empty (= world) and
            # anchors at (0,0,0) by design. Mark it so downstream checks
            # (zero-anchor, chain-collapse) can exclude it.
            is_world_anchor = (
                prim.GetTypeName() == "PhysicsFixedJoint"
                and not (prim.GetRelationship("physics:body0").GetTargets()
                         if prim.GetRelationship("physics:body0") else [])
            )
            joints.append({
                "name": prim.GetName(),
                "type": prim.GetTypeName(),
                "both_anchors_zero": lp0_zero and lp1_zero,
                "anchor_miss_m": anchor_miss_m,
                "is_world_anchor": is_world_anchor,
                "body0_path": str(t0[0]) if t0 else None,
                "body1_path": str(t1[0]) if t1 else None,
                "is_continuous": is_continuous,
                "pris_lo": pris_lo,
                "pris_hi": pris_hi,
                "axis": axis_str,
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
    # Per-rigid-body coverage: every rigid body with descendant meshes must
    # have ≥1 descendant collider. Bodies with NO descendant meshes (pure
    # coordinate-frame anchors for symmetric-pivot instruments like Clamps
    # where the central pivot has no geometry of its own) are allowed — they
    # serve only as joint anchors, not collision surfaces.
    bodies_without_colliders = []
    for rb in rigid_bodies:
        rb_prim = stage.GetPrimAtPath(rb["path"])
        if not rb_prim:
            continue
        has_col = False
        has_mesh = False
        for desc in Usd.PrimRange(rb_prim):
            if desc.IsA(UsdGeom.Mesh):
                has_mesh = True
            if desc.HasAPI(UsdPhysics.CollisionAPI):
                has_col = True
                break
        if has_mesh and not has_col:
            bodies_without_colliders.append(rb["path"])
    if bodies_without_colliders:
        c2_pass = False
        c2_detail += f" — {len(bodies_without_colliders)} rigid body(s) have NO colliders: {bodies_without_colliders}"
    # F47: zero-thickness collision meshes (flat decals, stickers, labels)
    # crash qhull at physics init. PhysX reports "Illegal BroadPhaseUpdateData"
    # and every rigid body's transform becomes "Invalid". Asset disappears
    # from sim. Fix: apply_collision_q1 / apply_collision_wheels must skip
    # meshes with any bbox axis < 1e-6 (see _is_degenerate_mesh).
    degenerate_colliders = [c["name"] for c in colliders if c.get("degenerate")]
    if degenerate_colliders:
        c2_pass = False
        c2_detail += (f" — F47: {len(degenerate_colliders)} zero-thickness "
                      f"collider(s) will crash qhull/PhysX broadphase "
                      f"(e.g. {degenerate_colliders[0]}); fix in "
                      f"apply_collision_q1 via _is_degenerate_mesh skip")
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
    # ArticulationRootAPI placement discipline (requested by Newton feedback,
    # 2026-04-18). V13 rule: applied to the default-prim Xform that contains
    # all rigid bodies, and nowhere else. Misplaced roots on a child prim
    # break Newton's articulation parser and confuse Isaac Sim's reduced-
    # coordinate solver.
    dp_str = str(dp_path)
    dp_has_root = dp_str in art_root_paths
    misplaced_roots = [p for p in art_root_paths if p != dp_str]
    if not dp_has_root:
        c4_pass = False
        c4_detail += (" — MISSING ArticulationRootAPI on default prim "
                      f"({dp_str}); fix in apply_physics (search for "
                      f"PhysicsArticulationRootAPI).")
    if misplaced_roots:
        c4_pass = False
        c4_detail += (f" — {len(misplaced_roots)} MISPLACED "
                      f"ArticulationRootAPI(s) (e.g. {misplaced_roots[0]}); "
                      f"V13 rule is default-prim-only.")
    results["C4 Flat Hierarchy"] = {"pass": c4_pass, "detail": c4_detail, "na": c4_na if len(rigid_bodies) <= 1 else False}

    # C5: Joints (existence + anchor validity + wheel split + wheel-in-footprint)
    has_movables = len(rigid_bodies) > 1
    if has_movables:
        enough_joints = len(joints) >= len(rigid_bodies) - 1
        # F14b: localPos0=localPos1=(0,0,0) is only a failure when we cannot
        # verify world-space anchor convergence. If anchor_miss_m is computable
        # and small, both body origins ARE at the pivot (legitimate for
        # symmetric-pivot instruments: scissors, clamps, pliers, forceps).
        # If anchor_miss_m is large, misaligned_joints catches it below.
        zero_anchor_joints = [
            j for j in joints
            if j.get("both_anchors_zero", False) and j.get("anchor_miss_m") is None
            and not j.get("is_world_anchor", False)
        ]
        # Joints where localPos0 and localPos1 don't map to the same world
        # point — parts will spring/detach at physics init.
        misaligned_joints = [j for j in joints
                             if j.get("anchor_miss_m") is not None
                             and j["anchor_miss_m"] > 0.01]

        # Wheel-split leak: a continuous-joint rigid body must not retain
        # structural-named descendants (frame/caps/bracket/brake/etc.). If
        # split_wheel_structural_parts failed silently, the bracket rotates with
        # the tire. Seen on EmergencyTrolley_A01_01 when structural parts were
        # wrapped in Xforms instead of direct Meshes.
        # Skip 2-DOF caster bracket joints — the bracket legitimately contains
        # mount / bolt / body meshes as structural members of its swivel body.
        # The audit heuristic (revolute + wide limits → is_continuous) fires on
        # bracket swivels with ±9999° limits; detect them by the "_bracket"
        # naming convention used by split_wheel_structural_parts.
        wheel_split_leaks = []
        for j in joints:
            if not j.get("is_continuous"):
                continue
            b1 = j.get("body1_path")
            if not b1:
                continue
            if "_bracket" in b1.lower() or "_bracket" in (j.get("name") or "").lower():
                continue
            # 2-DOF caster tires hinge off a bracket (not the chassis body).
            # The tire assembly legitimately contains hub/body/bolt as
            # rigid members that roll with the rubber. Skip the leak check
            # for any continuous joint whose body0 is a bracket.
            b0 = j.get("body0_path") or ""
            if "_bracket" in b0.lower():
                continue
            # Swivel seats and other non-wheel continuous joints legitimately
            # carry body/mount/bolt meshes; only check joints whose body1
            # looks like a wheel by name.
            b1_nm_lower = b1.lower()
            if not any(kw in b1_nm_lower for kw in ("wheel", "caster", "roller", "tire")):
                continue
            b1_prim = stage.GetPrimAtPath(b1)
            if not b1_prim:
                continue
            for desc in Usd.PrimRange(b1_prim):
                if desc == b1_prim:
                    continue
                n = desc.GetName().lower()
                if any(kw in n for kw in WHEEL_STRUCTURAL_KEYWORDS) and "tire" not in n:
                    wheel_split_leaks.append((j["name"], desc.GetName()))
                    break

        # Wheel outside chassis: each continuous-joint wheel's mesh centroid
        # should lie within body0's xy footprint (5% margin). If not, reparent
        # or unit conversion corrupted positions. Seen on EmergencyTrolley when
        # USD row-vector matrix order was wrong.
        wheels_out_of_footprint = []
        for j in joints:
            if not j.get("is_continuous"):
                continue
            b0, b1 = j.get("body0_path"), j.get("body1_path")
            if not (b0 and b1):
                continue
            b0p = stage.GetPrimAtPath(b0)
            b1p = stage.GetPrimAtPath(b1)
            if not (b0p and b1p):
                continue
            try:
                bb0 = mesh_world_bbox(stage, b0p.GetPath())
                bb1 = mesh_world_bbox(stage, b1p.GetPath())
            except Exception:
                continue
            if not (bb0 and bb1):
                continue
            wc = ((bb1[0][0]+bb1[1][0])/2, (bb1[0][1]+bb1[1][1])/2)
            mx = 0.05 * max(abs(bb0[1][0] - bb0[0][0]), abs(bb0[1][1] - bb0[0][1]))
            if not (bb0[0][0] - mx <= wc[0] <= bb0[1][0] + mx
                    and bb0[0][1] - mx <= wc[1] <= bb0[1][1] + mx):
                wheels_out_of_footprint.append((j["name"], wc))

        # Drawer direction: prismatic travel should move the drawer toward the
        # body's exterior face. Comparing in body-local frame so sign matches
        # joint axis direction. Seen on EmergencyTrolley (chassis rot 181.9°):
        # drawers pointed backward out of the chassis.
        backward_drawers = []
        # F46b: honor classify.json axis overrides (+X / -X). When the user
        # explicitly specifies a direction, skip the backward-drawer check
        # for that joint — the user's intent is authoritative.
        explicit_overrides = set()
        if classification is not None:
            for part_name, spec in classification.get("parts", {}).items():
                ax_raw = spec.get("axis", "")
                if isinstance(ax_raw, str) and len(ax_raw) == 2 and ax_raw[0] in "+-":
                    explicit_overrides.add(f"{part_name}_joint")
        for j in joints:
            if j.get("type") != "PhysicsPrismaticJoint":
                continue
            if j.get("name") in explicit_overrides:
                continue
            lo, hi, ax = j.get("pris_lo"), j.get("pris_hi"), j.get("axis")
            if lo is None or hi is None or ax not in ("X", "Y", "Z"):
                continue
            # Skip sliders (bidirectional ~ symmetric limits).
            if lo < -1e-4 and hi > 1e-4:
                continue
            b0, b1 = j.get("body0_path"), j.get("body1_path")
            if not (b0 and b1):
                continue
            b0p = stage.GetPrimAtPath(b0)
            b1p = stage.GetPrimAtPath(b1)
            if not (b0p and b1p):
                continue
            try:
                bb0 = mesh_world_bbox(stage, b0p.GetPath())
                bb1 = mesh_world_bbox(stage, b1p.GetPath())
            except Exception:
                continue
            if not (bb0 and bb1):
                continue
            body_w2l = UsdGeom.Xformable(b0p).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).GetInverse()
            # F46: prefer handle/lock/knob sub-mesh center over drawer-bbox
            # center when available — the handle identifies the opening face
            # on symmetric-bbox drawers (see apply_physics prismatic branch).
            handle_kws = ("handle", "knob", "pull", "lock", "rotor",
                          "grip", "latch")
            handle_center = None
            best_area = 0.0
            for desc in Usd.PrimRange(b1p):
                nm = desc.GetName().lower()
                if not any(kw in nm for kw in handle_kws):
                    continue
                mesh_prims = [desc] if desc.IsA(UsdGeom.Mesh) else _get_all_descendant_meshes(desc)
                hmin = Gf.Vec3d(1e30, 1e30, 1e30)
                hmax = Gf.Vec3d(-1e30, -1e30, -1e30)
                ok = False
                for mp in mesh_prims:
                    pts = mp.GetAttribute("points")
                    if not pts or not pts.HasValue():
                        continue
                    l2w = UsdGeom.Xformable(mp).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default())
                    for pt in pts.Get():
                        wp = l2w.TransformAffine(Gf.Vec3d(float(pt[0]), float(pt[1]), float(pt[2])))
                        hmin = Gf.Vec3d(min(hmin[0], wp[0]), min(hmin[1], wp[1]), min(hmin[2], wp[2]))
                        hmax = Gf.Vec3d(max(hmax[0], wp[0]), max(hmax[1], wp[1]), max(hmax[2], wp[2]))
                        ok = True
                if not ok:
                    continue
                area = (hmax[0]-hmin[0]) * (hmax[1]-hmin[1])
                if area > best_area:
                    best_area = area
                    handle_center = Gf.Vec3d((hmin[0]+hmax[0])/2, (hmin[1]+hmax[1])/2, (hmin[2]+hmax[2])/2)
            dc = body_w2l.TransformAffine(Gf.Vec3d(
                (bb1[0][0]+bb1[1][0])/2, (bb1[0][1]+bb1[1][1])/2, (bb1[0][2]+bb1[1][2])/2))
            bc = body_w2l.TransformAffine(Gf.Vec3d(
                (bb0[0][0]+bb0[1][0])/2, (bb0[0][1]+bb0[1][1])/2, (bb0[0][2]+bb0[1][2])/2))
            idx = {"X":0,"Y":1,"Z":2}[ax]
            if handle_center is not None:
                hc = body_w2l.TransformAffine(handle_center)
                # Opening direction = from drawer center toward handle.
                drawer_offset = hc[idx] - dc[idx]
            else:
                # Fallback: drawer vs body center.
                drawer_offset = dc[idx] - bc[idx]
            travel_sign = 1 if hi > abs(lo) else -1
            if drawer_offset * travel_sign < 0:
                backward_drawers.append(j["name"])

        anchors_ok = (len(zero_anchor_joints) == 0
                      and len(misaligned_joints) == 0
                      and len(wheel_split_leaks) == 0
                      and len(wheels_out_of_footprint) == 0
                      and len(backward_drawers) == 0)
        c5_pass = enough_joints and anchors_ok
        c5_detail = f"{len(joints)} joints for {len(rigid_bodies) - 1} movable parts"
        if not enough_joints:
            c5_detail += " — need more joints"
        if zero_anchor_joints:
            c5_pass = False
            c5_detail += (f" — {len(zero_anchor_joints)} joints have ZERO anchors "
                          f"(localPos0=localPos1=(0,0,0)) AND world-space anchor could not be "
                          f"resolved — check body0/body1 rels are set")
        if misaligned_joints:
            c5_pass = False
            worst = max(j["anchor_miss_m"] for j in misaligned_joints)
            c5_detail += (f" — {len(misaligned_joints)} joint(s) have misaligned anchors "
                          f"(worst: {worst*100:.1f}cm mismatch — parts will fly at physics init)")
        if wheel_split_leaks:
            c5_pass = False
            n, kw = wheel_split_leaks[0]
            c5_detail += (f" — {len(wheel_split_leaks)} continuous-joint wheel(s) still contain "
                          f"structural descendants (e.g. {n} has '{kw}') — bracket will rotate with tire; "
                          f"check split_wheel_structural_parts accepts Xform-wrapped children")
        if wheels_out_of_footprint:
            c5_pass = False
            n, wc = wheels_out_of_footprint[0]
            c5_detail += (f" — {len(wheels_out_of_footprint)} wheel(s) land outside chassis footprint "
                          f"(e.g. {n} at x={wc[0]:+.2f}, y={wc[1]:+.2f}) — check reparent matrix order "
                          f"for USD row-vector convention")
        if backward_drawers:
            c5_pass = False
            c5_detail += (f" — {len(backward_drawers)} drawer(s) open the wrong direction "
                          f"(e.g. {backward_drawers[0]}) — check prismatic direction-selection "
                          f"compares in body-local frame, not world")
        # F40: Implausible prismatic travel on a small asset. A 5mm button on
        # a 15cm tool should not have 60cm travel. Catches cases where the
        # bbox-derived travel over-shot (deeply nested part, inflated bbox)
        # and gemini_articulation wasn't consulted.
        implausible_prismatic = []
        try:
            stage_bb = mesh_world_bbox(stage, dp_path)
        except Exception:
            stage_bb = None
        if stage_bb:
            stage_size = max(
                abs(stage_bb[1][0] - stage_bb[0][0]),
                abs(stage_bb[1][1] - stage_bb[0][1]),
                abs(stage_bb[1][2] - stage_bb[0][2]),
            )
        else:
            stage_size = None
        for j in joints:
            if j.get("type") != "PhysicsPrismaticJoint":
                continue
            lo = j.get("pris_lo")
            hi = j.get("pris_hi")
            if lo is None or hi is None:
                continue
            travel = abs(hi - lo)
            if stage_size and travel > stage_size * 0.5:
                implausible_prismatic.append((j["name"], travel, stage_size))
        if implausible_prismatic:
            c5_pass = False
            n, tr, sz = implausible_prismatic[0]
            c5_detail += (f" — {len(implausible_prismatic)} prismatic joint(s) have implausible "
                          f"travel (e.g. {n}: {tr*100:.0f}cm on a {sz*100:.0f}cm asset) — "
                          f"check gemini_articulation range_meters is passed to apply_physics "
                          f"prismatic branch (F40)")
        # Serial-chain collapse: classifier declared N movables but pipeline
        # produced fewer joints. Root cause: nested movables dropped without
        # declared "parent" chain, or classifier omitted parent field entirely.
        # See usd-physx-schemas: Serial Kinematic Chains.
        if classification is not None:
            # Count canonical "movable:*" AND shorthand aliases ("wheel",
            # "caster") — both are accepted inputs; apply_physics normalizes
            # the shorthand to "movable:continuous" (F48).
            expected_movable = sum(
                1 for spec in classification.get("parts", {}).values()
                if str(spec.get("class", "")).startswith("movable:")
                or str(spec.get("class", "")) in _WHEEL_ALIASES
            )
            if len(joints) < expected_movable:
                c5_pass = False
                c5_detail += (f" — CHAIN COLLAPSE: classifier declared {expected_movable} "
                              f"movable parts but only {len(joints)} joints produced "
                              f"— check parent field in classify.json (see "
                              f"usd-physx-schemas: Serial Kinematic Chains)")
            # F48: flag any class value outside the accepted set. Catches
            # classifier drift early — unknown values otherwise fall through
            # the apply_physics dispatch and silently become structural.
            unknown_classes = []
            for pname, spec in classification.get("parts", {}).items():
                cv = str(spec.get("class", ""))
                if cv in _ACCEPTED_CLASSES:
                    continue
                if cv in _WHEEL_ALIASES:
                    continue
                unknown_classes.append((pname, cv))
            if unknown_classes:
                c5_pass = False
                nm, cv = unknown_classes[0]
                c5_detail += (f" — F48: {len(unknown_classes)} part(s) have unknown "
                              f"class value (e.g. {nm!r}={cv!r}); accepted: "
                              f"{_ACCEPTED_CLASSES + _WHEEL_ALIASES}. Fix in "
                              f"apply_physics via _normalize_class_aliases or "
                              f"update the classifier prompt.")
    else:
        c5_pass = True
        c5_detail = "no movable parts — joints N/A"
    results["C5 Joints"] = {"pass": c5_pass, "detail": c5_detail, "na": not has_movables}

    # C6: Joint Drives + stiffness/damping validation (F18, F32)
    # FixedJoints are 0-DOF and do not need drives — exclude them from the
    # expected count (required for welded structural links in serial chains,
    # e.g. plate1 rigidly attached to a column so it forms a non-adjacent
    # sibling for the sliding plate2 to collide with).
    if joints:
        joints_needing_drive = [j for j in joints if j.get("type") != "PhysicsFixedJoint"]
        c6_pass = len(drives) >= len(joints_needing_drive)
        c6_detail = f"{len(drives)} drives for {len(joints_needing_drive)} DOF joints ({len(joints)} total, {len(joints)-len(joints_needing_drive)} fixed)"
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

    # C7: Clean Asset (no scene, no contactOffset, meters, no residual xformOp:scale)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    c7_issues = []
    if has_physics_scene:
        c7_issues.append("PhysicsScene found")
    if has_contact_offset:
        c7_issues.append("contactOffset found")
    if abs(mpu - 1.0) > 0.01:
        unit_name = "centimeters" if abs(mpu - 0.01) < 0.001 else f"mpu={mpu}"
        c7_issues.append(f"stage in {unit_name}, not meters")
    # F43 regression guard: any non-unit xformOp:scale in the output means
    # bake_xform_scales either didn't run or failed. A pivot-sandwich scale
    # (scale between translate:pivot and !invert!translate:pivot) silently
    # shifts geometry by (1-s)·pivot when baked naively — wheels end up
    # orbiting an offset hinge. See usd-physx-schemas §"baking a non-unit
    # xformOp:scale inside a pivot sandwich" and bake_xform_scales for the
    # snapshot→reset→reauthor algorithm.
    residual_scales = []
    for prim in stage.Traverse():
        if prim.GetTypeName() not in ("Xform", "Mesh"):
            continue
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() != UsdGeom.XformOp.TypeScale:
                continue
            v = op.Get()
            if v is None:
                continue
            if any(abs(float(v[i]) - 1.0) > 1e-6 for i in range(3)):
                residual_scales.append((str(prim.GetPath()), tuple(float(v[i]) for i in range(3))))
                break
    if residual_scales:
        n = len(residual_scales)
        path0, s0 = residual_scales[0]
        c7_issues.append(f"{n} residual xformOp:scale(s) "
                         f"(e.g. {path0} = {s0}) — bake_xform_scales (F43) "
                         f"must run; geometry drifts by (1-s)·pivot on "
                         f"pivot-sandwich scales")
    c7_pass = len(c7_issues) == 0
    c7_detail = "clean (meters, no scene, unit scales)" if c7_pass else "; ".join(c7_issues)
    results["C7 Clean Asset"] = {"pass": c7_pass, "detail": c7_detail}

    # Tier 1 warning-level checks (2026-04-19 skill-library integration).
    # Advisory only — never flip a C1-C7 result. See skills/failure-modes
    # §Tier 2/5 for F50/F59/F61/F62/K12/K16 and skills/simready-criteria §C10.
    results["_warnings"] = _tier1_warnings(stage, rigid_bodies, joints,
                                           art_root_paths, colliders,
                                           classification)

    return results


def _tier1_warnings(stage, rigid_bodies, joints, art_root_paths, colliders,
                    classification):
    """Collect advisory warnings (F50, F59, F61, F62, K12, K16, C10).

    Returns a list of {code, msg} dicts. Empty list = nothing to flag.
    """
    w = []

    # F59 — principal axes misaligned with geometry. PhysX defaults
    # principalAxes to identity; if the body ships an explicit diagonalInertia
    # without an explicit principalAxes quaternion, the diagonal is
    # interpreted in the identity frame and can rotate the body at rest.
    # F62 — self-collision flag OFF at articulation root. Makes any rest-pose
    # "clean" audit unreliable (no pairs reported ≠ no geometric overlap).
    # K12 — condition number of diagonalInertia > 1e6 → ill-conditioned mass
    # matrix, solver instability risk.
    for rb in rigid_bodies:
        rb_prim = stage.GetPrimAtPath(rb["path"])
        if not rb_prim:
            continue
        di = rb_prim.GetAttribute("physics:diagonalInertia")
        pa = rb_prim.GetAttribute("physics:principalAxes")
        di_val = di.Get() if di and di.HasValue() else None
        pa_set = bool(pa and pa.HasValue())
        if di_val is not None:
            try:
                ixx, iyy, izz = (float(di_val[0]), float(di_val[1]), float(di_val[2]))
            except Exception:
                ixx = iyy = izz = None
            if ixx is not None:
                if not pa_set and max(ixx, iyy, izz) > 1.2 * min(ixx, iyy, izz) > 0:
                    w.append({
                        "code": "F59",
                        "msg": (f"{rb['path']} has anisotropic diagonalInertia "
                                f"({ixx:.3g}, {iyy:.3g}, {izz:.3g}) but no "
                                f"explicit principalAxes — body may rotate at "
                                f"rest; author physics:principalAxes"),
                    })
                lo = min(v for v in (ixx, iyy, izz) if v > 0) if any(v > 0 for v in (ixx, iyy, izz)) else 0.0
                hi = max(ixx, iyy, izz)
                if lo > 0 and hi / lo > 1e6:
                    w.append({
                        "code": "K12",
                        "msg": (f"{rb['path']} diagonalInertia cond = "
                                f"{hi/lo:.2e} (>1e6) — redistribute mass or "
                                f"regularize (K12)"),
                    })

    for art_path in art_root_paths:
        art_prim = stage.GetPrimAtPath(art_path)
        if not art_prim:
            continue
        sc = art_prim.GetAttribute("physxArticulation:enabledSelfCollisions")
        sc_val = sc.Get() if sc and sc.HasValue() else None
        if sc_val is not True:
            w.append({
                "code": "F62",
                "msg": (f"{art_path} self-collision is "
                        f"{'unset' if sc_val is None else sc_val}; overlap "
                        f"audits may be false-clean — set "
                        f"physxArticulation:enabledSelfCollisions=True before "
                        f"geometry review"),
            })

    # F61 — depenetration velocity cap. V13 assets don't ship a scene by
    # design (C7), so the cap must land in the host scene at load. Emit
    # one reminder per asset whenever an articulation root is present.
    if art_root_paths:
        w.append({
            "code": "F61",
            "msg": ("no scene-level maxDepenetrationVelocity cap ships with "
                    "the asset — Isaac Lab / Newton host must set "
                    "physxScene:maxDepenetrationVelocity ~5.0 during debug "
                    "loads to surface frame-1 overlaps"),
        })

    # F50 — swept overlap. Sample each revolute/prismatic joint at 10%
    # increments and flag when body1's world-frame bbox centroid penetrates
    # body0's bbox more than at rest. Heuristic but catches gross sweeps
    # (e.g. a drawer that opens into a wall). Skips continuous joints
    # (wheels) and world-anchor joints.
    for j in joints:
        if j.get("is_continuous") or j.get("is_world_anchor"):
            continue
        jtype = j.get("type")
        if jtype not in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            continue
        lo = j.get("pris_lo") if jtype == "PhysicsPrismaticJoint" else None
        hi = j.get("pris_hi") if jtype == "PhysicsPrismaticJoint" else None
        b0, b1 = j.get("body0_path"), j.get("body1_path")
        if not (b0 and b1):
            continue
        b0p, b1p = stage.GetPrimAtPath(b0), stage.GetPrimAtPath(b1)
        if not (b0p and b1p):
            continue
        if jtype == "PhysicsRevoluteJoint":
            lo_a = b1p.GetAttribute("physics:lowerLimit")
            hi_a = b1p.GetAttribute("physics:upperLimit")
            # revolute limits live on the joint prim, not body1 — correct below.
            jprim = _find_joint_prim(stage, j.get("name"))
            if not jprim:
                continue
            lo_a = jprim.GetAttribute("physics:lowerLimit")
            hi_a = jprim.GetAttribute("physics:upperLimit")
            lo_v = float(lo_a.Get()) if lo_a and lo_a.HasValue() else None
            hi_v = float(hi_a.Get()) if hi_a and hi_a.HasValue() else None
            if lo_v is None or hi_v is None or abs(hi_v - lo_v) > 1000:
                continue
            if abs(hi_v - lo_v) > 180.0:
                w.append({
                    "code": "F50",
                    "msg": (f"{j['name']} revolute range is "
                            f"{abs(hi_v - lo_v):.0f}° — plausibility flag; "
                            f"swept-overlap test not implemented, verify "
                            f"body1 doesn't pass through body0"),
                })
        else:  # prismatic — cheap travel-vs-body-depth sanity
            if lo is None or hi is None:
                continue
            travel = abs(hi - lo)
            try:
                bb1 = mesh_world_bbox(stage, b1p.GetPath())
            except Exception:
                bb1 = None
            if bb1:
                body1_depth_along_axis = 0.0
                ax = j.get("axis")
                idx = {"X": 0, "Y": 1, "Z": 2}.get(ax, None)
                if idx is not None:
                    body1_depth_along_axis = abs(bb1[1][idx] - bb1[0][idx])
                if body1_depth_along_axis > 0 and travel > 3.0 * body1_depth_along_axis:
                    w.append({
                        "code": "F50",
                        "msg": (f"{j['name']} prismatic travel "
                                f"{travel*100:.0f}cm is >3x body depth "
                                f"{body1_depth_along_axis*100:.0f}cm — body "
                                f"may exit parent volume mid-travel; verify"),
                    })

    # K16 — joint axis sanity against classifier intent. When classify.json
    # declares an axis (e.g. "+X", "Y"), the authored joint's physics:axis
    # should match the unsigned component. Catches frame-convention drift
    # between the classifier prompt and apply_physics.
    if classification is not None:
        parts = classification.get("parts", {}) or {}
        for j in joints:
            jname = j.get("name", "")
            if not jname.endswith("_joint"):
                continue
            pname = jname[: -len("_joint")]
            spec = parts.get(pname)
            if not spec:
                continue
            declared = str(spec.get("axis", "")).strip()
            if not declared:
                continue
            declared_letter = declared[-1].upper() if declared else ""
            if declared_letter not in ("X", "Y", "Z"):
                continue
            actual = j.get("axis")
            if actual and actual != declared_letter:
                w.append({
                    "code": "K16",
                    "msg": (f"{jname} classify declared axis={declared!r} but "
                            f"physics:axis={actual!r} — frame-convention "
                            f"drift; verify classifier prompt vs "
                            f"apply_physics dispatch"),
                })

    # C10 — tier certification heuristic. GPU-batchable requires: tree
    # articulation, ≤20 DOF, convex-only collision. CPU-tier triggers on
    # loops, high DOF, or many convex-decomposition hulls.
    dof_joints = [j for j in joints if j.get("type") != "PhysicsFixedJoint"
                  and not j.get("is_world_anchor", False)]
    n_dof = len(dof_joints)
    decomp_cols = sum(1 for c in colliders if c.get("approx") == "convexDecomposition")
    mesh_cols = sum(1 for c in colliders if c.get("approx") in ("none", "meshSimplification"))
    reasons = []
    if n_dof > 20:
        reasons.append(f"{n_dof} DOF joints (>20)")
    if decomp_cols > 5:
        reasons.append(f"{decomp_cols} convex-decomposition hulls (>5)")
    if mesh_cols > 0:
        reasons.append(f"{mesh_cols} non-convex colliders")
    if reasons:
        w.append({
            "code": "C10",
            "msg": ("likely CPU/offline tier — " + "; ".join(reasons)
                    + "; route to Newton Featherstone or PhysX CPU rather "
                    + "than Isaac Lab GPU batches"),
        })

    # Newton-compat authoring signatures (S-series from
    # `newton-physx-compat-matrix`). None of these are fatal in PhysX/Isaac
    # Sim, but they are the known-crashing authoring patterns for
    # Newton's USD importer. Discovered 2026-04-19 when
    # ResuscitationBed_A01_01_physics.usd segfaulted Newton 1.0.0's
    # `ModelBuilder.add_usd()` while the topologically-similar
    # InstrumentTrolley_B01_01 loaded cleanly; see skill §Known rough edges.
    OVERSIZED_VERTS = 5000
    DEEP_XFORM_DEPTH = 5  # number of "/" segments beyond the default prim
    UNWELDED_RATIO = 4.5  # indices-per-vertex; trolley ≈5.4, bed ≈3.88
    WHEEL_SPLIT_MIN_MESHES = 2  # a wheel body should be ≥2 siblings (tire+hub/detail)

    oversized = []
    unwelded = []
    deep_xforms = []
    dp_path = stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else None
    dp_depth = str(dp_path).count("/") if dp_path else 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            pts = prim.GetAttribute("points")
            idx = prim.GetAttribute("faceVertexIndices")
            if pts and pts.HasValue():
                pts_v = pts.Get()
                if pts_v is not None and len(pts_v) > OVERSIZED_VERTS:
                    oversized.append((prim.GetName(), len(pts_v)))
                if (pts_v is not None and idx and idx.HasValue()
                        and idx.Get() is not None and len(pts_v) > 0):
                    ratio = len(idx.Get()) / len(pts_v)
                    if ratio < UNWELDED_RATIO and len(pts_v) > 500:
                        unwelded.append((prim.GetName(), round(ratio, 2)))
        elif prim.IsA(UsdGeom.Xform):
            depth = str(prim.GetPath()).count("/") - dp_depth
            if depth > DEEP_XFORM_DEPTH:
                deep_xforms.append((str(prim.GetPath()), depth))

    # Wheel-split sanity: any continuous-joint body1 whose descendants include
    # just one Mesh (the tire itself) trips Newton on pure-quad meshes
    # (per the ResuscitationBed report). A proper wheel has tire+disc+detail
    # as siblings.
    unsplit_wheels = []
    for j in joints:
        if not j.get("is_continuous"):
            continue
        b1 = j.get("body1_path")
        if not b1:
            continue
        b1p = stage.GetPrimAtPath(b1)
        if not b1p:
            continue
        mesh_children = [c for c in b1p.GetChildren() if c.IsA(UsdGeom.Mesh)]
        if len(mesh_children) < WHEEL_SPLIT_MIN_MESHES:
            unsplit_wheels.append((j.get("name"), len(mesh_children)))

    if oversized:
        name, n = max(oversized, key=lambda x: x[1])
        w.append({
            "code": "S-verts",
            "msg": (f"{len(oversized)} mesh(es) >{OVERSIZED_VERTS} verts "
                    f"(worst: {name} at {n} verts) — Newton CPU fallback "
                    f"likely on convex hull (>64 vert GPU budget); decimate "
                    f"or set approximation=none if purely decorative"),
        })
    if unwelded:
        name, r = min(unwelded, key=lambda x: x[1])
        w.append({
            "code": "S-weld",
            "msg": (f"{len(unwelded)} mesh(es) look unwelded (indices/verts "
                    f"< {UNWELDED_RATIO}; worst: {name} ratio={r}) — enable "
                    f"'weld vertices on export' in your DCC; Newton's USD "
                    f"importer can crash on disconnected face-soup geometry"),
        })
    if deep_xforms:
        path, d = max(deep_xforms, key=lambda x: x[1])
        w.append({
            "code": "S-depth",
            "msg": (f"{len(deep_xforms)} Xform(s) nested >{DEEP_XFORM_DEPTH} "
                    f"levels (worst: {path} at depth {d}) — flatten the "
                    f"physics-layer hierarchy; Newton's importer traverses "
                    f"every organizational Xform and gains no physics from it"),
        })
    if unsplit_wheels:
        name, n = unsplit_wheels[0]
        w.append({
            "code": "S-wheel-split",
            "msg": (f"{len(unsplit_wheels)} continuous-joint wheel(s) have "
                    f"only {n} mesh child (expected ≥{WHEEL_SPLIT_MIN_MESHES} "
                    f"— e.g. tire+hub+detail) — pre-merged tire geometry is "
                    f"a known Newton crash trigger (ResuscitationBed pattern); "
                    f"split in DCC before re-export"),
        })

    return w


def _find_joint_prim(stage, joint_name):
    """Return the first UsdPhysics.Joint prim whose name matches `joint_name`.

    Used by F50 sweep check to re-fetch joint limits (the joints[] list
    captures prismatic limits but not revolute, to avoid bloating that dict).
    """
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint) and prim.GetName() == joint_name:
            return prim
    return None


def print_audit(results, label="AUDIT"):
    """Print a formatted scorecard."""
    print(f"\n  {label}:")
    total = 0
    passed = 0
    for name, info in results.items():
        if name.startswith("_"):
            continue
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
    warnings = results.get("_warnings") or []
    if warnings:
        print(f"    WARNINGS ({len(warnings)}):")
        for wn in warnings:
            print(f"      [{wn['code']}] {wn['msg']}")


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


_ACCEPTED_CLASSES = (
    "movable:revolute", "movable:prismatic", "movable:continuous",
    "structural", "decorative",
)
_WHEEL_ALIASES = ("wheel", "caster")


def _normalize_class_aliases(stage, classification, dp_path):
    """Normalize classifier shorthand to the canonical schema.

    - "wheel" / "caster"  →  "movable:continuous" + inferred axle axis
      (thinnest world-bbox dimension = axle direction).

    The Claude classifier occasionally drops to this shorthand even though
    the prompt specifies the canonical form; without normalization, the
    main dispatch (line ~1635) silently skips these parts and the asset
    has no rolling mechanism.
    """
    parts = classification.get("parts", {})
    for name, spec in parts.items():
        cls = str(spec.get("class", ""))
        if cls not in _WHEEL_ALIASES:
            continue
        # Resolve the part's world bbox to pick the axle axis (thinnest).
        part_prim = None
        for candidate in (dp_path.AppendChild(name),):
            if stage.GetPrimAtPath(candidate).IsValid():
                part_prim = stage.GetPrimAtPath(candidate)
                break
        if part_prim is None:
            for prim in stage.Traverse():
                if prim.GetName() == name and prim.GetTypeName() == "Xform":
                    part_prim = prim
                    break
        axle_axis = spec.get("axis")
        if not axle_axis and part_prim is not None:
            bb = mesh_world_bbox(stage, part_prim.GetPath())
            if bb:
                bmin, bmax = bb
                dims = [bmax[i] - bmin[i] for i in range(3)]
                axle_axis = ["X", "Y", "Z"][dims.index(min(dims))]
        spec["class"] = "movable:continuous"
        if axle_axis:
            spec["axis"] = axle_axis
        spec.setdefault("parent", "body")
        print(f"  [F48] Normalized '{name}' class={cls!r} → "
              f"'movable:continuous' axis={spec.get('axis')}")


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


def _is_degenerate_mesh(prim, eps=1e-6):
    """True if mesh has zero-thickness (any axis bbox < eps).

    Flat 2D decals/stickers/labels fail qhull (coplanar points produce
    NaN bounds) and crash PhysX broadphase. Such meshes must NOT get
    CollisionAPI. See usd-physx-schemas: Zero-thickness collision meshes.
    """
    pts_attr = prim.GetAttribute("points")
    if not pts_attr or not pts_attr.HasValue():
        return False
    pts = pts_attr.Get()
    if pts is None or len(pts) < 3:
        return True
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    return (max(xs) - min(xs) < eps
            or max(ys) - min(ys) < eps
            or max(zs) - min(zs) < eps)


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
    For movable parts: direct child meshes PLUS descendants of any structural
    child Xforms (welded attachments like a plate rigidly bolted to a column).
    Other rigid-body child Xforms are skipped — they have their own collision.
    Interior sub-Xform meshes (door shelves, rack bins) that belong to
    a rigid body descendant stay out, preventing them from clipping with
    body internals when closed.
    """
    prim = stage.GetPrimAtPath(xform_path)
    if not prim:
        return 0, 0

    if is_body:
        meshes = [(m, _mesh_vert_count(m)) for m in _get_all_descendant_meshes(prim)]
    else:
        # Direct Mesh children of the movable — filter interior/rail hardware
        # substrings ('frame', 'mechanism', etc.) to avoid clipping.
        direct_meshes = [m for m in prim.GetChildren() if m.GetTypeName() == "Mesh"]
        direct_meshes = _filter_movable_collision_meshes(direct_meshes)
        # Structural child Xforms (welded attachments without RigidBodyAPI):
        # include all their descendant meshes UNFILTERED — a plate welded to
        # a column carries its 'frame' mesh as actual collision geometry, not
        # rail hardware. Filtering drops the plate body.
        structural_meshes = []
        for child in prim.GetChildren():
            if child.GetTypeName() != "Xform":
                continue
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            structural_meshes.extend(_get_all_descendant_meshes(child))
        raw = direct_meshes + structural_meshes
        meshes = [(m, _mesh_vert_count(m)) for m in raw]
        # Fallback: if no meshes at all (deeply nested Xform→Xform→Mesh under
        # the movable itself), search recursively. Common on small tools
        # (scissors, forceps).
        if not meshes:
            raw = list(_get_all_descendant_meshes(prim))
            raw = _filter_movable_collision_meshes(raw)
            meshes = [(m, _mesh_vert_count(m)) for m in raw]
    if not meshes:
        return 0, 0

    # F44: for movable parts (drawers, doors, shelves, etc.), internal
    # organizer meshes (holders/cage/rack/grid/lattice) must be SKIPPED from
    # collision entirely — their hulls or decompositions still project
    # outside the drawer's outer envelope and collide with adjacent stacked
    # drawers. Drawers rely on base + front + handle for collision; the
    # internal dividers are visual-only. Seen on MedicalutilityCart_A03_01
    # drawer3 holders (2026-04-18) where the mesh spanned 47cm (entire
    # drawer-stack height) and a convexHull made drawers pass through each
    # other during teleop. Simply excluding `holders`-family meshes from
    # CollisionAPI keeps them visible but non-colliding.
    SKIP_COLLISION_KEYWORDS = ("holders", "holder", "cage", "rack",
                               "lattice", "grid", "divider", "organizer")
    meshes.sort(key=lambda x: x[1], reverse=True)
    n_col = 0
    n_decomp = 0
    for mesh_prim, npts in meshes:
        mesh_name_lower = mesh_prim.GetName().lower()
        if not is_body and any(kw in mesh_name_lower for kw in SKIP_COLLISION_KEYWORDS):
            # Visual-only — no CollisionAPI applied.
            continue
        # F47: zero-thickness meshes (flat decals, stickers, labels) crash
        # qhull (coplanar points → NaN bounds → PhysX "Illegal
        # BroadPhaseUpdateData" and every rigid body reports "Invalid
        # PhysX transform"). Seen on ResuscitationBed_A01_01 with 3 decal
        # meshes (Z-thickness = 0). See usd-physx-schemas: Zero-thickness
        # collision meshes.
        if _is_degenerate_mesh(mesh_prim):
            continue
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
    """Wheel meshes: tire-named meshes use convexHull (drum shape, no
    jitter); non-tire sub-parts (disc, detail, etc.) use convexDecomposition
    for non-convex geometry. Switched 2026-04-19 after the SurgicalChair
    caster build showed visible tire-cover popping under swivel torque —
    a torus-ish tire mesh decomposed into many small hulls produces
    contact separation that looks like the tire cover lifting off the rim.
    Hull is geometrically a filled drum, which for physics-grade casters
    is the right approximation (you don't need to model tread pattern).
    """
    prim = stage.GetPrimAtPath(xform_path)
    if not prim:
        return 0
    n = 0
    for desc in _get_all_descendant_meshes(prim):
        # F47: skip flat decals / stickers (qhull crash on coplanar points).
        if _is_degenerate_mesh(desc):
            continue
        UsdPhysics.CollisionAPI.Apply(desc)
        mc = UsdPhysics.MeshCollisionAPI.Apply(desc)
        dn = desc.GetName().lower()
        if "tire" in dn:
            mc.CreateApproximationAttr("convexHull")
        else:
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

def make_world_anchor_joint(stage, joint_path, body_path):
    """F49: portable world-anchor for fixture bodies.

    Creates a PhysicsFixedJoint with body0Rel empty (= world) and
    body1Rel pointing to the main body. Replaces the PhysX-specific
    `kinematicEnabled=True` idiom for pinning furniture: the body stays
    dynamic (kinematicEnabled=False) so Newton's articulation parser
    treats it as a normal articulated link, while PhysX still anchors
    it rigidly via the fixed joint.

    `localPos0` / `localRot0` are set to the body's current authored
    world transform so PhysX anchors the body where the DCC artist placed
    it. A naive (0,0,0) anchor would *teleport* any body whose authored
    world origin isn't at the origin — first surfaced on
    SurgicalChair_A01_01_physics.usd (2026-04-19): body origin authored
    at (0, 0.027, 0.244) was teleported to (0,0,0) at physics init, so
    the chair spawned with its top half above the floor and wheels 20cm
    below. DrugCabinet and Fridge (F49-verified 2026-04-18) only worked
    because their body origins happened to be at world (0,0,0).

    PhysX treats a fixed joint to world as infinitely stiff (same
    simulation semantics as the kinematic flag). Isaac Sim teleop's
    ArticulationCfg detection reads the body's kinematicEnabled = False
    and routes through the ArticulationCfg path — same path already
    used for dynamic bodies. The zero-stiffness ".*" actuator regex
    matches this fixed joint but PhysX ignores drives on 0-DOF joints,
    so no behavior change.
    """
    body_prim = stage.GetPrimAtPath(body_path)
    if body_prim and body_prim.IsValid():
        l2w = UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        t = l2w.ExtractTranslation()
        # Extract rotation as a quaternion; Gf returns (w, x, y, z) via .GetReal()
        # and .GetImaginary(). Keep identity fallback on degenerate transforms.
        try:
            rot = l2w.ExtractRotationQuat().GetNormalized()
            qw = float(rot.GetReal())
            im = rot.GetImaginary()
            qx, qy, qz = float(im[0]), float(im[1]), float(im[2])
        except Exception:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        anchor_pos = Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))
        anchor_rot = Gf.Quatf(qw, qx, qy, qz)
    else:
        anchor_pos = Gf.Vec3f(0, 0, 0)
        anchor_rot = Gf.Quatf(1, 0, 0, 0)

    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([])  # empty = world
    joint.CreateBody1Rel().SetTargets([body_path])
    joint.CreateLocalPos0Attr(anchor_pos)
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr(anchor_rot)
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    return joint


def make_revolute_joint(stage, joint_path, body0, body1, local_pos0, local_pos1,
                        axis="Z", hinge_edge="min_x", lower_deg=-120, upper_deg=120):
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateAxisAttr(axis)
    if hinge_edge == "continuous":
        # Bidirectional unlimited swivel — used for 2-DOF caster brackets
        # (Z rotation tracks push direction). Light angular drive below
        # keeps the bracket from oscillating under gravity.
        joint.CreateLowerLimitAttr(-9999.0)
        joint.CreateUpperLimitAttr(9999.0)
    elif hinge_edge == "min_x":
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
        local_mat = wmat * pw.GetInverse()

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        top = xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
        top.Set(local_mat)

    return moved


# --- Wheel structural splitting ---

WHEEL_STRUCTURAL_KEYWORDS = ("fixer", "bolt", "body", "mount", "stopper",
                             "frame", "caps", "bracket", "fork", "brake",
                             # Added 2026-04-18 after Mobilecartsandtables_C01_01
                             # shipped with bracket rotating as part of tire.
                             # Source USD named the caster bracket/cover parts
                             # `wheel_base_01` / `wheel_trim_01` which fell
                             # outside the original keyword set. Scope is safe:
                             # split_wheel_structural_parts only inspects DIRECT
                             # children of continuous-joint wheels (never body
                             # or chassis), so false-positive matching to a
                             # "base_link" elsewhere in the hierarchy cannot
                             # happen here.
                             "base", "trim")

# Sub-mesh keywords that identify a swivel-caster bracket (U-housing that
# rotates around vertical while the tire rolls inside it). Checked as direct
# children of a continuous-joint wheel Xform. Keep tight — broad matches like
# "body" on a fixed-wheel disc would mis-trigger caster mode.
CASTER_BRACKET_KEYWORDS = ("mount", "bracket", "housing", "fork", "yoke", "swivel")


def _is_swivel_caster(wheel_prim):
    """Detect swivel-caster pattern: wheel Xform has BOTH a tire mesh and a
    bracket-style sibling mesh (mount / bracket / housing / fork).

    Contrast with fixed wheels (InstrumentTrolley): direct children are
    tire + disc + detail — no bracket keyword, so 1-DOF path kicks in.
    """
    has_tire = False
    has_bracket = False
    for child in wheel_prim.GetAllChildren():
        if child.GetTypeName() not in ("Mesh", "Xform"):
            continue
        nm = child.GetName().lower()
        if "tire" in nm:
            has_tire = True
        if any(kw in nm for kw in CASTER_BRACKET_KEYWORDS):
            has_bracket = True
    return has_tire and has_bracket


def regroup_body_meshes_by_movable(stage, movables, body_path):
    """Move body-level structural meshes that visually belong to a movable
    INTO the movable's Xform so they transform together.

    The classifier receives a flat list of Xforms and routes each to body
    or to a movable. But the raw USD often scatters a movable's sub-meshes
    across the body layer using shared naming (`seat_body_01`, `seat_mount*`,
    `seat_bolts_01` for a rotating seat). If these stay on the chassis body
    they become static while the seat Xform rotates, and the soft cushion
    on the seat visually detaches from the "static" seat frame.

    Rule: for each non-wheel movable with name like `<prefix>_NN`, scan the
    body's direct Mesh children and reparent any whose name starts with
    `<prefix>_` into the movable's Xform (preserving world transform).
    Skipped for continuous wheels — those are handled by
    split_wheel_structural_parts with a different split direction.

    Surfaced on SurgicalChair_A01_01 (2026-04-19): seat_body/mount/bolts
    stayed on the leg while seat_01 rotated; cushion-on-rotating-seat
    appeared to detach from static-seat-frame-on-leg each time the seat
    swivelled.
    """
    body_prim = stage.GetPrimAtPath(body_path)
    if not body_prim:
        return {}
    moved = {}
    for name, info in movables.items():
        if info["joint"] == "continuous":
            continue  # wheels use split_wheel_structural_parts
        if info.get("is_caster_bracket"):
            continue  # brackets built by split_wheel_structural_parts
        # prefix e.g. "seat" from "sm_surgicalchair_a01_seat_01"
        parts = name.rsplit("_", 1)
        if len(parts) != 2 or not parts[1].isdigit():
            continue
        prefix_full = parts[0]  # "sm_surgicalchair_a01_seat"
        short_prefix = prefix_full.rsplit("_", 1)[-1]  # "seat"
        if len(short_prefix) < 3:
            continue  # reject absurdly short prefixes that'd over-match
        movable_path = info["path"]
        to_move = []
        for c in body_prim.GetChildren():
            if not c.IsA(UsdGeom.Mesh):
                continue
            cn = c.GetName().lower()
            if cn.startswith(prefix_full.lower() + "_"):
                # Don't move the movable Xform into itself (edge case when
                # the body is also the default prim wrapping the movable).
                if c.GetPath() == movable_path:
                    continue
                to_move.append(c.GetPath())
        if to_move:
            m = reparent_prims_preserve_world_xform(stage, to_move, movable_path)
            moved.update(m)
    return moved


def split_wheel_structural_parts(stage, movables, body_path):
    """Split each continuous-joint wheel into the right physics topology.

    Fixed wheels (InstrumentTrolley, EmergencyTrolley, ResuscitationBed):
    move structural meshes (fixer / body / bolts / frame / ...) from the
    wheel Xform to the chassis body so they stay welded to the chassis
    while only the tire sub-body spins.

    Swivel casters (SurgicalChair, office chairs): split each wheel into
    TWO rigid bodies — a bracket (U-housing containing mount / bolt / body
    meshes) that swivels on a revolute Z joint to the chassis, and a tire
    sub-body that rolls on a continuous joint to the bracket. True 2-DOF
    caster behavior: turn in place + roll in whatever direction the robot
    pushes. Mutates `movables` in place to inject the new bracket entry and
    re-parent the tire to it.

    Returns:
        all_moved          — prim-path map of every reparent, for logging
        caster_brackets    — {wheel_name: bracket_path} for each 2-DOF build
    """
    all_moved = {}
    caster_brackets = {}
    for name, info in list(movables.items()):
        if info["joint"] != "continuous":
            continue
        # Gate by wheel-name keyword so swivel seats and similar non-wheel
        # continuous joints don't match WHEEL_STRUCTURAL_KEYWORDS against their
        # own sub-meshes. Surfaced on SurgicalChair_A01_01 (2026-04-19): the
        # seat_01 continuous joint had seat_body/mount/bolts children, all
        # matched the wheel-structural keywords, and the split stripped the
        # seat frame to the leg — the soft cushion on the rotating seat then
        # visually detached from the "static seat frame" on every swivel.
        nm_lower = name.lower()
        if not any(kw in nm_lower for kw in ("wheel", "caster", "roller", "tire")):
            continue
        wheel_prim = stage.GetPrimAtPath(info["path"])
        if not wheel_prim:
            continue

        if _is_swivel_caster(wheel_prim):
            # --- 2-DOF caster: create bracket body, reparent bracket meshes ---
            # Only STRICT caster-bracket keywords (mount / bracket / housing /
            # fork / yoke / swivel) go into the bracket. The hub/drum, inner
            # body, and bolts that sit INSIDE the tire assembly stay with
            # the tire so they roll together. Surfaced 2026-04-19 on
            # SurgicalChair: wheel_body_01 is the plastic hub inside the
            # rubber tire; it had been lumped into the bracket because
            # "body" is in WHEEL_STRUCTURAL_KEYWORDS, so the hub swivelled
            # with the mount while the tire rolled independently — the
            # rubber appeared to "come off" the drum under swivel torque.
            bracket_children = []
            for child in wheel_prim.GetAllChildren():
                if child.GetTypeName() not in ("Mesh", "Xform"):
                    continue
                nm = child.GetName().lower()
                if "tire" in nm:
                    continue  # tire stays inside the wheel Xform
                if any(kw in nm for kw in CASTER_BRACKET_KEYWORDS):
                    bracket_children.append(child.GetPath())
            if not bracket_children:
                continue  # nothing to split out; treat as fixed-wheel-ish
            # New bracket Xform is a sibling of the wheel Xform under dp_parent.
            wheel_parent = wheel_prim.GetPath().GetParentPath()
            bracket_name = f"{wheel_prim.GetName()}_bracket"
            bracket_path = wheel_parent.AppendChild(bracket_name)
            # Author the bracket Xform's translate at the centroid of its
            # mount meshes (world space) BEFORE reparenting. If we leave the
            # bracket at identity, its Xform origin sits at world (0,0,0)
            # while its colliders land ~30 cm away — a massive origin-to-COM
            # moment arm that makes the rigid body unstable under gravity
            # (wheels visibly "fall apart" from the leg pivot). Placing the
            # bracket origin at the mount centroid means PhysX's auto-
            # computed COM ends up near the origin, which is what the solver
            # expects.
            # Compute centroid by walking points of each bracket mesh directly
            # (mesh_world_bbox walks only DESCENDANTS, so passing a Mesh path
            # returns None — wouldn't help here since bracket_children are
            # Mesh paths, not Xform paths).
            bmin = Gf.Vec3d(1e30, 1e30, 1e30)
            bmax = Gf.Vec3d(-1e30, -1e30, -1e30)
            found_any = False
            for mp in bracket_children:
                mprim = stage.GetPrimAtPath(mp)
                if not mprim or not mprim.IsValid():
                    continue
                meshes = []
                if mprim.IsA(UsdGeom.Mesh):
                    meshes.append(mprim)
                meshes.extend(_get_all_descendant_meshes(mprim))
                for m in meshes:
                    pts = m.GetAttribute("points")
                    if not pts or not pts.HasValue():
                        continue
                    ml2w = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default())
                    for pt in pts.Get():
                        wp = ml2w.TransformAffine(Gf.Vec3d(
                            float(pt[0]), float(pt[1]), float(pt[2])))
                        bmin = Gf.Vec3d(min(bmin[0], wp[0]),
                                        min(bmin[1], wp[1]),
                                        min(bmin[2], wp[2]))
                        bmax = Gf.Vec3d(max(bmax[0], wp[0]),
                                        max(bmax[1], wp[1]),
                                        max(bmax[2], wp[2]))
                        found_any = True
            UsdGeom.Xform.Define(stage, bracket_path)
            if found_any:
                centroid = Gf.Vec3d(
                    (bmin[0] + bmax[0]) / 2,
                    (bmin[1] + bmax[1]) / 2,
                    (bmin[2] + bmax[2]) / 2,
                )
                UsdGeom.Xformable(
                    stage.GetPrimAtPath(bracket_path)
                ).AddTranslateOp().Set(centroid)
            moved = reparent_prims_preserve_world_xform(
                stage, bracket_children, bracket_path)
            all_moved.update(moved)
            caster_brackets[name] = bracket_path
            # Inject bracket into movables as a revolute Z swivel on the body.
            # Continuous range so it can spin freely like a real caster.
            movables[bracket_name] = {
                "path": bracket_path,
                "joint": "revolute",
                "axis": "Z",
                "parent": "body",
                "hinge_edge": "continuous",  # ±continuous swivel
                "is_caster_bracket": True,
            }
            # The tire's parent is now the bracket, not the body.
            info["parent"] = bracket_name
            info["is_caster_tire"] = True
        else:
            # --- 1-DOF fixed wheel: move structural meshes to chassis body ---
            structural_paths = []
            for child in wheel_prim.GetAllChildren():
                if child.GetTypeName() not in ("Mesh", "Xform"):
                    continue
                if any(kw in child.GetName().lower() for kw in WHEEL_STRUCTURAL_KEYWORDS):
                    structural_paths.append(child.GetPath())
            if structural_paths:
                moved = reparent_prims_preserve_world_xform(
                    stage, structural_paths, body_path)
                all_moved.update(moved)
    return all_moved, caster_brackets


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
    """Find the body Xform by name — searches the whole subtree before
    falling back to first-child. Previously silently defaulted to the
    first direct Xform child when the named body was nested deeper,
    which made the classifier's body choice a no-op on assets with a
    doubly-nested default prim (e.g. scissors / retractor hierarchies
    where the real body is a grandchild)."""
    dp_path = default_prim.GetPath()
    candidate = dp_path.AppendChild(body_name)
    if stage.GetPrimAtPath(candidate).IsValid():
        return candidate
    for prim in stage.Traverse():
        if prim.GetName() == body_name and prim.GetTypeName() == "Xform":
            return prim.GetPath()
    for child in default_prim.GetChildren():
        if child.GetTypeName() == "Xform":
            return child.GetPath()
    return dp_path


def resolve_movable_parts(stage, body_path, dp_path, classification):
    """Resolve classified movable parts to prim paths and joint info.

    Each movable carries a "parent" name (classifier output, default "body")
    used to wire serial kinematic chains. Joint body0 will be the parent's
    prim path (after reparent), not the hard-coded body for every joint.
    """
    movables = {}
    for name, spec in classification["parts"].items():
        cls = spec.get("class", "")
        if not cls.startswith("movable:"):
            continue

        joint_type = cls.split(":")[1]
        axis_raw = spec.get("axis", "Z" if joint_type == "revolute" else "Y")
        # F46b: axis may carry an optional sign prefix ("+X", "-X") to override
        # the auto direction-select heuristic for prismatic drawers that open
        # AGAINST the handle-face convention (e.g. a top lid that opens toward
        # the back while other drawers open toward the front).
        axis_sign = 0
        if isinstance(axis_raw, str) and len(axis_raw) == 2 and axis_raw[0] in "+-":
            axis_sign = 1 if axis_raw[0] == "+" else -1
            axis = axis_raw[1]
        else:
            axis = axis_raw
        parent_name = spec.get("parent", "body")

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

        movables[name] = {
            "path": path,
            "joint": joint_type,
            "axis": axis,
            "axis_sign": axis_sign,  # 0 = auto, +1 = +axis, -1 = -axis
            "parent": parent_name,
        }
    return movables


def bake_xform_scales(stage):
    """Bake all non-unit xformOp:scale ops so every Xform ends with
    scale=(1,1,1), preserving world positions of all mesh geometry.

    F43: MedicalutilityCart_A03_01 raw USD had `xformOp:scale=(100,100,100)`
    on its inner chassis Xform AND nested compensating scales (0.02, 52, …) on
    arm/screencase/etc. Isaac Lab's ArticulationCfg spawned the asset but
    interpreted the inner scales inconsistently with the USD renderer —
    the cart physically functioned but appeared floating above the ground.

    Robust (snapshot→reset→reauthor) algorithm — correct regardless of op
    order, including `translate:pivot`/`!invert!translate:pivot` sandwiches
    wrapping the scale op. Prior implementation multiplied mesh points by
    the scalar scale, ignoring pivot-about semantics (s·P + (1-s)·pivot),
    which drifted tire geometry on InstrumentTrolley_B01_01 by ~5cm per
    wheel at teleop time. Discovered 2026-04-19 during Tier 1 regression.

    Three passes:
      A. Snapshot each Mesh's world-space points under the ORIGINAL
         transform chain, before any mutation.
      B. Reset every xformOp:scale to (1,1,1), and scale non-inverse
         translate ops by their ancestors' cum_scale so Xform pivots/
         translates stay in world-correct positions.
      C. For every snapshotted Mesh, compute the new L2W (with scales=1)
         and reauthor local points as new_L2W^-1 · world_points. This
         preserves world mesh positions exactly regardless of whether the
         original scale op sat inside a pivot sandwich.
    """

    def cum_scale_from_root(prim):
        """Product of all ancestor scales — excludes prim's OWN scale."""
        sx = sy = sz = 1.0
        p = prim.GetParent()
        while p and p.IsValid() and p.GetPath() != Sdf.Path.absoluteRootPath:
            xf = UsdGeom.Xformable(p)
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    v = op.Get()
                    if v is not None:
                        sx *= float(v[0]); sy *= float(v[1]); sz *= float(v[2])
            p = p.GetParent()
        return sx, sy, sz

    def own_scale(prim):
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                v = op.Get()
                if v is not None:
                    return float(v[0]), float(v[1]), float(v[2])
        return 1.0, 1.0, 1.0

    # --- Pre-scan: any non-unit scale anywhere? ---
    nonunit_paths = set()
    snapshot_cum = {}
    has_nonunit = False
    for prim in stage.Traverse():
        if prim.GetTypeName() not in ("Xform", "Mesh"):
            continue
        cx, cy, cz = cum_scale_from_root(prim)
        ox, oy, oz = own_scale(prim)
        snapshot_cum[str(prim.GetPath())] = (cx, cy, cz)
        if not (abs(ox-1.0) < 1e-6 and abs(oy-1.0) < 1e-6 and abs(oz-1.0) < 1e-6):
            nonunit_paths.add(str(prim.GetPath()))
            has_nonunit = True
    if not has_nonunit:
        return False

    # --- Pass A: snapshot world points for every Mesh whose chain carries scale ---
    def chain_has_scale(prim):
        p = prim
        while p and p.IsValid() and p.GetPath() != Sdf.Path.absoluteRootPath:
            if str(p.GetPath()) in nonunit_paths:
                return True
            p = p.GetParent()
        return False

    mesh_world_points = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not chain_has_scale(prim):
            continue
        pts_attr = prim.GetAttribute("points")
        if not pts_attr or not pts_attr.HasValue():
            continue
        l2w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        mesh_world_points[str(prim.GetPath())] = [
            l2w.TransformAffine(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
            for p in pts_attr.Get()
        ]

    # --- Pass B: reset scale ops to (1,1,1); scale translate ops by ancestor cum_scale ---
    # Translates get scaled by ancestors' cum_scale (not own scale) so that
    # an ancestor's baked-out scale is still reflected in descendants'
    # offset positions. Mesh points are handled separately in Pass C.
    for prim in stage.Traverse():
        if prim.GetTypeName() not in ("Xform", "Mesh"):
            continue
        cx, cy, cz = snapshot_cum[str(prim.GetPath())]
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.IsInverseOp():
                continue
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = op.Get()
                if v is not None and (abs(cx-1.0) > 1e-6 or abs(cy-1.0) > 1e-6 or abs(cz-1.0) > 1e-6):
                    new = (float(v[0])*cx, float(v[1])*cy, float(v[2])*cz)
                    op.Set(Gf.Vec3d(*new) if isinstance(v, Gf.Vec3d) else Gf.Vec3f(*new))
            elif op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                m = op.Get()
                if m is not None and (abs(cx-1.0) > 1e-6 or abs(cy-1.0) > 1e-6 or abs(cz-1.0) > 1e-6):
                    scaled = Gf.Matrix4d(m)
                    scaled.SetRow3(3, Gf.Vec3d(m[3][0]*cx, m[3][1]*cy, m[3][2]*cz))
                    op.Set(scaled)
            elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(1.0, 1.0, 1.0))

    # --- Pass C: reauthor mesh local points from snapshotted world points ---
    for path_str, world_pts in mesh_world_points.items():
        prim = stage.GetPrimAtPath(path_str)
        if not prim:
            continue
        new_l2w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        new_w2l = new_l2w.GetInverse()
        new_local = [new_w2l.TransformAffine(wp) for wp in world_pts]
        prim.GetAttribute("points").Set(
            [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in new_local]
        )
        ext_attr = prim.GetAttribute("extent")
        if ext_attr and new_local:
            xs = [p[0] for p in new_local]
            ys = [p[1] for p in new_local]
            zs = [p[2] for p in new_local]
            ext_attr.Set([
                Gf.Vec3f(float(min(xs)), float(min(ys)), float(min(zs))),
                Gf.Vec3f(float(max(xs)), float(max(ys)), float(max(zs))),
            ])

    print(f"    baked {len(nonunit_paths)} xformOp:scale ops "
          f"({len(mesh_world_points)} meshes reauthored via world-snapshot)")
    return True


def normalize_to_meters(stage):
    """Convert stage from any unit (cm, mm, etc.) to meters.

    Scales all mesh vertices and translation xformOps by metersPerUnit,
    then sets metersPerUnit to 1.0. This ensures the output USD works
    in any simulator without needing external scale factors.

    Also bakes out any lingering `xformOp:scale` ops into descendant vertex
    data — see bake_xform_scales (F43).
    """
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    if abs(mpu - 1.0) < 0.001:
        # Even when mpu is already 1.0, a non-unit xformOp:scale could still
        # be lurking on an inner Xform and confuse physics engines (F43).
        if bake_xform_scales(stage):
            print(f"\n  NORMALIZE: mpu already 1.0, but baked residual xformOp:scale ops")
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
    # F43: also bake any xformOp:scale residue so Isaac Lab + PhysX see the
    # same geometry the renderer does.
    bake_xform_scales(stage)
    return True


def apply_physics(stage, classification, output_usd, dynamic_body=False,
                  gemini_mass=None, gemini_density=None, gemini_articulation=None):
    """Phase 3: Apply all missing physics based on classification."""
    default_prim = stage.GetDefaultPrim()
    dp_path = default_prim.GetPath()

    normalize_to_meters(stage)
    mpu = 1.0

    # F48: classifier LLM drifts between the canonical "movable:continuous"
    # and the shorthand "wheel" / "caster". Treat the shorthand as an alias
    # so the pipeline is robust to prompt drift — otherwise wheels silently
    # become structural and the asset has no rolling mechanism. Seen on
    # ResuscitationBed_A01_01 (2026-04-18): 4 wheels labeled "wheel" were
    # dropped, producing a 139kg block that slid on friction instead of
    # rolling on casters.
    _normalize_class_aliases(stage, classification, default_prim.GetPath())

    # Strip existing physics
    n_j, n_a, n_m = strip_existing_physics(stage)
    if n_j + n_a + n_m > 0:
        print(f"\n  STRIPPED: {n_j} joints, {n_a} physics attrs, {n_m} materials")

    body_path = resolve_body_xform(stage, default_prim, classification["body"])
    movables = resolve_movable_parts(stage, body_path, dp_path, classification)

    # Guard: skip movables nested inside other movables ONLY when the nesting
    # is undeclared. A nested movable with a valid "parent" field pointing at
    # the enclosing movable is a kinematic-chain link (boom arms, robot arms)
    # — keep it; its joint will hinge to the declared parent after reparent.
    movable_path_strs = {str(info["path"]) for info in movables.values()}
    name_by_path = {str(info["path"]): name for name, info in movables.items()}
    nested_undeclared = []
    for name, info in movables.items():
        enclosing_paths = [mp for mp in movable_path_strs - {str(info["path"])}
                           if str(info["path"]).startswith(mp + "/")]
        if not enclosing_paths:
            continue
        parent_name = info.get("parent", "body")
        # Valid chain: declared parent matches an enclosing movable
        enclosing_names = {name_by_path[p] for p in enclosing_paths}
        if parent_name in enclosing_names:
            continue
        nested_undeclared.append(name)
    if nested_undeclared:
        print(f"\n  NESTED MOVABLES without declared parent chain (treating as structural — move with parent):")
        for name in nested_undeclared:
            print(f"    {name} (parent='{movables[name].get('parent','body')}')")
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

    # Regroup body-level meshes that share a movable's prefix into the
    # movable itself (keeps seat frame + cushion + bolts rotating together
    # on a swivel-seat chair; no-op on trolleys where wheels are handled by
    # split_wheel_structural_parts below). Must run BEFORE the wheel split
    # so its side-effects don't get undone.
    body_regroup_moved = regroup_body_meshes_by_movable(stage, movables, body_path)
    if body_regroup_moved:
        print(f"\n  BODY REGROUP: {len(body_regroup_moved)} body meshes → owning movable")
        for old, new in body_regroup_moved.items():
            print(f"    {old} -> {new}")

    # --- Wheel structural split (fixed wheels → chassis; casters → bracket body) ---
    wheel_moved, caster_brackets = split_wheel_structural_parts(stage, movables, body_path)
    if wheel_moved:
        print(f"\n  WHEEL SPLIT: {len(wheel_moved)} structural meshes → "
              f"{'bracket + ' if caster_brackets else ''}body")
        for old, new in wheel_moved.items():
            print(f"    {old} -> {new}")
    if caster_brackets:
        print(f"\n  CASTER 2-DOF: {len(caster_brackets)} caster bracket body(ies) created")
        for wheel_name, bracket_path in caster_brackets.items():
            print(f"    {wheel_name} → bracket {bracket_path}  "
                  f"(revolute Z swivel on body, tire rolls on bracket)")

    # Compute swivel anchor for each 2-DOF caster bracket injected by the
    # split. Anchor is at the bracket's top-center — the point where the
    # bracket mates with the leg under the chassis. Using the bracket's
    # volumetric centroid would place the Z-axis through the middle of the
    # caster, which works too, but top-center matches real-world caster
    # geometry more closely and keeps the swivel axis collinear with the
    # load path.
    for name, info in movables.items():
        if not info.get("is_caster_bracket"):
            continue
        bbox = mesh_world_bbox(stage, info["path"])
        if bbox:
            saved_anchors[name] = Gf.Vec3d(
                (bbox[0][0] + bbox[1][0]) / 2,
                (bbox[0][1] + bbox[1][1]) / 2,
                bbox[1][2],
            )
            print(f"    anchor {name} (bracket top-center): "
                  f"({saved_anchors[name][0]:.4f}, {saved_anchors[name][1]:.4f}, {saved_anchors[name][2]:.4f})")

    # Always recompute tire-center anchor for every continuous joint — whether or
    # not any structural parts were split. Prior behavior put this inside
    # `if wheel_moved:`, which silently skipped the fix on assets whose wheel
    # naming didn't match WHEEL_STRUCTURAL_KEYWORDS, causing wheels to detach
    # at physics init (symptom seen on EmergencyTrolley_A01_01, 2026-04-17).
    for name, info in movables.items():
        if info["joint"] == "continuous":
            bbox = mesh_world_bbox(stage, info["path"])
            if bbox:
                tire_center = Gf.Vec3d(
                    (bbox[0][0] + bbox[1][0]) / 2,
                    (bbox[0][1] + bbox[1][1]) / 2,
                    (bbox[0][2] + bbox[1][2]) / 2)
                saved_anchors[name] = tire_center
                # Thin bbox dimension is the wheel's axle. Extended 2026-04-19
                # to consider all three axes (was X-vs-Y only), and gated by a
                # name-keyword check — casters are near-cube assemblies
                # (tire + bracket), so bbox-thinness alone can't discriminate
                # them from swivel seats. The override is a wheel-specific
                # heuristic; applying it to a chair seat picks the wrong
                # rotation axis.
                sizes = [abs(bbox[1][i] - bbox[0][i]) for i in range(3)]
                nm_lower = name.lower()
                looks_like_wheel = any(kw in nm_lower for kw in
                                        ("wheel", "caster", "roller", "tire"))
                if looks_like_wheel:
                    # For casters (near-cube), prefer a HORIZONTAL axis (X or Y)
                    # over Z — a Z axis on a caster would swivel, not roll;
                    # only one DOF per wheel in V13, so roll wins.
                    if max(sizes[0], sizes[1]) > 1e-6:
                        detected_axis = "X" if sizes[0] <= sizes[1] else "Y"
                    else:
                        detected_axis = "X"
                    # If one horizontal dim is clearly thinnest across all three,
                    # use it; else fall back to min(X,Y).
                    if sizes[2] < min(sizes[0], sizes[1]) * 0.6:
                        # Legitimately Z-thin (unusual wheel orientation) —
                        # keep only if classifier also picked Z.
                        if info["axis"] == "Z":
                            detected_axis = "Z"
                    if detected_axis != info["axis"]:
                        print(f"    axis override {name}: {info['axis']} -> {detected_axis} "
                              f"(wheel X={sizes[0]:.4f} Y={sizes[1]:.4f} Z={sizes[2]:.4f})")
                        info["axis"] = detected_axis
                else:
                    print(f"    axis keep {name}: {info['axis']} "
                          f"(X={sizes[0]:.4f} Y={sizes[1]:.4f} Z={sizes[2]:.4f} "
                          f"— not wheel-named, trust classifier)")
                print(f"    anchor {name} (tire center): ({tire_center[0]:.4f}, {tire_center[1]:.4f}, {tire_center[2]:.4f})")

    # --- C1: Rigid Bodies + Mass ---
    print(f"\n  RIGID BODIES:")
    # Graspable props: if object is small (<3kg estimated), make body dynamic
    # so the robot can pick it up. Applies to both non-articulated tools
    # (Forceps) AND articulated handheld tools (HoldingDevice with button +
    # hinge arms, working scissors, syringes with plunger). Large furniture
    # and fixtures stay kinematic.
    has_movables = len(movables) > 0
    # Auto-dynamic if the asset has continuous joints (wheels/casters). A wheel
    # that rolls on the ground can only produce translation if its parent body
    # is free to move; anchoring the body with F49 pins it and wheels spin in
    # place. Matches the cross-pipeline rule documented in README §"--dynamic
    # decision tree": wheels → whole-thing-pushable → dynamic_body.
    # Surfaced on SurgicalChair_A01_01 (2026-04-19): chair classified with
    # 6 continuous joints but shipped world-anchored, so shift-drag did
    # nothing and the chair rotated on its seat axis instead of rolling.
    has_continuous = any(info.get("joint") == "continuous" for info in movables.values())
    if not dynamic_body and has_continuous:
        dynamic_body = True
        n_cont = sum(1 for info in movables.values() if info.get("joint") == "continuous")
        print(f"    (auto-dynamic: {n_cont} continuous joint(s) imply wheeled/mobile asset)")
    if not dynamic_body:
        est_mass = gemini_mass  # Use Gemini mass if available
        if not est_mass:
            est_mass = estimate_mass_from_mesh(stage, body_path, density=500)
        if not est_mass:
            body_bbox_check = mesh_world_bbox(stage, body_path)
            est_mass = estimate_mass(body_bbox_check, mpu, density=500) if body_bbox_check else 999
        if est_mass < 3.0:
            dynamic_body = True
            reason = ("no joints" if not has_movables
                      else f"{len(movables)} movable part(s) — handheld articulated tool")
            print(f"    (small object {est_mass:.2f}kg, {reason} — auto-dynamic for grasping)")
    body_kinematic = not dynamic_body
    # F49: portable encoding — the body is ALWAYS dynamic
    # (kinematicEnabled=False). Fixtures are anchored via an explicit
    # PhysicsFixedJoint to world rather than the PhysX-specific kinematic
    # flag. Equivalent in Isaac Sim / PhysX, but Newton's articulation
    # parser now sees a full articulation instead of orphan joints.
    apply_rigid_body(stage, body_path, kinematic=False, dynamic_body=dynamic_body)
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
        # Fixed-joint movables are welded structural links (e.g. a plate
        # rigidly attached to a column so a sibling prismatic can collide
        # with it). They need full mesh coverage — skip the rail-keyword
        # filter and descend all meshes, matching body collision treatment.
        is_fixed = info["joint"] == "fixed"
        if is_wheel:
            n_col = apply_collision_wheels(stage, info["path"])
            n_d = n_col
        elif is_fixed:
            n_col, n_d = apply_collision_q1(stage, info["path"], is_body=True)
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

    # F49: fixtures get a PhysicsFixedJoint to world instead of
    # kinematicEnabled=True. See make_world_anchor_joint docstring.
    if body_kinematic:
        anchor_path = joints_scope.AppendChild("world_anchor")
        make_world_anchor_joint(stage, anchor_path, body_path)
        print(f"    FixedJoint world_anchor → {body_path.name}  (F49)")

    for name, info in movables.items():
        path = info["path"]
        jtype = info["joint"]
        axis = info["axis"]
        joint_path = joints_scope.AppendChild(f"{name}_joint")

        # Resolve parent link. Default = body. For serial chains (boom arms,
        # robot arms) the classifier declares "parent" pointing at another
        # movable — joint body0 becomes that movable's reparented path so
        # joints hinge to their parent link, not always to the body.
        parent_name = info.get("parent", "body")
        if parent_name == "body" or parent_name not in movables:
            parent_path = body_path
            parent_bbox = body_bbox
        else:
            parent_path = movables[parent_name]["path"]
            parent_bbox = mesh_world_bbox(stage, parent_path) or body_bbox

        anchor = saved_anchors[name]
        lp0 = world_point_to_local(stage, parent_path, anchor)
        lp0_f = Gf.Vec3f(float(lp0[0]), float(lp0[1]), float(lp0[2]))
        lp1 = world_point_to_local(stage, path, anchor)
        lp1_f = Gf.Vec3f(float(lp1[0]), float(lp1[1]), float(lp1[2]))

        if jtype == "revolute":
            # Caster brackets declare hinge_edge="continuous" explicitly —
            # skip detect_hinge_edge to preserve the bidirectional ±unlimited
            # swivel. Everything else auto-detects hinge side from geometry.
            hinge = info.get("hinge_edge") or detect_hinge_edge(
                stage, path, anchor_world=anchor)
            make_revolute_joint(stage, joint_path, parent_path, path,
                                lp0_f, lp1_f, axis=axis, hinge_edge=hinge)
            print(f"    RevoluteJoint  {name}  axis={axis} hinge={hinge} parent={parent_name}")
        elif jtype == "prismatic":
            # F40: Gemini-reported travel range takes precedence over
            # bbox-derived travel for small components whose bbox includes
            # ancestor transforms (e.g. a 5mm push-button on a deeply-nested
            # valve assembly — bbox Y-extent is the full arm length, not
            # the button travel). Seen on HoldingDevice_A01_01 valvebutton
            # (2026-04-18): bbox gave 60cm travel, Gemini said 5mm.
            gemini_spec = (gemini_articulation or {}).get(name)
            gemini_range = gemini_spec.get("range_meters") if gemini_spec else None
            gemini_bidir = gemini_spec.get("limits_bidirectional", False) if gemini_spec else False

            # For prismatic joints, travel is computed against the PARENT
            # link's bbox (not the root body). In flat fan-out assets
            # parent_bbox == body_bbox so behavior is unchanged; for serial
            # chains (e.g. a height-adjust plate sliding on a column) the
            # slide range is correctly scoped to the column, not the whole
            # fixture.
            bbox = mesh_world_bbox(stage, path)
            axis_idx = {"X": 0, "Y": 1, "Z": 2}[axis]
            part_depth = abs(bbox[1][axis_idx] - bbox[0][axis_idx]) if bbox else 0.4
            body_depth = abs(parent_bbox[1][axis_idx] - parent_bbox[0][axis_idx]) if parent_bbox else part_depth
            # Use the overlap region between part and parent on the slide axis.
            # For overlapping parts (caliper blade over ruler, plate on column),
            # the useful travel is how far the part can slide before exiting
            # the parent.
            if bbox and parent_bbox:
                overlap_min = max(bbox[0][axis_idx], parent_bbox[0][axis_idx])
                overlap_max = min(bbox[1][axis_idx], parent_bbox[1][axis_idx])
                overlap = max(0, overlap_max - overlap_min)
                if overlap > 0 and overlap < part_depth * 0.95:
                    # Part overlaps parent partially (caliper, sliding tool,
                    # plate on column) — use full overlap as travel.
                    depth = overlap
                else:
                    depth = min(part_depth, body_depth)
            else:
                depth = min(part_depth, body_depth)
            # If drawer has rail mechanism meshes, limit travel to maintain
            # rail-track overlap (rail must not fully exit the body track).
            # Only applies to flat-topology prismatic (drawer in cabinet);
            # chained prismatics like a plate sliding on a column routinely
            # contain 'frame'/'mechanism' meshes that are part structure, not
            # rail hardware — false-positive caps travel to a few cm.
            has_rail = False
            if parent_name == "body":
                drawer_prim = stage.GetPrimAtPath(path)
                if drawer_prim:
                    for child in Usd.PrimRange(drawer_prim):
                        if child.IsA(UsdGeom.Mesh) and any(
                                kw in child.GetName().lower() for kw in _DRAWER_RAIL_KEYWORDS):
                            has_rail = True
                            break
            is_overlap_travel = (bbox and parent_bbox and overlap > 0 and overlap < part_depth * 0.95)
            if has_rail:
                travel = depth * 0.45   # ~45% of total depth keeps rail overlapped
                print(f"    (rail detected — limiting travel to {travel:.3f}m for overlap)")
            elif is_overlap_travel:
                travel = depth  # overlap IS the full useful range, no 85% reduction
                print(f"    (overlap-based travel: {travel:.3f}m = full ruler/slide range)")
            else:
                travel = depth * 0.85
            # Detect slider vs drawer: a slider (caliper, measuring tool, or
            # column height-slide) spans nearly the FULL parent length on the
            # slide axis (>70%). A drawer is much shorter than the parent.
            # Sliders need bidirectional limits; drawers need one-directional.
            is_slider = False
            if bbox and parent_bbox:
                part_extent = abs(bbox[1][axis_idx] - bbox[0][axis_idx])
                parent_extent = abs(parent_bbox[1][axis_idx] - parent_bbox[0][axis_idx])
                if parent_extent > 0:
                    span_ratio = part_extent / parent_extent
                    if span_ratio > 0.9:
                        is_slider = True

            # Chained slider: part is small relative to its (non-body) parent
            # and slides ALONG the parent (e.g. plate on a column). Travel is
            # the remaining space above/below the part within the parent,
            # computed bidirectionally from the part's current position.
            is_chain_slider = (
                parent_name != "body"
                and bbox and parent_bbox
                and abs(parent_bbox[1][axis_idx] - parent_bbox[0][axis_idx]) > 0
                and (abs(bbox[1][axis_idx] - bbox[0][axis_idx])
                     / abs(parent_bbox[1][axis_idx] - parent_bbox[0][axis_idx])) < 0.5
            )

            if is_chain_slider:
                parent_lo = parent_bbox[0][axis_idx]
                parent_hi = parent_bbox[1][axis_idx]
                part_lo = bbox[0][axis_idx]
                part_hi = bbox[1][axis_idx]
                # Up/positive = room above the part; down/negative = room below.
                upper_m = max(0.0, parent_hi - part_hi)
                lower_m = min(0.0, parent_lo - part_lo)
                travel = upper_m - lower_m
                parent_len = parent_hi - parent_lo
                print(f"    (chain slider — along parent axis: [{lower_m:.3f}, {upper_m:.3f}]m = {travel*100:.0f}cm, parent={parent_len*100:.0f}cm)")
            elif is_slider:
                # V13: Slider travels full parent length, both directions.
                # The driving part slides along the parent (ruler, caliper).
                # It can go left until its outer end reaches parent_lo,
                # and right until its inner end reaches parent_hi.
                if bbox and parent_bbox:
                    parent_lo = parent_bbox[0][axis_idx]
                    parent_hi = parent_bbox[1][axis_idx]
                    part_lo = bbox[0][axis_idx]
                    part_hi = bbox[1][axis_idx]

                    upper_m = parent_hi - part_lo
                    lower_m = parent_lo - part_hi

                    parent_len = parent_hi - parent_lo
                    travel = upper_m - lower_m
                    print(f"    (slider — full parent travel both ways: [{lower_m:.3f}, {upper_m:.3f}]m = {travel*100:.0f}cm, parent={parent_len*100:.0f}cm)")
                else:
                    parent_len = abs(parent_bbox[1][axis_idx] - parent_bbox[0][axis_idx]) if parent_bbox else depth
                    lower_m = -parent_len * 0.45
                    upper_m = parent_len * 0.45
                    print(f"    (slider — geometry fallback: [{lower_m:.3f}, {upper_m:.3f}]m)")
            elif bbox and parent_bbox:
                # Drawer / non-slider prismatic: one direction, face toward
                # parent exterior. Compare in PARENT-LOCAL frame so the sign
                # matches the joint axis direction regardless of the parent's
                # world rotation. Seen on EmergencyTrolley (chassis Z-rotation
                # 181.9°): doing this in world space inverted direction.
                parent_prim = stage.GetPrimAtPath(parent_path)
                parent_w2l = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()).GetInverse()
                # F46: prefer sub-mesh (handle/lock/knob/rotor) position over
                # full drawer bbox center for the direction decision. The
                # handle sits on the drawer's OPENING face; its offset along
                # the prismatic axis robustly identifies which face opens,
                # even when the drawer bbox is symmetric about the chassis
                # center (which fools the bbox-center heuristic). Seen on
                # MedicalutilityCart_A03_01 drawer1 (2026-04-18) — symmetric
                # 44cm-wide drawer with a lock on the -X edge, classifier
                # used bbox-center and defaulted to +X → drawer opened into
                # the cart's back face instead of the handle face.
                part_prim = stage.GetPrimAtPath(path)
                handle_kws = ("handle", "knob", "pull", "lock", "rotor",
                              "grip", "latch")
                handle_center_world = None
                if part_prim:
                    best_handle_area = 0.0
                    for desc in Usd.PrimRange(part_prim):
                        # Inspect BOTH Mesh prims and Xform wrappers named
                        # handle/lock/knob/rotor. Compute their mesh bbox
                        # directly from points (mesh_world_bbox expects an
                        # ANCESTOR of a mesh and returns None on a Mesh leaf).
                        nm = desc.GetName().lower()
                        if not any(kw in nm for kw in handle_kws):
                            continue
                        # Collect all descendant-mesh points under this
                        # sub-Xform (or the mesh itself).
                        mesh_prims = []
                        if desc.IsA(UsdGeom.Mesh):
                            mesh_prims.append(desc)
                        else:
                            mesh_prims.extend(_get_all_descendant_meshes(desc))
                        hmin = Gf.Vec3d(1e30, 1e30, 1e30)
                        hmax = Gf.Vec3d(-1e30, -1e30, -1e30)
                        hok = False
                        for mp in mesh_prims:
                            pts = mp.GetAttribute("points")
                            if not pts or not pts.HasValue():
                                continue
                            l2w = UsdGeom.Xformable(mp).ComputeLocalToWorldTransform(
                                Usd.TimeCode.Default())
                            for pt in pts.Get():
                                wp = l2w.TransformAffine(Gf.Vec3d(float(pt[0]), float(pt[1]), float(pt[2])))
                                hmin = Gf.Vec3d(min(hmin[0], wp[0]), min(hmin[1], wp[1]), min(hmin[2], wp[2]))
                                hmax = Gf.Vec3d(max(hmax[0], wp[0]), max(hmax[1], wp[1]), max(hmax[2], wp[2]))
                                hok = True
                        if not hok:
                            continue
                        area = (hmax[0]-hmin[0]) * (hmax[1]-hmin[1])
                        if area > best_handle_area:
                            best_handle_area = area
                            handle_center_world = Gf.Vec3d(
                                (hmin[0]+hmax[0])/2,
                                (hmin[1]+hmax[1])/2,
                                (hmin[2]+hmax[2])/2,
                            )
                dc_world = Gf.Vec3d((bbox[0][0] + bbox[1][0]) / 2,
                                    (bbox[0][1] + bbox[1][1]) / 2,
                                    (bbox[0][2] + bbox[1][2]) / 2)
                bc_world = Gf.Vec3d((parent_bbox[0][0] + parent_bbox[1][0]) / 2,
                                    (parent_bbox[0][1] + parent_bbox[1][1]) / 2,
                                    (parent_bbox[0][2] + parent_bbox[1][2]) / 2)
                bc_local = parent_w2l.TransformAffine(bc_world)
                # F46b: classify.json may carry an explicit sign (+X/-X) to
                # override the auto direction-select. Use that first.
                explicit_sign = info.get("axis_sign", 0)
                if explicit_sign == 1:
                    lower_m, upper_m = 0.0, travel
                    print(f"    (direction-select via classify override: axis +{axis})")
                elif explicit_sign == -1:
                    lower_m, upper_m = -travel, 0.0
                    print(f"    (direction-select via classify override: axis −{axis})")
                elif handle_center_world is not None:
                    # Use handle-vs-drawer-center along the prismatic axis:
                    # the handle sits on the opening face, so opening direction
                    # points FROM drawer center TOWARD the handle.
                    dc_local = parent_w2l.TransformAffine(dc_world)
                    hc_local = parent_w2l.TransformAffine(handle_center_world)
                    if hc_local[axis_idx] < dc_local[axis_idx]:
                        lower_m, upper_m = -travel, 0.0
                    else:
                        lower_m, upper_m = 0.0, travel
                    print(f"    (direction-select via handle/lock: axis {'−' if lower_m < 0 else '+'}{axis}, F46)")
                else:
                    dc_local = parent_w2l.TransformAffine(dc_world)
                    if dc_local[axis_idx] < bc_local[axis_idx]:
                        lower_m, upper_m = -travel, 0.0
                    else:
                        lower_m, upper_m = 0.0, travel
            else:
                lower_m, upper_m = 0.0, travel
            # F40 override: If Gemini gave an explicit range_meters and the
            # bbox-derived travel exceeds it by >3x, trust Gemini. This covers
            # the "deeply-nested small part, inflated bbox" case (buttons on
            # valve assemblies, small levers on complex tools) without
            # second-guessing bbox for normal drawers/sliders.
            if gemini_range and gemini_range > 0:
                bbox_travel = abs(upper_m - lower_m)
                if bbox_travel > gemini_range * 3.0:
                    if gemini_bidir:
                        lower_m, upper_m = -gemini_range, gemini_range
                    else:
                        lower_m, upper_m = 0.0, gemini_range
                    print(f"    (Gemini range override: bbox={bbox_travel:.3f}m → Gemini={gemini_range:.4f}m "
                          f"{'bidirectional' if gemini_bidir else 'one-way'} — F40)")
            make_prismatic_joint(stage, joint_path, parent_path, path,
                                 lp0_f, lp1_f, axis=axis,
                                 lower_m=lower_m, upper_m=upper_m)
            print(f"    PrismaticJoint {name}  axis={axis} travel=[{lower_m:.3f}, {upper_m:.3f}]m parent={parent_name}")
        elif jtype == "continuous":
            # Continuous joints are wheel-tires or caster-tires. Fixed wheels
            # attach to body; 2-DOF caster tires attach to their bracket
            # movable (parent_path already resolved above).
            make_continuous_joint(stage, joint_path, parent_path, path,
                                  lp0_f, lp1_f, axis=axis)
            print(f"    ContinuousJoint {name}  axis={axis} parent={parent_name}")
        elif jtype == "fixed":
            make_fixed_joint(stage, joint_path, parent_path, path, lp0_f, lp1_f)
            print(f"    FixedJoint      {name}  parent={parent_name}")

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
    changed = False
    if "PhysicsArticulationRootAPI" not in items:
        items.append("PhysicsArticulationRootAPI")
        changed = True
    # F45: Enable inter-link self-collisions. PhysX always skips collision
    # between directly-adjacent links (connected by a joint) — so a drawer
    # joined to the chassis will never collide with the chassis, regardless
    # of this flag. But two drawers both joined to the chassis are
    # NON-adjacent to each other, and need this flag set to True to collide.
    # Default=False made drawers pass through each other on
    # MedicalutilityCart_A03_01 (2026-04-18). Enabling it is safe for all
    # assets because adjacent-link collisions are still skipped.
    if "PhysxArticulationAPI" not in items:
        items.append("PhysxArticulationAPI")
        changed = True
    if changed:
        new_list = Sdf.TokenListOp()
        new_list.prependedItems = items
        dp_spec.SetInfo("apiSchemas", new_list)
    # Set the self-collision attr to True (can't be done through dp_spec.SetInfo
    # — need a UsdAttribute). Use the stage-level prim.
    sc_attr = default_prim.GetAttribute("physxArticulation:enabledSelfCollisions")
    if not sc_attr:
        sc_attr = default_prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool)
    sc_attr.Set(True)
    print(f"    ArticulationRootAPI on '{default_prim.GetName()}' (default prim)")
    print(f"    EnabledSelfCollisions=True (non-adjacent links collide — F45)")

    # --- Save ---
    stage.GetRootLayer().Save()
    print(f"\n  SAVED: {output_usd}")


def export_physics_json(usd_path, object_data=None):
    """Export Palatial-compatible physics sidecar JSON.

    Reads the physics USD and produces a JSON with:
    - parts: per-body metadata (mass, bounds, material, friction, description)
    - joint_relations: parent/child, type, axis, limits, damping, stiffness
    - base_link, object_description, total_mass
    """
    import math

    stage = Usd.Stage.Open(str(usd_path))
    if not stage:
        return None

    dp = stage.GetDefaultPrim()
    parts = []
    joints_list = []
    total_mass = 0
    base_link = None

    # ── Collect parts (rigid bodies) ──
    part_idx = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue

        name = prim.GetName()
        mass_attr = prim.GetAttribute("physics:mass")
        mass = mass_attr.Get() if mass_attr and mass_attr.HasValue() else 0
        total_mass += mass

        kin = prim.GetAttribute("physics:kinematicEnabled")
        is_kin = kin.Get() if kin and kin.HasValue() else False

        # Bounds
        bmin = [1e30]*3; bmax = [-1e30]*3; found = False
        for desc in Usd.PrimRange(prim):
            if desc.GetTypeName() != "Mesh": continue
            pts = desc.GetAttribute("points")
            if not pts or not pts.HasValue(): continue
            l2w = UsdGeom.Xformable(desc).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            for pt in pts.Get():
                wp = l2w.TransformAffine(Gf.Vec3d(float(pt[0]), float(pt[1]), float(pt[2])))
                for i in range(3):
                    bmin[i] = min(bmin[i], wp[i])
                    bmax[i] = max(bmax[i], wp[i])
                found = True

        bounds = None
        volume = 0
        if found:
            center = [round((bmin[i]+bmax[i])/2, 6) for i in range(3)]
            bounds = {
                "center": center,
                "max": [round(bmax[i], 6) for i in range(3)],
                "min": [round(bmin[i], 6) for i in range(3)],
            }
            w = abs(bmax[0]-bmin[0])
            d = abs(bmax[1]-bmin[1])
            h = abs(bmax[2]-bmin[2])
            volume = w * d * h

        # Friction from material binding
        sf = df = rest = 0
        bind = UsdShade.MaterialBindingAPI(prim)
        phys_bind = bind.GetDirectBinding("physics")
        if phys_bind and phys_bind.GetMaterialPath():
            mat_prim = stage.GetPrimAtPath(phys_bind.GetMaterialPath())
            if mat_prim:
                sfa = mat_prim.GetAttribute("physics:staticFriction")
                dfa = mat_prim.GetAttribute("physics:dynamicFriction")
                ra = mat_prim.GetAttribute("physics:restitution")
                sf = round(sfa.Get(), 4) if sfa and sfa.HasValue() else 0
                df = round(dfa.Get(), 4) if dfa and dfa.HasValue() else 0
                rest = round(ra.Get(), 4) if ra and ra.HasValue() else 0

        # Collider count
        n_col = sum(1 for d in Usd.PrimRange(prim) if d.HasAPI(UsdPhysics.CollisionAPI))

        # Determine if this is the base link (body/kinematic or first rigid body)
        if is_kin or (base_link is None and part_idx == 0):
            base_link = name
            is_root = True
        else:
            is_root = False

        # Material name from Gemini or guess from USD material
        material_name = "unknown"
        if object_data:
            material_name = object_data.get("material", "unknown")

        part = {
            "canonical_name": name,
            "part_name": name,
            "part_index": part_idx,
            "part_id": name,
            "material_name": material_name,
            "material": material_name,
            "mass": round(mass, 4),
            "density": 0,
            "volume": round(volume, 8),
            "bounds": bounds,
            "static_friction": sf,
            "dynamic_friction": df,
            "restitution": rest,
            "friction_combine_mode": "average",
            "restitution_combine_mode": "average",
            "physics_type": "rigid",
            "is_structural_root": is_root,
            "collider_count": n_col,
            "confidence": "high",
        }
        parts.append(part)
        part_idx += 1

    # ── Collect joints ──
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue

        jtype_raw = prim.GetTypeName()
        jtype = "fixed"
        if "Revolute" in jtype_raw: jtype = "revolute"
        elif "Prismatic" in jtype_raw: jtype = "prismatic"

        axis_attr = prim.GetAttribute("physics:axis")
        axis_val = axis_attr.Get() if axis_attr and axis_attr.HasValue() else ""
        axis_vec = {"X": [1,0,0], "Y": [0,1,0], "Z": [0,0,1]}.get(axis_val, [])

        lo_attr = prim.GetAttribute("physics:lowerLimit")
        hi_attr = prim.GetAttribute("physics:upperLimit")
        lo = lo_attr.Get() if lo_attr and lo_attr.HasValue() else None
        hi = hi_attr.Get() if hi_attr and hi_attr.HasValue() else None
        motion_limits = {"min": round(lo, 4), "max": round(hi, 4)} if lo is not None and hi is not None else None

        lp0 = prim.GetAttribute("physics:localPos0")
        lp0_val = lp0.Get() if lp0 and lp0.HasValue() else None
        origin = [round(float(lp0_val[i]), 6) for i in range(3)] if lp0_val else [0,0,0]

        b0 = prim.GetRelationship("physics:body0").GetTargets()
        b1 = prim.GetRelationship("physics:body1").GetTargets()
        parent = b0[0].name if b0 else ""
        child = b1[0].name if b1 else ""

        # Drive params
        damping = stiffness = 0
        for attr in prim.GetAttributes():
            n = attr.GetName()
            if "damping" in n.lower() and "drive" in n.lower() and attr.HasValue():
                damping = round(attr.Get(), 4)
            if "stiffness" in n.lower() and "drive" in n.lower() and attr.HasValue():
                stiffness = round(attr.Get(), 4)

        # Joint friction
        jf = prim.GetAttribute("physxJoint:jointFriction")
        friction = round(jf.Get(), 4) if jf and jf.HasValue() else 0

        joint = {
            "child": child,
            "parent": parent,
            "joint_type": jtype,
            "name": prim.GetName(),
            "damping": damping,
            "effort": 0,
            "friction": friction,
            "stiffness": stiffness,
            "velocity": 0,
            "origin": origin,
            "axis": axis_vec,
            "motion_limits": motion_limits,
            "score": 0.95,
            "joint_origin_local": origin,
        }
        if axis_vec:
            joint["joint_axis_vector_local"] = axis_vec
        if jtype == "revolute":
            joint["revolute_mode"] = "hinge"
        joints_list.append(joint)

    # ── Build final JSON ──
    obj_desc = ""
    if object_data:
        obj_desc = f"{object_data.get('object_name', 'unknown')}. {object_data.get('special_notes', '')}"

    physics_json = {
        "parts": parts,
        "joint_relations": joints_list,
        "base_link": base_link,
        "object_description": obj_desc.strip(),
        "total_mass": round(total_mass, 4),
        "center_of_mass": None,
        "inertia": None,
    }

    # Save next to USD
    json_path = str(usd_path).replace("_physics.usd", "_physics.json").replace(".usd", "_physics.json")
    import json as _json
    with open(json_path, "w") as f:
        _json.dump(physics_json, f, indent=2)
    print(f"\n  PHYSICS JSON: {json_path}")
    return json_path


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
    print(f"  make_simready (V13)")
    print(f"{'='*60}")
    print(f"  Input: {input_usd}")
    print(f"  Mode:  {'AUDIT + FIX' if fix else 'AUDIT ONLY'}")

    stage = Usd.Stage.Open(input_usd)

    # Phase 1: Audit
    results = audit(stage)
    print_audit(results, label="AUDIT (current state)")

    all_pass = all(r["pass"] for k, r in results.items() if not k.startswith("_"))
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

    # Re-audit — pass classification so C5 can catch serial-chain collapse
    # (classifier declared N movables, pipeline produced fewer joints).
    final_stage = Usd.Stage.Open(output_usd)
    final_results = audit(final_stage, classification=classification)
    print_audit(final_results, label="AUDIT (after fix)")

    # Summary
    n_rigid = sum(1 for p in final_stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    n_col = sum(1 for p in final_stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
    n_joints = sum(1 for p in final_stage.Traverse() if "Joint" in p.GetTypeName())
    print(f"\n  SUMMARY: {n_rigid} rigid bodies, {n_col} colliders, {n_joints} joints")

    # V13: Physics JSON sidecar (Palatial-compatible)
    obj_data = None
    if object_json and os.path.exists(object_json):
        with open(object_json) as f:
            obj_data = json.load(f)
    export_physics_json(output_usd, object_data=obj_data)

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

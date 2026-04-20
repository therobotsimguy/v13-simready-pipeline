#!/usr/bin/env python3
"""V13 Phase 6 — audit regression harness.

Every fix function in make_simready.py has an audit check that claims
to catch the failure if it reappears. But how do we know the audit
check is *sufficient*? The F35 scissors case proved an audit can exist
and still miss the regression (it checked global collider count instead
of per-body).

This test suite constructs minimal USD stages that exhibit specific
failure modes, runs audit(), and asserts the expected F-number is
cited in the FAIL message of the right criterion. If audit silently
passes, the regression would slip through — the test fails.

Covers 5 representative entries from fixes.json (ENFORCED set):
  F01   mpu != 1.0 not caught              (C7)
  F11   nested RigidBody not caught         (C1)
  F33   PhysicsScene in asset not caught    (C7)
  F47   zero-thickness collider not caught  (C2)
  F63   orphan sibling Xform not caught     (C2)

Extend by adding more `def test_F##_...` functions. Pattern:
  1. Build a baseline valid stage via `_minimal_stage()`
  2. Mutate it into the specific failing shape
  3. Call audit()
  4. Assert the F-number is in the right criterion's detail,
     and that criterion didn't pass

Usage:
  pytest test_audit_fixes.py -v
  # or:
  python3 test_audit_fixes.py            (runs all tests, prints PASS/FAIL)

Exit 0 if all pass, 1 otherwise.
"""
from __future__ import annotations

import sys
from pathlib import Path

V13 = Path(__file__).resolve().parent
sys.path.insert(0, str(V13 / "scripts/tools/simready_assets"))

try:
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
except ImportError:
    sys.exit("pxr (USD Python bindings) required — run inside Isaac Sim env")

from make_simready import audit  # type: ignore


# ─────────────────────────────────────────────────────────────────────
# Stage builders
# ─────────────────────────────────────────────────────────────────────

def _minimal_stage():
    """Valid baseline: /World default prim, ArticulationRootAPI, one
    RigidBody with MassAPI + descendant Mesh carrying CollisionAPI,
    metersPerUnit=1.0, no PhysicsScene."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(world.GetPrim())

    body = UsdGeom.Xform.Define(stage, "/World/body")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
    mass.CreateMassAttr(10.0)

    mesh = UsdGeom.Mesh.Define(stage, "/World/body/mesh")
    mesh.CreatePointsAttr([
        Gf.Vec3f(-0.1, -0.1, -0.1), Gf.Vec3f(0.1, -0.1, -0.1),
        Gf.Vec3f(0.1, 0.1, -0.1), Gf.Vec3f(-0.1, 0.1, -0.1),
        Gf.Vec3f(-0.1, -0.1, 0.1), Gf.Vec3f(0.1, -0.1, 0.1),
        Gf.Vec3f(0.1, 0.1, 0.1), Gf.Vec3f(-0.1, 0.1, 0.1),
    ])
    mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    mesh.CreateFaceVertexIndicesAttr([
        0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4,
        2, 3, 7, 6, 0, 3, 7, 4, 1, 2, 6, 5,
    ])
    coll = UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    approx = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    approx.CreateApproximationAttr("convexHull")

    # Physics material binding so C3 doesn't flag.
    mat = UsdShade.Material.Define(stage, "/World/PhysicsMat") \
        if False else None  # C3 is informational; skip for minimal stage
    return stage


# ─────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────

def _assert_detail_cites(results, criterion, fid, should_fail=True):
    assert criterion in results, \
        f"criterion {criterion!r} not in audit results keys: {list(results.keys())}"
    r = results[criterion]
    detail = r.get("detail", "") or ""
    passed = r.get("pass", True)
    assert fid in detail, (
        f"{fid} expected in {criterion} detail, got: {detail!r}"
    )
    if should_fail:
        assert not passed, (
            f"{criterion} expected to FAIL because of {fid}, "
            f"but passed. detail: {detail!r}"
        )


# ─────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────

def test_F01_detects_non_meter_stage():
    """C7 must FAIL with F01 cited when stage mpu != 1.0."""
    stage = _minimal_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)  # centimeters
    results = audit(stage)
    _assert_detail_cites(results, "C7 Clean Asset", "F01")


def test_F11_detects_nested_rigid_body():
    """C1 must FAIL with F11 cited when a RigidBody has a RigidBody parent."""
    stage = _minimal_stage()
    # Add a nested body under /World/body (which already has RigidBodyAPI).
    nested = UsdGeom.Xform.Define(stage, "/World/body/nested")
    UsdPhysics.RigidBodyAPI.Apply(nested.GetPrim())
    UsdPhysics.MassAPI.Apply(nested.GetPrim()).CreateMassAttr(1.0)
    results = audit(stage)
    _assert_detail_cites(results, "C1 Rigid Bodies", "F11")


def test_F33_detects_physics_scene_in_asset():
    """C7 must FAIL with F33 cited when a PhysicsScene prim exists."""
    stage = _minimal_stage()
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    results = audit(stage)
    _assert_detail_cites(results, "C7 Clean Asset", "F33")


def test_F47_detects_zero_thickness_collider():
    """C2 must FAIL with F47 cited when a collider mesh is degenerate
    (any bbox axis < eps)."""
    stage = _minimal_stage()
    # Add a flat decal: all Z coords identical → zero thickness.
    decal = UsdGeom.Mesh.Define(stage, "/World/body/decal")
    decal.CreatePointsAttr([
        Gf.Vec3f(-0.1, -0.1, 0.2), Gf.Vec3f(0.1, -0.1, 0.2),
        Gf.Vec3f(0.1, 0.1, 0.2), Gf.Vec3f(-0.1, 0.1, 0.2),
    ])
    decal.CreateFaceVertexCountsAttr([4])
    decal.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    UsdPhysics.CollisionAPI.Apply(decal.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(decal.GetPrim()).CreateApproximationAttr("convexHull")
    results = audit(stage)
    _assert_detail_cites(results, "C2 Collision Shapes", "F47")


def test_F63_detects_orphan_sibling_xform():
    """C2 must FAIL with F63 cited when a Mesh has no RigidBody
    ancestor (orphan sibling Xform with visible geometry)."""
    stage = _minimal_stage()
    # Orphan: Mesh outside body, no RigidBody in its ancestor chain.
    orphan_xform = UsdGeom.Xform.Define(stage, "/World/orphan")
    orphan_mesh = UsdGeom.Mesh.Define(stage, "/World/orphan/mesh")
    orphan_mesh.CreatePointsAttr([
        Gf.Vec3f(0.5, -0.05, -0.05), Gf.Vec3f(0.6, -0.05, -0.05),
        Gf.Vec3f(0.6, 0.05, -0.05), Gf.Vec3f(0.5, 0.05, -0.05),
        Gf.Vec3f(0.5, -0.05, 0.05), Gf.Vec3f(0.6, -0.05, 0.05),
        Gf.Vec3f(0.6, 0.05, 0.05), Gf.Vec3f(0.5, 0.05, 0.05),
    ])
    orphan_mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    orphan_mesh.CreateFaceVertexIndicesAttr([
        0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4,
        2, 3, 7, 6, 0, 3, 7, 4, 1, 2, 6, 5,
    ])
    results = audit(stage)
    _assert_detail_cites(results, "C2 Collision Shapes", "F63")


# ─────────────────────────────────────────────────────────────────────
# Negative tests — baseline stage must PASS all criteria it's built for
# ─────────────────────────────────────────────────────────────────────

def test_baseline_does_not_false_positive():
    """Minimal valid stage should NOT trigger any of the F##
    conditions we test for. Guards against audit becoming overly
    strict and failing clean assets."""
    stage = _minimal_stage()
    results = audit(stage)
    for criterion, fid in [
        ("C1 Rigid Bodies", "F11"),
        ("C7 Clean Asset", "F01"),
        ("C7 Clean Asset", "F33"),
        ("C2 Collision Shapes", "F47"),
        ("C2 Collision Shapes", "F63"),
    ]:
        detail = results[criterion].get("detail", "") or ""
        assert fid not in detail, (
            f"baseline stage should not cite {fid} in {criterion}, "
            f"got: {detail!r}"
        )


# ─────────────────────────────────────────────────────────────────────
# CLI runner (if pytest isn't used)
# ─────────────────────────────────────────────────────────────────────

TESTS = [
    test_F01_detects_non_meter_stage,
    test_F11_detects_nested_rigid_body,
    test_F33_detects_physics_scene_in_asset,
    test_F47_detects_zero_thickness_collider,
    test_F63_detects_orphan_sibling_xform,
    test_baseline_does_not_false_positive,
]


def main():
    # UsdShade is imported lazily by _minimal_stage if needed.
    global UsdShade
    try:
        from pxr import UsdShade as _UsdShade  # noqa: F401
        UsdShade = _UsdShade
    except ImportError:
        UsdShade = None

    print(f"running {len(TESTS)} audit regression tests...\n")
    failures = []
    for t in TESTS:
        try:
            t()
            print(f"  PASS  {t.__name__}")
        except AssertionError as e:
            print(f"  FAIL  {t.__name__}\n        {e}")
            failures.append((t.__name__, str(e)))
        except Exception as e:
            print(f"  ERROR {t.__name__}\n        {type(e).__name__}: {e}")
            failures.append((t.__name__, f"{type(e).__name__}: {e}"))

    print(f"\nresult: {len(TESTS) - len(failures)}/{len(TESTS)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

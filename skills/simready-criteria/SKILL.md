---
name: simready-criteria
description: >-
  The 7 criteria that define a SimReady USD asset. Use as a judge to evaluate
  any USD file — rigid or articulated — and determine what physics properties
  are present vs missing. Empirically derived from two working assets
  (InstrumentTrolley_B, Refrigerator_A) tested with Franka teleop in Isaac Sim.
  Extended with C8 (cross-solver validation), C9 (scale viability at IsaacLab
  batch sizes), C10 (GPU-batchable vs CPU-high-fidelity tier certification),
  and C11 (Newton / PhysX dual-output parity).
---

# SimReady Criteria Skill

## Purpose

This skill is a **judge**. Given any USD file, evaluate it against 7 criteria
and produce a pass/fail verdict. If it fails, the criteria tell you exactly
what to add.

## The 7 Criteria

### C1: Rigid Bodies

Every independently-moving group must have `RigidBodyAPI` + `MassAPI`.

| Check | Rule |
|-------|------|
| Main body | Has `RigidBodyAPI`. Dynamic with plausible mass, OR kinematic. |
| Each movable part | Has `RigidBodyAPI` + `MassAPI`. Mass > 0 and physically plausible. |
| Grandchildren | Must NOT have `RigidBodyAPI` (nested rigid body error). |

**How to estimate mass:** `bbox_volume_m3 * density_kg_m3`, clamped to plausible range.
Default density: 500 (wood/plastic mix). Body: 600. Metal parts: 800.
For per-category mass ranges, see **simready-joint-params**. For absolute clamps, see **failure-modes**.

### C2: Collision Shapes

Meshes under rigid bodies must have `CollisionAPI` + `MeshCollisionAPI` with
an approximation type.

| Check | Rule |
|-------|------|
| Every rigid body | Has at least one Mesh child with `CollisionAPI` |
| Approximation type | Set on each collider (not left as `none`) |
| Budget | Max ~5 `convexDecomposition` meshes per asset |

For collision geometry selection rules and anti-patterns, see **collision-physics** skill.

### C3: Friction Materials

Every collider must have friction bound via `material:binding:physics`.

| Check | Rule |
|-------|------|
| Every collision mesh | Has `material:binding:physics` relationship pointing to a Material prim |
| Target material | Has `PhysicsMaterialAPI` with `staticFriction` and `dynamicFriction` set |
| Handle/knob meshes | Bound to `GripMaterial` (sf=1.0, df=0.9) |
| Coverage | 100% — no collider without a physics material binding |

For full friction coefficient table, see **simready-joint-params**. Key rule: GripMaterial (sf=1.0, df=0.9) on handles. Default fallback: sf=0.5, df=0.4.

**How to bind:** If the mesh already has a visual material (via `material:binding`),
add `PhysicsMaterialAPI` to that material prim and create `material:binding:physics`
pointing to it. If no visual material exists, create/use a `DefaultPhysMaterial`.

### C4: Flat Hierarchy

Movable parts must be **siblings** of the body under the default prim,
not nested children.

| Check | Rule |
|-------|------|
| Each movable Xform | Is a direct child of the default prim |
| Structural parts | Remain under the body Xform (shelves, dividers, interior) |
| Structural sub-parts of movable assemblies | Moved under body (e.g., wheel fixer/bolts go under body, only rotating parts stay under wheel Xform) |

**Why:** PhysX swallows nested `RigidBodyAPI` prims into the parent body.
A door that is a child of the body will not move independently.

**Before (fails C4):**
```
/root
  /body          [RigidBody]
    /door1       [RigidBody]  <-- nested! PhysX merges into body
    /wheel1      [RigidBody]  <-- nested!
```

**After (passes C4):**
```
/root
  /body          [RigidBody]
  /door1         [RigidBody]  <-- sibling, independent
  /wheel1        [RigidBody]  <-- sibling, independent
  /joints
```

### C5: Joints (existence + anchor validity)

Every movable part must be connected to the body via a physics joint
with correct anchor positions.

| Check | Rule |
|-------|------|
| Joint scope | All joints under a `/joints` Scope prim |
| Every movable part | Has exactly one joint connecting it to body |
| Joint type | Matches the motion (see table below) |
| body0 | Points to the body Xform |
| body1 | Points to the movable part Xform |
| localPos0 / localPos1 | Computed via `world_point_to_local()` from anchor world point |
| **Anchor validity** | At least one of localPos0/localPos1 must be non-zero. Both at (0,0,0) = broken joint (part pinned to origin). |

**Critical:** Save joint anchors BEFORE reparenting. Reparenting clears
pivot xformOps — if anchors are read after, pivots are gone and all
localPos values fall back to (0,0,0).

**Joint type mapping:**

| Motion | Joint type | Axis | Limits |
|--------|-----------|------|--------|
| Door (vertical hinge) | `RevoluteJoint` | Z | [-120, 0] or [0, 120] based on hinge edge |
| Lid/flap (horizontal hinge) | `RevoluteJoint` | X | [-90, 0] or [0, 90] |
| Drawer | `PrismaticJoint` | Y | [0, depth*0.85] |
| Wheel/caster | `RevoluteJoint` | Axle direction (X or Y) | No limits (unlimited) |
| Button | `PrismaticJoint` | Z | [0, 0.005] |
| Fixed structural sub-body | `FixedJoint` | N/A | N/A |

**Joint anchor:** Use the prim's pivot xformOp if present (transformed by L2W),
otherwise use the Xform's world-space origin. Both localPos0 and localPos1 must
be computed via `world_point_to_local()` — never hardcode (0,0,0).

### C6: Joint Drives

Every joint must have `DriveAPI` with appropriate damping.

| Check | Rule |
|-------|------|
| Every joint | Has `PhysicsDriveAPI` (angular for revolute, linear for prismatic) |
| Damping | > 0 (prevents free-fall oscillation) |
| Stiffness | = 0 (no spring return) |

For per-category damping values, see **simready-joint-params**. For absolute clamps, see **failure-modes** Drive Parameters Reference.

### C7: Clean Asset (no scene, correct units)

The asset must NOT contain host-app responsibilities and must be in meters.

| Check | Rule |
|-------|------|
| No `PhysicsScene` | Host app provides the physics scene |
| No `contactOffset` | Set at runtime by teleop script (0.00005) |
| No `simulationOwner` | Not needed when no PhysicsScene in asset |
| **metersPerUnit = 1.0** | Output must be in meters. Assets in cm/mm are normalized during Phase 3. |
| `ArticulationRootAPI` | Optional — not required, not harmful |

**Why metersPerUnit matters:** If metersPerUnit = 0.01 (centimeters), Isaac Lab
spawns the asset 100x too large. The pipeline normalizes to meters during Phase 3,
but the audit must verify the output is correct.

### C8: Cross-Solver Validation

Advanced audit for assets that target multiple engines or require quantified fidelity. Applies especially to dual-output V13 assets.

| Check | Rule |
|-------|------|
| Cross-simulator trajectory match | State-trajectory MSE < 1e-4 over 10s gravity drop across PhysX + MuJoCo + Newton |
| Sensitivity to friction/mass | Behavior stable under ±10% perturbation on friction and mass |
| Sim-to-real error bounds | Gripper gap < 5mm, door-open time < 0.5s vs real-world measurement |
| Save/resume determinism | Never resume mid-contact (cross-ref S09). Always restart from beginning |

**Pass gate:** all 4 sub-checks pass. Warning-level failures are acceptable for V13.0; must escalate to blockers by V13.2.

<!-- source: bundle3/validation_report + bundle4/validation_gauntlet + bundle2/corrected_brief §17 + bundle5 s2r memo, confidence: HIGH -->

### C9: Scale Viability

Does the asset survive IsaacLab batched training?

| Check | Rule |
|-------|------|
| Single-env stability | No NaN, bounded oscillation at N=1 |
| Small-batch stability | Same behavior at N=4 as N=1 |
| Target-batch stability | Stable at target N (32 for articulated; 128–256 for soft bodies) |
| Solver tuning audit | `solver_position_iteration_count` and `dt` appropriate for asset complexity |

**Heuristic:** if the asset has more than ~5 convex decomposition hulls OR a GPU-incompatible topology (K13), flag for C10 tier review. Cross-ref F58.

<!-- source: bundle5 CI-0037 + bundle2/findings_file(2)/0_gpu_simulation_at_scale, confidence: HIGH -->

### C10: Tier Certification

Classify the asset at generation time as **GPU-batchable** OR **CPU/offline high-fidelity**. Drives pipeline routing.

**GPU-batchable tier** — must ALL be true:
- Tree articulation (no loops, OR loops handled via tree-plus-closure with explicit Baumgarte tuning)
- Low DOF (≤~20 joints)
- Convex collision only (primitives + single convex hull, OR convex decomposition within GPU hull budget ≤64 verts)
- Regular memory layout (SoA-compatible)

**CPU/offline high-fidelity tier** — assets with:
- Native closed loops (>2)
- Screw / gear transmissions with exact coupling
- Hydraulic compliance or complex deformable coupling
- Articulation + granular (MPM) — must use Newton
- Deformable assets with FEM (PhysX) or VBD (Newton)

**Never force a mismatched mechanism into GPU tier.** Soft fail (degraded physics) is worse than explicit routing to CPU offline.

**Routing:**
- GPU-batchable → Isaac Lab (PhysX GPU) + Newton GPU (Featherstone or VBD)
- CPU/offline → Newton offline (Featherstone or custom backend) OR PhysX CPU

<!-- source: bundle6/final_report §8 (GPU vs CPU tradeoffs), confidence: HIGH -->

### C11: Newton / PhysX Dual-Output Parity

If the asset ships to BOTH Isaac Sim (PhysX) and Newton as separate products, outputs must behave equivalently on a defined test battery. See `newton-physx-compat-matrix §7` for the full parity procedure.

| Test | Metric | Tolerance |
|------|-------|-----------|
| Gravity drop 10s | Final COM position delta | < 5 mm |
| Teleop push 100N lateral | Max joint deflection delta | < 5% |
| Joint sweep full range | Time-to-complete delta | < 10% |
| Contact force peaks | Max contact force ratio PhysX/Newton | 0.9 < ratio < 1.1 |
| State trajectory MSE | Position state vector, 10s | < 1e-4 |

**Known parity gaps (flag, don't silently fail):**
- PhysX SDF collision vs Newton — Newton has no SDF support (use convex decomp for dual-output).
- PhysX compliant contact (5.1+) vs Newton VBD — match within ±10%.
- PhysX articulation damping vs Featherstone — slightly different numerical integration.

**Initial rollout (V13.0):** C11 is warning-level. Promote to failure gate after empirical validation on the reference asset pair (Refrigerator_A, InstrumentTrolley_B).

<!-- source: bundle2/newton_report_corrections + bundle6/final_report §Architecture, confidence: HIGH (procedure) / MED (exact tolerances) -->

## Part Classification Guide (for LLM)

When reading a USD hierarchy, classify each Xform child of the body into:

| Classification | Description | Physics treatment |
|---------------|-------------|-------------------|
| **body** | Main structural Xform (largest, most meshes) | RigidBodyAPI + MassAPI + collision |
| **movable:revolute** | Doors, lids, flaps — hinged rotation | Sibling + RigidBody + RevoluteJoint |
| **movable:prismatic** | Drawers, sliding panels — linear motion | Sibling + RigidBody + PrismaticJoint |
| **movable:continuous** | Wheels, casters — unlimited rotation | Sibling + RigidBody + RevoluteJoint (no limits) |
| **structural** | Shelves, dividers, interior parts — don't move | Stay under body, no joint |
| **decorative** | Bolts, clips, logos, LEDs — no physics needed | No RigidBody, no collision |

**Classification signals:**
- Name contains "door", "lid", "flap" → movable:revolute
- Name contains "drawer", "slide" → movable:prismatic
- Name contains "wheel", "caster", "tire" → movable:continuous
- Name contains "shelf", "divider", "interior", "panel" → structural
- Name contains "bolt", "clip", "logo", "led", "light" → decorative
- Has pivot xformOp → likely movable (rotation center encoded)
- Thin tall bbox → door; box-shaped bbox → drawer; small round → wheel
- Mesh child named "handle" or "knob" → parent is movable

## Pass/Fail Scoring

| Asset type | Required criteria |
|-----------|-------------------|
| Rigid (static prop, no moving parts) | C1, C2, C3, C7 |
| Articulated (doors, drawers, wheels) | All 7: C1-C7 |

An asset is **SimReady** when all applicable criteria pass.

The audit script outputs:
```
PASS  — criterion fully satisfied
FAIL  — criterion violated (details of what's missing)
N/A   — criterion not applicable (e.g., no movable parts → C4/C5/C6 skip)
```

## What is NOT in this skill

- **Operational memory** (Blender bugs, historical pipeline lessons) → `memory/MASTER.md`
- **Running commands** (how to launch Isaac Sim, teleop) → `CLAUDE.md`
- **Collision details** (investigation tables, anti-patterns) → `simready-collision` skill
- **Robot constraints** (Franka grip force, reach) → `robot-model` skill
- **Math functions** (bbox, transforms, units) → `simready-math` skill

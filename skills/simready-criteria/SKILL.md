---
name: simready-criteria
description: >-
  The 7 criteria that define a SimReady USD asset. Use as a judge to evaluate
  any USD file — rigid or articulated — and determine what physics properties
  are present vs missing. Empirically derived from two working assets
  (InstrumentTrolley_B, Refrigerator_A) tested with Franka teleop in Isaac Sim.
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

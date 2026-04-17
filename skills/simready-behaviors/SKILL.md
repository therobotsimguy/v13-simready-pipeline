---
name: simready-behaviors
description: >-
  16 behavior types × 15 semantic constraints for SimReady articulated assets.
  Use when classifying how a part moves (door, drawer, knob, lever), setting
  joint parameters, choosing physics APIs, or validating that an asset's
  behavior is physically correct for Franka Panda manipulation.
---

# SimReady Behaviors Skill

## The 16 Behaviors

### Original 8 (Part 2 of source doc)

| # | Behavior | Joint Type | Axis | Example |
|---|----------|-----------|------|---------|
| 1 | **ROTATIONAL** | RevoluteJoint | Z (vertical hinge) | Cabinet door, jar lid, valve |
| 2 | **LINEAR TRANSLATIONAL** | PrismaticJoint | Y (depth) | Drawer, sliding door, button |
| 3 | **GRASPING/GRIPPING** | Gripper drive | N/A | Picking up objects |
| 4 | **INSERTION/ASSEMBLY** | Multi-axis | Varies | Peg-in-hole, plug, key |
| 5 | **DEFORMATION** | FEM soft body | N/A | Foam, rubber, cloth |
| 6 | **CONTACT-BASED** | PrismaticJoint (short) | Z | Button press, tap, stroke |
| 7 | **SEQUENTIAL/COMPOUND** | Multiple joints | Varies | Unlock then open |
| 8 | **DYNAMIC/BALLISTIC** | Free body | N/A | Throwing, dropping |

### Extended 8 (Part 4 of source doc)

| # | Behavior | Physics Model | Example |
|---|----------|--------------|---------|
| 9 | **SLIDING/FRICTION** | RigidBody + friction material | Pushing box on table |
| 10 | **WIPING/SWEEPING** | Impedance control + contact | Cleaning a surface |
| 11 | **TWISTING/TORQUE** | RevoluteJoint (continuous) | Turning a screw, bottle cap |
| 12 | **STACKING/PLACEMENT** | RigidBody + gravity settle | Stacking blocks |
| 13 | **COMPLIANT/FORCE-CONTROLLED** | Impedance/admittance control | Polishing, assembly with contact |
| 14 | **IMPACT/STRIKING** | High-velocity collision | Hammering, tapping |
| 15 | **PULLING/TENSION** | Prismatic + high friction | Pulling a stuck drawer, unplugging |
| 16 | **ROLLING** | RigidBody + friction + torque | Rolling a ball, cylinder |

## The 15 Semantic Constraint Domains

Every behavior is validated against these 15 domains:

1. **Directional** — force/motion direction matches intent
2. **Range Limits** — joint limits, workspace bounds
3. **Pivot Placement** — rotation axis at correct position
4. **Clearance/Tolerance** — no self-collision, gaps maintained
5. **Sequential Dependency** — unlock before open, grasp before pull
6. **Force/Torque Realism** — within Franka's 70N grip, 87Nm joints
7. **Contact/Friction** — friction coefficients, contact stability
8. **Symmetry** — symmetric vs asymmetric motion
9. **Material Properties** — stiffness, compliance, density
10. **Internal Volume** — contents prevent certain motions
11. **Kinematic Chain** — DOF count, joint types, singularities
12. **Energy** — gravity-driven vs powered, energy conservation
13. **Feedback** — force sensors, limit switches
14. **Safety** — hard stops, velocity limits, force limits
15. **Aesthetic** — visual appearance of motion

For joint parameters (damping, limits, mass, friction) per object type, see **simready-joint-params**.
For Franka force/reach constraints, see **robot-model**.

## Manifest Output (for make_simready.py)

When classifying an asset for the V8 pipeline, the LLM reads the USD hierarchy
and produces a `manifest.json` that `make_simready.py` consumes. The manifest
maps each part to a behavior from the tables above.

### Format

```json
{
  "body": "<name of the main body Xform>",
  "parts": {
    "<part_name>": {"joint": "revolute", "axis": "Z"},
    "<part_name>": {"joint": "prismatic", "axis": "Y"},
    "<part_name>": {"joint": "continuous", "axis": "X"},
    "<part_name>": {"joint": "fixed"},
    "<part_name>": {"joint": "structural"}
  }
}
```

### Behavior to joint mapping

| Behavior | `joint` value | `axis` | Notes |
|----------|--------------|--------|-------|
| ROTATIONAL (door, lid, flap) | `revolute` | `Z` (vertical hinge), `X` (horizontal hinge) | Limits from Quick Reference |
| LINEAR TRANSLATIONAL (drawer) | `prismatic` | `Y` (depth), `X` (lateral) | Travel computed from bbox |
| TWISTING/TORQUE (knob, cap) | `revolute` | `Z` | Use short angular limits |
| ROLLING (wheel, caster) | `continuous` | `X` or `Y` (axle axis) | Unlimited rotation [-9999, 9999]. **Detect from tire bbox:** thin dimension = axle. LLM often gets this wrong — always verify. |
| CONTACT-BASED (button) | `prismatic` | `Z` | Very short travel (5mm) |
| Structural (shelf, divider) | `fixed` or `structural` | — | `fixed` = separate rigid body with FixedJoint; `structural` = stays part of body |

### How the LLM should classify

1. Read the USD hierarchy (prim names, parent/child structure)
2. For each non-body Xform child, match to the closest behavior from the 16 types
3. Set `joint` and `axis` from the mapping table above
4. Parts not listed or marked `structural` stay part of the body (no RigidBodyAPI)

### Example

A fridge with two doors and one drawer:
```json
{
  "body": "sm_refrigerator_a01",
  "parts": {
    "door_top": {"joint": "revolute", "axis": "Z"},
    "door_bottom": {"joint": "revolute", "axis": "Z"},
    "drawer_freezer": {"joint": "prismatic", "axis": "Y"},
    "shelf_01": {"joint": "structural"},
    "shelf_02": {"joint": "structural"}
  }
}
```

## Full Reference

For complete behavior × constraint matrices, valid/invalid JSON specs, Isaac Sim API mappings, and Blender asset requirements, see:

- [COMPLETE_BEHAVIOR_SEMANTIC_MAPPING.md](../../../scripts/tools/simready_assets/COMPLETE_BEHAVIOR_SEMANTIC_MAPPING.md)

Parts 2 & 3 cover original 8 behaviors. Part 4 covers extended 8 behaviors.
Each behavior has: constraint matrix, Isaac Sim parameters, valid/invalid JSON, Blender requirements, validation protocol.

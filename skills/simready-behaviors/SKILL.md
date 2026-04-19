---
name: simready-behaviors
description: >-
  16 behavior types × 15 semantic constraints for SimReady articulated assets.
  Use when classifying how a part moves (door, drawer, knob, lever), setting
  joint parameters, choosing physics APIs, or validating that an asset's
  behavior is physically correct for Franka Panda manipulation. Extended with
  2 compliant-mechanism behaviors (compliant_hinge, compliant_slider via Howell
  PRBM) and the mimic-joint stability constraint (stiffness × dt² ≈ 1).
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

### Compliant Mechanism Behaviors (17–18)

For flexure-based parts: spring-loaded, leaf-spring, flexure bearings, bellows. Uses **Howell's Pseudo-Rigid-Body Model (PRBM)** — rigid link + torsional or linear spring approximates flexure behavior without full FEM.

| # | Behavior | Joint Type | Example | When to escalate to FEM |
|---|----------|-----------|---------|-------------------------|
| 17 | **COMPLIANT_HINGE** | PhysicsRevoluteJoint + PhysxJoint compliance (PhysX 5.1+) OR MuJoCo tendon/spring | Spring-loaded button lid, leaf-spring catch, flexure bearing | When flexure is distributed (not concentrated hinge) or large deformation |
| 18 | **COMPLIANT_SLIDER** | PhysicsPrismaticJoint + spring drive | Bellows, flexure slider, spring-loaded pin | When deformation exceeds PRBM validity window |

**Detection keywords** (classifier input): `flexure`, `spring-loaded`, `leaf-spring`, `bellows`, `compliant`.

**PRBM parameter mapping:** geometry (length, width, thickness) + material E-modulus → equivalent torsional/linear spring stiffness. Cross-ref `simready-joint-params` for parameter tables.

<!-- source: bundle4/section_file(1)/2_compliant_mechanisms_report (Howell PRBM), confidence: HIGH -->

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

### Additional Constraint: Mimic-Joint Stability (16th domain)

Applies whenever a mechanism uses mimic joints (gripper-finger pairs, symmetric linkages, coupled actuators).

**Quantitative rule:** stiffness × dt² ≈ 0.5–1.0. Products >> 1 oscillate and fail to converge.

- At PhysX default dt = 1/60 s: maximum stable stiffness ≈ 3600.
- At Isaac Sim teleop dt = 1/240 s: maximum stable stiffness ≈ 57,600 (much wider margin).
- If stiffness must be high for mechanical accuracy, reduce dt first; increase solver iterations second.

**Cross-ref:** failure mode F51 (mimic chatter), F57 (stiff drive + mimic non-convergence).

### Additional Constraint: Stribeck Friction (advisory)

Real materials show velocity-dependent friction. For authenticity in self-closing doors, damping feel, gripper-on-surface slip. See `simready-joint-params §Stribeck Friction Model` for the curve and implementation notes.

<!-- source: bundle1/KB §2,8 + bundle2/corrected_brief, confidence: HIGH -->

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

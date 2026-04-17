---
name: failure-modes
description: >-
  34 failure modes for SimReady articulated USD assets, organized by 8 physics
  pillars. Use as a checklist when classifying parts, reviewing physics, or
  diagnosing audit failures. Each failure has: symptom, root cause, and fix.
---

# SimReady Failure Modes

## How to Use

- **Classifier agent**: Check F04–F10 before outputting classify.json
- **Physics reviewer**: Check F11–F34 before approving classification
- **Auditor agent**: When audit fails, match the failure to an F-code and propose fix

## Tier 1: Foundation

| ID | Pillar | Failure | Root Cause | Fix |
|----|--------|---------|-----------|-----|
| F01 | Geometry/Units | Asset in cm not meters, everything 100x off | DCC defaults to cm | normalize_to_meters() detects mpu≠1.0 |
| F02 | Geometry/Units | Mesh has 0 vertices, collision crashes | Empty mesh prim in USD | Skip mesh in collision, warn |
| F03 | Geometry/Units | BBox returns None, anchors/mass undefined | Prim has no mesh descendants | Use fallback values, warn |
| F04 | Classification | LLM returns invalid JSON | Hallucinated format | Validate JSON before use, retry |
| F05 | Classification | LLM names part that doesn't exist in USD | Name mismatch | Resolve by searching stage, skip if missing |
| F06 | Classification | Structural part classified as movable | Bracket/fixer/mount wrongly gets joint | Check for structural keywords: fixer, bolt, body, mount, stopper |
| F07 | Classification | Movable part classified as structural | Door/drawer doesn't move at all | Check for movable keywords: door, drawer, wheel, lid, flap |
| F08 | Classification | Wrong joint type (revolute vs prismatic) | Misread part geometry | Door/lid=revolute, drawer/slider=prismatic, wheel=continuous |
| F09 | Classification | Wrong axis (Z vs X vs Y) | LLM guesses, doesn't measure | Vertical hinge=Z, horizontal=X. Wheel: thin bbox dimension=axle |
| F10 | Classification | Grandchild classified as movable | Can't be sibling of body | Only direct children of body can be movable |

## Tier 2: Physics Behavior

| ID | Pillar | Failure | Root Cause | Fix |
|----|--------|---------|-----------|-----|
| F11 | Hierarchy | Movables nested under body, don't move | PhysX merges child RigidBody into parent | Reparent as siblings (depth-sorted, separate batch edits) |
| F12 | Hierarchy | Reparent crashes (SdfBatchNamespaceEdit) | Wrong reparent order | Process deepest prims first |
| F13 | Hierarchy | Parts at wrong position after reparent | Mesh xformOps not cleared | Clear all xformOps, rewrite single world matrix |
| F14 | Position | Both joint anchors at (0,0,0), part pinned to origin | Anchors read AFTER reparent (pivots cleared) | Save anchors BEFORE reparent |
| F15 | Position | Wheel clips through bracket (~15mm off) | DCC pivot marks caster swivel, not tire center | Use tire bbox center AFTER structural split |
| F16 | Position | Hinge on wrong side of door | min_x vs max_x detection error | Measure anchor distance to both bbox edges |
| F17 | Position | Drawer opens into body instead of outward | Pull direction inverted | Compare drawer center vs body center on movement axis |
| F18 | Limits | Door jams at 0°, blocks gripper | Drive stiffness > 0 (spring return) | Always stiffness=0 on all joints |
| F19 | Limits | Wheel can't spin freely | Tight revolute limits | Use [-9999, 9999] for continuous joints |
| F20 | Limits | Drawer frozen, can't slide | Travel=0 (bbox estimation failed) | Fallback to 0.4m travel if bbox unavailable |
| F21 | Mass | Trolley body 318kg, can't drag | Auto-estimate with wrong density | Clamp dynamic body to 5–100kg, use density=80 |
| F22 | Mass | Door too heavy for robot | BBox overestimates mass | Clamp revolute mass to 2–100kg |
| F23 | Mass | Wheel blows away on contact | Mass below minimum | Clamp continuous mass to 0.05–1.0kg |
| F24 | Mass | PhysX assigns wrong default mass | Missing MassAPI on prim | Always apply MassAPI with explicit mass |

## Tier 3: Interaction

| ID | Pillar | Failure | Root Cause | Fix |
|----|--------|---------|-----------|-----|
| F25 | Collision | Invisible wall blocks robot | convexHull on large concave mesh | Use convexDecomposition on concave meshes >2000 verts |
| F26 | Collision | PhysX hangs on load (never finishes) | 42× convexDecomposition on one asset | Budget: max 5 decomposition meshes (MAX_DECOMP_BUDGET) |
| F27 | Collision | Wheel blob, poor rolling contact | convexHull on wheel tire | ALL wheel meshes must use convexDecomposition |
| F28 | Collision | Door jams when closing (PhysX overlap) | Collision on interior/clips/bolts inside door | Skip list: interior, clips, bolt, logo, rubber, etc. |
| F29 | Collision | Gripper passes through handle | No CollisionAPI on handle mesh | Detect handles by name, add colliders |
| F30 | Friction | PhysX ignores friction entirely | Only per-mesh attrs, no material:binding:physics | Must create binding relationship, not just material attrs |
| F31 | Friction | Gripper slides off handle | Metal friction sf=0.6 too low for grip | GripMaterial sf=1.0, df=0.9 on handles |
| F32 | Friction | Dynamic body oscillates wildly when dragged | No linear/angular damping on body | Set linearDamping=100, angularDamping=200 for dynamic |
| F33 | Clean | Host simulator conflict | PhysicsScene embedded in asset USD | Strip all PhysicsScene prims |
| F34 | Clean | Gripper gap 20mm instead of 0.5mm | contactOffset baked in asset USD | Strip contactOffset, set at runtime only (0.00005) |
| F35 | Collision | Movable part has zero collision — can't interact | Mesh is nested Xform→Xform→Mesh, collision code only checks direct children | Fallback to recursive mesh search when GetChildren() finds no Mesh |
| F36 | Collision | Gripper gap from robot finger convexHull | Franka finger.stl is concave, hull bloats 66% | Apply convexDecomposition on finger/hand meshes at runtime |
| F37 | Limits | Slider part only reaches half its range | Pipeline forced one-directional drawer limits on a bidirectional slider | Detect slider (part spans >90% of body on slide axis) → bidirectional limits |
| F38 | Hierarchy | Reparented child breaks DCC alignment (trigger exits slot, teeth misalign) | Assembly sub-component reparented as sibling + joint can't replicate parent-child precision | Don't reparent triggers/latches/handles — keep as children of their parent body |
| F39 | Position | Structural mesh in movable travel zone (wheels where drawer opens) | DCC model placed decorative parts in movable path | B8 detects overlap, auto-hides relocatable parts (wheels/bolts/clips) |

## Wheel Compound Failures

| ID | Combines | Symptom | Fix |
|----|----------|---------|-----|
| W01 | F06+F11 | Bracket tears off wheel under drag | split_wheel_structural_parts() moves fixer/bolt/mount to body |
| W02 | F09+F15+F27 | Wheel completely broken (won't roll, clips, blobs) | Correct axis from tire bbox + tire center anchor + decomposition |
| W03 | F09+F15 | Wheel detaches from trolley under force | Correct axis + correct anchor |

## Classifier Pre-Flight Checklist

Before outputting classify.json, verify:

- [ ] F05: Every part name exists in the USD hierarchy
- [ ] F06: Keywords fixer/bolt/body/mount/stopper → structural, not movable
- [ ] F07: Keywords door/drawer/wheel/lid/flap → movable, not structural
- [ ] F08: Joint type matches part geometry (hinged=revolute, sliding=prismatic)
- [ ] F09: Axis matches physics (vertical hinge=Z, horizontal=X, wheel=thin bbox dimension)
- [ ] F10: All movables are direct children of body, not grandchildren

## Auditor Diagnosis Guide

When C1-C7 audit fails, trace to failure mode:

| Criterion Failed | Likely Failure Modes |
|-----------------|---------------------|
| C1 (Rigid Bodies) | F21–F24 (mass), F11 (nested rigid) |
| C2 (Collision) | F25–F29 (collision strategy), F35 (nested mesh = zero colliders on movable) |
| C3 (Friction) | F30–F31 (binding, GripMaterial) |
| C4 (Hierarchy) | F11 (nested), F06/F10 (classification) |
| C5 (Joints) | F14–F17 (anchors, axis), F09 (wrong axis) |
| C6 (Drives) | F18 (stiffness), missing DriveAPI |
| C7 (Clean) | F33 (PhysicsScene), F34 (contactOffset), F01 (units) |

## Mass Clamp Reference

| Joint Type | Min (kg) | Max (kg) | Density (kg/m³) |
|-----------|---------|---------|-----------------|
| Body (kinematic) | — | — | 600 |
| Body (dynamic) | 5 | 100 | 80 |
| Revolute (door) | 2 | 100 | 500 |
| Prismatic (drawer) | 0.5 | 5 | 500 |
| Continuous (wheel) | 0.05 | 1.0 | 500 |
| Fixed | 0.1 | 10 | 500 |

## Drive Parameters Reference

| Joint Type | Damping | Stiffness | Limits |
|-----------|---------|-----------|--------|
| Revolute (door) | 2.0 | 0 | [-120°, 0] or [0, 120°] |
| Prismatic (drawer) | 5.0 | 0 | [0, depth×0.85] |
| Continuous (wheel) | 2.0 | 0 | [-9999, 9999] |

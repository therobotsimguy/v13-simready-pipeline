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
| F14b | Audit (false-pos) | C5 flags symmetric-pivot instruments (scissors/clamps/pliers/forceps) as zero-anchor | Both arms legitimately share origin AT the pivot; local (0,0,0) on both maps to the same world point | Resolve anchors in world-space; only fail when `anchor_miss_m > 0.01m`. See `make_simready.py:282`. |
| F14c | Classification | URDF/MuJoCo export fails "more than one to-neighbor" on revolute joint (AUDIT 7/7 but MUJOCO 0/1) | For symmetric-pivot instruments (scissors/clamps/pliers/forceps), classifier picked an arm as body instead of the default prim — leaves the other arm as a sibling with no joint chaining to the URDF root | Classifier MUST set `body = <default_prim_name>` for symmetric-pivot instruments; both arms become `movable:revolute`. See `simready-mechanism-lookup` → Symmetric-Pivot Instruments. Seen on Clamps_A01_01 (2026-04-18). |
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
| F40 | Limits | Prismatic button travel absurdly large (60cm on a 15cm tool) | bbox-derived travel inflates for deeply-nested parts; `gemini_articulation.range_meters` was extracted but never consulted inside `apply_physics` | Override bbox travel with Gemini `range_meters` when bbox travel > 3× Gemini value; audit FAIL when prismatic travel > 50% of asset bbox. Seen on HoldingDevice_A01_01 valvebutton (2026-04-18). |
| F41 | Mass | Small articulated handheld tool stays kinematic, robot can't pick it up | Auto-dynamic rule gated on `not has_movables` — articulated tools <3kg (holding devices, working scissors, syringes with plunger) never qualified | Drop the `not has_movables` guard; any body with Gemini mass <3kg becomes dynamic. Seen on HoldingDevice_A01_01 (2026-04-18). |
| F42 | Hierarchy | Wheel bracket/cover rotates with the tire under a continuous joint, breaking visual fidelity and risking detachment under drag | `split_wheel_structural_parts()` keyword list missed naming conventions outside the original `fixer/bolt/body/mount/stopper/frame/caps/bracket/fork/brake` set. Source USD called the caster parts `wheel1_base_01` / `wheel1_trim_01` — `base` and `trim` weren't matched, so both stayed inside the rotating wheel Xform. The C5 `wheel_split_leaks` audit uses the SAME keyword list, so it was blind to the leak too. | Extend `WHEEL_STRUCTURAL_KEYWORDS` whenever a new asset introduces different naming. Current list as of 2026-04-18: `fixer, bolt, body, mount, stopper, frame, caps, bracket, fork, brake, base, trim`. Scope is safe — the split function only inspects DIRECT children of continuous-joint wheels. Seen on Mobilecartsandtables_C01_01 (2026-04-18). |
| F43 | Transform | Asset appears to float above the ground in Isaac Lab teleop even though USD bbox says wheels touch Z=0 | Raw USD had `xformOp:scale=(100,100,100)` (or similar non-unit uniform scale) on an inner Xform, plus nested compensating scales. Isaac Lab's ArticulationCfg + PhysX interpret inner xformOp:scale inconsistently with the USD renderer (some pipelines ignore it, others apply it twice). `normalize_to_meters` only scaled metersPerUnit, leaving residual scale ops untouched. | `bake_xform_scales(stage)` added to the normalize step. Single-pass algorithm: snapshot every prim's (cum_scale, own_scale), then apply: mesh points scale by cum × own, translate ops scale by cum, all scale ops reset to (1,1,1). Two-pass snapshot prevents the ancestor-reset-during-traversal bug. Seen on MedicalutilityCart_A03_01 (2026-04-18). |
| F44 | Collision | Drawers (or stacked movables) physically merge / pass through each other because a concave internal mesh on one drawer gets hulled into a huge box that overlaps adjacent drawers' slots | `apply_collision_*` on movables applies `convexHull` to every mesh. Internal organizers (`holders`, `cage`, `rack`, `grid`, `lattice`, `divider`, `organizer`) are deeply concave — their hulls bloat to the full mesh bbox. On MedicalutilityCart_A03_01 drawer3, the `holders_01` mesh spanned 47cm vertically and hulled into a box that swallowed drawers 1+2 above it. | Skip CollisionAPI on movable-part descendant meshes whose name matches `SKIP_COLLISION_KEYWORDS` (holders/holder/cage/rack/lattice/grid/divider/organizer). Visual mesh preserved, just non-colliding. Body (chassis) meshes are unaffected because is_body path is separate. Seen on MedicalutilityCart_A03_01 (2026-04-18). |
| F45 | Articulation | Links joined to the same body via separate joints pass through each other — a drawer slides through an adjacent drawer, even though neither is connected to the other | PhysX articulations default `EnabledSelfCollisions = False`, which disables collision between ALL links of the articulation regardless of joint topology. The adjacency-skip rule (directly jointed links don't collide) is a separate mechanism and kicks in even when self-collisions ARE enabled — so turning the flag on is safe. | In `apply_physics` ARTICULATION step, apply `PhysxArticulationAPI` alongside `PhysicsArticulationRootAPI` on the default prim, and set `physxArticulation:enabledSelfCollisions = True`. Adjacent links still skip collision (drawer↔chassis), non-adjacent pairs now collide (drawer↔drawer). Seen on MedicalutilityCart_A03_01 (2026-04-18). |
| F46 | Limits | Prismatic drawer opens in the wrong direction — comes out the back or side of the cabinet instead of the front | Direction-select compared drawer-bbox center vs body-bbox center on the prismatic axis; fails when the drawer bbox is symmetric about the body center (e.g. a top-wide drawer spanning full chassis width). | Prefer handle/lock/knob/rotor sub-mesh center over drawer-bbox center when available — the handle sits on the opening face, so opening direction = drawer-center → handle-center. Applied in both apply_physics prismatic branch and the C5 backward_drawers audit. Seen on MedicalutilityCart_A03_01 drawer1 (2026-04-18). |
| F46b | Limits | Classifier geometry heuristic picks the wrong drawer direction for irregular layouts (e.g. a top lid that opens toward the BACK while other drawers open toward the FRONT) | F46 handle-heuristic still guesses wrong when the handle/lock is a locking mechanism that ENGAGES from a non-opening face. No geometric signal can reliably disambiguate this case. | `classify.json` accepts signed axis strings `"+X"` / `"-X"` / `"+Y"` etc. When present, pipeline honors the sign verbatim and the C5 backward_drawers audit skips the direction check for that joint (user intent is authoritative). Seen on MedicalutilityCart_A03_01 drawer1 (2026-04-18). |

## Wheel Compound Failures

| ID | Combines | Symptom | Fix |
|----|----------|---------|-----|
| W01 | F06+F11+F42 | Bracket/cover/trim tears off wheel under drag OR rotates with tire | split_wheel_structural_parts() moves every direct-child Xform matching `fixer/bolt/body/mount/stopper/frame/caps/bracket/fork/brake/base/trim` to body. Extend the list when a new asset introduces different naming (see F42). |
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

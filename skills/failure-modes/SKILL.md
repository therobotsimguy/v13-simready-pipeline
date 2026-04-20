---
name: failure-modes
description: >-
  34 failure modes for SimReady articulated USD assets, organized by 8 physics
  pillars. Use as a checklist when classifying parts, reviewing physics, or
  diagnosing audit failures. Each failure has: symptom, root cause, and fix.
  Extended with F50-F63 (continuation of classical F-series), D-series
  (deformables), K-series (kinematic/synthesis/format/standards), and S-series
  (solver/engine-specific Newton MPM + MuJoCo + MJX + Isaac Sim). All tiers
  provide symptom / root cause / fix / confidence.
---

# SimReady Failure Modes

## How to Use

- **Classifier agent**: Check F04–F10 before outputting classify.json
- **Physics reviewer**: Check F11–F39 and F50–F62 before approving classification
- **Auditor agent**: When audit fails, match the failure to a code (F / D / K / S) and propose fix
- **Deformable asset path**: also check Tier 4 (D01–D18) — cross-ref `deformable-physics-robotics`
- **Newton dual-output path**: also check Tier 6 S01 (MPM coupling) — cross-ref `newton-physx-compat-matrix`

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
| F50 | Collision | Parent-child overlap during joint motion (rest-pose clean) | Swept overlap untested | Sample joint range at 10% increments; regenerate collision where sweep conflicts |
| F51 | Limits/Drive | Mimic joint chatter / oscillation | stiffness × dt² >> 1 | Tune so stiffness × dt² ≈ 0.5–1.0; use compliant drive (dampingRatio > 1) |
| F52 | Limits/Drive | Joint floppy after PGS→TGS switch | TGS default spring interpretation ignores stiffness | Enable `eACCELERATION_SPRING` flag on D6 joints under TGS |
| F53 | Hierarchy | URDF-imported robot defies gravity, "Prim is not Articulation" error | URDF importer fails to auto-apply ArticulationRootAPI | Manually apply ArticulationRootAPI to root prim |
| F54 | Authoring | Asset spawns at (0,0,0) after ArticulationCfg wrap | Spawn doesn't inherit authored transform | Re-specify USD path in `spawn` config of ArticulationCfg |
| F55 | Geometry/Units | Scaled mesh makes robot move wrong speed or fly off | Visual scale ≠ physical scale (mass/inertia/gravity not updated) | Scale mass ∝ s³, inertia ∝ s⁵; preserve gravity |
| F56 | Authoring | Transform/axis gizmo jumps at simulation start | PhysX disables pivots at runtime | Bake orientation in source DCC; never rely on pivots for physics objects |
| F57 | Limits/Drive | Stiff drive + mimic joint: solver fails to converge | Competing hard constraints | Use compliant drives OR relax hard limits; never combine |
| F58 | Clean/Hardware | Sim stable at 4 envs, NaN at 32+ envs | Error accumulation at scale with under-tuned solver | Tune `solver_position_iteration_count`, dt, collision meshes at small N first; scale gradually |
| F59 | Inertial | Asset jitters at rest or rotates unintentionally | Principal inertia axes misaligned with geometry | Author `physics:principalAxes` explicitly; re-diagonalize inertia tensor; inspect in Physics Debugger |
| F60 | Inertial | Instability "fixed" by adding armature | Armature masking bad inertials (opaque fix) | Reject unexplained armature; recompute inertials properly; lower stiffness or damping instead |
| F61 | Clean/Hardware | Frame-1 explosion on load (asset flies apart immediately) | Uncapped depenetration velocity on initial penetrations | Limit max depenetration velocity during debug loading to surface visible overlaps |
| F62 | Collision | False-clean diagnosis (no overlaps reported but asset misbehaves) | Self-collision flag OFF at articulation root | Toggle self-collision ON before running overlap detection; absence of pairs ≠ clean geometry |
| F63 | Hierarchy/Collision | Main body drags, rest of tool stays visually pinned at author pose | Raw USD splits tool body into N sibling Xforms; classifier routes ONE as body root, others get no RigidBodyAPI and no CollisionAPI → render as static visuals | `weld_structural_siblings_into_body` in make_simready.py reparents body-sibling Xforms into the body before collision authoring. PhysX applies all descendant CollisionAPI meshes to the single body RigidBodyAPI. Surfaced on SurgicalpowerTool_B01_01 (10-Xform tool body, 9 orphans) |

<!-- source: bundle1/articulated_asset_generation_operational_kb.md §1-8 + bundle5/simulation_failure_rules_consolidated.md + SurgicalpowerTool_B01_01 field observation 2026-04-19, confidence: HIGH -->

## Tier 4: Deformables (D01–D18)

For deformable/non-rigid assets (cloth, ropes, cables, soft tissue). Cross-reference `deformable-physics-robotics`. Classifier should check these only when asset has deformable parts.

| ID | Pillar | Failure | Root Cause | Fix | Conf |
|----|--------|---------|-----------|-----|------|
| D01 | Element | Hourglassing in FEM | Zero-energy modes in linear tetrahedra | Reduced-integration or stabilization; monitor Jacobian | HIGH |
| D02 | Element | Over-stiff bending (shear locking) | Linear hex/tet lock | Higher-order elements OR XPBD reformulation | HIGH |
| D03 | Mesh | Inverted tetrahedra | Negative Jacobian from poor tet mesh | fTetWild with stricter quality thresholds; inspect input for self-intersections | HIGH |
| D04 | Mesh | Aspect ratio > 3 blows up sim | Elements stretched/crushed | Remesh with `--improve-quality`; target AR 1–3 | HIGH |
| D05 | Solver | Energy drift over long sim | Explicit integration accumulation | Implicit integrator; reduce dt or increase iterations | HIGH |
| D06 | Contact | Cloth/rope tunneling | CCD disabled | Enable CCD OR use constraint-stabilized capsule chains | HIGH |
| D07 | Contact | Gripper passes through deformable (Isaac Sim PhysX 5) | Known gap, forum #318907 | Validate in MuJoCo flex first; increase contact iterations | HIGH |
| D08 | Coupling | Deformable-rigid instability | Two-way coupling poorly damped | Explicit attachment API; keep deformable mass << rigid; tune damping | HIGH |
| D09 | Coupling | Multi-layer cloth jitter | PhysX soft-soft heuristics fail | Merge multi-layer into single self-colliding mesh; or Houdini Vellum offline | HIGH |
| D10 | Clean/HW | Warp GPU divergence on deformables | Irregular mesh connectivity causes thread divergence | Precompute adjacency; uniform particle layouts where possible | HIGH |
| D11 | Material | Sim material ≠ real object | Parameter mismatch | Differentiable-sim calibration from video/haptics; maintain cited material library | MED |
| D12 | Sim-to-Real | Gripper slip on deformable | Contact model too smooth / friction too low | Increase friction μ; real-gripper validation; domain randomization | HIGH |
| D13 | Mesh | Higher sim-mesh resolution → MORE deformation (not less) | Non-convergence with finer mesh | Co-tune solver iterations + dt when raising resolution; don't treat res as stiffness lever | HIGH |
| D14 | Authoring | Python-created rope unravels; UI rope stable | UI sets hidden solver/joint params scripted path misses | Reverse-engineer UI rope; replicate all hidden props | HIGH |
| D15 | Solver | Gradual drift over long sim | FP32 accumulation error | FP64 for critical calcs; symplectic integrator | MED |
| D16 | Contact | Stretched/torn deformable at contact | Solver under-iterating to satisfy all constraints | Increase solver iterations; robust nonlinear solver | MED |
| D17 | Contact | Missed collision in concave/tight geometry | Discrete collision detection edge case | Enable CCD; tighter geometry tolerance | MED |
| D18 | Material | Oscillation when over-damped | C-damping vs Rayleigh damping confusion | Expose damping model selection (C-damping preferred for robotics) | LOW |

<!-- source: bundle2/findings_file(1)_wide_research_unzipped/4_failure_catalog_final + bundle5 CI-0033/0034/0035, confidence: HIGH (D01-D14) / MED (D15-D17) / LOW (D18) -->

## Tier 5: Kinematic & Synthesis (K01–K18)

For assets generated or validated through synthesis pipelines. Orthogonal axis to F-series (synthesis-stage vs post-hoc physics). Cross-reference `articulation-pipelines §0` and `simready-mechanism-lookup`.

| ID | Pillar | Failure | Root Cause | Fix | Conf |
|----|--------|---------|-----------|-----|------|
| K01 | Synthesis | Kinematic chain cannot reach task goal | Topology invalidity (wrong linkage function space) | Validate topology pre-physics (reach + workspace analysis) | HIGH |
| K02 | Dynamic | Closed-loop constraint drift under actuation | Constraint softness mismatch between engines | Engine-specific: PhysX compliance vs MuJoCo `solimp`/`solref`; document tolerances | HIGH |
| K03 | Validation | Asset fragile under ±10% perturbations | Over-fit to one simulator | Cross-simulator consistency + sensitivity sweep | HIGH |
| K04 | Format | URDF closed-loop collapse on conversion | URDF tree-only limitation | Detect loops before URDF export; warn or use cut-joint formulation | HIGH |
| K05 | Format | MJCF actuator semantics dropped on USD round-trip | Format-specific semantic loss | Preserve as USD custom attrs; warn on lossy export | HIGH |
| K06 | Standards | Adjustable hospital bed has entrapment risk | IEC 60601-2-52 gap violation (rail gap < 120mm, frame < 60mm) | Automated gap audit; flag violators | HIGH |
| K07 | Standards | Office chair exceeds BIFMA cycle rating | Swivel 120k @ 13cpm or tilt 300k @ 19cpm limit exceeded | Reference BIFMA X5.1; design fatigue margins | HIGH |
| K08 | Sim-to-Real | Friction stiction mismatch | Constant μ in sim vs real Stribeck (velocity-dependent) curve | Velocity-dependent friction curve OR domain randomization | HIGH |
| K09 | Collision | Tunneling (fast-moving pass-through) | Discrete collision detection | Enable CCD; reduce dt | HIGH |
| K10 | Collision | Ghost collisions (adjacent-link false positives) | Missing collision filter pairs | Collision filters; geometry simplification | HIGH |
| K11 | Kinematic | Jacobian singularity in IK | Task configuration at singular manifold | Damped-least-squares IK fallback; trajectory avoidance; design constraints | HIGH |
| K12 | Dynamic | Ill-conditioned inertia matrix (cond > 10⁶) | Extreme mass ratio or bad mass distribution | Mass redistribution; solver regularization | HIGH |
| K13 | Performance | GPU-incompatible topology | Exotic joints / many loops | Simplify; use GPU-tailored solvers (Kamino pattern) | HIGH |
| K14 | Performance | Batch memory bottleneck at scale | AoS (Array-of-Structs) layout | Structure-of-Arrays (SoA) layout for batched dynamics | HIGH |
| K15 | Performance | Excessive substeps required for stability | Stiff dynamics | Implicit integrators; stiffness reduction | HIGH |
| K16 | Semantic | Joint axis misdefined | Frame-convention drift between tools | Automated axis sanity checks on classifier output | HIGH |
| K17 | Semantic | Inertial parameters physically implausible | No validation against reality | CAD-based or empirical database validation (MatWeb, CES Selector) | HIGH |
| K18 | Semantic | Friction model mismatch across engines | Engine-dependent parameterization | Rabinowicz/Bhushan-grounded defaults + per-engine tuning tables | HIGH |

<!-- source: bundle4/section_file(2)/0_failure_catalog + bundle4/part3 + bundle4/section_file(4)/1_industrial_design_standards_report, confidence: HIGH -->

## Tier 6: Solver & Engine-Specific (S01–S10)

Engine-level rules that manifest on specific physics backends. Different from asset-level F-series in that the asset may be correct but the engine configuration breaks it. Cross-reference `newton-physx-compat-matrix`.

| ID | Engine | Failure | Root Cause | Fix | Conf |
|----|--------|---------|-----------|-----|------|
| S01 | Newton | Articulated body on granular (MPM) terrain explodes or sinks | MPM solver sees only local part mass, not full body inertia | Manually add robot total mass to MPM-colliding parts in scene config; pipe forces back (Newton GitHub #1251) | HIGH |
| S02 | MuJoCo | `mjData.contact` forces show million-N spikes in control callback | `mjcb_control` fires between `mj_step1` (kinematics) and `mj_step2` (dynamics); forces uninitialized | Read forces AFTER `mj_step2()` OR use `mjSENS_CONTACTFORCE` sensor | HIGH |
| S03 | MuJoCo | Setting contact regularization R=0 allows MORE penetration over time | Iterative PGS solver doesn't fully enforce non-penetration without Baumgarte | Keep regularization > 0; use CG solver or Baumgarte stabilization via `solimp` | HIGH |
| S04 | MuJoCo | Object on flat hfield bounces continuously | Positive margin on hfield unsupported | Set margin = 0 on all hfield geometries | HIGH |
| S05 | MuJoCo/MJX | Training gradients noisy when contact is stiff | Stiff springs produce large noisy contact force gradients | Soften contacts for learning; use CFD (Contacts From Distance) technique. **Different domain from F18** — F18 is authoring-time drive stiffness; S05 is training-time contact stiffness | HIGH |
| S06 | MJX/JAX | Non-deterministic despite `TF_DETERMINISTIC_OPS=1` | XLA GPU reductions non-deterministic | `XLA_FLAGS="--xla_gpu_deterministic_ops=true"` (performance cost) | HIGH |
| S07 | Isaac Sim | Cannot simulate granular (sand/soil) | No DEM solver in Isaac Sim scope | Use Newton + Warp for granular | HIGH |
| S08 | Isaac Sim | Cloth multi-layer / tearing unsupported | Feature incomplete as of 2026 | Merge layers into single mesh OR Houdini Vellum offline | HIGH |
| S09 | Sim-to-Real | Save/resume mid-contact produces different results | Solver internal state not fully serialized | Always restart from beginning; never resume in-contact | HIGH |
| S10 | Havok | Cannot query hinge angle (API limitation); Coriolis ignored | Gaming-first design omits robotics features | Use 6D joint or different engine; manual Coriolis calculation. **Out-of-scope for V13 (we don't ship Havok) — flag only if imported asset comes from Havok source** | HIGH |

<!-- source: bundle5/simulation_failure_rules_consolidated.md + nested engine memos, confidence: HIGH -->

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
| C8 (Validation, new) | K03 (sensitivity), S09 (save-resume), D11 (material calibration) |
| C9 (Scale, new) | F58 (batch instability), D10 (Warp divergence), K14 (SoA layout) |
| C10 (Tier, new) | K13 (GPU-incompat topology), S07 (no DEM in Isaac Sim), S08 (cloth gaps) |

### Quantitative Diagnostic Thresholds
- **Condition number of inertia matrix > 10⁶** → instability risk (K12). Redistribute mass or regularize.
- **Closure residual for closed-loop mechanisms: position < 1e-6 m, angle < 1e-4 rad** → pass gate (K03).
- **Cross-solver state-trajectory MSE over 10s gravity drop: < 1e-4** → pass gate for C8.
- **Depenetration velocity uncapped at load** → frame-1 explosion (F61). Cap for debug.
- **Self-collision flag OFF** → absence of overlap reports is false-clean (F62). Toggle ON before audit.

<!-- source: bundle4/failure_catalog + bundle1/KB §8, confidence: HIGH -->

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

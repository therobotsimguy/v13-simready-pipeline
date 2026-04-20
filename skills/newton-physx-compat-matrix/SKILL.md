---
name: newton-physx-compat-matrix
description: >-
  NVIDIA Newton physics engine reference for dual-output V13 assets. Covers multi-
  solver backends (XPBD, VBD, MuJoCo, Featherstone, SemiImplicit), Warp substrate,
  OpenUSD-native schemas, MPM granular coupling, gradient-compatible differentiable
  paths, and parity requirements vs PhysX for assets that ship to both Isaac Sim and
  Newton. Also documents known rough edges (MPM articulated-body inertia config,
  multi-solver switching costs) and sim-to-real determinism rules relevant to asset
  validation. Use when authoring assets targeting Newton, ensuring PhysX/Newton
  behavioral parity, or routing between GPU-batchable and CPU-high-fidelity tiers.
---

# Newton / PhysX Compatibility Matrix Skill

Newton is NVIDIA's next-gen, open-source, Warp-based physics engine (built with Google DeepMind + Disney Research, announced March 2025). V13 targets it as a first-class output alongside PhysX. Use this skill to understand Newton's solver choices, its USD schemas, where it diverges from PhysX, and how to audit dual-output parity.

## When to Use
- Authoring an asset that must behave equivalently in Isaac Sim (PhysX) AND Newton
- Choosing a Newton solver backend for a given mechanism class
- Auditing dual-output parity (`simready-criteria` C11)
- Understanding Newton's relationship to Warp, MuJoCo Warp, and Featherstone
- Debugging Newton-specific rules (MPM articulated coupling, determinism paths)
- Deciding GPU-batchable vs CPU-high-fidelity routing when Newton is a target

## 1. Newton Overview

| Property | Value |
|---------|-------|
| **Status** | Open-source, announced March 2025 (NVIDIA + Google DeepMind + Disney Research) |
| **Substrate** | NVIDIA Warp (GPU-native kernel compiler) |
| **Format** | OpenUSD-native input/output |
| **License** | Open-source (Apache-style) |
| **Target domain** | Robotics physics + differentiable simulation |
| **Relationship to PhysX** | Complementary, not replacement. PhysX remains Isaac Sim's default. |
| **Relationship to MuJoCo** | MuJoCo Warp is one of Newton's backends — re-implementation of MuJoCo semantics on Warp |

**Key differentiator vs PhysX:** Newton is **multi-solver** — select the backend per scene or per asset class. PhysX is single-solver (PGS + TGS variants of its own solver).

<!-- source: bundle2/newton_vbd_correction_note.md + newton_coupling_answer.md + bundle6/research_notes_phase3_browser_findings.md, confidence: HIGH -->

## 2. Solver Backends — When to Use Each

| Solver | Use For | Production Ready? | Notes |
|--------|--------|------------------:|-------|
| **XPBD** | Cloth, soft bodies, constraint-based scenes | YES | General-purpose; particle/constraint formulation |
| **VBD** (Vertex Block Descent) | **Cable, cloth, rubber deformables** | YES | NVIDIA's 2025-2026 push; see `deformable-physics-robotics` |
| **MuJoCo Warp** | Contact-rich manipulation, rigid-body RL at scale | YES | Re-implementation of MuJoCo on Warp; primary deformable-rigid coupling path |
| **Featherstone** | Articulated robots (O(n) reduced-coord) | YES | `example_cloth_franka.py` pairs with VBD |
| **SemiImplicit** | Fast prototyping, low-fidelity scenes | YES | Explicit integration; cheap but drifty |

**Selection heuristic:**
- Pure rigid robot + rigid scene: **Featherstone** (MuJoCo Warp also fine).
- Robot + cloth/cable: **VBD + Featherstone** or **VBD + MuJoCo Warp** (see `deformable-physics-robotics §4`).
- Granular + rigid: **MPM + Featherstone** (with S01 caveats — §5).
- Gradient-based param ID: **Differentiable Warp kernel** (§6).

**Cross-solver switching cost:** assets must be authored to the richer IR. Newton supports per-scene backend selection but **semantic features differ per backend** — MuJoCo equality constraints don't map 1:1 to Featherstone.

<!-- source: bundle2/newton_coupling_answer.md + bundle6/final_report §7 + bundle5/nested 9_newton_research_report, confidence: HIGH -->

## 3. Newton USD Schemas vs PhysX — Side-by-Side

Newton reads OpenUSD natively. Most USD Physics base schemas work unchanged. PhysX extension schemas (PhysxRigidBodyAPI, PhysxArticulationAPI, etc.) **do not transfer** — Newton ignores them silently.

| Capability | PhysX (Isaac Sim) | Newton |
|-----------|-------------------|--------|
| Rigid body | `PhysicsRigidBodyAPI` + `PhysxRigidBodyAPI` | `PhysicsRigidBodyAPI` only — PhysX ext ignored |
| Collision | `PhysicsCollisionAPI` + `PhysxCollisionAPI` | `PhysicsCollisionAPI` only |
| Articulation root | `ArticulationRootAPI` + `PhysxArticulationAPI` | `ArticulationRootAPI` only; backend dictates semantics |
| Joint types | Revolute, Prismatic, Spherical, D6, Fixed, Distance | Same USD base; Featherstone prefers tree + explicit closure |
| Drives | `PhysicsDriveAPI` (with `acceleration` or `force` mode) | Same schema; per-backend interpretation |
| Mass / inertia | `PhysicsMassAPI` | Same |
| Scene / gravity | `PhysicsScene` | Same |
| Convex decomposition | `PhysxConvexDecompositionCollisionAPI` | Newton computes its own — ignores PhysX params |
| SDF collision | `PhysxSDFMeshCollisionAPI` | Not yet supported (as of 2026) |
| Deformables | `OmniPhysicsDeformableBodyAPI` family | Same schemas; Newton VBD/XPBD handles them |
| Material friction | `PhysicsMaterialAPI` | Same |

**Authoring pattern for dual-output:** use only USD Physics base schemas + OmniPhysicsDeformableBodyAPI family. Apply PhysX extension schemas only as optional per-engine overrides in a `physx.usda` layer (cross-ref Asset Structure 3.0 in `articulation-pipelines §0`).

<!-- source: bundle2/newton_vbd_verification_notes.md + bundle6/research_notes_phase3_browser_findings.md, confidence: HIGH -->

## 4. VBD-Specific Rules

Full treatment lives in `deformable-physics-robotics`. Newton-specific callouts:

- **VBD solver class coverage:** linear deformables (cables), thin deformables (cloth), volumetric deformables (rubber). Official NVIDIA blog March 2026.
- **Primary coupling path:** VBD + MuJoCo Warp. Documented production showcase.
- **Secondary coupling path:** VBD + Featherstone. See `example_cloth_franka.py` in Newton repo (header: "VBD for cloth, Featherstone for robot").
- **Not production for:** complex garments, food cutting, hemostasis — still research.

<!-- source: bundle2/newton_vbd_correction_note.md, confidence: HIGH -->

## 5. MPM Granular Coupling (S01 Rule)

Newton supports Material Point Method for granular scenes (sand, soil, snow). Isaac Sim PhysX 5 has no DEM solver — use Newton+Warp for granular per S07.

**S01 / CI-0045 — MPM articulated-body coupling failure:**
- **Symptom:** Articulated body (e.g., Franka) on MPM terrain explodes or sinks.
- **Root:** MPM solver only sees local part mass, not the full body's inertia.
- **Fix:** Manually add robot total mass to MPM-colliding parts in the scene config; pipe forces back to the articulation.
- **Reference:** GitHub Newton issue #1251, confirmed by core dev.
- **Confidence:** HIGH.

```python
# Illustrative; exact API may evolve
scene.set_mpm_particle_mass(
    part="franka/finger_left",
    effective_mass=robot.total_mass,  # instead of just finger mass
)
```

<!-- source: bundle5/research_file_wide_research_unzipped/9_newton_research_report + consolidated CI-0045, confidence: HIGH -->

## 6. Differentiable Paths (Warp Autograd)

Newton's Warp substrate supports gradient-based differentiation. Scope this carefully:

**In-scope for asset validation:**
- Forward-sim material calibration from real-world data (video/haptics).
- Sensitivity analysis on friction/mass/stiffness for C8 cross-solver validation.
- Parameter fitting against ground-truth trajectories.

**Out-of-scope (pure ML plumbing — skipped per Phase 2 decision):**
- Warp tile_matmul tiling rules for large ML batches (Bundle 5 CI-0038 — skipped).
- Warp autograd in-place array corruption (Bundle 5 CI-0039 — skipped).

These ML-framework rules are the responsibility of your RL infrastructure, not this skill.

**Asset-level differentiable caution:**
- Non-smooth contact breaks autograd. Smoothing + specialized solvers exist but no silver bullet.
- Use differentiable sim **for calibration only**, not as runtime. Freeze calibrated params before production.

<!-- source: bundle2/findings_file(2)_wide_research_unzipped/2_differentiable_deformable_simulation_2026 + 7_learned_physics_for_deformables, confidence: MED (active research) -->

## 7. PhysX / Newton Parity Requirements (C11)

V13 ships assets as **separate products** for Isaac Sim (PhysX) and Newton. `simready-criteria` C11 demands behavioral parity on a defined test battery.

### Test Battery
| Test | Metric | Tolerance |
|------|-------|-----------|
| Gravity drop (10 s) | Final COM position delta | < 5 mm |
| Teleop push (100 N lateral) | Max joint deflection delta | < 5% |
| Joint sweep (full range) | Time-to-complete delta | < 10% |
| Contact force peaks | Max contact force ratio | 0.9 < ratio < 1.1 |
| State trajectory MSE | Position state vector, 10 s | < 1e-4 |

### Known Parity Gaps (flag, don't silently fail)
- **PhysX compliant contact (5.1+) vs Newton VBD**: small-scale compliance differences. Spring-like behavior match within ±10%.
- **PhysX convex-decomp vs Newton auto-decomp**: Newton recomputes — vertex counts may differ. Collision silhouettes match; exact contact points do not.
- **PhysX SDF collision vs Newton**: Newton lacks SDF support — parity impossible; use convex decomposition for dual-output assets.
- **PhysX articulation damping vs Featherstone**: slightly different numerical integration. Asymmetric oscillation decay expected.

### Recommended workflow
1. Author in USD with base physics schemas only.
2. Add PhysX extensions in optional `physx.usda` layer (Asset Structure 3.0).
3. Run C11 test battery in both engines.
4. If parity fails: check `usd-physx-schemas §One-Way Lowering` for lossy semantics, or ship two per-engine variants with documented divergence.

<!-- source: bundle2/newton_report_corrections.md + bundle6/final_report §Architecture, confidence: HIGH / MED on exact tolerance numbers -->

## 8. Known Rough Edges

Items documented in research but not yet hardened:

- **Newton is not yet battle-tested in production** (2025-2026 rollout). Adopt cautiously; hedge with PhysX as primary for risk-critical assets.
- **Multi-solver switching within a scene** has semantic gaps — constraints that work in MuJoCo Warp may not exist in Featherstone.
- **MPM non-determinism** under GPU reordering — if reproducibility matters, pin seeds and verify across runs.
- **Warp CPU fallback** exists but performance characteristics vary by hardware. Measure before assuming.
- **SDF collision support** is a known gap relative to PhysX — use convex decomposition for Newton-targeted assets.
- **Save/resume mid-contact nondeterminism** — same rule as PhysX (S09): always restart from beginning, never resume mid-contact.

### Newton USD-Import Crash Signatures (observed 2026-04-19)

`newton.ModelBuilder.add_usd()` **segfaults / double-free / tcache-chunk-unaligned**
on SimReady USDs that pass Isaac Sim's PhysX importer cleanly. Cause is mesh-
authoring, not physics hierarchy. Four empirically-discriminating signals,
audited in V13 as the S-series below. All four fire on the crashing bed; at
most two fire mildly on the known-good trolley.

| Signal | Threshold | Rationale |
|---|---|---|
| **S-verts** | any Mesh > 5000 verts | Newton silently CPU-falls-back on >64-vert convex hulls (GPU budget per C9). Extreme cases (`bedtop_bolts_01` at 38,592 verts) trip parser limits. Decimate or set `approximation="none"` on decorative meshes. |
| **S-weld** | faceVertexIndices / points < 4.5 | Unwelded (face-soup) geometry. Trolley ≈5.4, bed ≈3.88. Enable "weld vertices on export" in DCC before re-exporting. |
| **S-depth** | Xform nested > 5 levels from default prim | Newton traverses every organizational Xform and gains no physics from it. Flatten the physics-layer hierarchy; keep visual layering separate. |
| **S-wheel-split** | continuous-joint wheel body has <2 Mesh children | Single merged tire mesh (pre-decomposition) is a known crash trigger. Proper wheel = tire + hub + detail as separate Mesh siblings, each with its own CollisionAPI + convexDecomposition. |

**Case study** — `ResuscitationBed_A01_01_physics.usd`:
- 17 meshes > 5000 verts (worst: 38,592)
- 111 meshes unwelded (worst ratio 2.84)
- 27 Xforms at depth > 5 (worst: 8 levels; IV-pole chained inside hydraulic cylinders)
- 4/4 wheels with a single merged tire mesh

Segfaults 100% on `add_usd()` even with all joints / DriveAPIs / CollisionAPIs
stripped. Crash survives down to "main body + one wheel, no collision,
no joint" — the wheel prim **alone** is sufficient. Upstream bug filed.

Trolley (known-good in Newton) shows 5 meshes >5000 verts (worst 6406) and
4 mild welding warnings but loads cleanly — signals are proportional to
severity.

**V13 audit behavior**: these are advisory warnings in WARNINGS block,
never fail C1-C7. They point DCC re-export work, not physics logic.

<!-- source: bundle2/newton_vbd_verification_notes.md + bundle5/research_file_wide_research_unzipped/9_newton_research_report + 12_sim_to_real_gaps + 2026-04-19 ResuscitationBed crash report (scripts/tools/view_simready_scene.py), confidence: HIGH -->

## 9. Decision Guide — When to Use Newton vs PhysX

| Scenario | Preferred | Reason |
|----------|-----------|--------|
| Franka manipulating rigid cabinet | **PhysX (Isaac Sim)** | Mature; audit tooling; F49 world-anchor verified |
| Franka manipulating cloth/drape | **Newton VBD + MuJoCo Warp** | PhysX gripper-deformable gap (D07) |
| Franka on sand/granular terrain | **Newton MPM + Featherstone** | Isaac Sim PhysX has no DEM (S07) |
| Large-batch RL training (>256 envs) | Depends on assets: **PhysX GPU** for rigid; **Newton Warp** for deformable | GPU-batchable tier per C10 |
| Material parameter calibration | **Newton Warp (differentiable)** | PhysX not differentiable |
| Teleop demo / Franka real-time | **PhysX** | Proven stack; V13 reference assets built here |
| Dual-output required | **Both** | Author in USD base schemas; test C11 parity |

<!-- source: bundle2/corrected_brief §1-2 + bundle6/final_report §7-8, confidence: HIGH -->

## Reference

**Source files:**
- `deformable_research_complete_bundle/newton_vbd_correction_note.md`
- `deformable_research_complete_bundle/newton_coupling_answer.md`
- `deformable_research_complete_bundle/newton_coupling_research_notes.md`
- `deformable_research_complete_bundle/newton_vbd_verification_notes.md`
- `deformable_research_complete_bundle/newton_report_corrections.md`
- `simulation_failure_rules_package_all_in_one/research_file_wide_research_unzipped/9_newton_research_report`
- `mechanism_research_bundle/research_notes_phase3_browser_findings.md`
- `generative_articulated_mechanism_research_all_files/final_mechanism_design_research_expanded.md §7`

**External authorities:**
- NVIDIA Newton repo (GitHub)
- Newton issue #1251 (MPM articulated coupling)
- NVIDIA blog: "Newton Physics for Robotics" (March 2025, updated 2026)
- NVIDIA blog: VBD cloth manipulation (Sept 2025, March 2026)

**Cross-references:**
- `deformable-physics-robotics` — VBD/XPBD solver detail, deformable asset pipeline
- `usd-physx-schemas` — PhysX side of the dual-output story, one-way lowering
- `simready-criteria §C10, §C11` — tier certification + parity audit
- `articulation-pipelines §0` — Asset Structure 3.0 layering for dual-output

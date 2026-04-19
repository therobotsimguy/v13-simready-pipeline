---
name: deformable-physics-robotics
description: >-
  Deformable object simulation for robotic manipulation. Covers solver selection
  (FEM, XPBD, VBD, MPM), dual-mesh asset pipelines (fTetWild tetrahedralization,
  Scaled Jacobian > 0.2, aspect ratio 1-3), coupling strategies (PhysX attachments,
  Newton VBD + MuJoCo Warp primary path, VBD + Featherstone example_cloth_franka.py),
  USD deformable APIs (OmniPhysicsDeformableBodyAPI family), 18 failure modes
  (D01-D18), 4-stage validation gauntlet, material calibration (hyperelastic,
  Stribeck friction), and sim-to-real gaps (gripper-deformable contact, electrostatic
  cling). Use when building or simulating cloth, ropes, cables, soft tissue,
  bedsheets, surgical drapes, IV tubing — any non-rigid asset.
---

# Deformable Physics for Robotics Skill

Non-rigid assets have a distinct solver ecosystem and distinct failure modes. This skill is the entry point for any cloth, rope, cable, or soft-body asset. For rigid articulation, see `articulation-pipelines`. For rigid-deformable coupling, both skills apply in sequence.

## When to Use
- Building bedsheets, surgical drapes, IV tubing, cables, soft tissue, hair, foliage — any asset that deforms under load
- Choosing between FEM / XPBD / VBD / MPM solvers for a given asset class
- Preparing tetrahedral or surface meshes for simulation (fTetWild, Scaled Jacobian quality)
- Coupling a rigid robot (Franka) with a deformable (cloth, rope) — PhysX attachments or Newton VBD paths
- Debugging gripper-pass-through, cloth tunneling, hourglassing, mesh inversion
- Calibrating material parameters from video/haptics
- Validating sim-to-real transfer for contact-rich manipulation of soft objects

## 1. Solver Selection Matrix

| Solver | Strength | Use For | Limitation |
|--------|---------|---------|-----------|
| **FEM** (PhysX 5 volumetric) | High fidelity for volumetric soft bodies | Rubber, organ proxy, tissue phantoms | Expensive; linear tet prone to hourglassing (D01) |
| **XPBD** (PhysX 5 cloth / Newton XPBD) | Constraint-based, GPU-friendly | Cloth, shells, thin deformables | Less physically accurate than FEM |
| **VBD** (Newton Vertex Block Descent) | Unifies cable/cloth/rubber on Warp substrate | Production robotics deformables (2026) | Newton-only; not yet in PhysX |
| **MPM** (Newton Material Point Method) | Granular + soft coupling | Sand, snow, flowable materials, fracture | Expensive; S01 coupling caveats |
| **Differentiable (Warp)** | Gradient-based parameter fitting | Calibrating material from video/haptics | Contact non-smoothness breaks autograd |

**Rule of thumb:** cloth → XPBD or VBD. Cable → VBD (capsule-chain fallback for simple cases). Volumetric soft body → FEM (PhysX) or VBD (Newton). Granular → MPM (Newton only — Isaac Sim PhysX has no DEM per S07).

<!-- source: bundle2/deformable_simulation_research_brief_corrected.md §1-5 + findings_file_wide_research_unzipped/0_solver_family_taxonomy, confidence: HIGH -->

## 2. Newton + PhysX 5 + MuJoCo Stack — What's Ready Today

**Newton VBD (authoritative, from official NVIDIA materials):**
- Solver is one of Newton's five backends (alongside XPBD, MuJoCo, Featherstone, SemiImplicit).
- Covers **linear deformables** (cables), **thin deformables** (cloth), **volumetric deformables** (rubber parts).
- Built on Warp substrate → GPU-native, OpenUSD-native.

**Primary rigid-deformable coupling path: VBD + MuJoCo Warp**
- NVIDIA 2026 showcase path.
- Use for cloth manipulation where gripper contact is critical.

**Secondary coupling path: VBD + Featherstone**
- See `example_cloth_franka.py` in official Newton repo (header: "VBD for cloth, Featherstone for robot").
- Use when you need reduced-coordinate articulated-body dynamics for the robot side.

**PhysX 5 deformable stack:**
- FEM for volumetric bodies, XPBD for cloth.
- Single-collider limitation on deformable body (one CollisionAPI prim per deformable).
- Gripper-deformable contact is **unstable as of 2026** (NVIDIA forum #318907) — see §9.

**MuJoCo flex:**
- CPU-only. Not in MJX (GPU). Production blocker for large-scale batched training.
- Volumetric deformables require tetrahedral mesh input (error: "edge ordering is incoherent" if non-tet mesh provided). **fTetWild is mandatory** for MuJoCo deformable assets.

<!-- source: bundle2/newton_vbd_correction_note.md + newton_coupling_answer.md + newton_vbd_verification_notes.md, confidence: HIGH -->

## 3. Asset Preparation Pipeline

Every deformable asset follows this five-step prep:

1. **Start from a watertight input mesh.** Triangle soup fails downstream tetrahedralization.
2. **Tetrahedralize with fTetWild** (volumetric) or keep triangle mesh (cloth/shell).
3. **Quality-gate the output:**
   - Volumetric: **Scaled Jacobian > 0.2** (strict; negative or zero = automatic failure, inverted element).
   - Volumetric: **aspect ratio in [1, 3]** (outside = instability).
   - Cloth: triangle AR < 1.5.
4. **Build dual-mesh system:**
   - Simulation mesh (low-res tet or triangle).
   - Collision mesh (often the same as sim mesh, embedded).
   - Render mesh (high-res, skinned to sim).
5. **Apply USD schema** (see §5).

### Tetrahedralization with fTetWild
```python
# Strict production defaults
ftetwild.run(
    input_mesh="body.obj",
    output_tet="body.msh",
    epsilon=1e-3,         # envelope size (fraction of bbox diagonal)
    edge_length_rel=0.05, # target edge length (fraction of bbox diagonal)
    stop_energy=10,       # conformal-AM-IPC convergence
    improve_quality=True, # critical: reject AR > 3, inverted tets
)
# Reject if Scaled Jacobian min < 0.2
```

### Dual-Mesh Rationale
Simulation cost scales with mesh resolution; visual quality demands higher res. Simulate at low res, render at high res, skin via barycentric interpolation. This is a production-best-practice, not optional.

<!-- source: bundle2/findings_file_wide_research_unzipped/5_meshing_tetrahedralization + corrected_brief §6,11, confidence: HIGH -->

## 4. Coupling Strategies

### 4.1 PhysX Attachments (rigid ↔ deformable)
`OmniPhysicsDeformableAttachmentAPI` supports three attachment types:
- **point** — pin to single location. Simple; high mass-ratio instability risk.
- **spring** — springy anchor. Damping tuning critical.
- **fixed** — rigid coupling. Expensive; use sparingly.

Rule: keep deformable mass << rigid mass to avoid D08 two-way coupling instability.

### 4.2 Newton VBD + MuJoCo Warp (primary path, 2026)
```python
# From NVIDIA Newton 2026 cloth manipulation showcase
import newton
scene = newton.Scene(solver="vbd_mujoco_warp")
scene.add_cloth(...)           # VBD
scene.add_robot_mjcf("franka.xml")  # MuJoCo-parsed rigid body
scene.step(dt=1/240)
```

### 4.3 Newton VBD + Featherstone (articulated robot + deformable)
See `example_cloth_franka.py` in the Newton repo. Use when you need Featherstone reduced-coordinate dynamics (e.g., custom IK solvers) on the robot side while simulating cloth via VBD.

### 4.4 Deformable-Deformable Coupling
**PhysX limitation:** inter-body deformable-deformable contact is unreliable. Workaround: merge multi-layer cloth into single self-colliding mesh (D09). For genuinely separate layered deformables, use Houdini Vellum offline.

<!-- source: bundle2/findings_file(1)_wide_research_unzipped/0_deformable_rigid_coupling + 1_deformable_coupling_research, confidence: HIGH -->

## 5. USD Deformable Schemas

See `usd-physx-schemas §Deformable Body APIs` for the full property reference. Canonical authoring pattern:

```usd
def Xform "Drape" (
    prepend apiSchemas = ["OmniPhysicsDeformableBodyAPI"]
)
{
    float omniphysics:mass = 0.3
    rel omniphysics:simMesh = </Drape/SimMesh>
    rel omniphysics:collisionMesh = </Drape/SimMesh>  # same as sim for thin cloth
    rel omniphysics:renderMesh = </Drape/RenderMesh>

    def Mesh "SimMesh" (
        prepend apiSchemas = ["OmniPhysicsSurfaceDeformableSimAPI"]
    )
    {
        float omniphysics:youngsModulus = 1e5   # soft cloth
        float omniphysics:poissonsRatio = 0.3
        float omniphysics:density = 300         # kg/m³
        # For pre-stressed cloth, add:
        # point3f[] omniphysics:restShapePoints = [...]
    }
}
```

**Critical rules:**
- Deformable body can have **only ONE** CollisionAPI prim.
- Collision must be on the simulation mesh (not the render mesh).
- `restShapePoints` is needed for pre-stressed cloth or tensioned cables. Not obvious in USD round-tripping.

<!-- source: bundle2/findings_file_wide_research_unzipped/10_usd_physics_deformables + corrected_brief §11, confidence: HIGH -->

## 6. Failure Modes D01–D18

Cross-reference `failure-modes §Tier 4: Deformables`. Summary table:

| ID | Pillar | Symptom | Fix |
|----|--------|---------|-----|
| D01 | Element | Hourglassing in FEM | Reduced-integration; monitor Jacobian |
| D02 | Element | Shear locking (over-stiff bending) | Higher-order elements OR XPBD |
| D03 | Mesh | Inverted tetrahedra (Jacobian < 0) | fTetWild stricter quality |
| D04 | Mesh | AR > 3 blowup | Remesh; target AR 1-3 |
| D05 | Solver | Energy drift in long sim | Implicit integrator; reduce dt |
| D06 | Contact | Cloth/rope tunneling | Enable CCD OR capsule-chain |
| D07 | Contact | Gripper passes through deformable | Validate in MuJoCo first |
| D08 | Coupling | Deformable-rigid instability | Keep deformable mass << rigid |
| D09 | Coupling | Multi-layer cloth jitter | Merge into single self-colliding mesh |
| D10 | Clean/HW | Warp GPU divergence on deformables | Precompute adjacency; uniform particles |
| D11 | Material | Sim material ≠ real | Differentiable-sim calibration |
| D12 | Sim-to-Real | Gripper slip on deformable | Increase friction; domain randomization |
| D13 | Mesh | Higher sim-mesh res → MORE deformation | Co-tune iters + dt |
| D14 | Authoring | Python rope unravels; UI rope stable | Reverse-engineer UI rope params |
| D15 | Solver | Long-sim drift | FP64 critical calcs; symplectic integrator |
| D16 | Contact | Stretched/torn deformable | Increase solver iterations |
| D17 | Contact | Missed collision in concave geometry | CCD; tighter tolerance |
| D18 | Material | Oscillation when over-damped | C-damping selection (preferred for robotics) |

<!-- source: bundle2/findings_file(1)_wide_research_unzipped/4_failure_catalog_final, confidence: HIGH (D01-D14) / MED (D15-D17) / LOW (D18) -->

## 7. Validation Gauntlet (4-stage)

| Stage | Checks | Pass Criteria |
|-------|--------|----------------|
| 1. Mesh QA | Scaled Jacobian > 0.2; AR ∈ [1,3]; no inverted elements; watertight | Hard gate |
| 2. Static/Dynamic | Rest-state stability (no unprompted motion); gravity drop; pinch test | No NaN, bounded oscillation |
| 3. Cross-Solver | Same asset in PhysX + Newton + MuJoCo; state-trajectory MSE | < 1e-4 over 10s |
| 4. GPU Batch Viability | Simulate N parallel envs in IsaacLab | Stable at target N (32/128/256 per class) |

<!-- source: bundle2/findings_file(1)_wide_research_unzipped/5_validation_gauntlet, confidence: HIGH -->

## 8. Asset Classes — Ready Today vs Frontier

**READY TODAY (short-tail):**
| Asset | Simulator(s) | Notes |
|-------|-------------|-------|
| Bedsheets, surgical drapes | PhysX 5 cloth OR Newton VBD | Electrostatic cling is a known sim blind-spot |
| IV tubing (thin deformable) | Newton VBD OR PhysX FEM | Capsule-chain for simple routing |
| Surgical drapes (single-layer) | PhysX 5 cloth | Multi-layer = merge via §4.4 |
| Cables (power, umbilical) | Newton VBD OR MuJoCo tendon | Routing constraints non-standard |
| Soft tissue (liver, muscle proxy) | PhysX FEM OR Genesis MPM | Per-tissue material calibration required |

**FRONTIER (2028+):**
| Asset | Blocker |
|-------|---------|
| Organ cutting/tearing | DiSECt differentiable cutting not production |
| Complex garments (seams/layers) | PhysX no multi-deformable; workaround = offline CLO 3D bake |
| Hemostasis / coagulation | No simulator models phase-change or clotting |
| Food cutting / granular+soft | DiSECt + MPM still research; benchmarks lacking |

**V13 recommended focus for medical OR:** bedsheets + IV tubing + surgical drapes + cables as the anchor short-tail set.

<!-- source: bundle2/findings_file(2)_wide_research_unzipped/9_short_tail_long_tail_strategy + 10_deformable_horizons, confidence: HIGH -->

## 9. Sim-to-Real Gaps

**Known gaps as of 2026:**
- **Gripper-deformable contact in Isaac Sim PhysX 5** — unstable; validate via MuJoCo flex first (NVIDIA forum #318907).
- **Electrostatic cling in surgical drapes** — not modeled in any mainstream engine.
- **Friction on deformables** — simplified Coulomb; real materials exhibit Stribeck (velocity-dependent) behavior. Use velocity-dependent friction curve or domain randomization.
- **Contact gradients (differentiable sim)** — non-smooth contact breaks autograd. Smoothing + specialized solvers exist but no silver bullet.

**Mitigation strategies:**
- Domain-randomize friction, mass, stiffness by ±10–30%.
- Validate in two engines before committing to one (cross-solver MSE < 1e-4 per §7 Stage 3).
- Use real-object photogrammetry + differentiable-sim fitting for material params.

### Textile Industry Calibration Sources
CLO 3D, Browzwear, Optitex ship proprietary physics engines with validated cloth parameters. Treat as calibration benchmarks — reverse-engineer material params or partner for dataset access.

<!-- source: bundle2/findings_file(1)_wide_research_unzipped/2_grasp_manipulation_coupling + findings_file(2)_wide_research_unzipped/5_textile_apparel_industry_systems, confidence: HIGH -->

## Reference

**Source bundles:**
- `deformable_research_complete_bundle/deformable_simulation_research_brief_corrected.md` (authoritative, 268 KB)
- `deformable_research_complete_bundle/newton_*_note.md` (5 Newton correction notes — empirical truth)
- `deformable_research_complete_bundle/findings_file*_wide_research_unzipped/` (29 topical memos)

**External authorities:**
- NVIDIA Newton repo (`example_cloth_franka.py`, VBD docs)
- NVIDIA blog: VBD cloth manipulation (Sept 2025, March 2026)
- NVIDIA forum #318907 (gripper-deformable gap, Isaac Sim PhysX 5)
- fTetWild (Hu et al., ACM TOG 2020) — tetrahedralization
- OpenUSD Physics WG AOUSD Schema spec — deformable USD APIs

**Cross-references:**
- `usd-physx-schemas §Deformable Body APIs` — full schema property reference
- `failure-modes §Tier 4: Deformables` — D01-D18 rows
- `newton-physx-compat-matrix §VBD-Specific Rules` — Newton backend detail
- `collision-physics §Deformable Collision Special Cases` — cloth/tet collision rules
- `simready-joint-params §Stribeck Friction Model` — velocity-dependent friction curves

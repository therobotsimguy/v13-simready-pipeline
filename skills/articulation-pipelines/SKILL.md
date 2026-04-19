---
name: articulation-pipelines
description: >-
  State-of-the-art approaches for converting static 3D meshes into simulation-ready
  articulated assets. Covers joint estimation algorithms, contact interface extraction,
  physics-constrained optimization, part segmentation, and physical limit correction.
  Use when designing or improving the SimReady pipeline, or when debugging articulation
  failures. Derived from 15 research papers (2024-2026). Also covers the graph-IR
  compiler-pipeline architecture (topology as source-of-truth, formats as compiled
  output), Asset Structure 3.0 layering, CoACD collision decomposition with GPU hull
  budgets, closed-loop tree-plus-closure decision, and mechanism synthesis context
  (Freudenstein / GA / GNN).
---

# Articulation Pipelines Skill

## When to Use
- Designing or debugging the static-to-articulated pipeline
- Choosing joint estimation algorithms
- Understanding why articulation fails (inter-penetration, kinematic hallucinations, axis errors)
- Evaluating new approaches against the state of the art
- Architecting the pipeline (internal IR, format lowering, tier routing)

## §0: System Architecture (Graph-IR Compiler Pipeline)

Before any stage: decide that the pipeline's **source of truth is an internal graph IR, not a file format**. URDF, MJCF, and USD are *compiled output*, not source-of-record.

### Internal Graph IR

The authoring IR is an **attributed multigraph**:

- **Nodes:** rigid parts + virtual frames (no mass, used for joint placement)
- **Edges:** joints (type / axis / transform), couplings, actuators
- **Attributes per node:** mass, inertia, collision geometry, provenance
- **Attributes per edge:** drive params, limits, friction, solver hints
- **Contact exclusions:** explicit list, not derived from hierarchy

Lower the IR to engine-specific output:
- PhysX (Isaac Sim) via USD + physics extensions
- Newton via USD (base schemas) — see `newton-physx-compat-matrix`
- MuJoCo via MJCF (lossy — flag closed loops, tendon semantics)
- URDF for legacy consumers (lossy — no closed loops, no mimic native)

### Asset Structure 3.0 (Layer Split)

Separate concerns into distinct USD layers so dual-output + format lowering are tractable:

```
asset_root/
├── geometry.usdz      # Mesh data, materials bindings (visual only)
├── physics.usda       # PURE USD Physics APIs (cross-engine)
├── physx.usda         # PhysX-specific overrides (optional layer)
├── newton.usda        # Newton-specific overrides (optional layer)
└── metadata.usda      # Provenance, classification, audit history
```

Rule: `physics.usda` is pure OpenUSD Physics — no PhysX or Newton extension APIs. Engine-specific parameters go in side layers, composed at runtime per target engine.

### Two-Tier Routing Policy

Classify every asset at generation time (see `simready-criteria §C10`):

- **GPU-batchable tier:** tree articulation, low DOF, ≤K closure joints, convex collision, regular memory layout. Routes to Isaac Lab (PhysX GPU).
- **CPU/offline high-fidelity tier:** loops, screw/gear transmissions, hydraulic compliance. Routes to offline Newton (Featherstone or custom backend).

Never force a mismatched mechanism into GPU tier — soft fail is worse than explicit routing.

<!-- source: bundle1/KB §5 + bundle6/final_report §Architecture + research_notes_phase3_browser_findings, confidence: HIGH -->

## The 3-Stage Pipeline Pattern

All modern pipelines follow the same 3-stage architecture (MotionAnymesh, ArtLLM, PhysX-Anything):

```
[Static Mesh] --> Stage 1: Part Segmentation --> Stage 2: Joint Estimation --> Stage 3: Asset Finalization
```

### Stage 1: Part Segmentation

| Method | Approach | Strengths | Weaknesses |
|--------|----------|-----------|------------|
| **MotionAnymesh** | 3D-native P3-SAM + SP4D kinematic priors + VLM clustering | Preserves geometric boundaries; no 2D lifting artifacts | Requires multi-view renders |
| **ArtLLM** | 3D LLM autoregressive prediction from point cloud | Single-pass; handles variable part counts | Limited to trained categories |
| **PhysX-Anything** | VLM-based with voxel tokenization (193x compression) | Works from single image; rich physical attributes | Requires fine-tuned VLM |
| **Part2GS** | 3D Gaussian splatting with learnable part-identity embeddings | Motion-aware; discovers parts from observation | Needs two-state observations |
| **Kinematic Kitbashing** | Reuses parts from existing part libraries | Fast assembly; leverages existing assets | Limited by library coverage |

**Critical lesson:** Pure semantic segmentation fails on irregular mechanical components. Always ground VLM reasoning with geometric or kinematic priors (MotionAnymesh achieves 0.86 mIoU vs 0.68 without priors).

### Stage 2: Joint Estimation

#### Contact Interface Extraction
For any joint type, first extract the contact surface between child part K_i and parent K_P(i):

```
S_contact = { x in boundary(K_i) | min_{y in boundary(K_P(i))} ||x - y||_2 < tau }
```
- **tau = 0.01m** (MotionAnymesh empirical threshold)
- Contact shape determines joint type: circular = spin, elongated = hinge, planar = prismatic

#### Type-Aware Kinematic Initialization

| Joint Type | Contact Shape | Axis Method | Pivot Method |
|-----------|--------------|-------------|-------------|
| **Spin** (wheel, knob) | Circular/annular band | PCA on S_contact: smallest eigenvalue normal = axis | RANSAC circle fit on 2D projection (delta=0.005m), back-project center |
| **Hinge** (door, lid) | Elongated strip | PCA on S_contact: primary eigenvector = rotation axis | Centroid of contact point cloud |
| **Prismatic** (drawer, slider) | Planar patch | Global PCA on entire part: 3 bbox axes as candidates | N/A (pivot undefined for prismatic) |

#### Prismatic Axis Selection: Dual-Penalty Verification
For prismatic joints, test all 3 candidate axes from global PCA:

```
C(v) = L_collide(v) + omega * L_derail(v)   (omega = 20)
```
- **L_collide**: fraction of translated points that penetrate parent (epsilon_c = 0.005m)
- **L_derail**: average divergence of contact points from original sliding surface
- Select axis v* = argmin C(v) -- minimum collision + minimum derailment

#### Physics-Constrained Trajectory Optimization
After initial estimation, refine via continuous non-linear optimization:

```
L_opt = sum_{phi in states} sum_{x in S_contact} ||D_SDF(T(x; v, q, phi), M_static)||^2
```
- Minimizes SDF distance to static parent across all motion states
- Optimized via Levenberg-Marquardt
- **Impact**: Physical Executability jumps from 65% to 87% (MotionAnymesh ablation)

### Stage 3: Asset Finalization

#### Physical Limit Estimation (Geometry-Based)

| Joint Type | Method | Details |
|-----------|--------|---------|
| **Revolute** | Forward-simulation collision detection | Rotate from 0 in both directions up to +/-180; limits = angles where mesh intersection occurs |
| **Prismatic (inward)** | Collision with backplate | Translate inward until drawer hits cabinet back |
| **Prismatic (outward)** | Contact-loss criterion | Translate outward along axis; limit = point where mutual contact area drops to zero |

**ArtLLM's approach**: Articulate child through predicted range, compute collision volume at discrete steps. Sharp spikes in collision volume derivative = joint limit. Hierarchical search within spike window for precise angle.

#### Physical Limit Correction (ArtLLM)
Before correction: self-collisions during articulation. After: smooth, collision-free motion.
- Expand predicted bounding boxes to enclose all points
- Compute collision volume at discrete steps
- Detect spikes in derivative -> refine with hierarchical search

## Benchmark: Method Comparison

| Method | Part Seg (mIoU) | Joint Type Err | Axis Err | Pivot Err | Phys. Executability |
|--------|----------------|---------------|----------|-----------|-------------------|
| PARIS | 0.17 | 0.67 | 1.56 | 1.14 | 11% |
| URDFormer | 0.21 | 0.72 | 1.31 | 1.53 | 21% |
| SINGAPO | 0.52 | 0.24 | 0.73 | 0.57 | 43% |
| Articulate-Anything | 0.47 | 0.21 | 0.86 | 0.64 | 46% |
| Articulate-AnyMesh | 0.59 | 0.35 | 0.64 | 0.44 | 35% |
| **MotionAnymesh** | **0.86** | **0.08** | **0.12** | **0.10** | **87%** |

## Common Failure Modes (From Literature)

| Failure | Cause | Fix |
|---------|-------|-----|
| Kinematic hallucination | VLM invents non-existent joints | Ground with geometric/kinematic priors (SP4D) |
| 2D-to-3D lifting artifacts | Segmentation via rendered views | Use 3D-native segmentation (P3-SAM) |
| Axis drift during long-range motion | Micro-misalignment in initial estimate | Physics-constrained trajectory optimization |
| Inter-penetration at limits | Static limit estimation misses dynamic collision | Forward-simulation collision check at discrete steps |
| Over-segmentation | Local features split single parts | Kinematic-aware clustering with VLM |
| Under-segmentation | Irregular shapes merged by VLM | Fine-grained 3D primitive extraction first |

For dataset selection and comparison, see **sim-ready-datasets** skill.

---

## Extended Pipeline Rules (2026 Production Hardening)

Additional rules layered on top of the 3-stage pipeline. Apply these to every asset. Cross-reference `failure-modes` F50–F62 for corresponding audit checks.

### Stage 1 Additions — Collision Decomposition & Offsets

- **CoACD preferred over V-HACD** for every new pipeline. Parameters: real-metric mode, threshold 0.002–0.01 m, ≤64 verts/hull, 16–32 hulls max. V-HACD remains legacy-only fallback.
- **90/10 collider distribution rule:** ~90% of articulated links should use primitives or a single convex hull; only ~10% should escalate to decomposition or SDF. Escalation is expensive — budget and audit.
- **GPU convex hull budget: ≤64 verts total, ≤32 verts/face.** Exceeding forces silent CPU fallback per-pair → kills batch throughput. Audit every cooked hull post-decomposition (cross-ref F58).
- **Scale-aware contact offset:** 0.5–2% of the smallest contact-relevant feature. Mm-scale gripper: 0.001–0.003 m. Meter-scale furniture: 0.005–0.02 m. Don't copy a single value globally.
- **Pre-cooked collision is mandatory.** Runtime cooking is a content pipeline failure. Version-pin cooked data.
- **Negative scale is unsupported** on convex meshes — bake orientation in DCC (cross-ref F56).

<!-- source: bundle1/KB §1 + research_notes_section1 + bundle6/CSV3, confidence: HIGH -->

### Stage 2 Additions — Frame Placement, Body Ordering

- **Joint frames MUST sit on the physical kinematic interface** (hinge pin, slider centerline, contact reference). Body origins are not acceptable. Implicit-in-existing-skill; now explicit.
- **Body 0 = parent, Body 1 = child** for articulation joints. Preserves tree semantics and consistent force propagation.
- **Fixed-base flag vs external fixed joint:** for a fixed root, use the articulation fixed-base flag at creation time, NOT an external fixed joint (which is a soft constraint and accumulates drift).
- **Overconstrained detection:** Grübler-Kutzbach count is insufficient for special geometries (scissor linkages, parallel structures). Add a screw-theoretic rank check; flag overconstrained mechanisms before physics instantiation (cross-ref K01).
- **Mimic-joint stability quantitative target:** tune so stiffness × dt² ≈ 0.5–1.0. Products >> 1 oscillate (F51).

<!-- source: bundle1/KB §2 + bundle4/section_file/0 mechanism_synthesis + part3, confidence: HIGH -->

### §2.5: Closed-Loop Mechanism Decision Tree

Closed-loop mechanisms (four-bar linkages, scissor jacks, parallelograms, gear trains with multiple rigid-body constraints) cannot be represented as a pure tree.

**Decision procedure:**
1. **Detect loops** via cycle detection on the joint graph.
2. **Extract spanning tree** — choose which edges are tree edges vs closure edges. Prefer tree edges that carry the primary motion; closure edges absorb geometric constraints.
3. **Identify loop-break joint** — the closure edge. Select the edge whose elimination has the smallest functional impact (e.g., in a four-bar linkage, make the opposite-link connection the closure).
4. **Apply external maximal-coordinate closure:**
   - PhysX: `excludeFromArticulation` + external D6 or distance joint on the closure edge.
   - MuJoCo: `equality/connect` constraint.
   - Newton Featherstone: explicit loop-closure constraint API.
5. **Reduce closure stiffness** relative to tree joints — closures are soft constraints and benefit from lower stiffness to avoid chattering.
6. **Baumgarte stabilization** — tune constraint stabilization parameters for the closure joint; too little = drift, too much = artificial stiffness.

**Closure residual thresholds (pass gate for K03):** position < 1e-6 m, angle < 1e-4 rad after 10 s gravity drop.

<!-- source: bundle1/KB §4 + bundle4/part3 + bundle4/section_file/1_multibody_dynamics_report, confidence: HIGH -->

### Stage 3 Additions — Full-Sweep Overlap, Inertials

- **Full joint-sweep overlap test** — sample the joint range at 10% increments; rest-pose clear ≠ swept clear. Any overlap at non-rest pose = regenerate collision (cross-ref F50).
- **Inertial authoring hierarchy:** CAD > proxy > auto-generated. Never touch the COM without recomputing inertia (parallel-axis theorem; see `simready-math`).
- **Principal-axes alignment:** author `physics:principalAxes` explicitly after re-diagonalizing inertia. Misalignment = jitter at rest (F59).
- **Reject "armature-as-stabilizer" anti-pattern:** if an asset is only stable with high armature, that's a bug masking bad inertials. Recompute inertials properly; lower stiffness/damping instead (F60).
- **Condition number of inertia matrix > 10⁶** = instability risk (K12). Redistribute mass or regularize solver.
- **Depenetration velocity capping at load** surfaces visible overlaps instead of catapulting the asset (F61). Remove cap once overlaps resolved.

<!-- source: bundle1/KB §3 + §8 + bundle4/failure_catalog, confidence: HIGH -->

## Mechanism Synthesis Context

Before joint estimation, reframe generation as a synthesis problem. This determines the algorithm to use and sets the stage for validation.

### Classification-First, Not Joint-First

Follow the descent from function to topology:

1. **Function** — serial / parallel / hybrid chain?
2. **Pair type** — lower pairs (R, P, H, C, S, F) or higher pairs (gear, cam, point)?
3. **Chain structure** — open or closed?
4. **Topology graph** — nodes (links) + edges (joints)?

Only after this descent should joint type selection occur. See `simready-mechanism-lookup §Classification Path`.

### Synthesis Method by Class

| Mechanism Class | Type Synthesis | Dimensional Synthesis |
|-----------------|----------------|------------------------|
| Simple hinge, slider | Enumeration (trivial) | Direct from geometry |
| Four-bar linkage | Freudenstein equation (closed-form) | Gradient optimization |
| Multi-link parallel | Enumeration + isomorphism rejection (≤8 links production-proven) | GA or gradient w/ singularity avoidance |
| Compliant flexure | Howell PRBM (see `simready-behaviors`) | Analytical + FEM validation |
| Novel topology | GNN proposal (research-stage) | Differentiable simulator (research) |

### Hybrid Synthesis Architecture (Production Pattern)

Not end-to-end black box. Pipeline:

1. **Retrieval** — find nearest existing mechanism in library (100 mechanisms in `simready-mechanism-lookup`).
2. **Proposal** — ML model proposes parameter adjustments or topology deltas.
3. **Constrained optimization** — optimize under physical + manufacturability constraints.
4. **Validation** — 4-stage gauntlet (static / dynamic / cross-simulator / performance; see `simready-criteria §C8`).

### Cross-Simulator Parameter Mapping

When lowering the IR to a specific engine, map parameters explicitly. Examples:

| Semantic | PhysX | MuJoCo | Newton |
|----------|-------|--------|--------|
| Soft joint limit | `compliance = 0.01` | `solimp = [0.9, 0.95, 0.001]`, `solref = [0.01, 0.99]` | Featherstone native limits + damping |
| Damping ratio 1.0 | `damping` (computed from stiffness) | `dampratio = 1.0` on actuator | Featherstone damping scalar |
| Mimic (coupling ratio k) | PhysX Fixed tendon | MJCF `<equality/>` with joint1/joint2 + polycoef | Featherstone equality constraint |

Document lossy semantics upfront (closed loops lost in URDF, tendon semantics format-specific in MJCF).

<!-- source: bundle4/section_file/0_mechanism_synthesis_report + part3 + part4 + bundle6/final_report §2, confidence: HIGH -->

---

## Implementation Patterns

Concrete algorithms from papers, ready to integrate into `make_simready.py` / `simready_agent.py`.

### Pattern 1: Contact Interface Extraction (MotionAnymesh)

Extract the contact surface between a movable part and its parent body. This tells you
the joint type, axis, and pivot — all from geometry, no LLM guessing.

```python
import numpy as np
from scipy.spatial import KDTree

def extract_contact_interface(child_verts, parent_verts, tau=0.01):
    """Extract vertices on child that are within tau meters of parent surface.
    
    Args:
        child_verts: (N, 3) array of child part boundary vertices
        parent_verts: (M, 3) array of parent body boundary vertices
        tau: distance threshold in meters (0.01 = 1cm, empirical from MotionAnymesh)
    
    Returns:
        S_contact: (K, 3) array of contact interface vertices
    """
    tree = KDTree(parent_verts)
    distances, _ = tree.query(child_verts)
    mask = distances < tau
    return child_verts[mask]

def classify_joint_from_contact(S_contact):
    """Determine joint type from contact interface shape using PCA.
    
    Returns: ('spin', axis, pivot) | ('hinge', axis, pivot) | ('prismatic', axis, None)
    """
    if len(S_contact) < 10:
        return ('fixed', None, None)  # Too few contact points
    
    # PCA on contact points
    centered = S_contact - S_contact.mean(axis=0)
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    # eigenvalues sorted ascending: [smallest, middle, largest]
    
    # Ratio test: how "flat" vs "elongated" vs "circular" is the contact?
    ratio_flat = eigenvalues[0] / (eigenvalues[2] + 1e-10)
    ratio_elongated = eigenvalues[1] / (eigenvalues[2] + 1e-10)
    
    if ratio_flat < 0.05 and ratio_elongated > 0.3:
        # Flat + elongated = hinge (door hinge strip)
        axis = eigenvectors[:, 2]  # Primary eigenvector = hinge line direction
        pivot = S_contact.mean(axis=0)  # Centroid of contact
        return ('hinge', axis, pivot)
    
    elif ratio_flat < 0.05 and ratio_elongated < 0.15:
        # Flat + circular = spin joint (wheel axle, knob shaft)
        axis = eigenvectors[:, 0]  # Smallest eigenvalue normal = rotation axis
        # Use RANSAC circle fit for precise pivot (see Pattern 2)
        pivot = S_contact.mean(axis=0)  # Fallback: centroid
        return ('spin', axis, pivot)
    
    else:
        # Planar/spread = prismatic (drawer slides along body)
        # Use global PCA on entire part for axis candidates (see Pattern 3)
        return ('prismatic', None, None)
```

**When to use in your pipeline:** Before `detect_hinge_edge()`. If contact interface extraction gives a clear hinge/spin/prismatic classification, trust it over LLM classification. Falls back to current approach if contact points are too sparse.

### Pattern 2: RANSAC Circle Fitting for Spin Joints (MotionAnymesh)

For wheels and knobs, find the exact pivot by fitting a circle to the contact boundary.

```python
import numpy as np
from numpy.random import default_rng

def ransac_circle_fit_2d(points_2d, delta=0.005, max_iterations=1000, min_inliers_ratio=0.5):
    """Fit a circle to 2D points via RANSAC. 
    
    Args:
        points_2d: (N, 2) projected contact points
        delta: inlier distance threshold (0.005m = 5mm, from MotionAnymesh)
        
    Returns:
        center_2d: (2,) circle center
        radius: float
        inlier_count: int
    """
    rng = default_rng(42)
    best_center, best_radius, best_inliers = None, None, 0
    N = len(points_2d)
    
    for _ in range(max_iterations):
        # Sample 3 random points to define a circle
        idx = rng.choice(N, 3, replace=False)
        p1, p2, p3 = points_2d[idx]
        
        # Compute circumcenter
        ax, ay = p1
        bx, by = p2
        cx, cy = p3
        D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(D) < 1e-10:
            continue
        ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / D
        uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / D
        center = np.array([ux, uy])
        radius = np.linalg.norm(p1 - center)
        
        # Count inliers
        dists = np.abs(np.linalg.norm(points_2d - center, axis=1) - radius)
        inliers = np.sum(dists < delta)
        
        if inliers > best_inliers:
            best_center, best_radius, best_inliers = center, radius, inliers
    
    return best_center, best_radius, best_inliers

def find_spin_pivot(S_contact, axis):
    """Find pivot for a spin joint by projecting contact to 2D and fitting circle.
    
    Args:
        S_contact: (K, 3) contact interface vertices
        axis: (3,) rotation axis from PCA
        
    Returns:
        pivot_3d: (3,) world-space pivot point
    """
    # Build local 2D coordinate system perpendicular to axis
    centroid = S_contact.mean(axis=0)
    # Find two orthogonal vectors perpendicular to axis
    arbitrary = np.array([1, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1, 0])
    b1 = np.cross(axis, arbitrary)
    b1 /= np.linalg.norm(b1)
    b2 = np.cross(axis, b1)
    
    # Project to 2D
    centered = S_contact - centroid
    points_2d = np.column_stack([centered @ b1, centered @ b2])
    
    # RANSAC circle fit
    center_2d, radius, inliers = ransac_circle_fit_2d(points_2d)
    
    # Back-project to 3D
    pivot_3d = centroid + center_2d[0] * b1 + center_2d[1] * b2
    return pivot_3d
```

**When to use:** Replace current wheel anchor detection (`tire_bbox_center`) with this when the contact interface is available. RANSAC is more robust than bbox center — it handles asymmetric tire geometry and offset axles. Falls back to tire bbox center if contact extraction fails.

### Pattern 3: Dual-Penalty Prismatic Axis Selection (MotionAnymesh)

For drawers/sliders, test all 3 bbox axes and pick the one with least collision + least derailment.

```python
import numpy as np
from scipy.spatial import KDTree

def score_prismatic_axis(child_verts, parent_verts, S_contact, axis_candidate, 
                          steps=np.linspace(-0.3, 0.3, 30), epsilon_c=0.005, omega=20):
    """Score a candidate prismatic axis by simulating translation.
    
    Lower score = better axis. Tests both forward and backward translation.
    
    Args:
        child_verts: (N, 3) all vertices of the movable part
        parent_verts: (M, 3) all vertices of the static parent
        S_contact: (K, 3) contact interface vertices
        axis_candidate: (3,) unit vector candidate axis
        steps: translation distances to test (meters)
        epsilon_c: penetration threshold (0.005m from MotionAnymesh)
        omega: derailment weight (20 from MotionAnymesh)
        
    Returns:
        score: float (lower = better)
    """
    parent_tree = KDTree(parent_verts)
    L_collide = 0.0
    L_derail = 0.0
    
    for d in steps:
        # Translate child
        translated = child_verts + d * axis_candidate
        
        # Collision: fraction of points penetrating parent
        dists, _ = parent_tree.query(translated)
        penetrating = np.sum(dists < epsilon_c)
        L_collide += penetrating / (len(steps) * len(child_verts))
        
        # Derailment: how far contact points drift from original sliding surface
        translated_contact = S_contact + d * axis_candidate
        contact_dists, _ = parent_tree.query(translated_contact)
        L_derail += np.mean(contact_dists) / len(steps)
    
    return L_collide + omega * L_derail

def select_prismatic_axis(child_verts, parent_verts, S_contact):
    """Select best prismatic axis from 3 PCA candidates.
    
    Returns:
        best_axis: (3,) unit vector
        scores: dict of axis_name -> score
    """
    # Global PCA on child part for 3 candidate axes
    centered = child_verts - child_verts.mean(axis=0)
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    
    candidates = {
        'primary': eigenvectors[:, 2],    # Largest variance
        'secondary': eigenvectors[:, 1],  # Middle variance
        'tertiary': eigenvectors[:, 0],   # Smallest variance
    }
    
    scores = {}
    for name, axis in candidates.items():
        axis = axis / np.linalg.norm(axis)
        scores[name] = score_prismatic_axis(child_verts, parent_verts, S_contact, axis)
    
    best_name = min(scores, key=scores.get)
    return candidates[best_name], scores
```

**When to use:** When `auto-detect from body center` gives ambiguous direction (e.g., slider that could go X or Y). The dual-penalty test tries all 3 axes physically and picks the one that actually slides without collision. Particularly useful for non-standard drawers, sliding panels, and shelf sliders.

### Pattern 4: Geometry-Based Joint Limit Detection (ArtLLM)

Find exact joint limits by detecting collision volume spikes — more precise than `depth * 0.85`.

```python
import numpy as np
import trimesh

def find_revolute_limits(child_mesh, static_meshes, axis, pivot, 
                          angle_range=(-180, 180), coarse_steps=72, fine_steps=20):
    """Find revolute joint limits by detecting collision onset.
    
    Args:
        child_mesh: trimesh.Trimesh of the movable part
        static_meshes: list of trimesh.Trimesh for all static parts
        axis: (3,) rotation axis
        pivot: (3,) pivot point
        angle_range: (min_deg, max_deg) search range
        coarse_steps: number of coarse angle samples
        fine_steps: number of fine samples within spike window
        
    Returns:
        (lower_limit_deg, upper_limit_deg)
    """
    static_combined = trimesh.util.concatenate(static_meshes)
    
    # Phase 1: Coarse sweep — find approximate collision angles
    angles = np.linspace(angle_range[0], angle_range[1], coarse_steps)
    collision_volumes = []
    
    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.add_object('static', static_combined)
    
    for angle_deg in angles:
        angle_rad = np.radians(angle_deg)
        # Rotate child around axis through pivot
        rotation = trimesh.transformations.rotation_matrix(angle_rad, axis, pivot)
        rotated = child_mesh.copy()
        rotated.apply_transform(rotation)
        
        # Check collision (boolean is fast; volume is more informative)
        is_collision, contact_data = collision_manager.in_collision_single(
            rotated, return_data=True)
        collision_volumes.append(len(contact_data) if is_collision else 0)
    
    collision_volumes = np.array(collision_volumes, dtype=float)
    
    # Phase 2: Find spike boundaries via derivative
    derivative = np.abs(np.diff(collision_volumes))
    threshold = np.max(derivative) * 0.3  # 30% of max spike = significant
    
    # Find lower limit (negative direction)
    lower_limit = angle_range[0]
    for i in range(len(angles) // 2, -1, -1):  # Search from center outward
        if i < len(derivative) and derivative[i] > threshold:
            # Phase 3: Hierarchical search in this window
            window_start = angles[max(0, i-1)]
            window_end = angles[min(len(angles)-1, i+1)]
            fine_angles = np.linspace(window_start, window_end, fine_steps)
            for fine_angle in fine_angles:
                rotation = trimesh.transformations.rotation_matrix(
                    np.radians(fine_angle), axis, pivot)
                rotated = child_mesh.copy()
                rotated.apply_transform(rotation)
                if collision_manager.in_collision_single(rotated):
                    lower_limit = fine_angle + 1  # 1 degree safety margin
                    break
            break
    
    # Find upper limit (positive direction)
    upper_limit = angle_range[1]
    mid = len(angles) // 2
    for i in range(mid, len(derivative)):
        if derivative[i] > threshold:
            window_start = angles[i]
            window_end = angles[min(len(angles)-1, i+2)]
            fine_angles = np.linspace(window_start, window_end, fine_steps)
            for fine_angle in fine_angles:
                rotation = trimesh.transformations.rotation_matrix(
                    np.radians(fine_angle), axis, pivot)
                rotated = child_mesh.copy()
                rotated.apply_transform(rotation)
                if collision_manager.in_collision_single(rotated):
                    upper_limit = fine_angle - 1
                    break
            break
    
    return (lower_limit, upper_limit)

def find_prismatic_limits(child_mesh, static_meshes, S_contact, axis,
                           max_travel=0.8, coarse_steps=40):
    """Find prismatic joint limits via collision (inward) and contact-loss (outward).
    
    Returns:
        (lower_limit_m, upper_limit_m)  — negative = push in, positive = pull out
    """
    static_combined = trimesh.util.concatenate(static_meshes)
    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.add_object('static', static_combined)
    
    parent_tree = trimesh.proximity.ProximityQuery(static_combined)
    
    # Inward limit: collision with backplate
    inward_limit = 0.0
    for d in np.linspace(0, -max_travel, coarse_steps):
        translated = child_mesh.copy()
        translated.apply_translation(d * axis)
        if collision_manager.in_collision_single(translated):
            inward_limit = d + 0.005  # 5mm safety margin
            break
    
    # Outward limit: contact-loss criterion
    # Monitor mutual contact area — stop when it drops to zero
    outward_limit = max_travel
    initial_contact_count = len(S_contact)
    
    for d in np.linspace(0, max_travel, coarse_steps):
        translated_contact = S_contact + d * axis
        # Check how many contact points are still near the parent
        closest, distances, _ = parent_tree.on_surface(translated_contact)
        still_in_contact = np.sum(distances < 0.01)  # 1cm threshold
        contact_ratio = still_in_contact / initial_contact_count
        
        if contact_ratio < 0.05:  # Less than 5% contact remaining = detached
            outward_limit = d
            break
    
    return (inward_limit, outward_limit)
```

**When to use:** Replace `depth * 0.85` drawer limit estimation. This approach:
- Handles non-standard geometries (L-shaped drawers, angled panels)
- Detects when a drawer has internal rails that limit travel
- Works for revolute too (replacing fixed 120-degree assumption with actual collision boundary)
- Requires trimesh (already in your environment)

### Pattern 5: Enhanced Joint Drive Equations (ArtVIP)

Go beyond basic spring-damper for realistic behaviors in Isaac Sim.

```python
# === Velocity-dependent friction (ArtVIP Eq. 3a-3c) ===
# Configure via Isaac Sim's "Joint Friction" parameter on the joint prim
#
# Static friction: F_friction = -F_ext  (holds still)
#   when q_dot = 0 AND |F_ext| <= mu_s * (|F| + |T|)
#
# Breakaway:  F_friction = -mu_s * (|F| + |T|) * sign(F_ext)
#   when q_dot = 0 AND |F_ext| > mu_s * (|F| + |T|)
#
# Dynamic:    F_friction = -D * q_dot * sign(q_dot)
#   when q_dot != 0
#
# USE FOR: self-closing doors, soft-close drawers, door closers

# === Position-dependent latch release (ArtVIP Eq. 4a-4b) ===
# Implemented as runtime controller, not USD property
#
# q_target = q_upper_bound  if q > q_threshold AND S_open = 1
# q_target = q_lower_bound  if q < q_threshold AND S_open = 0
#
# USE FOR: 
#   - Trash can foot pedal (depress pedal -> lid springs open)
#   - Button-triggered door (press button -> door pops open)
#   - Spring-loaded latch (pull past detent -> releases)

# === Configuring in Isaac Sim USD ===
# For velocity-dependent friction on a revolute joint:
#   joint_prim.CreateAttribute("physxJoint:jointFriction", Sdf.ValueTypeNames.Float).Set(mu_s)
#
# For position-dependent behavior: requires runtime Python controller
#   (cannot be baked into USD — must run in simulation loop)
```

**When to use:** After basic joint setup works. These enhance realism for:
- Refrigerator doors that self-close in the last 10 degrees
- Drawers with soft-close dampers
- Button-triggered mechanisms (trash can pedals, microwave doors)
- Any mechanism where the force profile changes during the motion

### Pattern 6: LLM Articulation Prompting (ArtLLM)

How to structure prompts for LLM-based joint prediction.

```python
# ArtLLM's tokenized representation for articulation prediction
# Key insight: Discretize continuous values into tokens for LLM prediction

# Part bounding box: normalized to [-0.9, 0.9], quantized to 128 bins per axis
PART_TEMPLATE = """
@dataclass
class BBox:
    min_x: int  # 0-127 (quantized from [-0.9, 0.9])
    min_y: int
    min_z: int
    max_x: int
    max_y: int
    max_z: int
"""

# Joint types with discrete parameters
JOINT_TEMPLATES = {
    'revolute': {
        'parent_box_id': 'int',
        'child_box_id': 'int',
        'axis_direction': 'int',        # Discretized to codebook on unit sphere
        'axis_position': '[int, int, int]',  # Quantized 3D position
        'rotation_limit': '[int, int]',      # Quantized angle range
    },
    'prismatic': {
        'parent_box_id': 'int',
        'child_box_id': 'int',
        'axis_direction': 'int',
        'translation_limit': '[int, int]',
    },
    'continuous': {
        'parent_box_id': 'int',
        'child_box_id': 'int',
        'axis_direction': 'int',
        'axis_position': '[int, int, int]',
    },
    'screw': {
        'parent_box_id': 'int',
        'child_box_id': 'int',
        'axis_direction': 'int',
        'axis_position': '[int, int, int]',
        'translation_limit': '[int, int]',
    },
}

# ArtLLM axis direction codebook:
# 128-entry codebook on unit sphere, sampled uniformly
# Additional points on XY, YZ, XZ planes for common axes
# Total: ~150 discrete axis directions
#
# Key insight: Most real-world joints align with cardinal axes (X, Y, Z)
# The codebook is constructed to oversample these regions
```

**When to use:** When designing the Claude prompt for `simready_agent.py`. The discrete token approach works better than asking the LLM to output continuous float values. ArtLLM's multi-task training strategy (3 stages: layout prediction, kinematic prediction, end-to-end) improves stability.

### Pattern 7: Repulsion-Guided Collision Prevention (Part2GS)

Prevent inter-penetration during articulation by placing repel points at contact interfaces.

```python
import numpy as np

def place_repel_points(static_verts, movable_verts, N_R=2000, proximity_threshold=1.5):
    """Place repulsion points at static-movable interface to prevent interpenetration.
    
    From Part2GS: repel points are placed on the static base at locations where
    movable parts are nearby. They generate repulsive forces during articulation.
    
    Args:
        static_verts: (N, 3) static body vertices
        movable_verts: (M, 3) movable part vertices  
        N_R: number of repel points (2000 from Part2GS; stable for 3-7 part objects)
        proximity_threshold: distance threshold for "nearby" (1.5 units)
        
    Returns:
        repel_points: (N_R, 3) positions on static body
    """
    from scipy.spatial import KDTree
    
    # Find static vertices that are near movable parts
    tree = KDTree(movable_verts)
    distances, _ = tree.query(static_verts)
    near_mask = distances < proximity_threshold
    candidates = static_verts[near_mask]
    
    if len(candidates) < N_R:
        # Not enough candidates — use all + pad with random static verts
        extra_idx = np.random.choice(len(static_verts), N_R - len(candidates), replace=True)
        repel_points = np.vstack([candidates, static_verts[extra_idx]])
    else:
        # Subsample uniformly
        idx = np.random.choice(len(candidates), N_R, replace=False)
        repel_points = candidates[idx]
    
    return repel_points

def compute_repulsion_force(movable_position, repel_points, k_r=5e-4, tau_max=0.1):
    """Compute repulsive force on a movable part from repel points.
    
    Force = clip(sum over repel points of k_r * (r_j - mu) / ||r_j - mu||^3)
    
    Args:
        movable_position: (3,) current position of movable part center
        repel_points: (N_R, 3) repulsion point positions
        k_r: repulsion strength (5e-4 from Part2GS)
        tau_max: maximum force magnitude clamp
        
    Returns:
        force: (3,) repulsive force vector
    """
    diff = repel_points - movable_position  # (N_R, 3)
    dist = np.linalg.norm(diff, axis=1, keepdims=True)  # (N_R, 1)
    dist = np.maximum(dist, 1e-5)  # Prevent division by zero
    
    # Inverse cubic falloff (strong at close range, negligible far away)
    forces = k_r * diff / (dist ** 3)
    total_force = forces.sum(axis=0)
    
    # Magnitude clamp for stability
    magnitude = np.linalg.norm(total_force)
    if magnitude > tau_max:
        total_force = total_force * (tau_max / magnitude)
    
    return total_force
```

**When to use:** As a validation tool — after setting joint params, simulate the full range of motion and check if repulsion forces are needed. If they are, the joint params (axis, pivot) are slightly off and need refinement. Part2GS uses this during training, but the concept applies to any collision-free verification.

## Reference Papers
Located at: `scripts/tools/simready_assets/reference_library/papers_articulation/`
- MotionAnymesh (05), ArtLLM (05), PhysX-Anything (10), Part2GS (04)
- Kinematic Kitbashing (06), MagicArticulate (26), SINGAPO, Articulate-Anything

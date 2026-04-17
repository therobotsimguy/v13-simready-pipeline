---
name: collision-physics
description: >-
  Collision detection, collision geometry selection, sim-to-real gap bridging, and
  impulse-based dynamics for articulated rigid bodies. Covers convex hull vs
  decomposition trade-offs, continuous collision detection (CCD), contact parameters,
  and physics engine integration. Use when selecting collision geometry, debugging
  collision issues, or optimizing simulation fidelity. Derived from 7 collision papers
  plus ArtVIP dataset guidelines.
---

# Collision Physics Skill

## When to Use
- Choosing collision geometry type (convexHull vs convexDecomposition vs SDF vs mesh)
- Debugging collision artifacts (invisible walls, pass-through, jams, gaps)
- Setting contact parameters (friction, damping, restitution, contactOffset)
- Bridging sim-to-real gap for collision-rich tasks
- Understanding CCD for fast-moving articulated parts

## Collision Geometry Selection Guide

### Decision Matrix

| Geometry | When to Use | Pros | Cons | Budget Impact |
|----------|------------|------|------|--------------|
| **convexHull** | Simple convex shapes (boxes, cylinders, spheres) | Fast; exact for convex | Bloats concave shapes; invisible walls | Cheapest |
| **convexDecomposition** | Concave shapes (doors, handles, wheels, L-shapes) | Accurate for complex geometry | Slow to compute; PhysX can hang on many pieces | 1 decomp = ~5x hull cost |
| **SDF (Signed Distance Field)** | High-precision contact (gripper fingers, tight clearances) | Sub-mm accuracy; no bloat | GPU-only; memory intensive; asset-level only | Highest |
| **triangle mesh** | Static/kinematic bodies only | Exact shape | Cannot be used for dynamic rigid bodies | N/A |
| **none** | Decorative/interior parts | Saves budget | No interaction | Free |

### The ArtVIP Collision Strategy (Best Practice)
From ArtVIP (992 assets, ICLR 2026):
1. **Simple/regular meshes** -> convex hull (Isaac Sim default)
2. **Complex meshes that can't decompose** -> Split collision volume into multiple primitive shapes (cubes, cylinders)
3. **Neither works** -> Isaac Sim's built-in convex decomposition tool (uses mesh normals)
4. **Budget rule**: max ~5 convexDecomposition meshes per asset (F26 in failure-modes)

### Collision Selection by Part Type

| Part Type | Recommended | Why |
|-----------|------------|-----|
| Main body (cabinet, fridge shell) | convexDecomposition (if concave) or convexHull (if boxy) | Large, often concave |
| Door/lid | convexHull (if flat) or convexDecomposition (if L-shaped) | Usually planar |
| Drawer | convexHull | Box-shaped |
| Handle | convexDecomposition | Small, concave grip surface critical for robot |
| Wheel tire | convexDecomposition | Rolling contact quality matters |
| Wheel bracket/fixer | convexDecomposition | Complex shape around axle |
| Interior shelves/racks | collisionEnabled=False OR convexHull | Usually not interacted with directly |
| Bolts/clips/logos | Skip (no collision) | Decorative; causes jams if included |
| Robot fingers | convexDecomposition at runtime | convexHull bloats fingers 66% (F36) |

## Contact Parameters

### Friction Model (Isaac Sim / PhysX)
Two friction values per material pair:
- **staticFriction (sf)**: force to initiate sliding. Range: [0.0, 2.0]
- **dynamicFriction (df)**: force during sliding. Always df <= sf
- GripMaterial (handles): sf=1.0, df=0.9
- Full friction coefficient table by material pair: see **simready-joint-params** skill

### Critical: material:binding:physics
PhysX ignores friction if only PhysicsMaterialAPI attributes are set. You MUST create a `material:binding:physics` relationship on the mesh. (F30 in failure-modes)

### Contact Offset Rules
- **contactOffset**: gap where PhysX starts computing contacts
  - In asset USD: **DO NOT SET** (F34) -- causes gripper gap
  - At runtime only: set to 0.00005 (0.05mm) for tight interaction
- **restOffset**: penetration depth before separation force
  - Default: 0.0 (fine for most cases)

### Damping for Dynamic Bodies
When main_body is dynamic (trolleys, draggable objects):
- **linearDamping = 100**: prevents wild oscillation when dragged
- **angularDamping = 200**: prevents spinning
- Without these: F32 (body oscillates wildly when dragged)

## Sim-to-Real Collision Gap (CLASH Framework)

### Key Parameters Causing Gap

| Parameter | Sim Default | Real-World Range | Impact |
|-----------|------------|-----------------|--------|
| Friction coefficient | 0.5 | [0.05, 1.0] | Sliding behavior completely wrong |
| Damping ratio | 0.1 | [0.1, 3.0] | Post-collision energy dissipation |
| Restitution | 0.0 | [0.0, 0.8] | Bounce behavior |
| Contact stiffness | Solver default | Material-dependent | Penetration depth |

### CLASH 3-Step Process
1. **Pre-train surrogate model** on 100K simulated collision pairs (MuJoCo)
2. **System identification**: gradient-based optimization to find real-world parameters from ~10 real samples
3. **Fine-tune surrogate** on real-world residuals (10 small-step updates)
- Result: 35% reduction in positioning error for semi-cylinder; 27% for square

### Materials Matter
Real-world collision tested across: Resin, Nylon, PETG-CF (high-flow friction sand)
- Anisotropic geometry (semi-cylinder) makes collision outcome highly sensitive to impact point
- Material pair affects friction and damping more than geometry

## Continuous Collision Detection (CCD) for Articulated Models

### Three-Stage CCD Pipeline (Redon et al.)

```
Stage 1: Dynamic BVH Culling (CPU)
  - Build AABB hierarchy around line-swept-spheres (LSS) of each link
  - Cull links far from environment

Stage 2: Dynamic Swept Volume Culling (GPU)
  - Approximate swept volume of each link using LSS tessellation
  - GPU visibility queries to detect potential collisions

Stage 3: Exact Contact Computation (CPU+GPU)
  - OBB-tree overlap tests between potentially colliding links
  - Interval arithmetic for continuous overlap testing
  - Compute exact time-of-contact (TOC)
```

### Performance (Puma robot in 187K triangle environment)
| Motion | BVH Culling | SV Culling | Exact Contact | Total CCD |
|--------|------------|-----------|--------------|----------|
| 1 degree | 0.33ms | 18.5ms | 7.01ms | 49ms |
| 15 degrees | 0.33ms | 23.3ms | 15.0ms | 44ms |
| 60 degrees | 0.34ms | 43.1ms | 19.1ms | 91ms |

### When to Use CCD
- Fast-moving parts (wheels, flaps, thrown objects)
- Thin geometry (sheet metal, glass panels) that can tunnel through
- Robot arm motion planning (collision-free path verification)

## Impulse-Based Articulation Dynamics

### Articulation Constraint Enforcement (Weinstein et al.)
For articulated rigid bodies with frequent contact and collision:

1. **Pre-stabilization**: Compute target joint state J^target, apply impulses to reach it BEFORE collision resolution
2. **Post-stabilization**: After collision, project velocities back to constraint manifold
3. **Key insight**: Process joints one at a time; solve 6-DOF nonlinear system per joint using Newton iteration

### Joint Constraint Types
- **Revolute**: constant joint position x_j; only rotation around axis allowed
- **Prismatic**: constant orientation q_j; only translation along axis allowed
- **Rigid/Fixed**: both x_j and q_j constant

### Scaling
- Articulation constraints: O(n) in number of joints (not bodies)
- Contact processing: linear in bodies, constant time per contact
- Closed loops: multiple overlapping constraint sets (handled by iterative refinement)

## ArtVIP: Advanced Joint Drive Equations

### Enhanced Joint Drive (Beyond Basic Spring-Damper)
Basic: `tau = K(q - q_target) + D(q_dot - q_dot_target)`

ArtVIP extends for complex behaviors:

**Velocity-dependent friction:**
```
F_friction(q_dot) = {
  -F_ext                                    if q_dot=0 and |F_ext| <= mu_s*(|F|+|T|)
  -mu_s*(|F|+|T|)*sign(F_ext)              if q_dot=0 and |F_ext| > mu_s*(|F|+|T|)
  -D*q_dot*sign(q_dot)                      if q_dot != 0
}
```
- Configurable via Isaac Sim `Joint Friction` parameter
- Enables: self-closing doors, button snap-back, door closers

**Position-dependent latch release:**
```
q_target = {
  q_upper_bound    if q > q_threshold and S_open = 1
  q_lower_bound    if q < q_threshold and S_open = 0
}
```
- Enables: trash can foot pedal, button-triggered door release

For ArtVIP's 5 behavior primitives (latching, damping, cross-asset, within-asset, hover/hold), see **simready-behaviors** skill.

## Reference Papers
Located at: `scripts/tools/simready_assets/reference_library/papers_collision/`
- ArtVIP (02), CLASH (03), Part2GS (04), MotionAnymesh (05)
- Kinematic Kitbashing (06), Dynamic Simulation (07), Fast CCD (08)

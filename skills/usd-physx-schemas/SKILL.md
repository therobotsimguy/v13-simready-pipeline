---
name: usd-physx-schemas
description: >-
  USD Physics and PhysX schema compatibility matrix. Which APIs can coexist, which
  silently conflict, hierarchy rules, kinematic vs dynamic restrictions, collision
  geometry constraints, and articulation gotchas. Use when applying physics APIs to
  USD prims, debugging silent physics failures, or deciding between ArticulationRootAPI
  vs flat rigid bodies. Derived from OpenUSD spec, PhysX 5.5 docs, and Isaac Sim
  empirical testing.
---

# USD + PhysX Schema Compatibility Skill

## When to Use
- Applying physics APIs to a USD prim and unsure what combinations are valid
- Debugging a silent physics failure (body doesn't move, joint ignored, collision missing)
- Choosing between ArticulationRootAPI vs independent rigid bodies
- Setting up kinematic vs dynamic bodies
- Understanding why something works in USD but breaks in PhysX at runtime

## Schema Hierarchy

```
USD Physics Schema (OpenUSD standard)
├── UsdPhysicsRigidBodyAPI        — marks prim as simulated body
├── UsdPhysicsCollisionAPI        — enables collision on geometry
├── UsdPhysicsMeshCollisionAPI    — collision approximation type for meshes
├── UsdPhysicsMassAPI             — explicit mass/density/inertia
├── UsdPhysicsMaterialAPI         — friction and restitution (on Material prim)
├── UsdPhysicsJoint               — base joint constraint
│   ├── UsdPhysicsRevoluteJoint   — 1-DOF rotation
│   ├── UsdPhysicsPrismaticJoint  — 1-DOF translation
│   ├── UsdPhysicsFixedJoint      — 0-DOF weld
│   ├── UsdPhysicsSphericalJoint  — ball-in-socket
│   └── UsdPhysicsDistanceJoint   — distance constraint
├── UsdPhysicsLimitAPI            — joint motion limits (multi-apply)
├── UsdPhysicsDriveAPI            — joint motors/springs (multi-apply)
├── UsdPhysicsArticulationRootAPI — reduced-coordinate articulation marker
├── UsdPhysicsCollisionGroup      — collision filtering groups
├── UsdPhysicsFilteredPairsAPI    — pairwise collision disable
└── UsdPhysicsScene               — simulation parameters (gravity, etc.)

PhysX Schema (NVIDIA extension — from Isaac Sim 4.5 / omni.usd.schema.physx 107.3)
├── PhysxRigidBodyAPI             — extends RigidBodyAPI
├── PhysxCollisionAPI             — extends CollisionAPI
├── PhysxArticulationAPI          — extends ArticulationRootAPI
├── PhysxJointAPI                 — extends Joint
├── PhysxLimitAPI                 — extends LimitAPI
├── PhysxMaterialAPI              — extends MaterialAPI
├── PhysxSceneAPI                 — extends Scene
├── PhysxSDFMeshCollisionAPI      — SDF collision params
├── PhysxConvexDecompositionCollisionAPI — decomposition params
├── PhysxConvexHullCollisionAPI   — hull params
├── PhysxTriangleMeshCollisionAPI — triangle mesh params
├── PhysxTriangleMeshSimplificationCollisionAPI — mesh simplification
├── PhysxSphereFillCollisionAPI   — sphere fill approximation
├── PhysxMeshMergeCollisionAPI    — merge multiple meshes
├── PhysxMimicJointAPI            — mimic/gear joints
├── PhysxForceAPI                 — apply external forces
└── (+ Vehicle, Camera, Particle, Deformable, Tendon APIs — not relevant for rigid SimReady)
```

## PhysX Extension API Properties (Complete Reference)

Properties extracted from Isaac Sim's PhysxSchema module. These EXTEND the base USD Physics properties.

### PhysxRigidBodyAPI (on any RigidBodyAPI prim)

| Property | Type | SimReady Default | Notes |
|----------|------|-----------------|-------|
| LinearDamping | float | 0 (kinematic), 100 (dynamic trolley) | Resistance to linear motion. F32: dynamic body oscillates without this |
| AngularDamping | float | 0 (kinematic), 200 (dynamic trolley) | Resistance to rotation |
| DisableGravity | bool | false | Per-body gravity override |
| EnableCCD | bool | false | Continuous collision detection for fast-moving parts |
| EnableSpeculativeCCD | bool | false | Cheaper CCD alternative |
| MaxLinearVelocity | float | default | Clamp velocity to prevent explosion |
| MaxAngularVelocity | float | default | Clamp angular velocity |
| MaxDepenetrationVelocity | float | default | How fast objects separate after overlap |
| MaxContactImpulse | float | default | Cap contact forces |
| SolverPositionIterationCount | int | 4 | More iterations = more accurate but slower |
| SolverVelocityIterationCount | int | 1 | Velocity-level constraint iterations |
| SleepThreshold | float | default | Energy below which body sleeps |
| RetainAccelerations | bool | false | Keep accelerations across frames |
| EnableGyroscopicForces | bool | false | Spinning top effects |
| LockedPosAxis | int | 0 | Bitfield: lock translation axes |
| LockedRotAxis | int | 0 | Bitfield: lock rotation axes |

### PhysxCollisionAPI (on any CollisionAPI prim)

| Property | Type | SimReady Default | Notes |
|----------|------|-----------------|-------|
| **ContactOffset** | float | **DO NOT SET in asset** | Gap where PhysX starts computing contacts. Set at runtime only (0.00005). F34 |
| **RestOffset** | float | **DO NOT SET in asset** | Penetration depth before separation. Runtime only |
| TorsionalPatchRadius | float | default | Torsional friction patch size |
| MinTorsionalPatchRadius | float | default | Minimum torsional patch |

### PhysxArticulationAPI (on ArticulationRootAPI prim)

| Property | Type | Default | Notes |
|----------|------|---------|-------|
| ArticulationEnabled | bool | true | Master enable for articulation |
| EnabledSelfCollisions | bool | false | Collisions between links of same articulation |
| SolverPositionIterationCount | int | 32 | Higher = more accurate joints. Robots typically need 32+ |
| SolverVelocityIterationCount | int | 1 | Velocity constraint iterations |
| SleepThreshold | float | 0.005 | Energy threshold for articulation sleep |
| StabilizationThreshold | float | 0.001 | Joint stabilization threshold |

### PhysxJointAPI (on any Joint prim)

| Property | Type | SimReady Default | Notes |
|----------|------|-----------------|-------|
| **JointFriction** | float | 0 | Coulomb friction on joint. Use for self-closing doors (ArtVIP Eq. 3) |
| **Armature** | float | 0 | Virtual inertia added to joint. Stabilizes high-ratio drives |
| **MaxJointVelocity** | float | default | Clamp joint velocity. Prevents PhysX#164 crash |

### PhysxLimitAPI (on joint with LimitAPI)

| Property | Type | Default | Notes |
|----------|------|---------|-------|
| BounceThreshold | float | default | Velocity above which limit bounces |
| Stiffness | float | 0 | Limit spring stiffness |
| Damping | float | 0 | Limit damping |
| Restitution | float | 0 | Limit bounce coefficient |

### PhysxConvexDecompositionCollisionAPI

| Property | Type | SimReady Default | Notes |
|----------|------|-----------------|-------|
| **MaxConvexHulls** | int | 128 | Max hulls in decomposition. Budget: ~5 per asset (F26) |
| **VoxelResolution** | int | 500000 | Voxelization resolution for decomposition |
| HullVertexLimit | int | 64 | Max vertices per hull (PhysX limit: 255) |
| ErrorPercentage | float | default | Acceptable approximation error |
| MinThickness | float | default | Minimum hull thickness |
| ShrinkWrap | bool | false | Tighter hull fitting |

### PhysxSDFMeshCollisionAPI

| Property | Type | Default | Notes |
|----------|------|---------|-------|
| SdfResolution | int | 256 | SDF grid resolution. Higher = more accurate + more memory |
| SdfSubgridResolution | int | 6 | Sparse SDF subgrid resolution |
| SdfMargin | float | default | Collision margin around SDF surface |
| SdfNarrowBandThickness | float | default | Active SDF region thickness |
| SdfBitsPerSubgridPixel | int | default | Compression for sparse SDF |
| SdfEnableRemeshing | bool | false | Remesh for better SDF quality |

### PhysxSceneAPI (on PhysicsScene prim — host app, NOT in asset)

| Property | Type | Default | Notes |
|----------|------|---------|-------|
| **EnableGPUDynamics** | bool | false | GPU-accelerated physics |
| **SolverType** | token | PGS | "PGS" or "TGS". TGS better for articulations but can crash (PhysX#164) |
| BroadphaseType | token | default | "GPU" or "MBP" or "SAP" |
| EnableCCD | bool | false | Scene-level CCD enable |
| FrictionType | token | default | "patch" or "oneDirectional" or "twoDirectional" |
| TimeStepsPerSecond | int | 60 | Simulation rate |
| EnableStabilization | bool | false | Extra stabilization pass |
| EnableEnhancedDeterminism | bool | false | Bit-exact determinism (slower) |

## The Compatibility Matrix

### API Combinations: What Works vs What Breaks

| API A | API B | Result | Notes |
|-------|-------|--------|-------|
| RigidBodyAPI | CollisionAPI (on child mesh) | **WORKS** | Standard pattern. Collision shapes on mesh descendants. |
| RigidBodyAPI | MassAPI | **WORKS** | MassAPI overrides auto-computed mass. Can apply on body or child meshes. |
| RigidBodyAPI | kinematicEnabled=true | **WORKS** | Body follows animated pose, pushes dynamic bodies with infinite mass. |
| RigidBodyAPI (parent) | RigidBodyAPI (child) | **CAUTION** | USD spec says nested rigid body creates independent subtree. **PhysX behavior differs**: child body is merged into parent in many cases. Reparent to siblings instead. |
| RigidBodyAPI (grandchild) | — | **BREAKS SILENTLY** | PhysX swallows grandchild rigid bodies. They become part of the nearest ancestor rigid body. F11 in failure-modes. |
| ArticulationRootAPI | kinematicEnabled=true | **BREAKS SILENTLY** | Articulation links CANNOT be kinematic (PhysX restriction). The body won't move. Use FixedJoint to world instead. |
| ArticulationRootAPI | RigidBodyAPI (same prim) | **WORKS** | Standard for floating-base articulation root. |
| ArticulationRootAPI | Flat sibling hierarchy | **WORKS** | Our pipeline pattern: body + parts as siblings under /root, ArticulationRootAPI on /root or body. |
| ArticulationRootAPI | Nested hierarchy | **CAUTION** | Works IF joints connect the tree correctly. But reparenting is safer for SimReady assets. |
| CollisionAPI | No RigidBodyAPI ancestor | **WORKS** | Static collider (world-fixed). No simulation owner needed. |
| CollisionAPI | Triangle mesh + dynamic RigidBody | **NEEDS SDF** | Triangle meshes on dynamic bodies require SDF collision. Without SDF, collision silently fails. |
| MeshCollisionAPI | approximation="none" | **STATIC ONLY** | Raw triangle mesh collision only works for static/kinematic. Dynamic bodies need convexHull, convexDecomposition, or SDF. |
| PhysicsMaterialAPI | material:binding (visual only) | **BREAKS SILENTLY** | PhysX ignores friction if only visual binding exists. MUST use `material:binding:physics` relationship. F30 in failure-modes. |
| PhysicsScene | Inside asset USD | **BAD PRACTICE** | Host app provides PhysicsScene. Asset-embedded scene conflicts with host. F33 in failure-modes. |
| DriveAPI | stiffness > 0 | **CAUTION** | Creates spring-return behavior. For free-moving doors/drawers, stiffness MUST be 0. F18 in failure-modes. |

### Kinematic vs Dynamic: Decision Matrix

| Property | Dynamic Body | Kinematic Body | Static Collider |
|----------|-------------|---------------|-----------------|
| RigidBodyAPI | Yes | Yes (kinematicEnabled=true) | No |
| Responds to forces/gravity | Yes | No (infinite mass) | No |
| Moves other dynamic bodies | Via collision | Pushes with infinite mass | Via collision |
| Collides with kinematic | Yes | **NO** (silent miss) | **NO** |
| Collides with static | Yes | **NO** (silent miss) | N/A |
| Can be in articulation | Yes | **NO** (silent fail) | No |
| Collision shape restriction | convexHull, convexDecomp, or SDF | Any | Any |
| Use case | Movable parts (doors, drawers) | Animated body, main_body | Walls, floors |

**Critical**: Kinematic-to-kinematic and kinematic-to-static collisions generate NO contact response. If your main_body is kinematic and a wall is static, they pass through each other.

### Collision Geometry Restrictions by Body Type

| Geometry Type | Static | Kinematic | Dynamic | Max Verts/Faces | Notes |
|--------------|--------|-----------|---------|-----------------|-------|
| convexHull | Yes | Yes | Yes | 255 | Fastest. Bloats concave shapes. |
| convexDecomposition | Yes | Yes | Yes | Per-hull 255 | Multiple convex hulls. Budget ~5 per asset. |
| triangle mesh (no SDF) | Yes | Yes | **NO** | Unlimited | Static/kinematic only without SDF. |
| triangle mesh + SDF | Yes | Yes | Yes | Unlimited | SDF required for dynamic. Heavy memory. |
| boundingBox / boundingSphere | Yes | Yes | Yes | N/A | Very rough approximation. |
| heightfield | **Static only** | No | No | Grid | Terrain only. |

**Negative scale is NOT supported for convex meshes** — will silently produce wrong collision shape.

## Articulation Rules

### When to Use ArticulationRootAPI

| Scenario | Use ArticulationRootAPI? | Why |
|----------|------------------------|-----|
| Robot arm (Franka, UR5) | **YES** | Reduced coordinates = zero joint error, better mass ratio handling |
| Cabinet with doors/drawers | **NO** (for SimReady) | Kinematic main_body is incompatible with articulations |
| Trolley with wheels (dynamic) | **MAYBE** | Works if ALL links are dynamic. But AssetBaseCfg can't use it. |
| Ragdoll / humanoid | **YES** | Tree of rigid bodies, no kinematic links |

### ArticulationRootAPI Restrictions (from PhysX 5.5)

1. **No kinematic links** — articulation links CANNOT be kinematic. Silent failure.
2. **No breakable joints** — articulation joints cannot break at runtime.
3. **No direct pose setting** — cannot `SetTranslate()` on individual links. Must set joint positions.
4. **No topology changes after scene insertion** — adding/removing links is silently blocked. Must remove articulation from scene, modify, re-add.
5. **Tree topology required** — no loops allowed natively. Loops require external rigid-body joints.
6. **No per-link sleep control** — entire articulation sleeps/wakes as unit.
7. **Spherical joints can drift** — locked axes on spherical joints may drift due to quaternion integration.

### Fixed-Base vs Floating

| Type | How to Set | Root Behavior | Use Case |
|------|-----------|---------------|----------|
| **Fixed-base** | `PxArticulationFlag::eFIX_BASE` | Anchored to world (perfect, no joint needed) | Robot arm on table |
| **Floating** | Default (no flag) | Root moves freely | Mobile robot, ragdoll |

**Fixed-base is superior to FixedJoint-to-world** because the immovable property is solved perfectly, not approximately.

## Joint Schema Details

### Joint Properties (all joint types inherit these)

| Property | Type | Default | Notes |
|----------|------|---------|-------|
| body0 | relationship | — | First body (usually main_body) |
| body1 | relationship | — | Second body (movable part) |
| localPos0 | point3f | (0,0,0) | Joint anchor in body0's local space |
| localPos1 | point3f | (0,0,0) | Joint anchor in body1's local space |
| localRot0 | quatf | (1,0,0,0) | Joint frame orientation relative to body0 |
| localRot1 | quatf | (1,0,0,0) | Joint frame orientation relative to body1 |
| collisionEnabled | bool | false | Enable collision between jointed bodies |
| breakForce | float | inf | Force to break joint (inf = unbreakable) |
| excludeFromArticulation | bool | false | Use maximal coordinates instead of reduced |

**Both localPos at (0,0,0) = broken joint** — part pinned to origin. F14 in failure-modes.

### DriveAPI Formula

```
force = stiffness * (targetPosition - position) + damping * (targetVelocity - velocity)
```

| Parameter | SimReady Default | Notes |
|-----------|-----------------|-------|
| stiffness | **0** | MUST be 0 for free-moving parts. Non-zero = spring return (F18). |
| damping | 2.0 (revolute), 5.0 (prismatic) | Prevents free-fall oscillation |
| targetPosition | 0 | Rest position |
| targetVelocity | 0 | Rest velocity |
| maxForce | inf | Drive force limit |
| type | "force" | "force" or "acceleration" |

### LimitAPI Behavior

| low vs high | Result |
|-------------|--------|
| low < high | Normal range limit |
| low > high | DOF is **locked** |
| low = -inf, high = inf | No limit (free) |
| low = high | Locked at that position |

## PhysX Runtime Gotchas

### Silent Failures (things that break without error)

| # | What Happens | Why | Fix |
|---|-------------|-----|-----|
| 1 | Body doesn't move | kinematicEnabled on articulation link | Remove kinematicEnabled or don't use ArticulationRootAPI |
| 2 | Friction has no effect | Only visual material binding, no `material:binding:physics` | Create physics material binding relationship |
| 3 | Nested body merges into parent | RigidBodyAPI on child/grandchild prim | Reparent as sibling under /root |
| 4 | Dynamic mesh has no collision | Triangle mesh without SDF on dynamic body | Use convexHull, convexDecomposition, or add SDF |
| 5 | Kinematic passes through walls | Kinematic-to-static = no collision response | Use dynamic body if collision needed |
| 6 | Joint at world origin | Both localPos0 and localPos1 = (0,0,0) | Compute anchors with world_point_to_local() BEFORE reparent |
| 7 | contactOffset creates gripper gap | contactOffset baked in asset USD | Strip from asset, set at runtime only (0.00005) |
| 8 | Sleeping bodies ignore gravity change | PhysX doesn't auto-wake on gravity change | Call wakeUp() after changing gravity |
| 9 | Convex hull bloated | Negative scale on convex mesh | Ensure all scales are positive |
| 10 | Drive oscillates wildly | stiffness too high relative to timestep | Reduce stiffness or reduce timestep |
| 11 | Joint limit violated | Contact impulses override limit resolution | Increase solver iteration count |
| 12 | Friction degrades on complex shapes | >32 friction patches per contact manager | Simplify collision geometry |
| 13 | Child parts shifted when parent has non-identity rotation | Reparent uses wrong matrix-multiplication order | USD is row-vector: `new_local = wmat * parent_world.GetInverse()`, NOT `parent_world.GetInverse() * wmat` |
| 14 | Drawers open backward out of chassis | Prismatic direction decided in world frame instead of body-local | Transform drawer and body centers to body-local, then compare on the joint axis |

### Rule: USD uses row-vector matrix convention

USD composes transforms as `p_world = p_local * L2W`. Matrices multiply
left-to-right as applications: `M = M1 * M2` means M1 applied first, then M2.
When reparenting a prim while preserving world pose, the correct recomputation is:

```python
new_local = old_world_matrix * new_parent_L2W.GetInverse()
```

**Not** `GetInverse() * wmat` — that's column-vector convention (OpenGL-style),
not USD's. Hidden until parent has non-identity rotation, because with identity
parent both orders give the same result.

**Symptom seen on:** `EmergencyTrolley_A01_01` — chassis has 181.9° Z-rotation.
Wheels, after pipeline reparent, ended up at sign-flipped (X,Y) positions:
raw (±0.34, ±0.22), pipeline output (±0.54, ±0.33) and (±0.99, ±0.89). Fix was
a one-line order swap in `reparent_prims_preserve_world_xform`.

### Solver Differences: PGS vs TGS

| Aspect | PGS (default CPU) | TGS (GPU default) |
|--------|-------------------|-------------------|
| Friction processing | Final 3 position iterations only | Every iteration |
| Friction symmetry | Can be asymmetric | Symmetric |
| Mass ratio handling | Degrades above ~10:1 | Better (up to ~100:1) |
| Joint error | Accumulates | Lower accumulation |
| When to use | Simple scenes, CPU | Complex articulations, GPU |

## SimReady Asset Pattern (Recommended)

```
/root (DefaultPrim)
  /main_body    [RigidBodyAPI, kinematicEnabled=true, MassAPI, CollisionAPI on meshes]
  /door_left    [RigidBodyAPI, MassAPI, CollisionAPI on meshes]
  /door_right   [RigidBodyAPI, MassAPI, CollisionAPI on meshes]
  /drawer_01    [RigidBodyAPI, MassAPI, CollisionAPI on meshes]
  /wheel_FL     [RigidBodyAPI, MassAPI, CollisionAPI on meshes]
  /joints
    /joint_door_left   [RevoluteJoint, body0=/root/main_body, body1=/root/door_left]
    /joint_door_right  [RevoluteJoint, body0=/root/main_body, body1=/root/door_right]
    /joint_drawer_01   [PrismaticJoint, body0=/root/main_body, body1=/root/drawer_01]
    /joint_wheel_FL    [RevoluteJoint, body0=/root/main_body, body1=/root/wheel_FL]
  /Looks
    /DefaultMaterial   [PhysicsMaterialAPI, material:binding:physics on all meshes]
    /GripMaterial      [PhysicsMaterialAPI sf=1.0 df=0.9, bound to handles]
```

**Rules:**
- NO ArticulationRootAPI (main_body is kinematic = incompatible)
- NO PhysicsScene (host app provides it)
- NO contactOffset in USD (runtime only)
- NO RigidBodyAPI on grandchildren
- ALL movable parts as siblings of main_body (not children)
- ALL collision meshes bound via `material:binding:physics`
- ALL joints connect to main_body as body0
- Use `--dynamic` flag for trolleys (removes kinematicEnabled, adds damping)

## Real-World Issues from GitHub (PhysX + Newton)

Bugs and gotchas reported by users in the wild — things the docs don't warn about.

### PhysX Articulation Issues

| Issue | Problem | Root Cause | Workaround |
|-------|---------|-----------|-----------|
| [PhysX#164](https://github.com/NVIDIA-Omniverse/PhysX/issues/164) | Articulations crash on collision with assertions: `motionAccelerations not finite` | TGS solver goes unstable at high velocities between articulation links | Switch to PGS solver; add joint velocity limits; keep mass ratios < 100:1 |
| [PhysX#161](https://github.com/NVIDIA-Omniverse/PhysX/issues/161) | Spring/damper has zero effect on articulation joints | Using deprecated `TARGET` drive mode (hardcodes stiffness internally, ignores user values) | Use `FORCE` drive mode. TARGET mode deprecated in PhysX 5.3+ |
| [PhysX#204](https://github.com/NVIDIA-Omniverse/PhysX/issues/204) | Joint angles instantly change after single `simulate()` call with no forces | Float32 precision loss in quaternion inversion during Featherstone forward dynamics | Known PhysX bug in specific topologies; issue persists across PhysX 4, 5.1, 5.3 |
| [PhysX#200](https://github.com/NVIDIA-Omniverse/PhysX/issues/200) | Broken joints come back unbroken when removed/re-added to scene | `eBROKEN` flag not preserved on re-insertion | Never reuse broken joints — create new ones from scratch |
| [PhysX#286](https://github.com/NVIDIA-Omniverse/PhysX/issues/286) | D6 Joint doesn't report drive force | Force reporting missing for certain joint configurations | Use articulation joints instead of D6 joints for force feedback |

### PhysX Collision / SDF Issues

| Issue | Problem | Root Cause | Workaround |
|-------|---------|-----------|-----------|
| [PhysX#383](https://github.com/NVIDIA-Omniverse/PhysX/issues/383) | Access violation crash when triangle mesh is very small (mm scale) | SDF computation fails on sub-millimeter geometry | Use convexHull/convexDecomposition for small meshes; ensure units are meters not mm |
| [PhysX#247](https://github.com/NVIDIA-Omniverse/PhysX/issues/247) | SDF + kinematic toggle causes CUDA error and driver crash | Switching rigid body to kinematic while SDF collision is active corrupts GPU state | Don't toggle kinematic on bodies with SDF collision at runtime |
| [PhysX#467](https://github.com/NVIDIA-Omniverse/PhysX/issues/467) | Unstable contact on GPU with TGS solver | TGS GPU solver generates NaNs under certain contact configurations | Reduce simulation complexity or switch to PGS for problematic scenes |
| [PhysX#407](https://github.com/NVIDIA-Omniverse/PhysX/issues/407) | TGS simulation blows up with NaNs | Numeric instability in TGS solver with complex articulated + contact scenarios | Add velocity damping; reduce timestep; switch to PGS if unstable |
| [PhysX#182](https://github.com/NVIDIA-Omniverse/PhysX/issues/182) | Triangle mesh collision errors | Mesh cooking removes/modifies triangles silently | Validate mesh after cooking; watch for zero-area triangles |

### Newton (Isaac Sim Next-Gen) Issues

| Issue | Problem | Root Cause | Workaround |
|-------|---------|-----------|-----------|
| [Newton#980](https://github.com/newton-physics/newton/issues/980) | USD Franka from MuJoCo fails to load: "inertia must have positive values" | URDF→USD conversion produces zero/negative inertia on some links | Ensure all links have valid positive inertia tensors after conversion |
| [Newton#1616](https://github.com/newton-physics/newton/issues/1616) | SDF tests corrupt CUDA context, causing subsequent test failures | CUDA memory corruption from SDF computation leaks across tests | Isolate SDF computations; reset CUDA context between uses |
| [Newton#1384](https://github.com/newton-physics/newton/issues/1384) | `Builder.replicate` doesn't replicate gravity setting | Gravity not copied when duplicating model builders | Set gravity explicitly on replicated builders |
| [Newton#1293](https://github.com/newton-physics/newton/issues/1293) | Gravity direction ignored (only magnitude used) | Code hardcodes `-magnitude` on Y axis, ignores direction vector | Bug — fixed in later versions. Always verify gravity after USD load |
| [Newton#973](https://github.com/newton-physics/newton/issues/973) | No consistent sentinel for "unlimited" joint limits | Different thresholds used across codebase for encoding unlimited joints | Use consistent sentinel values; Newton now standardizing |
| [Newton#911](https://github.com/newton-physics/newton/issues/911) | H1 humanoid URDF gives wrong simulation results | Inertia/mass parsing issues in URDF→Newton conversion | Validate simulation against MuJoCo reference before trusting Newton output |

### Key Takeaways from GitHub Issues

1. **TGS solver is faster but less stable** — PhysX#164, #407, #467 all involve TGS instability. PGS is safer for complex scenes.
2. **SDF collision is fragile** — crashes on small meshes (#383), corrupts GPU on kinematic toggle (#247), corrupts CUDA context (Newton#1616). Use convexHull/convexDecomposition as default; only use SDF when needed.
3. **Drive mode matters** — `TARGET` mode is deprecated and silently broken (#161). Always use `FORCE` mode.
4. **Quaternion precision** — Float32 quaternion math can silently corrupt joint angles in specific topologies (#204). No fix — it's a known limitation.
5. **Don't toggle kinematic at runtime** on bodies with SDF or complex collision (#247). Set once and leave it.
6. **URDF/USD conversion drops properties** — inertia (#980), gravity direction (#1293), joint limits (#973). Always validate after conversion.

## Sources
- OpenUSD Physics Schema: https://openusd.org/release/api/usd_physics_page_front.html
- PhysX 5.5 Rigid Body Dynamics: nvidia-omniverse.github.io/PhysX/physx/5.5.0/docs/RigidBodyDynamics.html
- PhysX 5.5 Articulations: nvidia-omniverse.github.io/PhysX/physx/5.5.0/docs/Articulations.html
- PhysX 5.5 Geometry: nvidia-omniverse.github.io/PhysX/physx/5.5.0/docs/Geometry.html
- Omniverse Physics Schema Overview: docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/usd_schema_api.html
- PhysX GitHub Issues: github.com/NVIDIA-Omniverse/PhysX/issues
- Newton GitHub Issues: github.com/newton-physics/newton/issues

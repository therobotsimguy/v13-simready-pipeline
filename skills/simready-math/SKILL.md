---
name: simready-math
description: >-
  Deterministic math computations for SimReady asset pipeline. Use when
  calculating positions, quaternions, unit conversions, bounding boxes,
  hinge positions, spawn positions, mass estimates, layout grids, or any
  numerical value. Never do mental arithmetic — always call these functions.
  Includes Blow's tetrahedral inertia decomposition and parallel-axis-theorem
  helpers for SimReady asset inertial authoring.
---

# SimReady Math Skill

## Rule
**Never compute numbers in your head.** Always use the functions below or run `python3 -c "..."`.

## Modules

All functions live in `scripts/tools/simready_assets/math_skill/`:

### geometry.py — Positions & Layout

```python
from scripts.tools.simready_assets.math_skill.geometry import (
    LayoutGrid,              # Column/row centers for any rectangular grid
    bbox_center,             # Center of bounding box
    bbox_size,               # (width, depth, height) from bbox
    bbox_volume,             # Volume from bbox
    hinge_position_left,     # center_x - width/2
    hinge_position_right,    # center_x + width/2
    slide_anchor_back_face,  # center_y - depth/2
    vertex_shift_for_pivot,  # How far to shift vertices
    spawn_position_facing_robot,  # X position for desired gap from robot
)
```

**LayoutGrid** — replaces mental column/row math:
```python
grid = LayoutGrid(width=1.2, depth=0.5, height=0.9, columns=3, row_heights=[0.41, 0.41])
grid.col_center_x(0)       # → -0.3867
grid.divider_x_positions()  # → [-0.1933, 0.1933]
grid.shelf_z_positions()    # → [0.43]
```

### units.py — Conversions & Quaternions

```python
from scripts.tools.simready_assets.math_skill.units import (
    mm_to_m, m_to_mm, cm_to_m, m_to_cm,
    deg_to_rad, rad_to_deg,
    quat_from_axis_angle_deg,  # ('Z', 90) → (0.707, 0, 0, 0.707)
    quat_identity,
    quat_to_axis_angle_deg,
    quat_multiply,
    estimate_mass_from_bbox,   # bbox + density → kg
    scale_factor_to_meters,
)
```

### transforms.py — Rotations & Spawn Positions

```python
from scripts.tools.simready_assets.math_skill.transforms import (
    rotate_point_around_z,
    transform_point_by_quat,
    front_face_position_after_rotation,
    spawn_pos_for_gap,  # gap_m + asset_half_depth + rotation → spawn xyz
)
```

## Quick Examples

**Spawn position for 80cm gap:**
```bash
python3 -c "
from scripts.tools.simready_assets.math_skill.geometry import spawn_position_facing_robot
print(spawn_position_facing_robot(0.80, 0.25))
"
```

**Quaternion for -90° around Z:**
```bash
python3 -c "
from scripts.tools.simready_assets.math_skill.units import quat_from_axis_angle_deg
print(quat_from_axis_angle_deg('Z', -90))
"
```

**Mass from bounding box (cm units):**
```bash
python3 -c "
from scripts.tools.simready_assets.math_skill.units import estimate_mass_from_bbox
print(estimate_mass_from_bbox((-60,-67,0), (60,0,152), density_kg_m3=200, mpu=0.01))
"
```

## Inertia Helpers (for SimReady asset authoring)

Deterministic inertia computations — use these instead of the PhysX auto-generated inertia when accuracy matters.

### Blow's Tetrahedral Inertia Decomposition

Compute a body-space inertia tensor from a triangle mesh by tetrahedralizing (signed-volume tets from the origin to each face) and accumulating per-tet contributions. Exact for closed meshes; works for non-convex.

```python
def tetrahedral_inertia_tensor(mesh_points, mesh_faces, density=1.0):
    """Body-space inertia tensor from triangle mesh via Blow's algorithm.

    Args:
        mesh_points: (N, 3) array of mesh vertex positions (world or local body frame)
        mesh_faces: (M, 3) array of triangle vertex indices
        density: kg / m³

    Returns:
        dict with 'mass' (kg), 'com' (3,), 'inertia_tensor' (3, 3) about the COM
    """
    # Reference: Jonathan Blow, "How to find the inertia tensor of a polyhedron"
    # For each triangle face (a, b, c), form a tet with the origin and accumulate
    # signed-volume-weighted contributions to mass, first moments, and second moments.
    # ... (full implementation in simready_assets/math_skill/inertia.py)
```

**When to use:**
- Asset requires accurate inertia (tall or thin parts — wheels, drawers, doors where PhysX auto-inertia is unreliable).
- Any time F59 (principal-axes misalignment) or F60 (armature-as-stabilizer) is suspected.
- Cross-ref `simready-joint-params §Inertial Authoring Precedence`.

### Parallel-Axis Shift

Move an inertia tensor from the centroid to an arbitrary reference point.

```python
def parallel_axis_shift(I_C, mass, d):
    """Shift inertia tensor by vector d (from centroid to new origin).

    I_O = I_C + m * ((d² I) - (d outer d))

    Args:
        I_C: (3, 3) inertia at centroid
        mass: scalar (kg)
        d: (3,) shift vector from centroid to new reference point

    Returns:
        I_O: (3, 3) inertia at new reference point
    """
    import numpy as np
    d = np.asarray(d, dtype=float)
    d_sq = d @ d  # ||d||²
    I_3 = np.eye(3)
    return I_C + mass * (d_sq * I_3 - np.outer(d, d))
```

**Rule:** never adjust COM without re-running this shift. Ignoring the shift invalidates the tensor (F59 / F60).

<!-- source: bundle4/section_file/4_physical_parameter_identification (Blow algorithm + parallel-axis theorem), confidence: HIGH -->

## For USD-specific transforms

Use pipeline functions (require `pxr`):
- `_world_point_to_local_body()` in `stage_f.py`
- `_mesh_world_bbox_via_vertices()` in `stage_f.py`
- `world_point_to_local()` in `make_simready.py`
- `mesh_world_bbox()` in `make_simready.py`

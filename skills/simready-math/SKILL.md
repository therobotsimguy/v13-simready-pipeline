---
name: simready-math
description: >-
  Deterministic math computations for SimReady asset pipeline. Use when
  calculating positions, quaternions, unit conversions, bounding boxes,
  hinge positions, spawn positions, mass estimates, layout grids, or any
  numerical value. Never do mental arithmetic — always call these functions.
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

## For USD-specific transforms

Use pipeline functions (require `pxr`):
- `_world_point_to_local_body()` in `stage_f.py`
- `_mesh_world_bbox_via_vertices()` in `stage_f.py`
- `world_point_to_local()` in `make_simready.py`
- `mesh_world_bbox()` in `make_simready.py`

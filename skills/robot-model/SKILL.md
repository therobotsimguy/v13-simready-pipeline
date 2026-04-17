---
name: robot-model
description: >-
  Franka Emika Panda robot specs as reference for SimReady asset design.
  Use when building assets that a robot must manipulate — ensures handles
  are graspable, forces are achievable, clearances fit the gripper, and
  joint parameters are within robot capabilities.
---

# Robot Model Skill — Franka Emika Panda

## Why This Matters
Every SimReady asset must be manipulable by the Franka. If a handle is too thick for the gripper, or a door requires more torque than the robot can exert, the asset is useless for simulation. Use these specs as hard constraints when designing assets.

## Franka Panda Specifications

### Gripper (from URDF + franka.py)
| Parameter | Real Franka | Isaac Lab Sim | Asset Constraint |
|-----------|------------|---------------|-----------------|
| Max opening | **80mm** (0.04m × 2 fingers) | Same | Handle cross-section < 80mm |
| Finger force (URDF) | **20 N per finger** | **200 N** `effort_limit_sim` | Sim is 10× stronger than real |
| Finger travel | 0–0.04m per finger | Same | |
| Finger stiffness | — | 2000 N/m | |
| Finger damping | — | 100 N·s/m | |

### Arm (from URDF + franka.py)
| Parameter | URDF / Real | Isaac Lab Sim | Source |
|-----------|------------|---------------|--------|
| Joints 1-4 torque | **87 Nm** | **87 Nm** `effort_limit_sim` | URDF + franka.py |
| Joints 5-7 torque | **12 Nm** | **12 Nm** `effort_limit_sim` | URDF + franka.py |
| Joints 1-4 velocity | **2.175 rad/s** | URDF default | URDF |
| Joints 5-7 velocity | **2.61 rad/s** | URDF default | URDF |
| DOF | 7 + 2 fingers | Same | |
| Payload | **3 kg** | Not in config | Franka datasheet |
| Reach | **0.855 m** | Not in config | Franka datasheet |

### Derived Force Limits (what the robot can exert on assets)

| Action | Real Franka | In Sim (stronger) |
|--------|------------|-------------------|
| Open door (handle at 0.3m) | 20N × 0.3m = **6 Nm** | 200N × 0.3m = **60 Nm** |
| Pull drawer | **20 N** per finger | **200 N** sim |
| Push object | Wrist 12 Nm | Same |
| Turn knob (0.02m radius) | 20N × 0.02m = **0.4 Nm** | 200N × 0.02m = **4 Nm** |

**WARNING:** Isaac Lab sim gripper is ~10× real Franka. Assets designed for sim-only gripper force (200N) will fail on real hardware (20N). Design for real limits if targeting sim-to-real transfer.

## Asset Design Rules

### Handle Sizing
```
Handle cross-section < 80mm (gripper opening)
Handle diameter: 15-40mm ideal for stable grasp
Handle length: > 40mm for two-finger contact
Handle protrusion from door: > 25mm for finger clearance
```

### Joint Friction Limits
```
FOR REAL FRANKA (20N finger):
  Door hinge static friction: < 6 Nm (20N × 0.3m handle)
  Drawer rail friction: < 20 N (finger force)
  Knob rotation friction: < 0.4 Nm (20N × 0.02m radius)

FOR SIM (200N effort_limit_sim):
  Door hinge static friction: < 60 Nm
  Drawer rail friction: < 200 N
  Knob rotation friction: < 4 Nm

Design for REAL limits if targeting sim-to-real transfer.
```

### Collision Geometry
```
Graspable surfaces: convexHull (not triangle mesh — causes penetration)
Handle collision: must exist (gripper passes through visual-only meshes)
Handle mesh width: strictly < 80mm for gripper to close around it
```

### Mass Limits
```
Graspable objects: < 3 kg (payload limit)
Doors/drawers (attached): any mass, but friction must be within force limits
Recommended door mass: 2-10 kg
Recommended drawer mass: 1-5 kg
```

### Spawn Distance
```
Interaction point must be within 0.855m of robot base
Recommended gap: 0.6-0.8m from base to asset front face
Asset handle height: 0.3-1.2m (Franka comfortable workspace)
```

## Validation Checklist

Before declaring an asset SimReady for Franka:

- [ ] All handles < 80mm cross-section
- [ ] Handle colliders present (`CollisionAPI` on handle meshes)
- [ ] Door hinge friction < 5 Nm
- [ ] Drawer rail friction < 30 N
- [ ] Graspable object mass < 3 kg
- [ ] Interaction points within 0.855m reach
- [ ] Joint damping low enough for Franka forces (damping < 10 for revolute, < 20 for prismatic)
- [ ] convexHull collision on all graspable surfaces

## Isaac Lab Integration

The Franka in teleop is spawned at origin `(0, 0, 0)` facing +X:

```python
# Spawn asset 80cm from robot, front facing robot
env_cfg.scene.cabinet = AssetBaseCfg(
    prim_path="{ENV_REGEX_NS}/CustomAsset",
    spawn=UsdFileCfg(usd_path=path),
    init_state=AssetBaseCfg.InitialStateCfg(
        pos=(1.05, 0.0, 0.0),   # 80cm gap + half depth
        rot=(0.707, 0, 0, -0.707),  # front faces -X (toward robot)
    ),
)
```

## Reference

Full Franka × behavior parameter tables in:
- [COMPLETE_BEHAVIOR_SEMANTIC_MAPPING.md](../../../scripts/tools/simready_assets/COMPLETE_BEHAVIOR_SEMANTIC_MAPPING.md) — Part 3 (Isaac Sim implementations) and Part 4 (extended behaviors)

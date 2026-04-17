---
name: simready-joint-params
description: >-
  Joint parameter lookup tables for 20 categories of articulated objects. Given an
  object type (door, drawer, wheel, valve, etc.), returns: joint type, axis, limits,
  mass range, stiffness, damping, friction values, and robot interaction notes.
  Derived from the reference library's articulated_object_catalog and
  articulated_object_reference, plus ArtVIP empirical data. Use on EVERY asset to
  get initial parameter estimates before geometry-based refinement.
---

# SimReady Joint Parameters Skill

## When to Use
- Setting initial joint parameters for any new asset
- Validating that make_simready.py output is in the right ballpark
- Quick lookup during classification to match part type to physics

## Quick Reference: Common Household Objects

### Doors (Revolute)

| Door Type | Axis | Limits (deg) | Damping | Stiffness | Mass (kg) | Friction (s/d) |
|-----------|------|-------------|---------|-----------|-----------|---------------|
| Cabinet door (side hinge) | Z | [0, 120] or [-120, 0] | 2.0 | 0 | 2-15 | 0.4/0.3 |
| Fridge door | Z | [0, 120] | 2.0 | 0 | 15-40 | 0.5/0.4 |
| Oven door (top hinge) | X | [-90, 0] | 3.0 | 0 | 5-20 | 0.5/0.4 |
| Microwave door | Z | [0, 120] | 2.0 | 0 | 1-3 | 0.3/0.2 |
| Washing machine door | Z | [0, 150] | 2.0 | 0 | 3-8 | 0.4/0.3 |
| Toilet lid | X | [0, 90] | 1.5 | 0 | 0.5-2 | 0.3/0.2 |
| Laptop lid | X | [0, 135] | 1.0 | 0 | 0.3-1.0 | 0.4/0.3 |
| Car door | Z | [0, 75] | 5.0 | 0 | 15-30 | 0.5/0.4 |
| Barn door (sliding) | -- | See prismatic | -- | -- | 10-40 | -- |

### Drawers (Prismatic)

| Drawer Type | Axis | Limits (m) | Damping | Stiffness | Mass (kg) | Friction (s/d) |
|------------|------|-----------|---------|-----------|-----------|---------------|
| Kitchen drawer | Y | [0, depth*0.85] | 5.0 | 0 | 1-5 | 0.3/0.2 |
| Filing cabinet drawer | Y | [0, 0.6] | 5.0 | 0 | 2-8 | 0.3/0.2 |
| Desk drawer | Y | [0, 0.45] | 5.0 | 0 | 0.5-3 | 0.3/0.2 |
| Tool chest drawer | Y | [0, 0.5] | 5.0 | 0 | 3-10 | 0.4/0.3 |
| Nightstand drawer | Y | [0, 0.35] | 5.0 | 0 | 0.5-2 | 0.3/0.2 |

### Wheels (Continuous)

| Wheel Type | Axis | Limits (deg) | Damping | Mass (kg) | Notes |
|-----------|------|-------------|---------|-----------|-------|
| Caster wheel (swivel) | Y or X (thin dim) | [-9999, 9999] | 2.0 | 0.05-1.0 | Axis = thin bbox dimension |
| Chair wheel | Y or X | [-9999, 9999] | 2.0 | 0.1-0.5 | All parts convexDecomposition |
| Cart wheel | Y or X | [-9999, 9999] | 2.0 | 0.2-1.0 | Structural parts under body, rotating under wheel |
| Steering wheel | Z | [-540, 540] | 3.0 | 1-3 | Axis perpendicular to wheel plane |

### Knobs and Valves (Revolute)

| Type | Axis | Limits (deg) | Damping | Mass (kg) |
|------|------|-------------|---------|-----------|
| Door knob | Y or Z | [0, 90] | 1.0 | 0.2-0.5 |
| Faucet handle | Z | [0, 90] | 1.5 | 0.3-0.8 |
| Stove knob | Z | [0, 270] | 1.0 | 0.1-0.3 |
| Ball valve | Z | [0, 90] | 2.0 | 0.5-3 |
| Gate valve (handwheel) | Z | [0, 3600] | 2.0 | 1-5 |

### Lids and Covers (Revolute)

| Type | Axis | Limits (deg) | Damping | Mass (kg) |
|------|------|-------------|---------|-----------|
| Trash can lid (foot pedal) | X | [0, 75] | 2.0 | 0.3-1.0 |
| Storage chest lid | X | [0, 110] | 2.0 | 1-5 |
| Bottle cap (screw) | Z | [0, 720] | 0.5 | 0.01-0.05 |
| Car hood | X | [0, 60] | 5.0 | 10-25 |
| Car trunk | X | [0, 75] | 5.0 | 8-20 |

### Buttons and Switches (Prismatic/Revolute)

| Type | Joint | Axis | Limits | Damping | Mass (kg) |
|------|-------|------|--------|---------|-----------|
| Push button | Prismatic | Z | [0, 0.005] | 1.0 | 0.01-0.05 |
| Toggle switch | Revolute | X | [-30, 30] | 0.5 | 0.02-0.1 |
| Rocker switch | Revolute | X | [-15, 15] | 0.5 | 0.02-0.1 |
| Keyboard key | Prismatic | Z | [0, 0.004] | 0.5 | 0.005-0.02 |

### Sliders and Sliding Doors (Prismatic)

| Type | Axis | Limits (m) | Damping | Mass (kg) |
|------|------|-----------|---------|-----------|
| Sliding door | X or Y | [0, width*0.9] | 5.0 | 5-30 |
| Sliding window | X | [0, width*0.5] | 3.0 | 2-10 |
| Pocket door | X | [0, width*0.95] | 5.0 | 10-30 |
| Sliding shelf | Y | [0, depth*0.8] | 3.0 | 0.5-3 |

## Material Density Reference

| Material | Density (kg/m3) | Common Objects |
|----------|----------------|----------------|
| Softwood (pine) | 400-600 | Shelves, light furniture |
| Hardwood (oak) | 600-900 | Doors, cabinets, desks |
| Steel | 7,850 | Hinges, locks, tools |
| Aluminum | 2,700 | Laptop, light frames |
| ABS Plastic | 1,040-1,070 | Appliance housings |
| Glass | 2,400-2,800 | Windows, screens |
| Foam | 30-50 | Cushions, padding |
| MDF/Particle board | 600-800 | Cabinet bodies, shelves |
| Rubber | 1,100-1,300 | Wheels, gaskets |
| Ceramic | 2,300-2,500 | Toilets, sinks |

## Friction Coefficient Reference

| Material Pair | Static | Dynamic |
|--------------|--------|---------|
| Wood-Wood | 0.25-0.50 | 0.20 |
| Steel-Steel | 0.74 | 0.57 |
| Steel-Aluminum | 0.61 | 0.47 |
| Rubber-Concrete | 1.0 | 0.80 |
| Rubber-Metal | 0.80 | 0.60 |
| Teflon-Steel | 0.04 | 0.04 |
| Plastic-Metal | 0.35 | 0.30 |
| GripMaterial (SimReady) | 1.00 | 0.90 |

## Axis Convention

| Scenario | Axis | Notes |
|----------|------|-------|
| Vertical hinge (door) | Z | World up = Z |
| Horizontal hinge (lid, oven door) | X | Perpendicular to front face |
| Drawer pull direction | Y | Depth axis (into/out of cabinet) |
| Wheel axle | Thin bbox dimension | Measured from tire: Y if thin in Y, X if thin in X |
| Button press | Z (usually) | Into surface |
| Sliding door | X (usually) | Along wall |

For Franka robot constraints (grip force, reach, payload, handle sizing), see **robot-model** skill.

## Reference
Full tables: `scripts/tools/simready_assets/reference_library/articulated_object_catalog.md`
Engineering foundations: `scripts/tools/simready_assets/reference_library/articulated_object_reference.md`

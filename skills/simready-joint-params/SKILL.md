---
name: simready-joint-params
description: >-
  Joint parameter lookup tables for 20 categories of articulated objects. Given an
  object type (door, drawer, wheel, valve, etc.), returns: joint type, axis, limits,
  mass range, stiffness, damping, friction values, and robot interaction notes.
  Derived from the reference library's articulated_object_catalog and
  articulated_object_reference, plus ArtVIP empirical data. Use on EVERY asset to
  get initial parameter estimates before geometry-based refinement. Includes
  Rabinowicz/Bhushan tribological friction values, Stribeck velocity-dependent
  friction curves, BIFMA/IEC/ASME standards-compliance cyclic loads, inertial
  authoring precedence (CAD > proxy > auto), parallel-axis-theorem helpers, and
  cross-simulator parameter mapping (PhysX compliance ↔ MuJoCo solimp/solref).
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

## Additional Columns for Every Category Table

Every category table above can be augmented with these columns for production-grade provenance:

| New Column | Domain | Example Values |
|------------|--------|----------------|
| `max_velocity` | deg/s or m/s | 200–300 deg/s (training-safe default for revolute); 0.3–1.0 m/s (prismatic) |
| `friction_source` | string | `steel_dry`, `steel_lubricated`, `teflon`, `elastomer`, `assumed` |
| `density_source` | string | `MatWeb`, `CES_Selector`, `assumed`, `CAD_derived` |
| `provenance_stack` | enum | `analytic`, `cad_derived`, `spec_derived`, `empirical_prior`, `fitted`, `learned` |
| `confidence` | float [0, 1] | 0.95 = empirically validated; 0.5 = inferred from class; 0.2 = fallback default |
| `physx_compliance` | float | PhysX soft-limit compliance, e.g., 0.01 |
| `mujoco_solimp` | list[float] | MuJoCo impedance, e.g., `[0.9, 0.95, 0.001]` |
| `mujoco_solref` | list[float] | MuJoCo time constant, e.g., `[0.01, 0.99]` |

The `provenance_stack` + `confidence` columns let the classifier emit an explicit uncertainty trail — downstream auditors can gate on confidence thresholds.

<!-- source: bundle1/KB §2 + bundle4/section_file/4_physical_parameter_identification + bundle6/research_parameters_physx_and_gpu_tradeoffs.csv, confidence: HIGH -->

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

## Rabinowicz Tribological Friction Reference

Empirical static (μ_s) and kinetic (μ_k) friction coefficients from Rabinowicz (1995) and Bhushan (2013) tribology references. Use for `friction_source` column.

| Contact pair | μ_s (static) | μ_k (kinetic) | Notes |
|--------------|-------------:|-------------:|-------|
| Steel on steel (dry) | 0.74 | 0.57 | Default for unlubricated metal mechanisms |
| Steel on steel (lubricated) | 0.16 | 0.09 | Use for bearings, lubricated joints |
| Aluminum on steel | 0.61 | 0.47 | Common for aluminum extrusion drawer slides |
| Copper on steel | 0.53 | 0.36 | |
| Brass on steel | 0.51 | 0.44 | |
| Teflon on Teflon | 0.04 | 0.04 | Lowest μ of common engineered surfaces — slider guides |
| Rubber (elastomer) on concrete (dry) | 1.00 | 0.80 | Caster tires on floors |
| Rubber on linoleum | 0.80 | 0.70 | Hospital/OR trolley wheels |
| Wood on wood | 0.50 | 0.25 | |
| Glass on glass | 0.94 | 0.40 | |

**Stribeck friction model (velocity-dependent):** real materials show μ as a function of sliding velocity. Regime transitions: static → boundary lubrication → mixed → hydrodynamic. For robotics authenticity (self-closing doors, gripper slip):
- Below breakaway velocity: μ ≈ μ_s (static)
- Just above breakaway: Stribeck dip (lowest friction)
- Higher velocities: rises back to a hydrodynamic value

Use velocity-dependent friction curve in PhysX or MuJoCo, OR apply domain randomization around Rabinowicz values for sim-to-real robustness. Cross-ref K08 failure mode.

<!-- source: bundle4/section_file/4_physical_parameter_identification (Rabinowicz 1995, Bhushan 2013) + bundle2/corrected_brief (Stribeck), confidence: HIGH -->

## Standards-Compliance Cyclic Loads

For assets subject to industrial/medical standards. Use as hard constraints during generation and as fatigue-life validation targets.

### BIFMA X5.1 — Office Chairs

| Test | Functional Load (N) | Proof Load (N) | Cycles | Implication |
|------|---------:|-----------:|--------:|-------------|
| Backrest static | 667 | 1001 | — | Material selection, cross-section design |
| Drop dynamic | 1001 | 1334 | — | Base / caster structural integrity |
| Swivel cyclic | 1201 | — | 120,000 @ 13 cpm | Bearing/swivel-joint fatigue design |
| Tilt cyclic | 1068 | — | 300,000 @ 19–20 cpm | **Most critical articulated component** |

### IEC 60601-2-52 — Medical Beds (Hard Geometric Constraints)

| Feature | Constraint | Rationale |
|---------|-----------|-----------|
| Internal rail gaps | < 120 mm | Head entrapment avoidance |
| Frame component gaps | < 60 mm | Neck-entrapment prevention |
| Pinch-point clearance | > 25 mm | Finger-injury prevention |
| Side rail height | > 220 mm | Fall prevention |

**Automated gap audit:** detect any gap in the forbidden range 4.9–11.8 cm; flag for redesign. Cross-ref K06 failure.

### ASME B30.20 / BTH-1 — Below-the-Hook Lifting Devices

- **Design Category A** (normal use) vs **B** (severe/unusual).
- **Service Class 0–5** (load cycles × severity).
- Determines safety factors + fatigue-analysis requirements. Inputs directly to FEA stress simulation during design phase.

<!-- source: bundle4/section_file(4)/1_industrial_design_standards_report, confidence: HIGH -->

## Inertial Authoring Precedence

When assigning inertia tensors to rigid bodies:

1. **CAD-derived** (best): compute from meshes + density via volumetric integral. Use Blow's tetrahedral inertia algorithm — see `simready-math`.
2. **Proxy geometry**: fit inertia to bounding primitives when full mesh is unavailable.
3. **Auto-generated**: fallback only; PhysX default inertia is often wrong for off-axis assets.

**Rule:** never adjust COM without recomputing inertia. Use parallel-axis theorem: `I_O = I_C + m((d²)I − dd^T)` where `d` is the shift vector and `I_C` is the tensor at the centroid. See `simready-math §parallel_axis_shift`.

Cross-ref F59 (principal-axes misalignment), F60 (armature-masking-bad-inertia anti-pattern).

<!-- source: bundle1/KB §3 + bundle4/part1 + section_file/4, confidence: HIGH -->

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

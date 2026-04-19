---
name: simready-mechanism-lookup
description: >-
  Lookup DOF count, joint types, assembly hierarchy, and behavioral description
  for 100 industrial and mechanical mechanisms. Covers robotic arms, hydraulic
  systems, gearboxes, valves, pumps, motors, clutches, brakes, transmissions,
  linkages, casters, hinges, locks, tools, and more. Use when encountering an
  unknown mechanism or industrial object that needs articulation parameters.
  Derived from industrial_assets_part1-4 in the reference library. Also includes
  Reuleaux kinematic-pair primitives (R/P/H/C/S/F lower pairs), biomechanical
  joint equivalents (ball-socket 3-DOF, saddle 2-DOF, knee 6-DOF), musculoskeletal
  benchmarks (OpenSim, MyoSuite), bio-inspired actuator advisory (hydrogel/SMA/
  EAP/TCP), and classification-path metadata (function → pair → chain → topology).
---

# SimReady Mechanism Lookup Skill

## When to Use
- Encountering an unfamiliar industrial or mechanical object
- Need to know DOF count, joint types, or assembly structure for a mechanism
- Classifying parts of complex mechanical assemblies
- Setting up articulation for tools, industrial equipment, or vehicles

## Quick DOF Lookup (Top 50 Most Useful)

| # | Mechanism | DOF | Primary Joints | Key Motion |
|---|-----------|-----|---------------|------------|
| 1 | 6-axis robotic arm | 6 | 6x revolute | Each axis rotates independently |
| 2 | Hydraulic excavator | 10+ | Revolute + prismatic (hydraulic) | Boom, stick, bucket rotation + cylinder extension |
| 3 | CNC machining center | 5 | 3 prismatic (XYZ) + 2 revolute (tilt/rotate) | 5-axis milling |
| 4 | Planetary gearbox | 1 (effective) | Multiple meshing gears | Speed reduction, torque multiplication |
| 5 | Ball screw actuator | 1 | Helical (screw) | Rotation -> linear translation |
| 6 | Pneumatic cylinder | 1 | Prismatic | Linear push/pull |
| 7 | Hydraulic cylinder | 1 | Prismatic | Linear push/pull (high force) |
| 8 | Parallel gripper | 1 | 2x prismatic (mirrored) | Symmetric open/close |
| 9 | Harmonic drive | 1 | Continuous revolute | High ratio speed reduction |
| 10 | Delta robot | 3 | 3x revolute (base) + passive | Fast pick-and-place |
| 11 | SCARA robot | 4 | 2 revolute + 1 prismatic + 1 revolute | Horizontal plane assembly |
| 12 | Universal joint | 2 | 2x revolute (perpendicular) | Angle transmission |
| 13 | Rack and pinion | 1 | Revolute -> prismatic | Rotation to linear (steering) |
| 14 | Gate valve | 1 | Revolute (handwheel) -> prismatic (gate) | Flow control: full open/close |
| 15 | Ball valve | 1 | Revolute (90 deg) | Quarter-turn flow control |
| 16 | Butterfly valve | 1 | Revolute | Disc rotation for flow |
| 17 | Centrifugal pump | 1 | Continuous revolute | Impeller spin |
| 18 | Disc brake caliper | 1 | Prismatic (piston) | Pad squeeze on rotor |
| 19 | Manual transmission (5-speed) | 2 | 1 revolute (shift) + 1 prismatic (select) | Gear engagement |
| 20 | Differential gear | 3 | 3x revolute (ring, 2 side) | Wheel speed compensation |
| 21 | CV joint | 2 | 2x revolute | Constant velocity angle drive |
| 22 | Scissor lift | 1 | Prismatic (actuator) -> linkage | Vertical platform lift |
| 23 | Pantograph | 1 | 4-bar linkage (1 DOF input) | Parallelogram motion |
| 24 | Toggle clamp | 1 | Revolute (over-center) | Quick clamp/release |
| 25 | Four-bar linkage | 1 | 4x revolute (1 DOF effective) | Coupler curve motion |
| 26 | Crank-slider | 1 | Revolute -> prismatic | Engine piston |
| 27 | Geneva drive | 1 | Intermittent revolute | Indexing (step rotation) |
| 28 | Ratchet and pawl | 1 | Revolute (one-way) | Prevents reverse rotation |
| 29 | Cam and follower | 1 | Revolute (cam) -> prismatic (follower) | Custom motion profile |
| 30 | Chain drive | 1 | 2x revolute (sprockets) + chain | Power transmission |
| 31 | Belt drive + tensioner | 1 | 2x revolute (pulleys) + 1 prismatic (tensioner) | Power + tension adjustment |
| 32 | Conveyor belt | 1 | Continuous revolute (drive roller) | Material transport |
| 33 | Electric linear actuator | 1 | Revolute (motor) -> prismatic (screw) | Precise linear positioning |
| 34 | Pan-tilt unit | 2 | 2x revolute (pan + tilt) | Camera/sensor pointing |
| 35 | Door hinge | 1 | Revolute | Door swing |
| 36 | Door knob + latch | 2 | Revolute (knob) + prismatic (bolt) | Turn to retract bolt |
| 37 | Deadbolt lock | 1 | Prismatic (bolt via thumb turn) | Security locking |
| 38 | Telescoping drawer slide | 1 | Prismatic (3-stage) | Full-extension drawer |
| 39 | Gas spring/strut | 1 | Prismatic (damped) | Controlled lift (car hood, cabinet) |
| 40 | Industrial caster | 2 | 1 revolute (swivel) + 1 revolute (roll) | Direction + rolling |
| 41 | Swivel chair base | 1-6 | 1 revolute (swivel) + 1 prismatic (gas lift) + 5 casters | Office chair mobility |
| 42 | Adjustable wrench | 1 | Helical (worm screw) | Jaw width adjustment |
| 43 | Scissors | 1 | Revolute (pivot) | Cutting action |
| 44 | Stapler | 1 | Revolute (press) | Staple driving |
| 45 | Laptop hinge | 1 | Revolute (friction) | Screen angle |
| 46 | Retractable pen | 1 | Prismatic (click) | Tip extend/retract |
| 47 | Drill chuck | 1 | Helical | Jaw tightening |
| 48 | Hole punch | 1 | Revolute (lever) -> prismatic (punch) | Paper punching |
| 49 | Carabiner | 1 | Revolute (spring-loaded gate) | Quick-connect |
| 50 | Ratchet tie-down | 2 | 1 revolute (ratchet) + 1 prismatic (strap) | Tensioning |

## Joint Type Decision Guide

| If the mechanism... | Then use... | Examples |
|--------------------|------------|---------|
| Rotates around a fixed axis | **Revolute** | Doors, hinges, knobs, levers |
| Slides along a fixed axis | **Prismatic** | Drawers, pistons, slides, buttons |
| Spins continuously (no limits) | **Continuous** | Wheels, rollers, fans, drill bits |
| Rotates AND translates (screw) | **Helical** | Bottle caps, adjustable wrenches, lead screws |
| Connects with no relative motion | **Fixed** | Bolts, welds, structural connections |
| Has 2 rotation axes (perpendicular) | **Universal** | Drive shafts, gimbal joints |
| Has 3+ rotation axes | **Ball/Spherical** | Shoulder joints, trackballs |

## Assembly Hierarchy Pattern

Most mechanical assemblies follow this parent-child pattern:

```
/root
  /main_body (base frame / housing / chassis)
    -- Fixed to world or kinematic
  /part_A (first movable -- e.g., door)
    -- Joint to main_body
    /sub_part_A1 (child of A -- e.g., handle)
      -- Fixed joint to part_A (moves with it)
  /part_B (second movable -- e.g., drawer)
    -- Joint to main_body
```

**Key rule**: Only direct children of main_body get joints. Sub-components stay as children of their parent (F38).

## Reuleaux Kinematic-Pair Primitives

Formal taxonomy for classifying any joint. Backends prefer lower pairs; higher pairs are only used when lower pairs cannot capture the motion.

### Lower Pairs (6 canonical)

| Symbol | Name | DOF | USD Joint Type | Example |
|--------|------|-----|----------------|---------|
| R | Revolute | 1 rot | `PhysicsRevoluteJoint` | Door hinge, knob |
| P | Prismatic | 1 trans | `PhysicsPrismaticJoint` | Drawer, slider |
| H | Helical (screw) | 1 coupled rot+trans | `PhysicsJoint` w/ coupling | Ball-screw actuator, lead screw |
| C | Cylindrical | 2 (rot + trans same axis) | Decomposed R + P OR D6 | Piston with rotation |
| S | Spherical | 3 rot | `PhysicsSphericalJoint` (PhysX uses pyramidal limits) | Ball-and-socket, wrist |
| F | Flat / Planar | 3 (2 trans + 1 rot in plane) | D6 w/ 3 locked axes | Planar slider |

### Higher Pairs (use sparingly)

| Pair | When | PhysX/MuJoCo representation |
|------|------|------------------------------|
| Gear | Coupled rotation with ratio | Fixed tendon OR equality constraint OR compliant contact |
| Cam | Nonlinear position mapping | Equality with polycoef (MuJoCo) OR custom drive |
| Point contact | Rolling without constraint | Dynamic contact only (no joint) |

**Rule:** descend the classification tree function → pair type → chain → topology. Decide the pair type FIRST, not the joint name. Cross-ref `articulation-pipelines §Mechanism Synthesis Context`.

<!-- source: bundle4/section_file(4)/3_topic_19 + bundle6/research_mechanism_taxonomy_and_synthesis.csv + KMODDL Cornell Reuleaux Collection, confidence: HIGH -->

## Classification Path (Metadata Columns)

Augment the 100-mechanism DOF table with these classification columns:

| Column | Values | Purpose |
|--------|--------|---------|
| `function_class` | serial, parallel, hybrid | Mechanism function topology |
| `pair_type` | lower (R/P/H/C/S/F), higher (gear/cam/point) | Kinematic pair classification |
| `chain_structure` | open, closed, mixed | Graph closure |
| `classification_path` | e.g., `serial → R → open → tree` | Full descent string |
| `type_synthesis_method` | Freudenstein, GA, enumeration, GNN (research) | Recommended synthesis approach |
| `number_synthesis_method` | enumeration, optimization | Parameter count search |
| `is_overconstrained` | bool | Flag for screw-theoretic rank check |
| `gpu_batchable_safe` | bool | Tier-routing input (C10) |
| `patent_references` | list of patent numbers | Prior-art for design derivation |

Rule: populate these columns on every new mechanism added to the 100-entry table. Cross-ref `simready-criteria §C10`.

<!-- source: bundle6/recover_missing_taxonomy_and_synthesis_research.csv + bundle4/section_file(1)/4_patent_landscape_report, confidence: HIGH -->

## Biomechanical Joint Equivalents

For humanoid / medical / prosthetic assets that must match human anatomy:

| Joint | DOF | Approximation | Real Complexity |
|-------|-----|---------------|-----------------|
| Hip | 3 rot (ball-socket) | `PhysicsSphericalJoint` | Capsule limits approximate but miss femoral-head-specific ROM |
| Shoulder | 3 rot (ball-socket) | `PhysicsSphericalJoint` | Scapula motion adds effective 2 DOF — often omitted |
| Elbow | 1 rot (hinge) | `PhysicsRevoluteJoint` | Coupled rotation (pronation/supination) requires second revolute |
| Wrist | 2 rot (saddle) | D6 with 2 axes locked | Ulnar/radial deviation + flexion/extension independent |
| Knee | nominally 1 rot | `PhysicsRevoluteJoint` | **Actually 6 DOF** — translation + rotation coupling during flexion; full model requires 6-DOF joint with constraints |
| Ankle | 1 rot | `PhysicsRevoluteJoint` | Subtalar joint adds second rotation axis |
| Thumb | 2 rot (saddle) | D6 | Flexion/extension + abduction/adduction |
| Finger (per joint) | 1 rot | `PhysicsRevoluteJoint` | Coupled between DIP/PIP in natural grasp |
| Spine (per vertebra) | 3 rot (ball-socket) | `PhysicsSphericalJoint` × N | Simplified — real spine has bend/twist/translation |

**Production reference models:**
- **OpenSim full-body**: 22 bodies / 29 DOF / 80 muscles (Rajagopal 2016). Simbody backend. Gait analysis, rehabilitation.
- **MyoSuite hand**: 28 DOF / 39 muscles. MuJoCo backend. RL-ready for dexterous manipulation.
- **Parametric Human Project** (Autodesk): population-varying anatomical atlas for ergonomic studies.

<!-- source: bundle4/section_file(4)/0_biomechanics_and_anatomy + bundle3/surgical_simulation_memo, confidence: HIGH -->

## Bio-Inspired Actuators (Advisory)

Non-traditional actuator classes that may appear in medical/prosthetic assets. Currently out-of-scope for V13 first-pass physics (use rigid-body proxy), but flag for future compliant-mechanism extension:

| Actuator | Typical Use | Sim Strategy |
|----------|-------------|--------------|
| Hydrogel | Slow, soft, biocompatible — medical implants | Rigid proxy; future: deformable |
| Shape-memory alloy (SMA) | High force/weight; hysteresis — aerospace/robotics | Rigid proxy + custom drive curve |
| Electroactive polymer (EAP) | Fast, lightweight — haptics, medical | Compliant drive (PhysX 5.1+) |
| Twisted-coiled polymer (TCP) | Cheap, high force/strain, thermal-cycling — prosthetics, exoskeletons | Tendon approximation (MuJoCo) |

<!-- source: bundle4/section_file(4)/0_biomechanics_and_anatomy, confidence: MED (advisory — not production now) -->

## Reference
Full mechanism descriptions (100 objects with assembly sequences, component lists, and behavioral descriptions):
- `scripts/tools/simready_assets/reference_library/industrial_assets_part1.md` (1-25: robotic arms, CNC, gearboxes, cylinders, valves)
- `scripts/tools/simready_assets/reference_library/industrial_assets_part2.md` (26-50: pumps, motors, clutches, brakes, transmissions, linkages)
- `scripts/tools/simready_assets/reference_library/industrial_assets_part3.md` (51-75: gears, belts, actuators, hinges, locks, springs)
- `scripts/tools/simready_assets/reference_library/industrial_assets_part4.md` (76-100: suspension, tools, switches, pens, scissors, staplers)

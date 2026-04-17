---
name: simready-mechanism-lookup
description: >-
  Lookup DOF count, joint types, assembly hierarchy, and behavioral description
  for 100 industrial and mechanical mechanisms. Covers robotic arms, hydraulic
  systems, gearboxes, valves, pumps, motors, clutches, brakes, transmissions,
  linkages, casters, hinges, locks, tools, and more. Use when encountering an
  unknown mechanism or industrial object that needs articulation parameters.
  Derived from industrial_assets_part1-4 in the reference library.
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

## Reference
Full mechanism descriptions (100 objects with assembly sequences, component lists, and behavioral descriptions):
- `scripts/tools/simready_assets/reference_library/industrial_assets_part1.md` (1-25: robotic arms, CNC, gearboxes, cylinders, valves)
- `scripts/tools/simready_assets/reference_library/industrial_assets_part2.md` (26-50: pumps, motors, clutches, brakes, transmissions, linkages)
- `scripts/tools/simready_assets/reference_library/industrial_assets_part3.md` (51-75: gears, belts, actuators, hinges, locks, springs)
- `scripts/tools/simready_assets/reference_library/industrial_assets_part4.md` (76-100: suspension, tools, switches, pens, scissors, staplers)

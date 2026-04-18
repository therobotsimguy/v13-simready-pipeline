---
name: simready-collision
description: >-
  SimReady collision & friction — bodies, wheels, grip. Full rules, trolley wheel
  tables, and anti-patterns live in V8 PRINCIPLES_FRIDGE_TROLLEY.md on GitHub;
  use this skill as a pointer and quick reminder.
---

# SimReady Collision Skill

## Canonical document (read this first)

**All** detailed rules — **body hybrid hull vs decomposition**, **friction / GripMaterial / contactOffset**, **wheels & casters** (bracket vs tire, axle axis, anchor vs pivot, structural split keywords), **full trolley investigation tables**, **door/drawer/handle notes**, and **anti-patterns** — are in the V8 repo:

**[PRINCIPLES_FRIDGE_TROLLEY.md](https://github.com/therobotsimguy/v8-usd-to-simready/blob/main/PRINCIPLES_FRIDGE_TROLLEY.md)**

Local clone (if present): `/home/msi/v8-usd-to-simready/PRINCIPLES_FRIDGE_TROLLEY.md`

Do **not** duplicate long tables here; update the V8 file when behavior or learnings change, then sync `make_simready.py` to V8.

## Three-rule reminder (summary only)

1. **Body:** Hybrid collision — `convexDecomposition` only where concavity / cloak risk demands it; `convexHull` for small parts; **hard cap** on decomp count per asset (see principles + `MAX_DECOMP_BUDGET` in code). **Wheels:** decomp on all tire/disc/detail meshes under the wheel RB after structural split.

2. **Grip:** **`material:binding:physics`** on every collider; **GripMaterial** on handles; **`contactOffset`** only at runtime in teleop — not in asset USD.

3. **Wheel joint anchors:** for every `continuous` joint (caster/wheel), the anchor (`localPos1` on the rotating part) must be at the **wheel's bbox center** — the tire/axle. The wheel Xform origin is often **not** at the axle; using it causes wheels to detach at physics init. V13's `make_simready.py` computes tire center unconditionally for every continuous joint. Structural child meshes that must be reparented out of the rotating wheel Xform (so the bracket stays with the body) use keywords: `fixer, bolt, body, mount, stopper, frame, caps, bracket, fork, brake, base, trim`. Extend that list rather than accepting a bracket-rotating-with-tire failure. `base` and `trim` were added 2026-04-18 after Mobilecartsandtables_C01_01 shipped with the wheel bracket spinning — source USD named caster parts `wheel1_base_01` / `wheel1_trim_01` / `wheel1_brake_01` / `wheel1_tire_01`, and neither `base` nor `trim` matched the original set. The scope is safe: `split_wheel_structural_parts` only inspects DIRECT children of continuous-joint wheels, never the chassis or body, so a `base_link` elsewhere cannot be mis-matched. **Structural children may be direct `Mesh` prims OR `Xform` prims that wrap a mesh** — `split_wheel_structural_parts` must check both types, not just `Mesh`. EmergencyTrolley wraps each structural part in an Xform (`wheel2_frame_01/wheel2_frame_01`), so a `Mesh`-only filter silently skips them and the entire wheel-plus-bracket rotates together.

## simready-criteria

Use **simready-criteria** skill (C2, C3) as the **audit scorecard**; use **PRINCIPLES_FRIDGE_TROLLEY.md** as the **design reference** for why those choices exist.

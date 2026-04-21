# V13 SimReady Pipeline

Convert a raw USD asset into a SimReady asset — full physics, joint limits,
friction, collision, articulation — ready for Isaac Sim teleop, training, or
simulation.

---

## Entry point

```bash
python3 scripts/tools/simready_assets/simready_agent.py --input /path/to/asset.usd
```

Add `--dynamic` when the main body should be pushable by the robot
(trolleys, carts, movable furniture). Omit it for kinematic-body assets
(refrigerators, cabinets, fixtures).

```bash
# Cart you can push
python3 scripts/tools/simready_assets/simready_agent.py --input trolley.usd --dynamic

# Fridge you open doors on but doesn't move
python3 scripts/tools/simready_assets/simready_agent.py --input fridge.usd
```

### `--dynamic` decision tree

**Use `--dynamic`** if the whole asset should translate under a robot push:
- trolleys, carts, mobile utility carts
- beds on casters (ResuscitationBed, EmergencyTrolley)
- wheelchairs, mobile IV stands
- surgical tables if meant to be repositioned
- any asset with **wheels/casters** that should roll

**Omit `--dynamic`** for fixtures — the body is anchored to world via a
`PhysicsFixedJoint` (F49 encoding; Newton-compatible), but **articulated
children still move freely** on their joints:
- fridge doors open (revolute)
- drug-cabinet drawers slide (prismatic)
- IV-pole tubes telescope (prismatic)
- surgical-arm joints rotate (revolute)

Common misconception: drawers/sliders need `--dynamic`. They don't —
joint articulation is independent of whether the root is anchored.
`--dynamic` only controls the **whole-body** mobility.

Symptoms if you pick wrong:
- Missing `--dynamic` on a trolley → wheels spin in place but body won't shift-drag.
- Wrong `--dynamic` on a fixture → robot brushes the fridge, fridge walks across the room; 6-DOF free body wastes solver time in RL batches.

That is the **only** command you need. It orchestrates:

1. `read_hierarchy` — parse USD structure
2. `geometric_fingerprint.py` — emit per-part bbox / aspect / thin_axis /
   long_axis / pivot ground-truth from the USD, injected into the classifier
   prompt so wheel-axle and slider-direction decisions come from exact
   geometry, not pixel inference
3. `gemini_vision.py` — render views, analyze geometry
4. `object_understanding.py` — infer mass + material density
5. Claude classifier — decide which parts move and how
6. `make_simready.py` — apply physics APIs, collision, joints, drives
7. `verify_visual.py` — render before/after for visual sanity
8. Auto-push debug data + classify JSONs to this repo

End-to-end: ~3–5 min per asset (most of it is LLM calls).

`--dynamic` is auto-inferred when the classifier produces any
`movable:continuous` part (wheels/casters imply a pushable body). Manual
`--dynamic` still works and takes precedence.

---

## Do NOT call `make_simready.py` directly

`make_simready.py` is a **sub-step**, not an entry point. It's invoked by
`simready_agent.py` as a subprocess. Calling it directly skips vision,
object understanding, classification, and visual verify — you'll get an
untested asset that may fail in ways the agent would have caught.

The one exception is rebuilding a previously-classified asset for a
regression test — then you can pass `--classify-json` + `--object-json`
with previously-generated JSONs from `classify/`.

---

## Output

```
~/SimReady_Output/simready/<asset_name>/
├── <asset_name>_physics.usd    # the SimReady asset
├── <asset_name>_physics.json   # physics sidecar (mass, bounds, joints)
└── Textures/                   # relative-path-resolved textures
```

Assets live outside this repo by design. The repo only tracks code, skills,
classification JSONs, and debug history.

---

## Teleop a built asset

```bash
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent_cinematic.py \
  --asset ~/SimReady_Output/simready/<asset_name>/<asset_name>_physics.usd \
  --device cpu
```

Shift+drag to push. For assets with `--dynamic` body, this rolls the asset
around. For kinematic-body assets, only movable parts (doors, drawers)
respond.

---

## Pipeline knowledge: `skills/`

**15 skills** describe everything the classifier needs to know about USD
physics, PhysX gotchas, joint parameters, failure modes, robot constraints,
collision strategy, deformables, and Newton-engine compatibility. They load
automatically into the classifier's system prompt at runtime — no manual
setup.

Skill inventory:

| Skill | Scope |
|---|---|
| `articulation-pipelines` | 3-stage pipeline + graph-IR compiler architecture, CoACD, synthesis context |
| `usd-physx-schemas` | PhysX compat matrix, OmniPhysics deformable APIs, one-way lowering, GPU PCM |
| `failure-modes` | **98 modes across 4 axes** — F (classical), D (deformables), K (kinematic/synthesis), S (solver-engine) |
| `simready-criteria` | **C1–C11 audit** — basic physics + cross-solver validation + tier certification + Newton parity |
| `simready-joint-params` | 20 object categories + Rabinowicz/Bhushan friction, Stribeck, BIFMA/IEC/ASME standards |
| `simready-mechanism-lookup` | 100 mechanisms + Reuleaux pairs, biomechanical joints, OpenSim/MyoSuite |
| `simready-behaviors` | 18 behaviors (incl. compliant_hinge/slider via Howell PRBM) × 16 constraints |
| `collision-physics` | Rigid + deformable collision, gripper-deformable gap, production-tier labels |
| `blender-3d-generation` | 8 patterns (incl. PRBM-aware, collision-aware), ArtLLM/ArticFlow/AKD |
| `robot-model` | Franka Panda specs + OR deployment constraints (sterility, 150N cap, haptic absence) |
| `sim-ready-datasets` | Dataset catalog + musculoskeletal + cloth calibration (CLO 3D, Browzwear) |
| `simready-math` | Deterministic math + Blow's tetrahedral inertia + parallel-axis theorem |
| `simready-collision` | Trolley wheel / grip rules pointer |
| **`deformable-physics-robotics`** (new) | Cloth, ropes, cables, soft-body — FEM/XPBD/VBD/MPM, fTetWild, D01-D18 modes |
| **`newton-physx-compat-matrix`** (new) | Newton solver backends (VBD/Featherstone/MuJoCo Warp), MPM coupling, dual-output parity |

When pipeline behavior needs to change, **edit the skills first, then the
code**. The classifier reads skills; future Claude sessions read skills;
audit checks cite skills by name in their error messages. They are the
durable record of pipeline decisions.

---

## Audit criteria (`C1`–`C11`)

Every asset must pass the core criteria before it's considered SimReady.
`simready_agent.py` runs the audit automatically; `make_simready.py --fix`
runs it before and after applying physics.

**Core (required, blocks release):**
- **C1** Rigid Bodies, **C2** Collision Shapes, **C3** Friction Materials,
  **C4** Flat Hierarchy, **C5** Joints, **C6** Drives, **C7** Clean Asset

**Extended (warning-level in V13.0, promoted to blockers as tooling matures):**
- **C8** Cross-solver validation (state-trajectory MSE < 1e-4 across PhysX + MuJoCo + Newton over 10s gravity drop)
- **C9** Scale viability (stable at IsaacLab batch sizes — 32+ envs)
- **C10** Tier certification (GPU-batchable vs CPU/offline high-fidelity — drives Isaac Lab vs Newton routing)
- **C11** Newton / PhysX dual-output parity (gravity drop, teleop push, joint sweep, contact force comparison)

7/7 on core means the asset is structurally valid. C8–C11 quantify fidelity
and dual-output readiness. Passing the audit does **not** guarantee it feels
right in teleop — always test physically after building.

---

## Local checkout

The owner's local clone lives at:

    ~/IsaacLab/scripts/tools/simready_v13/

This is a **nested git repo** — it sits inside `~/IsaacLab/` (a separately
git-tracked Isaac Lab checkout) but has its own `.git/` and is not a
submodule. All V13 commits are made from inside this folder and push to
`origin`. Anyone else can clone V13 to any location (e.g.
`~/v13-simready-pipeline/`) and it will work standalone, because the
pipeline now ships its own `skills/`.

---

## Requirements

- Python 3.10+
- NVIDIA Isaac Sim (for USD schemas + teleop) or standalone USD Core
- `claude-agent-sdk` Python package
- `~/.claude/api_keys.json` with `anthropic.api_key` and `github.pat`
- `~/IsaacLab/` checkout for teleop script (asset building works without it)

---

## Current asset set — medical operating-room library

V13 is being validated against a 20-asset library of operating-room /
surgery USDs. Raw inputs are **not** in this repo (they're local, large,
and under a separate license); they live at:

    ~/SimReady_Output/raw_challenge_assets/*.usd

Built SimReady outputs land at `~/SimReady_Output/simready/<asset_name>/`
as usual.

**Reference baseline** (shipped in this repo for regression / smoke-testing):

| Asset | Location | Role |
|---|---|---|
| `InstrumentTrolley_B01_01` | `examples/trolleyB/` | Known-good caster-trolley reference — clean identity-chassis input, used to verify pipeline doesn't regress when fixing rotated-chassis assets. |

**Progress on the 20-asset library:**

| # | Asset | Type | Status |
|---|---|---|---|
|  1 | ArticulatedsupportArm_A01_01 | arm / mount | **built** (drove the serial-kinematic-chain + adjacent-link-self-collision fixes on 2026-04-17) |
|  2 | BipolardissectingScissors_A01_01 | surgical tool | **built** (2026-04-18 — body + screw merged + two revolute sibling blades; AUDIT 7/7, MUJOCO 12/12, teleop PASS, 107s) |
|  3 | Clamps_A01_01 | surgical tool | **built** (2026-04-18 — drove F14b/F14c fixes: C5 world-space anchor resolution for symmetric-pivot instruments, classifier rule `body = default prim`, C2 allows mesh-less body. AUDIT 7/7, MUJOCO 11/12) |
|  4 | DrugCabinet_A03_01 | storage | **built** |
|  5 | EmergencyTrolley_A01_01 | cart | **built** (drove the rotated-chassis fixes on 2026-04-17) |
|  6 | Forceps_A01_01 | surgical tool | **built** (2026-04-18 — single-mesh USD, classified as non-articulated graspable prop; teleop pickup PASS) |
|  7 | HoldingDevice_A01_01 | mount | **built** (2026-04-18 — drove F40/F41 fixes: Gemini `range_meters` override for prismatic travel, auto-dynamic rule for articulated handheld tools, orchestrator prompt preserves full object JSON. AUDIT 7/7, MUJOCO 15/16; valvebutton 5mm travel + 2 revolute arms, body kinematic stand-mount) |
|  8 | MedicalutilityCart_A03_01 | cart | **partial** (2026-04-18 — drove F42 base/trim keyword, F43 bake_xform_scales, F44 skip-concave-organizer, F45 articulation self-collisions, F46 handle-based direction, F46b signed-axis override, and the teleop ArticulationCfg fix for dynamic roots. Wheels + ground + physics all PASS. Drawers still open in wrong face — Gemini keeps classifying axis=Y; requires deeper classifier work to reliably pick the correct face on this asset) |
|  9 | Mobilecartsandtables_C01_01 | cart / table | **built** (2026-04-18 — drove F42: `base`/`trim` added to `WHEEL_STRUCTURAL_KEYWORDS`, fixing bracket-rotates-with-tire. AUDIT 7/7, MUJOCO 27/28; 4 casters + height-adjust table + handle. Teleop PASS) |
| 10 | ResuscitationBed_A01_01 | bed | **built** (2026-04-18 — drove F47 zero-thickness collider skip + F48 wheel/caster class alias. 4 wheels had been silently dropped as class="wheel" (not `movable:continuous`); 3 flat decals crashed PhysX broadphase via qhull NaN. AUDIT 7/7, MUJOCO 38/44, teleop rolls on casters) |
| 11 | Retractor_A01_01 | surgical tool | **built** (2026-04-19 — single-mesh USD, classified as non-articulated graspable prop; AUDIT 7/7, MUJOCO 4/4, 95.9s, teleop PASS) |
| 12 | RoboticSystem_A01_01 | system | pending |
| 13 | RoboticSystem_B01_Console_01 | system | pending |
| 14 | Scissors_A01_01 | surgical tool | **built** |
| 15 | SelfretainingRetractor_A01_01 | surgical tool | **built** (motion PASS — shift-drag arms at `--asset_scale 5.0`; prongs visually clip at close position — geometry limitation, not a pipeline bug; see `LEARNINGS.md` → scissor self-collision) |
| 16 | SurgicalChair_A01_01 | chair | **still broken — F68 + F69 landed, asset STILL falls apart in teleop (2026-04-21)**. Fix wave continues from 2026-04-20 (F64d/F64e/F65/F66/F67 + F45 v2). **2026-04-21 session**: chassis-rebuild of SurgicalChair AUDIT 7/7, InstrumentTrolley_B regression clean, but teleop shift-drag STILL detaches casters AND other parts fly off. Root-cause chase landed two more F-numbers, neither fully solved it: **F68** — raw USD authors each splayed caster's mount mesh with a Z rotation facing its leg (wheel3+75°, wheel5−70°); `reparent_prims_preserve_world_xform` collapses that into a `TypeTransform` matrix on the bracket child (first cut of F68 missed this — only checked Euler/Orient ops). Fix: `normalize_caster_bracket_mount_rotation` snapshot→reset→reauthor now covers TypeTransform too. Audit C5 FAILs residual >5° rotation or non-identity 3×3 in TypeTransform. **F69** — revolute Z bracket swivel joints inherited the door-hinge default `damping=2.0`. For a 2-DOF caster chain (chassis ~7kg → bracket 0.34kg → tire 0.34kg, bracket:tire ≈ 1:1), any chassis impulse resonates down the chain — tire inertia whips the bracket faster than 2.0 can absorb, solver budget blows, constraint visually separates. Real caster bearings damp at 5–20 N·m·s/rad; V13 was modeling them as near-frictionless. Fix: `make_revolute_joint` caster-swivel branch (`hinge_edge="continuous"`) authors `damping=15.0`. Audit C6 FAILs any `_bracket_joint` with `drive:angular:physics:damping < 5.0`. **Reality check**: joint-anchor mismatches are 0.00mm on both chair and trolley (authoring correct on both); the difference was always dynamics, not topology. After F68 + F69 direct-patch on the post-rebuild USD, AUDIT 7/7 + all criteria PASS, but the user reports the chair still falls apart in shift-drag teleop. Remaining suspects: **F62** `enabledSelfCollisions=False` on this articulation (set off by F45 v2 because PhysX produced NaN at bracket↔wheel hull level on 2-DOF casters) — may be letting bracket and tire interpenetrate under impulse; **F61** no `physxScene:maxDepenetrationVelocity` cap ships with the asset — when frame-1 contact produces any penetration, PhysX launches bodies at unbounded velocity; **chassis mass 7.2kg vs trolley 16kg** — the chair is literally half the trolley chassis, compounding every impulse propagation problem. **Patched reference USDs**: `SurgicalChair_A01_01_physics_f68.usd` (F68 only), `SurgicalChair_A01_01_physics_f68_f69.usd` (F68 + F69). Pipeline fixes landed + enforced by audit; teleop verdict still FAIL. |
| 17 | SurgicalChair_B01_01 | chair | pending |
| 18 | SurgicalMicroScope_A01_01 | system | pending |
| 19 | SurgicalpowerTool_B01_01 | surgical tool | **built** (2026-04-19 — drove F63 orphan-structural-siblings fix: raw USD authored tool body as 10 sibling Xforms (main_01 + cylinderpart1-4 + handlebase + handle + drillbit + decals), classifier marked 1 as body root, other 9 had no RigidBodyAPI → dragged main detached from rest. Added `weld_structural_siblings_into_body` to make_simready.py, F63 audit check, skill docs. Post-fix body: 1 → 11 colliders; AUDIT 7/7; 0 orphans; teleop PASS with all parts moving together. Regression-tested clean on InstrumentTrolley_B / Refrigerator_A / EmergencyTrolley / DrugCabinet / BipolardissectingScissors) |
| 20 | SurgicalTable_A01_01 | table | **built** (2026-04-19 — drove F64/F64b/F64c caster fixes: 4 no-tire casters (cap+core only, bbox aspect 1.06) skidded on cube colliders, chassis parked on `wheel*_base` plates + `base1` foot at floor level. Added `synthesize_wheel_cylinder_collider` (primitive Cylinder, axis Y, r=4cm h=7.6cm, rubber-bound), `_strip_chassis_wheel_blockers` (hides + strips wheel-prefix blockers), `strip_chassis_floor_blockers` (hides + strips non-wheel floor blockers). AUDIT 7/7, 14 links / 13 joints (4 casters + table tilt + headrest + frame1/2 + joint1-4 leg-rest chain + lader4 prismatic lift). First live verification of classifier rationale tracking: 52 rule citations, 14 parts each with `rationale` field (F06/F07/F09/F11/F20/BEH/MEC). Teleop PASS.) |

Score: **15 / 20 built + 1 partial + 1 still broken** (MedicalutilityCart:
physics correct, drawers mis-faced; **SurgicalChair: AUDIT 7/7, runtime
teleop still fails** — F68 mount-rotation bake + F69 bracket-swivel
damping=15 both landed and enforced, but shift-drag still detaches
casters and flies parts off as of 2026-04-21). Remaining 7 assets can be
run with the single entry-point command; no per-asset tuning is required
unless V13 surfaces a new silent-failure class, in which case follow the
3-step fix rule below.

**New feature (2026-04-19):** **2-DOF swivel casters.** When a wheel Xform
contains both a tire mesh AND a bracket-keyword mesh (mount / bracket /
housing / fork / yoke / swivel), `split_wheel_structural_parts` now builds
a 2-body kinematic chain per caster — a bracket body that swivels on a
revolute Z joint to the chassis, plus a tire body that rolls on a continuous
joint to the bracket. Real office-chair / surgical-chair caster behavior.
Fixed-axle wheels (InstrumentTrolley, EmergencyTrolley, ResuscitationBed)
fall through unchanged — their wheels have no bracket-keyword meshes.

**Recent fix wave (2026-04-18):** F40 Gemini prismatic-range override,
F41 handheld-tool auto-dynamic, F42 wheel-split keyword growth (base/trim),
F43 bake residual xformOp:scale (fixes "floats in air" class), F44 skip
concave organizer hulls, F45 enable articulation self-collisions, F46
handle-based prismatic direction, F46b signed-axis classify override,
teleop spawn-path branch (ArticulationCfg for dynamic roots), F47
zero-thickness collider skip (qhull NaN → broadphase crash), F48
wheel/caster class-alias normalization (silently-dropped rolling joints),
F49 world-anchor FixedJoint for fixtures (Newton-compatible; replaces
`kinematicEnabled=True` on fixtures — DrugCabinet + Fridge verified).

**2026-04-21 — SurgicalChair struggle, still unresolved:** F68 bake rotated
mount meshes inside caster brackets (handles both Euler ops and collapsed
`TypeTransform` matrices via F43's snapshot-reset-reauthor pattern),
F69 raise caster-swivel damping 2.0 → 15.0 (door default was starving a
2-DOF chassis→bracket→tire chain of the friction it needs to absorb
resonant whip). Both fixes ENFORCED in audit (C5 for F68, C6 for F69),
both verified by direct-patch on the post-rebuild USD, and the asset
still falls apart in teleop shift-drag. See the **"Next session work"**
section below for what's been ruled out and what the remaining suspects
are — we are not starting from zero on the next investigation.

**Skill-library integration (2026-04-19):** 6 research bundles (2.8 MB,
152 files) absorbed via 3-phase review (inventory → plan → execute).
Results:
- **2 new skills:** `deformable-physics-robotics`, `newton-physx-compat-matrix`
- **Failure modes: 39 → 98** across 4 segregated axes (F/D/K/S)
- **Audit criteria: C1–C7 → C1–C11**
- **Behaviors: 16 → 18** (added compliant_hinge + compliant_slider via Howell PRBM)
- **+1,453 lines** of new skill content, all with provenance tags

**Next session work (continuation of 2026-04-21 SurgicalChair struggle):**

The SurgicalChair_A01_01 teleop problem has now outlasted seven F-number
fixes (F64d, F64e, F65, F66, F67, F68, F69). Audit says 7/7; reality says
parts still fly off. This section is the honest record of what's been
ruled out and what's next, so we don't re-run the same experiments.

**Ruled out so far (2026-04-20 → 2026-04-21):**
- Chassis-collider authoring (F64d, preserve ≥1 collider)
- Bracket misclassification as cube-wheel (F64e)
- Swivel seat mis-dispatched as wheel (F65)
- Non-adjacent bbox overlap → filterPairs (F66) + conditional F45 v2
- Redundant-wrapper Xform + matrix xform (F67)
- Rotated mount mesh in bracket via Euler ops (F68 initial)
- Rotated mount mesh via `TypeTransform` matrix (F68 extended, 2026-04-21)
- Bracket swivel damping too low (F69, 2.0 → 15.0, 2026-04-21)
- Joint anchor authoring — verified 0.00mm mismatch on both chair and
  trolley, so not a fly-apart-at-init problem

**Still suspected (to investigate):**
1. **F45 v2 disabled self-collision is hiding an intersecting-geometry bug,
   not fixing it.** The trolley has `enabledSelfCollisions=True` and works;
   the chair has it `False` specifically because F45 on-mode produced NaN
   contact normals on bracket↔wheel hull overlaps. But turning self-
   collision off just defers the problem: whatever hull pair was generating
   NaN is still geometrically overlapping — it's just not being contact-
   resolved. Task: identify the hull pair (likely bracket mount mesh
   intersecting tire rim), fix the geometry with `physxCollision:
   convexDecomposition` or mesh-volume split rather than disabling
   self-collision globally.
2. **No `physxScene:maxDepenetrationVelocity` cap ships with the asset**
   (Tier 1 warning, F61). Frame-1 PhysX penetration → unbounded launch
   velocity → visually the "parts fly off" symptom. Task: author a
   `physxSceneAPI` on the default_prim with `maxDepenetrationVelocity=5.0`
   as a per-asset safety cap (even though scene-level attrs usually live
   host-side; some engines honor asset-level overrides).
3. **Chassis mass 7.2 kg vs trolley 16 kg.** The chair's total mass is half
   the trolley's. Every impulse impact is proportionally larger. Task:
   check whether `estimate_mass` is under-weighting the pedestal/leg
   assembly — a real surgical stool with steel base is 10–20 kg.
4. **F68 + F69 only verified via direct USD patching, not end-to-end
   rebuild.** A fresh `simready_agent.py` run of SurgicalChair was
   triggered 2026-04-21 and produced AUDIT 7/7 in 480s — but the log did
   NOT show F68's "normalized N mount(s)" output even though brackets
   were created. Need to confirm F68 actually fires during a real rebuild
   (subprocess stdout may be filtered) vs only firing on direct-apply. If
   it's being silently skipped in the rebuild path, the rebuilt USD still
   carries the rotated mounts.

**Pre-F68/F69 reference USDs preserved:**
- `~/SimReady_Output/simready/SurgicalChair_A01_01/SurgicalChair_A01_01_physics.usd` — rebuilt via simready_agent.py 2026-04-21, AUDIT 7/7, teleop FAIL
- `SurgicalChair_A01_01_physics_f68.usd` — F68 mount-rotation direct-patch on top, C5 clean
- `SurgicalChair_A01_01_physics_f68_f69.usd` — F68 + F69 damping direct-patch, all criteria PASS, teleop FAIL per user 2026-04-21

**Previously planned work, still pending:**
5. **Add C4 audit check for the redundant-Xform pattern** — FAIL if
   ArticulationRootAPI sits on a prim whose direct child is a same-named
   Xform with no RigidBody but a grandchild with one.
6. **Unblocked pending assets:** `RoboticSystem_A01_01`,
   `RoboticSystem_B01_Console_01`, `SurgicalChair_B01_01`,
   `SurgicalMicroScope_A01_01`. The drift detector + expanded audit
   citations (F01–F69) should catch most asset-specific issues during
   classification + apply + audit.
7. **Deformable-asset track** (bedsheets, IV tubing, surgical drapes,
   cables) still unblocked via `deformable-physics-robotics` — Newton VBD
   + MuJoCo Warp path. Needs asset pilot first.

**The honest lesson from this saga:** Audit passing is necessary but not
sufficient. The 2-DOF caster chain is a fragile topology that introduces
runtime failure modes audit cannot see (solver iteration budget, hull-
level NaN, unbounded depenetration velocity). Future work on 2-DOF
assets (`SurgicalChair_B01_01`, office chairs) should start from a known-
working reference and diff against it, rather than running new rebuilds
and trusting 7/7 audits.

---

## Dual output: PhysX (Isaac Sim) + Newton

V13 produces SimReady USD assets that target **both** NVIDIA Isaac Sim
(PhysX 5) and NVIDIA Newton as separate products. The skill library is
structured to support this:

- **Base USD Physics schemas** (`physics.usda`) are engine-agnostic
- **Engine-specific overrides** go in `physx.usda` (PhysX) or `newton.usda` (Newton)
- **C11 parity audit** verifies behavior equivalence across both engines

When to use which engine:
- **PhysX (Isaac Sim)** — default for rigid articulated assets. Mature tooling,
  validated teleop path, F49 world-anchor fixtures verified.
- **Newton** — required for deformables (cloth, cable, soft body via VBD),
  granular/MPM coupling, differentiable physics for param calibration.
  Primary path: VBD + MuJoCo Warp. Secondary: VBD + Featherstone.
- **Both (dual-output)** — required for assets that must behave identically
  in both environments. Validate via C11 test battery.

See `skills/newton-physx-compat-matrix/SKILL.md` for the full compatibility
matrix and `skills/deformable-physics-robotics/SKILL.md` for the deformable
solver stack.

---

## Contributing fixes

When you find and fix a pipeline bug, apply the **3-step fix rule**:

1. **CODE** — fix in `scripts/tools/simready_assets/make_simready.py` (or
   relevant file).
2. **SKILL** — document the rule in `skills/<skill>/SKILL.md` under the
   right domain (e.g. `usd-physx-schemas` for schema-level gotchas,
   `simready-collision` for wheel/grip rules).
3. **AUDIT** — extend `audit()` in `make_simready.py` to FAIL when the
   condition reappears, with a message that names the fix location.

Validated by rebuilding a known-good asset (e.g. `InstrumentTrolley_B01_01`)
and confirming the audit catches the regression when the fix is reverted.

### Drift detector: `fixes.json` + `lint_fixes_manifest.py`

The 3-step rule is honor-system unless something checks it. `fixes.json` is
the structured source of truth for every F / D / K / S failure-mode ID —
one row per ID with pointers to its skill location, its fix function (if
named), and whether `audit()` cites it. `lint_fixes_manifest.py` validates
that every row still resolves to a real location and classifies each
entry by propagation state.

```bash
make lint    # validate fixes.json → reports CODE_NO_AUDIT / CODE_NO_SKILL drift
make scan    # regenerate fixes.json from skill file + code scan
```

Status categories the linter reports:

| Status          | Meaning                                                      |
|-----------------|--------------------------------------------------------------|
| `ENFORCED`      | skill + named fix function + audit citation all present     |
| `AUDIT_INLINE`  | skill + audit citation, fix is inline (no named function)   |
| `CODE_NO_AUDIT` | skill + code, **no audit citation** — silent regression risk |
| `CODE_NO_SKILL` | cited in code but missing from failure-modes skill — F-number hole |
| `SKILL_ONLY`    | documented, zero enforcement — backlog                       |

Linter exits 1 on broken refs (manifest points at a missing file /
renamed function / removed audit citation). Run before every commit
that touches skills, `make_simready.py`, or adds a new F-number.

### Propagator: `propagate_learning.py`

Automates the 3-step fix rule. Given a diagnosed failure (observation +
root cause + proposed fix), it drafts the SKILL row, the CODE function
and call site, and the AUDIT check as one markdown document for human
review. Allocates the next available F/D/K/S number from the manifest
so numbering stays consistent.

```bash
python3 propagate_learning.py \
  --observation "wheels don't rotate when chassis is dragged" \
  --diagnosis "no tire mesh, cube-aspect collider" \
  --proposed-fix "synthesize primitive Cylinder sized to wheel bbox" \
  --dry-run            # show prompt, no LLM call

# Live run (default model: claude-sonnet-4-5, ≤$0.05 per draft)
python3 propagate_learning.py --observation ... --diagnosis ... --proposed-fix ...
```

The propagator **never writes to skill/code/audit files directly** and
**never commits**. It emits a draft to stdout (and `.research_delta/
propagations/`) for human review. Workflow:

1. Review the draft
2. Copy edits into the three files
3. `make scan && make lint` → confirm new ID status = `ENFORCED`
4. Rebuild `InstrumentTrolley_B01_01` → confirm no regression
5. Rebuild the failing asset → confirm the fix resolves the observation

The gold-standard examples (F63 weld orphans, F64 cube-wheel cylinders)
are included in the prompt so the LLM mirrors their precise shape —
same level of detail in the skill row, same naming discipline for the
fix function, same F-number-prefixed FAIL message in the audit check.

### Runtime rationale drift: `check_rationale_drift.py`

The classifier emits a `rationale` list per part (e.g.
`["F49:world-anchor-applied", "F09:thin-axis-from-fingerprint"]`).
Audit checks separately. This tool cross-references per build:

```bash
python3 check_rationale_drift.py \
  --classify ~/SimReady_Output/simready/classify/<asset>_classify.json \
  --usd ~/SimReady_Output/simready/<asset>/<asset>_physics.usd
```

Reports three drift classes:
- **CONTRADICTION** — classifier claimed `F##` AND audit FAILs with `F##` cited (rule supposed to fire, output violates it — highest-priority drift)
- **BLIND SPOT** — audit FAIL with no matching classifier claim (classifier didn't anticipate; informational)
- **UNVERIFIABLE** — classifier claim has no corresponding audit check (use `make lint` to see if it's `SKILL_ONLY` vs inline)

Exit 1 on contradictions.

### Audit regression harness: `test_audit_fixes.py`

Every fix function claims an audit check catches regressions. But audits
can be insufficient (F35 scissors — audit existed but checked the wrong
invariant). This test suite constructs minimal failing USD stages
in-memory and asserts audit FAILs with the expected F-number:

```bash
make test-audit     # or: python3 test_audit_fixes.py
```

Currently covers F01, F11, F33, F47, F63 plus a baseline-doesn't-false-
positive test. Extend by adding `def test_F##_...` functions following
the same pattern (build stage → mutate → audit → assert).

---

## Next session — Learning Propagator (architectural priority)

The 3-step fix rule (CODE + SKILL + AUDIT) is currently applied manually —
each new learning (F63 weld orphans, F64 cube-wheel cylinders, F64b/F64c
chassis blockers) requires editing make_simready.py, a skill file, and
the audit function by hand. This scales badly as rule count grows and
drift between the three artifacts becomes a risk.

**The plan: an LLM-driven `propagate_learning.py` subagent** that takes a
diagnosed learning as input and automatically lands it in the right
places across the pipeline:

```
propagate_learning.py \
  --observation "wheels skid on cube-shape casters when dragged" \
  --asset /path/to/failing/asset.usd \
  --diagnosis "wheel Xform has no tire sub-mesh, bbox aspect 1.06, convex-decomp produces non-rolling collider" \
  --proposed-fix "synthesize primitive cylinder sized to wheel bbox"
```

**What the propagator does:**

1. **Reads** the pipeline structure (phases of `apply_physics`, existing
   fix functions, skill file layouts, audit criterion blocks).
2. **Decides** which phase the fix belongs in — collision-apply for wheel
   geometry, pre-classify for topology, joint-apply for drive params, etc.
3. **Writes** the Python fix function in the right location with the
   right call site inside `apply_physics`.
4. **Updates** the matching skill file with an F-number entry
   (symptom / root cause / fix) in the correct tier.
5. **Adds** an audit check in the right criterion block, citing the fix
   function by name in the FAIL message.
6. **Runs** the rebuilt pipeline against a baseline asset
   (`InstrumentTrolley_B01_01`) to confirm no regression.
7. **Emits a diff** for human review — does NOT auto-commit.

**Why this is the next move:**

- Every learning propagates to the right place automatically — no manual
  distribution across three files, no drift possible by construction.
- The LLM is already the tool we use to write each fix; formalizing it
  as a pipeline step removes the human copy-paste between locations.
- Learnings stay in the pipeline code where they belong for runtime
  determinism (no registry, no DSL, no new abstraction layer — just
  functions in the right phases).
- Each new asset that surfaces a bug becomes training signal for
  self-improvement: observation → propagator drafts fix → human approves
  → pipeline is smarter for the next asset.

**Guardrails (non-negotiable):**

- Propagator NEVER auto-commits. Always emits a diff + regression report
  for human review.
- Must pass post-patch audit against the full goldens corpus before the
  diff is surfaced.
- If the proposed fix fails audit on any existing built asset, propagator
  re-drafts or flags "needs human design input."

**Estimate:** ~1 day to prototype the propagator prompt + validation
harness. Payoff: every future learning lands correctly without the human
editing three files and risking drift. The current 3-step fix rule
becomes "describe what you learned; the propagator distributes it."

**Why not a rule registry / DSL / idempotent-fixer abstraction instead?**
Those were considered and rejected in the same session this section was
written. The pipeline IS the registry — fixes already live in it. The
remaining problem is the mechanical work of editing three files in sync,
which is exactly the kind of work LLMs do well. No new abstraction
layer is needed; just automate the distribution.

---

## Roadmap — pending code work (2026-04-19 skill integration follow-up)

The 2026-04-19 skill-library expansion completed **step 2 (SKILL) only** of
the 3-step fix rule for ~44 new failure modes and 4 new audit criteria.
Steps 1 (CODE) and 3 (AUDIT) are pending. The 15 skills *describe* pipeline
expectations; `make_simready.py` does not yet *enforce* them.

Full follow-up list is in `.research_delta/INTEGRATION_REPORT_PHASE3.md §8`
(local-only). Summary:

### Tier 1 — Low-risk, code-only (next session, ~1 session of work)
- **Implement Blow's tetrahedral inertia** in `scripts/tools/simready_assets/math_skill/inertia.py` (skill declares it; code is missing)
- **Add warning-level checks** for low-risk failure modes that won't break current builds:
  - F50 full joint-sweep overlap test (extends existing rest-pose check)
  - F59 principal axes alignment
  - F61 depenetration velocity cap at load
  - F62 self-collision flag audit
  - K12 condition-number-of-inertia > 10⁶ warning
  - K16 axis-sanity check on classifier output
- **C10 tier certification** as warning (GPU-batchable vs CPU-only, heuristic)
- Regression-test against `InstrumentTrolley_B01_01` + `Refrigerator_A` before committing

### Tier 2 — Requires infrastructure (4–8 weeks)
- **`validate_simready.py` module** — 4-stage gauntlet (static → dynamic → cross-simulator → performance) for C8
- **C11 test battery** — concrete PhysX/Newton parity scripts for Refrigerator_A + InstrumentTrolley_B
- **MatWeb / tribology integration** — Rabinowicz/Bhushan friction defaults per `simready-joint-params §Rabinowicz Table`
- **fTetWild tetrahedralization** — prerequisite for deformable asset builds
- **PRBM detection path** — classifier hook for `flexure` / `spring-loaded` parts → compliant_hinge/compliant_slider behaviors

### Tier 3 — Deformable asset pilot (Q3 2026)
- **First deformable asset** — surgical drape on Refrigerator_A, end-to-end via `deformable-physics-robotics`
- **Newton VBD + MuJoCo Warp path** — production wiring for cloth + Franka coupling
- **Cross-simulator consistency harness** — automated PhysX + MuJoCo + Newton comparison

### Tier 4 — Research-stage, do not commit roadmap (18+ months)
- GNN topology synthesis (ArtLLM / ArticFlow / NDGM) — monitor, don't build
- AKD video-diffusion enrichment for existing assets — monitor, don't build
- World-models-to-asset (Cosmos / Genie 3 / V-JEPA) — 5-year opportunity

**Skip list** (research content explicitly rejected, do NOT revisit):
aerospace/civil/climate/molecular/game-engine memos, Havok/Bullet/ODE/Flex
engine rules, Warp ML-plumbing internals (tile_matmul, autograd). See
Phase 3 report §5 for the full skip inventory with reasons.

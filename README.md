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
| 16 | SurgicalChair_A01_01 | chair | **partial** (2026-04-19 — drove 2-DOF swivel-caster support, caster-bracket centroid-origin fix, `--dynamic` auto-inference from continuous joints, F49 authored-pose anchor, wheel-keyword gating on swivel seats, `_is_degenerate_mesh` eps raise 1e-6 → 3mm to catch thin trim, caster bracket mass override (wheel-class, not revolute-class). AUDIT 7/7; chair still spawns with PhysX broadphase error on some geometry combinations — investigation ongoing) |
| 17 | SurgicalChair_B01_01 | chair | pending |
| 18 | SurgicalMicroScope_A01_01 | system | pending |
| 19 | SurgicalpowerTool_B01_01 | surgical tool | **built** (2026-04-19 — drove F63 orphan-structural-siblings fix: raw USD authored tool body as 10 sibling Xforms (main_01 + cylinderpart1-4 + handlebase + handle + drillbit + decals), classifier marked 1 as body root, other 9 had no RigidBodyAPI → dragged main detached from rest. Added `weld_structural_siblings_into_body` to make_simready.py, F63 audit check, skill docs. Post-fix body: 1 → 11 colliders; AUDIT 7/7; 0 orphans; teleop PASS with all parts moving together. Regression-tested clean on InstrumentTrolley_B / Refrigerator_A / EmergencyTrolley / DrugCabinet / BipolardissectingScissors) |
| 20 | SurgicalTable_A01_01 | table | **built** (2026-04-19 — drove F64/F64b/F64c caster fixes: 4 no-tire casters (cap+core only, bbox aspect 1.06) skidded on cube colliders, chassis parked on `wheel*_base` plates + `base1` foot at floor level. Added `synthesize_wheel_cylinder_collider` (primitive Cylinder, axis Y, r=4cm h=7.6cm, rubber-bound), `_strip_chassis_wheel_blockers` (hides + strips wheel-prefix blockers), `strip_chassis_floor_blockers` (hides + strips non-wheel floor blockers). AUDIT 7/7, 14 links / 13 joints (4 casters + table tilt + headrest + frame1/2 + joint1-4 leg-rest chain + lader4 prismatic lift). First live verification of classifier rationale tracking: 52 rule citations, 14 parts each with `rationale` field (F06/F07/F09/F11/F20/BEH/MEC). Teleop PASS.) |

Score: **15 / 20 built + 1 partial** (MedicalutilityCart: physics correct,
drawers mis-faced; SurgicalChair: 2-DOF caster topology correct, spawn-time
PhysX broadphase still under investigation). Remaining 7 assets can be run
with the single entry-point command; no per-asset tuning is required unless
V13 surfaces a new silent-failure class, in which case follow the 3-step
fix rule below.

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

**Skill-library integration (2026-04-19):** 6 research bundles (2.8 MB,
152 files) absorbed via 3-phase review (inventory → plan → execute).
Results:
- **2 new skills:** `deformable-physics-robotics`, `newton-physx-compat-matrix`
- **Failure modes: 39 → 98** across 4 segregated axes (F/D/K/S)
- **Audit criteria: C1–C7 → C1–C11**
- **Behaviors: 16 → 18** (added compliant_hinge + compliant_slider via Howell PRBM)
- **+1,453 lines** of new skill content, all with provenance tags

**Next up:** pick from `Retractor_A01_01`, `RoboticSystem_A01_01`,
`SurgicalChair_A01_01`, or another pending asset. Deformable-asset track
(bedsheets, IV tubing, surgical drapes, cables) is now unblocked via
`deformable-physics-robotics` — Newton VBD + MuJoCo Warp path.

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

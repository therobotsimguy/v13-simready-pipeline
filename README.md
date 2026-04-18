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

That is the **only** command you need. It orchestrates:

1. `read_hierarchy` — parse USD structure
2. `gemini_vision.py` — render views, analyze geometry
3. `object_understanding.py` — infer mass + material density
4. Claude classifier — decide which parts move and how
5. `make_simready.py` — apply physics APIs, collision, joints, drives
6. `validate_dynamics.py` — MuJoCo stability check
7. `verify_visual.py` — render before/after for visual sanity
8. Auto-push debug data + classify JSONs to this repo

End-to-end: ~3–5 min per asset (most of it is LLM calls).

---

## Do NOT call `make_simready.py` directly

`make_simready.py` is a **sub-step**, not an entry point. It's invoked by
`simready_agent.py` as a subprocess. Calling it directly skips vision,
object understanding, classification, MuJoCo validation, and visual verify —
you'll get an untested asset that may fail in ways the agent would have
caught.

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

13 skills describe everything the classifier needs to know about USD
physics, PhysX gotchas, joint parameters, failure modes, robot constraints,
and collision strategy. They load automatically into the classifier's
system prompt at runtime — no manual setup.

When pipeline behavior needs to change, **edit the skills first, then the
code**. The classifier reads skills; future Claude sessions read skills;
audit checks cite skills by name in their error messages. They are the
durable record of pipeline decisions.

---

## Audit criteria (`C1`–`C7`)

Every asset must pass all 7 criteria before it's considered SimReady.
`simready_agent.py` runs the audit automatically; `make_simready.py --fix`
runs it before and after applying physics. 7/7 means the asset is
structurally valid. It does **not** guarantee it feels right in teleop —
always test physically after building.

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
| 10 | ResuscitationBed_A01_01 | bed | pending |
| 11 | Retractor_A01_01 | surgical tool | pending |
| 12 | RoboticSystem_A01_01 | system | pending |
| 13 | RoboticSystem_B01_Console_01 | system | pending |
| 14 | Scissors_A01_01 | surgical tool | **built** |
| 15 | SelfretainingRetractor_A01_01 | surgical tool | **built** (motion PASS — shift-drag arms at `--asset_scale 5.0`; prongs visually clip at close position — geometry limitation, not a pipeline bug; see `LEARNINGS.md` → scissor self-collision) |
| 16 | SurgicalChair_A01_01 | chair | pending |
| 17 | SurgicalChair_B01_01 | chair | pending |
| 18 | SurgicalMicroScope_A01_01 | system | pending |
| 19 | SurgicalpowerTool_B01_01 | surgical tool | pending |
| 20 | SurgicalTable_A01_01 | table | pending |

Score: **11 / 20 built** (MedicalutilityCart counts as partial — physics
correct, drawers mis-faced but usable for teleop). Remaining 9 assets can
be run with the single entry-point command; no per-asset tuning is required
unless V13 surfaces a new silent-failure class, in which case follow the
3-step fix rule below.

**Recent fix wave (2026-04-18):** F40 Gemini prismatic-range override,
F41 handheld-tool auto-dynamic, F42 wheel-split keyword growth (base/trim),
F43 bake residual xformOp:scale (fixes "floats in air" class), F44 skip
concave organizer hulls, F45 enable articulation self-collisions, F46
handle-based prismatic direction, F46b signed-axis classify override,
teleop spawn-path branch (ArticulationCfg for dynamic roots).

**Next up:** `ResuscitationBed_A01_01` — bed, likely kinematic with
adjustable sections.

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

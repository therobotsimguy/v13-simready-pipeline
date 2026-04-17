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

## Requirements

- Python 3.10+
- NVIDIA Isaac Sim (for USD schemas + teleop) or standalone USD Core
- `claude-agent-sdk` Python package
- `~/.claude/api_keys.json` with `anthropic.api_key` and `github.pat`
- `~/IsaacLab/` checkout for teleop script (asset building works without it)

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

# V13 Learnings

Curated working rules and notable fixes for the V13 SimReady pipeline. This
file lives inside the V13 repo so learnings travel with the code — clone on
any machine and they come along. Deep technical content stays in `skills/`;
this is the index + narrative record.

New rules land here *and* in the appropriate `skills/*.md` (see `README.md`
for the 3-step fix rule: skill → code → audit). Do not duplicate the
physics details — cross-reference the skill.

---

## Workflow rules

### 1. Every fix touches three places — skill, code, audit.

1. `skills/<skill>/SKILL.md` — add or update the F-code (symptom, root
   cause, how to detect).
2. `scripts/tools/simready_assets/` — fix it.
3. `audit()` in `make_simready.py` — add a check that would fail when the
   bug reappears. The error message names the fix location.

Skipping step 3 means the same bug passes 7/7 on the next asset. The
scissors regression is the canonical cautionary tale — F35 (no collision on
movable blade) slipped past because C2 only checked global collider count,
not per-body coverage.

### 2. Rebuild from raw USD — never patch `_physics.usd` directly.

If something needs changing, fix it in `classify.json` or pipeline code and
rerun `make_simready.py --fix` on the original raw USD. Layered manual
edits accumulate state that `strip_existing_physics` cannot fully undo.
The self-retaining retractor was perfect on clean rebuild and broke after
five manual edits.

Corollary: don't reparent correctly-positioned children (triggers,
latches) unless they truly need independent physics. The DCC artist's
sub-mm positioning is a gift.

### 3. Every output is a self-contained folder.

```
AssetName/
├── AssetName_physics.usd      (paths remapped to ./Textures/)
├── AssetName_physics.json     (physics sidecar)
└── Textures/
    ├── T_*.png
    └── ...
```

Absolute texture paths (`C:/Assets/dt_w/...` from the original DCC tool)
break the moment someone opens the USD elsewhere. Remap to relative before
shipping. The entire folder must zip and run on any machine.

### 4. Assets never live in this repo.

`simready_agent.py` auto-pushes **only** `debug_history/*.json`,
`classify/*.json`, and `summary.json`. USD + Textures + OBJ + URDF stay at
`~/SimReady_Output/simready/<AssetName>/`. A single 220 MB auto-push on
2026-04-17 (DrugCabinet_A03_01) exhausted git's default `http.postBuffer`
and failed; the repo is a learnings flywheel, not an artifact store.

If asset versioning is ever needed, use Git LFS or an artifact store — not
plain git. Fix committed at `ee94bbf`.

---

## Environment gotchas

### Unset `ANTHROPIC_API_KEY` when credits are on OAuth

`claude-agent-sdk` prefers the `ANTHROPIC_API_KEY` env var over Claude
Code's OAuth session. If the env key points at a zero-balance workspace
while Claude Code itself is working (different workspace, funded), the
pipeline fails at Phase 2 with *"Credit balance is too low"* even though
you can see balance drop somewhere.

Run with:

```bash
env -u ANTHROPIC_API_KEY python3 scripts/tools/simready_assets/simready_agent.py ...
```

to force the SDK onto the OAuth session. Diagnose this **before**
retrying blindly — each failed Phase 2 burns ~$0.30+ in Gemini for Phase
1a/1b/1c (no caching across runs).

---

## Notable fixes (narrative record)

### Serial kinematic chains + adjacent-link self-collision (boom arm, 2026-04-17)

Two coupled gaps shipped on `ArticulatedsupportArm_A01_01`:

1. **Serial chains** — V13 used to flatten every movable and hinge every
   joint to the single `body_path`. Boom arms, robot arms, any nested
   articulation collapsed into one joint. Fixed by adding a `"parent"`
   field to each movable in `classify.json`; joint `body0` now resolves to
   the declared parent's reparented path.

2. **Adjacent-link self-collision** — PhysX reduced-coordinate
   articulations disable collision between any two links joined by a
   single joint. A welded sub-part classified as `structural` becomes part
   of its parent link's geometry, so chained siblings cannot collide with
   it. Symptom: `plate2` (prismatic on the column) slid straight through
   `plate1` (welded higher on the column). Fix: classify welded blockers
   as `"movable:fixed"` — they become non-adjacent siblings via FixedJoint
   and collision re-applies.

Full pattern + JSON example: `skills/usd-physx-schemas/SKILL.md` → *Serial
Kinematic Chains* → *Adjacent-link self-collision is disabled by the
solver*. Classifier guidance: `skills/simready-behaviors/SKILL.md`.

Commits: `76e4e56` (code), `ded2284` (docs).

### Scissor-style two-arm self-collision — unresolved (2026-04-17)

**Status:** unresolved limitation.

Self-retaining retractor rebuilt cleanly with the classify pattern:
body = central screw (kinematic pivot), `sx_01` + `dx_01` as
`movable:revolute` siblings of the body, trigger as structural under dx.
Shift-drag works once the asset is spawned via `--asset_scale 5.0`
(teleop spawn-time scale) rather than a baked USD scale — baking scale
into vertices appeared to break something in the collision path.

The prongs of the two arms continue to clip through each other under
drive force or shift-drag pressure, even though each arm has proper
colliders. Things tried that did not resolve it:

- `physxArticulation:enabledSelfCollisions = True` on the articulation
  root (plus applying `PhysxArticulationAPI` via apiSchemas).
- `physics:collisionEnabled = True` on each joint.
- Swapping `convexHull` → `convexDecomposition` with tight params
  (maxHulls=128, voxelRes=500k) on both arms — colliders visually fit
  the prong shape, still clip.
- Enabling `physxRigidBody:enableCCD` with PhysxRigidBodyAPI applied.
- Bumping `physxCollision:contactOffset` from 50μ default to 2mm.
- Removing `ArticulationRootAPI` entirely (maximal-coords constraints).
- Making the screw body dynamic (per the skill: kinematic in an
  articulation tree breaks the solver silently) plus a FixedJoint to
  world as the anchor.

Diagnosis is inconclusive — the `usd-physx-schemas` skill's
adjacent-link rule ("two links joined by ONE joint don't collide") would
imply sx↔dx should collide (2 joints apart via screw), but they don't
in practice. Contrast the boom arm fix (plate1 `movable:fixed` sibling
of plate2 under column) which does block correctly — suggests the
boom-arm case involves a FixedJoint that avoids whatever filter is
catching the revolute-revolute sibling pair.

**What worked adjacent to this:**

- `resolve_body_xform` subtree search fix (commit
  [`ef5d692`](https://github.com/therobotsimguy/v13-simready-pipeline/commit/ef5d692))
  — unblocks nested body names like `screw_01` from being silently
  remapped. Pre-requisite for the scissor pattern even attempting to
  work.
- Teleop script `contactOffset` override scoped to robot prims only
  (IsaacLab-side patch) — prevents every custom asset getting forced to
  50μ offset, which is too tight for thin geometry.

**Next steps when resuming:**

1. Build a minimal 2-body repro USD (two rigid bodies + shared
   common-parent revolute joints + articulation root, nothing else)
   and isolate whether the filter is PhysX-level or Isaac Lab-side.
2. Try explicit `UsdPhysicsCollisionGroup` whitelisting the
   arm↔arm pair.
3. Try SDF mesh collision on the arms (`approximation="sdf"` + PhysxSDFMeshCollisionAPI).
4. Compare to a known-working Isaac Lab asset that has two revolute
   links on one kinematic pivot (if any exist in the standard library).

### EmergencyTrolley world-vs-body-local fixes (2026-04-17)

Three bugs exposed by `EmergencyTrolley_A01_01`, all rooted in a single
pattern: a choice about a **body-local quantity** (xform origin, joint
axis, joint direction) was accidentally computed in **world frame**.
Hidden on identity-chassis assets like InstrumentTrolley_B; surfaced on
rotated-chassis assets (EmergencyTrolley has 181.9° Z rotation).

1. **Matrix order in `reparent_prims_preserve_world_xform`** — USD is row-
   vector convention. Correct: `new_local = wmat * parent_L2W.GetInverse()`.
   Reversed order produced sign-flipped wheel positions.
2. **`split_wheel_structural_parts` missed Xform-wrapped structural
   meshes** — EmergencyTrolley wraps each wheel sub-part (`wheel2_frame_01`,
   `caps`, `brake`) in an Xform. The pre-fix loop only accepted direct
   Mesh children, so the entire wheel-plus-bracket rotated together.
3. **Prismatic drawer direction decided in world space** — joint axis is
   body-local; comparing drawer vs body center in world bounds flipped
   the sign on a rotated chassis → drawers opened out the back.

Audit C5 now catches each regression by symptom: wheels outside footprint
→ matrix order; structural descendants in continuous-joint bodies →
wheel-split filter; prismatic drawer opens inward → direction-select
frame. Skills updated: `usd-physx-schemas` (matrix + drawer direction),
`simready-collision` (wheel-split child type).

If a future asset shows wrong positions/rotation/direction on a rotated-
chassis asset, **first suspect body-local-vs-world confusion in
`make_simready.py`**.

---

## Canonical state (2026-04-17)

- V13 is the only SimReady pipeline. V11 archived at
  `github.com/therobotsimguy/v11-simready-pipeline`; V12 removed. The
  IsaacLab working directory's only pipeline code is under
  `scripts/tools/simready_v13/` (this repo, nested clone).
- Raw assets live at `~/SimReady_Output/raw_challenge_assets/*.usd`.
- Built assets live at `~/SimReady_Output/simready/<AssetName>/`.
- The 20-asset medical operating-room library is the current validation
  set. Progress table in `README.md`.

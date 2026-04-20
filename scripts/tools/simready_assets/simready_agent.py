#!/usr/bin/env python3
"""
simready_agent.py — V13 SimReady Agent Pipeline (originally authored in V9 era)

Agent-driven USD → SimReady conversion using Claude Agent SDK.
Independent from make_simready.py (calls it as a black-box CLI tool).

Usage:
  python3 simready_agent.py --input /path/to/asset.usd
  python3 simready_agent.py --input /path/to/asset.usd --dynamic
"""

import argparse
import asyncio
import json
import os
import sys
from pathlib import Path

# ── USD hierarchy extraction (deterministic, no LLM) ──
from pxr import Usd, UsdGeom, UsdPhysics, Gf

# ── Claude Agent SDK ──
from claude_agent_sdk import (
    query,
    ClaudeAgentOptions,
    AgentDefinition,
    AssistantMessage,
    ResultMessage,
    SystemMessage,
    TextBlock,
    ToolUseBlock,
)

# ═══════════════════════════════════════════════════════════════════
# PATHS
# ═══════════════════════════════════════════════════════════════════

SCRIPT_DIR = Path(__file__).parent.resolve()
MAKE_SIMREADY = SCRIPT_DIR / "make_simready.py"
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent
V13_ROOT = SCRIPT_DIR.parent.parent.parent   # scripts/tools/simready_assets → simready_v13/
ISAACLAB_ROOT = Path(os.path.expanduser("~/IsaacLab"))
# V13 ships its skills under v13/skills/ so a fresh clone is self-contained.
# IsaacLab-colocated paths are kept as fallbacks for existing checkouts.
SKILLS_DIRS = [
    V13_ROOT / "skills",
    ISAACLAB_ROOT / ".cursor" / "skills",
    ISAACLAB_ROOT / ".claude" / "skills",
]
OUTPUT_ROOT = Path(os.path.expanduser("~/SimReady_Output/simready"))
CLASSIFY_TMP = OUTPUT_ROOT / "classify" / "agent_classify.json"


# ═══════════════════════════════════════════════════════════════════
# SKILL LOADER
# ═══════════════════════════════════════════════════════════════════

def load_skill(name: str) -> str:
    """Load a skill markdown file as text for injection into agent system prompts."""
    for skills_dir in SKILLS_DIRS:
        path = skills_dir / name / "SKILL.md"
        if path.exists():
            return path.read_text()
    searched = ", ".join(str(d) for d in SKILLS_DIRS)
    return f"[Skill '{name}' not found in: {searched}]"


# ═══════════════════════════════════════════════════════════════════
# USD HIERARCHY READER
# ═══════════════════════════════════════════════════════════════════

def _replay_classification_rationales(dbg, classify_json_path) -> None:
    """Read agent_classify.json and replay per-part rationale citations as
    skill activations on the debugger.

    The classifier output schema includes a `rationale` list per part, e.g.
    `["F09:axis-from-thin-bbox", "JP:caster-damping-2.0"]`. Each entry is
    parsed into (skill, impact, body) and forwarded to dbg.log_skill so the
    final debugger report shows which rules the LLM actually applied while
    classifying — not just which skill files were loaded into its prompt.

    Unknown / malformed rationale entries are silently dropped.
    """
    import re
    import json as _json
    from pathlib import Path as _Path
    p = _Path(str(classify_json_path))
    if not p.exists():
        return
    try:
        with open(p) as f:
            classification = _json.load(f)
    except Exception:
        return
    parts = classification.get("parts", {})
    if not isinstance(parts, dict):
        return

    NAMED_PREFIX = {
        "JP":       ("simready-joint-params",       "info"),
        "RM":       ("robot-model",                  "info"),
        "MEC":      ("simready-mechanism-lookup",    "info"),
        "COL":      ("simready-collision",           "info"),
        "USD":      ("usd-physx-schemas",            "info"),
        "BEH":      ("simready-behaviors",           "info"),
        "OVERRIDE": ("classifier",                   "override"),
        "WARN":     ("classifier",                   "warning"),
    }

    n_logged = 0
    for part_name, info in parts.items():
        if not isinstance(info, dict):
            continue
        rationale = info.get("rationale", [])
        if not isinstance(rationale, list):
            continue
        for rule in rationale:
            if not isinstance(rule, str) or ":" not in rule:
                continue
            head = rule.split(":", 1)[0].strip()
            if re.fullmatch(r"[FKDS]\d+", head):
                skill, impact = "failure-modes", "confirmation"
            elif re.fullmatch(r"C\d+", head):
                skill, impact = "simready-criteria", "confirmation"
            elif head in NAMED_PREFIX:
                skill, impact = NAMED_PREFIX[head]
            else:
                continue
            dbg.log_skill(skill, f"{part_name}: {rule}", impact=impact)
            n_logged += 1
    if n_logged:
        print(f"\n  RATIONALE TRACE: {n_logged} skill rule citation(s) "
              f"replayed from classifier output")


def read_usd_hierarchy(usd_path: str) -> str:
    """Extract USD hierarchy as structured text for LLM classification.

    Returns a human-readable tree showing prim types, mesh counts, vertex counts,
    bounding box sizes, pivot xformOps, and child mesh names — everything the
    classifier needs to decide what each part is.
    """
    stage = Usd.Stage.Open(str(usd_path))
    if not stage:
        raise FileNotFoundError(f"Cannot open USD: {usd_path}")

    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        for p in stage.GetPseudoRoot().GetChildren():
            default_prim = p
            break
    if not default_prim:
        raise ValueError("No default prim found in USD")

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)

    lines = [
        f"FILE: {usd_path}",
        f"metersPerUnit: {mpu}",
        f"upAxis: {up_axis}",
        f"defaultPrim: {default_prim.GetName()}",
        "",
        "HIERARCHY:",
    ]

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

    def _bbox_size_str(prim):
        try:
            bbox = bbox_cache.ComputeWorldBound(prim)
            rng = bbox.ComputeAlignedRange()
            if rng.IsEmpty():
                return ""
            s = rng.GetMax() - rng.GetMin()
            return f" bbox=({s[0]:.4f}, {s[1]:.4f}, {s[2]:.4f})"
        except Exception:
            return ""

    def _count_meshes(prim):
        """Count meshes and total vertices under a prim."""
        meshes = 0
        verts = 0
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Mesh):
                meshes += 1
                pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
                if pts:
                    verts += len(pts)
        return meshes, verts

    def _has_pivot(prim):
        xf = UsdGeom.Xformable(prim)
        if not xf:
            return False
        for op in xf.GetOrderedXformOps():
            if "pivot" in op.GetOpName().lower() and "invert" not in op.GetOpName().lower():
                return True
        return False

    def _apis(prim):
        tags = []
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            tags.append("RigidBody")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            tags.append("Collision")
        if prim.HasAPI(UsdPhysics.MassAPI):
            tags.append("Mass")
        return f" [{','.join(tags)}]" if tags else ""

    def describe(prim, depth=0):
        indent = "  " * depth
        typ = prim.GetTypeName() or "Prim"
        name = prim.GetName()
        nm, nv = _count_meshes(prim)
        bbs = _bbox_size_str(prim)
        pivot = " [pivot]" if _has_pivot(prim) else ""
        apis = _apis(prim)

        lines.append(f"{indent}{typ} '{name}'{apis} ({nm} meshes, {nv} verts){bbs}{pivot}")

        # List direct mesh children by name (classifier uses these for handle/bolt/etc detection)
        for child in prim.GetChildren():
            if child.IsA(UsdGeom.Mesh):
                pts = UsdGeom.Mesh(child).GetPointsAttr().Get()
                nv_child = len(pts) if pts else 0
                lines.append(f"{indent}  Mesh '{child.GetName()}' ({nv_child} verts)")

        # Recurse into Xform/Scope children
        for child in prim.GetChildren():
            if child.IsA(UsdGeom.Xform) or child.IsA(UsdGeom.Scope):
                describe(child, depth + 1)

    describe(default_prim, depth=0)
    return "\n".join(lines)


# ═══════════════════════════════════════════════════════════════════
# MAIN PIPELINE
# ═══════════════════════════════════════════════════════════════════

async def run_pipeline(input_usd: str, dynamic: bool = False, max_retries: int = 2):
    """Run the V13 SimReady agent pipeline with debugger integration."""

    input_path = Path(input_usd).resolve()
    if not input_path.exists():
        print(f"ERROR: {input_path} does not exist")
        sys.exit(1)

    if not MAKE_SIMREADY.exists():
        print(f"ERROR: make_simready.py not found at {MAKE_SIMREADY}")
        sys.exit(1)

    # ── V13: Initialize debugger ──
    from pipeline_debugger import PipelineDebugger
    asset_name = input_path.stem
    dbg = PipelineDebugger(asset_name, object_type="unknown")

    # Check history before starting
    dbg.check_history(scale=1.0)

    # ── Phase 1: Extract hierarchy (deterministic, no LLM) ──
    print(f"\n{'=' * 70}")
    print(f"  V13 SimReady Agent Pipeline")
    print(f"  Input:  {input_path}")
    print(f"  Mode:   {'dynamic (trolley/mobile)' if dynamic else 'kinematic (cabinet/fridge)'}")
    print(f"  Engine: make_simready.py at {MAKE_SIMREADY}")
    print(f"{'=' * 70}\n")

    dbg.start_stage("read_hierarchy")
    print("[Phase 1] Reading USD hierarchy...")
    try:
        hierarchy_text = read_usd_hierarchy(str(input_path))
    except Exception as e:
        print(f"ERROR reading USD: {e}")
        sys.exit(1)

    print(hierarchy_text)
    dbg.end_stage(decisions={"prims": hierarchy_text.count("Xform")})
    print()

    # ── Phase 1a+: Geometric fingerprint (per-part ground truth from USD) ──
    # Gives the classifier exact bbox sizes, aspect class, thin/long axes,
    # parent offsets, and pivot coords — so wheel-axle and slider-direction
    # decisions are grounded in geometry instead of inferred from names.
    # A/B tested 2026-04-19 on ResuscitationBed: axis correctness went
    # 8/12 → 12/12 when fingerprint was added to the classifier prompt.
    dbg.start_stage("geometric_fingerprint")
    fingerprint_text = ""
    try:
        from geometric_fingerprint import fingerprint as build_fingerprint, to_prompt_text
        fp = build_fingerprint(Usd.Stage.Open(str(input_path)))
        fingerprint_text = to_prompt_text(fp)
        print(f"[Phase 1a+] Geometric fingerprint: {len(fp['parts'])} parts "
              f"(~{len(fingerprint_text)//4} tokens)")
        dbg.end_stage(decisions={"parts": len(fp["parts"]),
                                 "chars": len(fingerprint_text)})
    except Exception as e:
        print(f"[Phase 1a+] Fingerprint skipped: {e}")
        dbg.end_stage(decisions={"skipped": str(e)})

    # ── Phase 1b: Visual analysis (V3 — Blender + Gemini) ──
    dbg.start_stage("visual_analysis")
    vision_report = ""
    try:
        from gemini_vision import analyze_asset_visually
        print("[Phase 1b] Visual analysis (Blender + Gemini)...")
        vision_result = analyze_asset_visually(
            str(input_path), hierarchy_text=hierarchy_text, verbose=True)
        if "error" not in vision_result:
            # Format for classifier consumption
            parts = vision_result.get("movable_parts", [])
            materials = vision_result.get("materials", {})
            issues = vision_result.get("issues", [])
            lines = ["GEMINI VISUAL ANALYSIS:"]
            if parts:
                lines.append(f"  Movable parts seen ({len(parts)}):")
                for p in parts:
                    handle = " [handle visible]" if p.get("handle_visible") else ""
                    hinge = f" hinge={p.get('hinge_side')}" if p.get("hinge_side") else ""
                    lines.append(f"    {p.get('name','?')} → {p.get('type','?')} axis={p.get('axis','?')}{hinge}{handle}")
            if materials:
                lines.append(f"  Materials detected:")
                for surface, mat in materials.items():
                    lines.append(f"    {surface}: {mat}")
            if issues:
                lines.append(f"  Issues flagged:")
                for issue in issues:
                    lines.append(f"    - {issue}")
            vision_report = "\n".join(lines)
            print(vision_report)
        else:
            print(f"  Vision analysis returned error: {vision_result['error']}")
    except ImportError:
        print("[Phase 1b] Skipped — gemini_vision.py not available")
    except Exception as e:
        print(f"[Phase 1b] Vision analysis failed: {e}")
    print()

    dbg.end_stage(decisions={"n_movable_seen": len(vision_result.get("movable_parts", [])) if "vision_result" in dir() and vision_result else 0})

    # ── Phase 1c: Object Understanding (V10) ──
    dbg.start_stage("object_understanding")
    object_description = ""
    object_data = {}
    try:
        from object_understanding import understand_object
        print("[Phase 1c] Object understanding (Gemini)...")
        # Reuse rendered views from Phase 1b if available
        import glob, tempfile
        views = glob.glob("/tmp/v9_vision_*/front.png")
        view_dir = str(Path(views[0]).parent) if views else None
        rendered = [str(p) for p in Path(view_dir).glob("*.png")] if view_dir else None

        object_data = understand_object(
            str(input_path), hierarchy_text=hierarchy_text,
            rendered_views=rendered, verbose=True)

        if "error" not in object_data:
            lines = ["OBJECT UNDERSTANDING:"]
            lines.append(f"  Name: {object_data.get('object_name', '?')}")
            lines.append(f"  Type: {object_data.get('object_type', '?')}")
            lines.append(f"  Material: {object_data.get('material', '?')} ({object_data.get('material_density_kg_m3', '?')} kg/m³)")
            lines.append(f"  Mass: {object_data.get('estimated_mass_kg', '?')} kg")
            lines.append(f"  Articulated: {object_data.get('is_articulated', '?')}")
            for p in object_data.get("movable_parts", []):
                bidir = " BIDIRECTIONAL" if p.get("limits_bidirectional") else ""
                lines.append(f"    {p.get('name','?')} → {p.get('behavior','?')} range={p.get('range_description','?')}{bidir}")
            notes = object_data.get("special_notes", "")
            if notes:
                lines.append(f"  Notes: {notes}")
            object_description = "\n".join(lines)
            print()
    except ImportError:
        print("[Phase 1c] Skipped — object_understanding.py not available")
    except Exception as e:
        print(f"[Phase 1c] Object understanding failed: {e}")

    # Save object data for make_simready.py to use
    OBJECT_TMP = OUTPUT_ROOT / "classify" / "agent_object.json"
    if object_data and "error" not in object_data:
        import json as _json
        with open(OBJECT_TMP, "w") as f:
            _json.dump(object_data, f, indent=2)
        print(f"  Object data saved to {OBJECT_TMP}")
    dbg.object_type = object_data.get("object_type", "unknown") if object_data else "unknown"
    dbg.gemini_output = object_data
    dbg.end_stage(decisions={
        "object_name": object_data.get("object_name", "?") if object_data else "?",
        "mass_kg": object_data.get("estimated_mass_kg", "?") if object_data else "?",
    })
    print()

    # ── Load all 8 relevant skills ──
    dbg.start_stage("skill_loading")
    # Core 5 (always needed)
    behaviors_skill = load_skill("simready-behaviors")
    criteria_skill = load_skill("simready-criteria")
    failure_skill = load_skill("failure-modes")
    joint_params_skill = load_skill("simready-joint-params")
    robot_model_skill = load_skill("robot-model")
    # Situational 3 (loaded always, agent uses when relevant)
    collision_skill = load_skill("simready-collision")
    mechanism_skill = load_skill("simready-mechanism-lookup")
    physx_schemas_skill = load_skill("usd-physx-schemas")

    ALL_SKILLS = [
        "simready-behaviors", "simready-criteria", "failure-modes",
        "simready-joint-params", "robot-model",
        "simready-collision", "simready-mechanism-lookup", "usd-physx-schemas",
    ]
    for skill_name in ALL_SKILLS:
        dbg.log_skill(skill_name, "Loaded into classifier agent system prompt", impact="info")

    # ── Build agent options ──
    dynamic_flag = " --dynamic" if dynamic else ""

    classifier_system = f"""You are a SimReady asset classifier for robotic simulation.
Given a USD hierarchy, classify each part so physics can be applied by make_simready.py.

## Behavior Knowledge
{behaviors_skill}

## Joint Parameters Reference
{joint_params_skill}

## Robot Model (Franka Panda) — Hard Constraints
{robot_model_skill}

## SimReady Criteria
{criteria_skill}

## Failure Modes to Avoid
{failure_skill}

## Collision Strategy
{collision_skill}

## Mechanism Lookup (for unknown objects)
{mechanism_skill}

## USD PhysX Schema Compatibility
{physx_schemas_skill}

## Your Task

If a GEOMETRIC FINGERPRINT block is present in the user message, it is
**ground truth** extracted directly from the USD — exact per-part bbox,
aspect label (disk_AXIS/elongated_AXIS/flat_AXIS/blocky), thin_axis, long_axis,
parent offsets, and pivot coords. Trust these numbers over name-based guesses,
but apply them CORRECTLY per joint type:

- `aspect=disk_<axis>` → wheel/caster/puck. Joint axis = `thin_axis` (the axle).
- `aspect=elongated_<axis>` + **PRISMATIC** joint (slider/telescoping tube) →
  joint axis = `long_axis` (the travel direction).
- `aspect=elongated_<axis>` + **REVOLUTE** joint (lever/pedal/door/arm) → joint
  axis is **PERPENDICULAR** to `long_axis`. Use Gemini's hinge analysis and
  pivot coords to decide which perpendicular axis (e.g. a pedal elongated along
  X that tips up/down typically rotates about Y, not X). NEVER use `long_axis`
  as the rotation axis for a revolute joint — that would be the arm's own
  length, not its swing direction.
- `aspect=flat_<axis>` → panel/plate/shelf; the `thin_axis` is the surface
  normal. Usually structural unless name suggests a lid/flap.

If a Gemini visual analysis report is provided alongside the hierarchy,
use it to cross-check — especially for parts with ambiguous names. Gemini
sees handles, hinges, and materials you can't infer from prim names alone.

1. Identify the BODY — the main structural Xform (largest, most meshes/vertices).
2. For each Xform in the hierarchy, classify:
   - Door/lid/flap (hinged): "movable:revolute" + axis (Z=vertical hinge, X=horizontal)
   - Drawer/slider: "movable:prismatic" + axis (Y=depth, X=lateral) — prefer fingerprint long_axis
   - Wheel/caster: "movable:continuous" + axis — **always** thin_axis from fingerprint if available
   - Lever/pedal/handle/foot-pedal (hinged arm): "movable:revolute" — axis is PERPENDICULAR
     to the arm's long_axis (typically Y if arm runs along X); do NOT use long_axis as rotation axis
   - Kinematic-chain link (boom arm, robot arm segment): "movable:revolute" or "movable:prismatic" + parent
   - Shelf/divider/interior: "structural"
   - Bolts/clips/LEDs/logos: "decorative"
3. Use name AND fingerprint (bbox, aspect, thin_axis, long_axis) AND Gemini visual cues together.
4. Declare a "parent" for every movable:
   - Flat topology (wheels on trolley, doors on fridge): parent = body.
   - Serial chain (arm segments, nested pivots): parent = the movable it hinges to. Walk the pivot chain Gemini identified.
   - If unsure, default parent = body.
5. Grandchildren CAN be movable ONLY when declared as a kinematic-chain link with a movable parent (not body). Otherwise treat grandchildren as structural (PhysX swallows them silently).
6. Output ONLY valid JSON. No markdown fences, no explanation.

## Output Format
{{"body": "<body_xform_name>", "parts": {{
  "<part>": {{"class": "movable:revolute", "axis": "Z", "parent": "body",
              "rationale": ["F09:axis-from-thin-bbox", "JP:wheel-revolute-default"]}},
  "<part>": {{"class": "structural", "rationale": []}}
}}}}

## Rationale (REQUIRED — list rules you actually applied to this part)

For EACH part, add a `"rationale"` field listing the skill rule IDs that
informed your decision. List ONLY rules you actually applied while
reasoning. Do NOT fabricate citations.

Prefix taxonomy:
- `F##:<short>` / `K##:<short>` / `D##:<short>` / `S##:<short>` — failure-modes rule applied (e.g. "F09:axis-from-thin-bbox", "F11:declared-parent-chain")
- `C##:<short>` — simready-criteria (e.g. "C5:joint-per-movable")
- `JP:<short>` — joint-params lookup (e.g. "JP:caster-damping-2.0")
- `RM:<short>` — robot-model constraint (e.g. "RM:franka-150N-cap")
- `MEC:<short>` — mechanism lookup match (e.g. "MEC:swivel-caster-2DOF")
- `COL:<short>` — collision strategy (e.g. "COL:tire-convexHull")
- `USD:<short>` — USD/PhysX schema rule (e.g. "USD:no-grandchild-RB")
- `BEH:<short>` — behavior taxonomy match (e.g. "BEH:DOOR-revolute-Z")
- `OVERRIDE:<short>` — overrode a Gemini hint with a skill rule
- `WARN:<short>` — flagged a potential issue without changing the class

Empty `"rationale": []` is fine for trivially-obvious classifications
(body root, plain structural shell). For movables, expect 1–4 entries.

## Kinematic-Chain Example (medical boom arm)
Hierarchy:
  body
  ├── base                (structural mount)
  └── mechanism [pivot]
      └── arm [pivot]
          └── column [pivot]
              ├── plate1 [pivot]
              └── plate2 [pivot]

Correct output:
{{"body": "body", "parts": {{
  "base":      {{"class": "structural"}},
  "mechanism": {{"class": "movable:revolute", "axis": "Z", "parent": "body"}},
  "arm":       {{"class": "movable:revolute", "axis": "Y", "parent": "mechanism"}},
  "column":    {{"class": "movable:revolute", "axis": "Z", "parent": "arm"}},
  "plate1":    {{"class": "movable:revolute", "axis": "Y", "parent": "column"}},
  "plate2":    {{"class": "movable:revolute", "axis": "Y", "parent": "column"}}
}}}}

## Pre-Flight Checks (MUST verify before output)
- F05: Every part name you output must exist in the hierarchy above
- F06: Structural keywords (fixer/bolt/body/mount/stopper) → structural
- F07: Movable keywords (door/drawer/wheel/lid/flap) → movable
- F08: Joint type matches part type (hinged=revolute, sliding=prismatic, spinning=continuous)
- F09: Axis matches physics (vertical hinge=Z, horizontal=X, wheel=thin bbox dimension)
- F10: Every movable has a "parent" field; parent is either "body" or another movable's name
- F11: If Gemini lists N movable_parts nested in hierarchy, produce N movables with declared parent chain (do NOT collapse to 1)
- F14c: SYMMETRIC-PIVOT INSTRUMENTS (scissors, clamps, pliers, forceps, bipolar tools):
  If the hierarchy has two symmetric arm Xforms (e.g. `*_dx_*`/`*_sx_*`, `*_left_*`/`*_right_*`) pivoting around a shared pin, set `body = <default_prim_name>` (the top-level root Xform — same name as the USD file stem). Both arms become `movable:revolute`. Do NOT pick one arm as body — URDF export will fail with "more than one to-neighbor" on the revolute joint even though AUDIT reports 7/7. See `simready-mechanism-lookup` → Symmetric-Pivot Instruments."""

    auditor_system = f"""You are a SimReady audit diagnostician.
When a USD asset fails the 7-criteria audit after make_simready.py --fix, you diagnose
WHY it failed and propose a corrected classify.json that will fix the issue.

## SimReady Criteria
{criteria_skill}

## Failure Modes
{failure_skill}

## Joint Parameters Reference
{joint_params_skill}

## Robot Model Constraints
{robot_model_skill}

## Collision Strategy
{collision_skill}

## Your Task

1. Read the make_simready.py output (audit scores, warnings, errors).
2. For each failed criterion, identify which failure mode (F01-F34) caused it.
3. Determine if the failure is fixable by changing the classification JSON.
4. If fixable: output a COMPLETE corrected classify.json.
5. If not fixable (pipeline bug): describe the issue clearly.

## Output Format

Always output valid JSON:
{{"diagnosis": "what failed and why",
  "fixable": true,
  "corrected_classification": {{"body": "...", "parts": {{...}}}}
}}

Or if not fixable:
{{"diagnosis": "what failed and why",
  "fixable": false,
  "pipeline_issue": "description of the bug"
}}

## Serial-Chain Fix Pattern

If the audit reports `C5 CHAIN COLLAPSE: classifier declared N movable parts
but only M joints produced`, the previous classify.json was missing the
`"parent"` field for kinematic-chain links (boom arms, robot arms, support
arms). The corrected classification MUST include a `"parent"` field per
movable — either `"body"` or the name of another movable that the link
hinges to. Example:

{{"parts": {{
  "mechanism": {{"class": "movable:revolute", "axis": "Z", "parent": "body"}},
  "arm":       {{"class": "movable:revolute", "axis": "Y", "parent": "mechanism"}},
  "column":    {{"class": "movable:revolute", "axis": "Z", "parent": "arm"}}
}}}}"""

    options = ClaudeAgentOptions(
        model="claude-opus-4-6",
        allowed_tools=["Read", "Write", "Bash", "Glob", "Grep", "Agent"],
        permission_mode="bypassPermissions",
        max_turns=40,
        cwd=str(SCRIPT_DIR),
        agents={
            "classifier": AgentDefinition(
                description="Classifies USD asset parts into behavior types (door/drawer/wheel/structural) for physics simulation",
                prompt=classifier_system,
                tools=["Read"],
                model="opus",
            ),
            "auditor": AgentDefinition(
                description="Diagnoses SimReady audit failures and proposes corrected classify.json",
                prompt=auditor_system,
                tools=["Read", "Bash"],
                model="opus",
            ),
        },
    )

    # ── Orchestrator prompt ──
    orchestrator_prompt = f"""You are the V9 SimReady pipeline orchestrator.
Your job: take a raw USD asset and make it simulation-ready by classifying its parts,
then running make_simready.py to apply physics, then verifying the result passes audit.

## Context

INPUT ASSET: {input_path}
MODE: {"dynamic (trolley/mobile — draggable body)" if dynamic else "kinematic (cabinet/fridge — body stays fixed)"}

The USD hierarchy has already been extracted:

```
{hierarchy_text}
```
{('## Geometric Fingerprint (ground truth from USD)' + chr(10) + chr(10) + '```' + chr(10) + fingerprint_text + chr(10) + '```' + chr(10) + chr(10) + 'Use thin_axis for wheel axle direction and flat-panel normal; use long_axis for slider travel direction and arm orientation. Values are exact — do NOT guess from part name when fingerprint disagrees.' + chr(10)) if fingerprint_text else ''}
{('## Gemini Visual Analysis' + chr(10) + chr(10) + vision_report + chr(10)) if vision_report else ''}
{('## Object Understanding (V10)' + chr(10) + chr(10) + object_description + chr(10)) if object_description else ''}
## Tools Available

- "classifier" agent: Send it the hierarchy, it returns classify.json
- "auditor" agent: Send it failed audit output, it diagnoses and returns corrected classify.json
- Bash: Run make_simready.py and other commands
- Write: Save classify.json to disk

## Steps — Execute in Order

### STEP 1: CLASSIFY
Use the "classifier" agent. Send it:
- The full USD hierarchy text above.
- The Geometric Fingerprint block above, IF present — per-part bbox, aspect
  (disk/elongated/flat/blocky), thin_axis (wheel-axle candidate), long_axis
  (slider direction candidate), parent offsets, pivots.
- Gemini Visual Analysis, IF present.
- Object Understanding data, IF present.
IMPORTANT: If Object Understanding data is available above, the classifier MUST use it:
- Use the object's identified behavior ("slider" vs "drawer") for joint classification
- Use the object's range_meters for travel limits if available
- If the object is identified as non-articulated, classify all parts as structural
IMPORTANT: When the fingerprint is available, the classifier MUST prefer its
thin_axis for continuous-joint wheel axles (aspect=disk_*) and its long_axis
for prismatic sliders/telescoping tubes (aspect=elongated_*). Name-based guesses
are only a fallback when fingerprint data is missing.
Tell the classifier to return JSON in this exact format:
{{"body": "name", "parts": {{"part": {{"class": "movable:revolute", "axis": "Z", "parent": "body", "rationale": ["F09:axis-from-thin-bbox"]}}}}}}
The `rationale` field is REQUIRED per part — see classifier system prompt
for the prefix taxonomy. Use `[]` only for trivially-obvious classifications.

### STEP 2: SAVE
Parse the classifier's JSON response. Write it to {CLASSIFY_TMP}
Verify the JSON is valid before saving.

DO NOT modify or re-write {OBJECT_TMP} — it already contains the full
Gemini object-understanding output (including `movable_parts[]` with
`range_meters` per part). Overwriting it with a simplified schema drops
the per-part range data that `make_simready.py` needs for prismatic
travel override (F40). The pipeline already persisted it before you ran.

### STEP 3: APPLY PHYSICS
Run this exact command:
```
python3 {MAKE_SIMREADY} --input {input_path} --fix{dynamic_flag} --classify-json {CLASSIFY_TMP} --output-dir {OUTPUT_ROOT / input_path.stem}{' --object-json ' + str(OBJECT_TMP) if object_data else ''}
```
Capture the full output.

### STEP 4: CHECK RESULTS
Look for "SCORE:" in the output.
- If "7/7" appears: SUCCESS — go to STEP 7.
- If less than 7/7: go to STEP 5.

### STEP 5: DIAGNOSE
Send the complete make_simready.py output to the "auditor" agent.
The auditor will return a diagnosis with a corrected classify.json if possible.

### STEP 6: RETRY (max {max_retries} retries)
If the auditor provided a corrected classification:
- Write it to {CLASSIFY_TMP}
- Go back to STEP 3.
If the auditor says it's not fixable by classification, report the issue and stop.

### STEP 7: REPORT
Find the output file path (look for "_physics.usd" in the make_simready.py output).
Print this final report:

```
=== V9 RESULT ===
Status: SUCCESS (7/7) or FAILED
Output: <path to _physics.usd>
Classification: <what was classified as what>

Test with Franka teleop:
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent_cinematic.py --asset <output_path> --device cpu
```

## Important Rules
- Do NOT modify make_simready.py or any other existing code
- Do NOT run Isaac Sim or any GPU commands
- The classify.json format uses "class" (not "type" or "joint") as the key
- Valid class values: "movable:revolute", "movable:prismatic", "movable:continuous", "structural", "decorative"
- Axis values when applicable: "X", "Y", "Z"
"""

    # ── Run the agent ──
    dbg.end_stage()
    dbg.start_stage("agent_classify_and_build")
    print("[Phase 2-6] Agent pipeline starting...\n" + "-" * 70)

    session_id = None
    async for message in query(prompt=orchestrator_prompt, options=options):
        if isinstance(message, SystemMessage):
            pass  # suppress system init messages
        elif isinstance(message, AssistantMessage):
            for block in message.content:
                if isinstance(block, TextBlock):
                    print(block.text)
                elif isinstance(block, ToolUseBlock):
                    if block.name == "Agent":
                        agent_name = block.input.get("description", "agent")
                        print(f"\n  >> Spawning subagent: {agent_name}")
                    elif block.name == "Bash":
                        cmd = block.input.get("command", "")
                        if len(cmd) > 100:
                            cmd = cmd[:100] + "..."
                        print(f"\n  >> Running: {cmd}")
        elif isinstance(message, ResultMessage):
            if hasattr(message, "session_id"):
                session_id = message.session_id
            result_text = getattr(message, "result", "")
            if result_text:
                print(f"\n{result_text}")
            cost = getattr(message, "total_cost_usd", None)
            if cost is not None:
                print(f"\n[Cost: ${cost:.4f}]")

    dbg.end_stage()

    # ── Replay classifier rationales as skill activations ──
    # Each part in agent_classify.json may carry a `rationale` list of
    # rule IDs (e.g. ["F09:axis-from-thin-bbox", "JP:caster-damping-2.0"]).
    # Parse them so the debugger report shows which skill rules the LLM
    # actually applied, not just which skills were loaded into its prompt.
    _replay_classification_rationales(dbg, CLASSIFY_TMP)

    # ── Locate output physics USD for downstream phases ──
    # (Phase 7 MuJoCo validation removed 2026-04-19 — Isaac Sim teleop is
    # ground truth; URDF-based MuJoCo check produced noisy false negatives
    # on parallel-sibling joints. Cross-solver validation will return as
    # the Newton C11 parity battery, not as a MuJoCo URDF check.)
    output_usd = None
    v13_output = OUTPUT_ROOT / input_path.stem
    if v13_output.exists():
        for f in v13_output.glob("*_physics.usd"):
            output_usd = f
            break
    if not output_usd:
        output_dir = input_path.parent / "simready_out"
        if output_dir.exists():
            for f in output_dir.glob("*_physics.usd"):
                output_usd = f
                break
    if not output_usd:
        candidate = input_path.with_name(input_path.stem + "_physics.usd")
        if candidate.exists():
            output_usd = candidate

    # ── Phase 8: Post-build visual verification (V3 enhancement) ──
    dbg.start_stage("visual_verification")
    if output_usd:
        try:
            from verify_visual import verify_post_build
            print(f"\n[Phase 8] Post-build visual verification (Blender + Gemini)...")
            print("-" * 70)
            vv_result = verify_post_build(str(output_usd), verbose=True)
            overall = vv_result.get("overall", "UNKNOWN")
            if overall == "FAIL":
                print(f"\n  WARNING: Visual verification FAILED")
                print("  The asset passes audit + behavioral but LOOKS wrong.")
                print("  Review the issues above.")
            elif overall == "PASS":
                print(f"\n  Visual verification: PASS")
        except ImportError:
            print(f"\n[Phase 8] Skipped — verify_visual.py not available")
        except Exception as e:
            print(f"\n[Phase 8] Visual verification error: {e}")

    # ── Phase 9: URDF export (dual-format) ──
    if output_usd:
        try:
            from export_urdf import export_urdf
            print(f"\n[Phase 9] URDF export (dual-format)...")
            print("-" * 70)
            urdf_path = export_urdf(str(output_usd), verbose=True)
            print(f"\n  Asset is now dual-format: USD (PhysX) + URDF (MuJoCo/PyBullet/Drake)")
        except ImportError:
            print(f"\n[Phase 9] Skipped — export_urdf.py not available")
        except Exception as e:
            print(f"\n[Phase 9] URDF export error: {e}")

    dbg.end_stage()

    # ── Physics diagnostics ──
    if output_usd:
        dbg.start_stage("physics_diagnostics")
        dbg.run_diagnostics(str(output_usd))
        dbg.end_stage()

    # ── Debugger report (pre-verdict) ──
    dbg.audit_score = "7/7"  # If we got here, agent achieved 7/7
    dbg.print_report()

    # ── Terminal verdict collection ──
    if output_usd:
        print(f"\n  {'─' * 58}")
        print(f"  TEST THE ASSET:")
        print(f"    ./isaaclab.sh -p ~/v13-simready-pipeline/scripts/environments/teleoperation/teleop_se3_agent_cinematic.py \\")
        print(f"      --asset {output_usd} --device cpu")
        print(f"  {'─' * 58}")
        print(f"  Run the command above in another terminal, then come back here.\n")

        try:
            verdict = ""
            while verdict not in ("PASS", "FAIL", "SKIP"):
                verdict = input("  Verdict (PASS / FAIL / SKIP): ").strip().upper()
            notes = input("  Quick note (or Enter to skip): ").strip()
            dbg.set_verdict(verdict, notes)
        except (EOFError, KeyboardInterrupt):
            print(f"\n  Skipped — verdict set to PENDING")
            dbg.set_verdict("PENDING", "User skipped verdict")
    else:
        dbg.set_verdict("FAIL", "No output USD produced")

    dbg.save()

    # ── Auto-push to GitHub ──
    # Push debug data + classify JSONs + output asset (on PASS/PENDING) to V13 repo.
    # Never pushes pipeline code changes — only data.
    # Derive V13 repo root from this script's location (resilient to clone path)
    V13_REPO = Path(__file__).resolve().parents[3]
    try:
        import subprocess as _sp
        print(f"\n[Auto-push] Syncing data to GitHub...")

        # Copy debug history
        repo_debug = V13_REPO / "debug_history"
        repo_debug.mkdir(exist_ok=True)
        import shutil as _sh
        for f in Path(os.path.expanduser("~/SimReady_Debug")).glob("*.json"):
            _sh.copy2(str(f), str(repo_debug / f.name))

        # Copy classify JSONs
        repo_classify = V13_REPO / "classify"
        repo_classify.mkdir(exist_ok=True)
        classify_dir = OUTPUT_ROOT / "classify"
        if classify_dir.exists():
            for f in classify_dir.glob("*.json"):
                _sh.copy2(str(f), str(repo_classify / f.name))

        # Git add + commit + push — only learnings (debug_history + classify).
        # Asset USD/Textures stay at ~/SimReady_Output/simready/ — never pushed to git.
        _sp.run(["git", "add", "debug_history/", "classify/"],
                cwd=str(V13_REPO), capture_output=True)
        commit_msg = f"data: {dbg.run_id} {asset_name} — {dbg.verdict or 'PENDING'}"
        result = _sp.run(
            ["git", "-c", "user.name=therobotsimguy",
             "-c", "user.email=therobotsimguy@users.noreply.github.com",
             "commit", "-m", commit_msg],
            cwd=str(V13_REPO), capture_output=True, text=True)
        if result.returncode == 0:
            # Build authenticated push URL from ~/.claude/api_keys.json (fallback to origin)
            push_target = "origin"
            _pat = None
            try:
                with open(os.path.expanduser("~/.claude/api_keys.json")) as _kf:
                    _keys = json.load(_kf)
                _gh = _keys.get("github", {})
                _pat = _gh.get("pat")
                _user = _gh.get("user", "therobotsimguy")
                if _pat:
                    push_target = f"https://{_user}:{_pat}@github.com/{_user}/v13-simready-pipeline.git"
            except Exception:
                pass
            push = _sp.run(["git", "push", push_target, "main"],
                          cwd=str(V13_REPO), capture_output=True, text=True)
            # Redact PAT from any error output
            _stderr = push.stderr[:200]
            if _pat:
                _stderr = _stderr.replace(_pat, "<REDACTED>")
            if push.returncode == 0:
                print(f"  [Auto-push] Pushed: {commit_msg}")
            else:
                print(f"  [Auto-push] Commit OK but push failed: {_stderr}")
        else:
            if "nothing to commit" in result.stdout:
                print(f"  [Auto-push] No new data to push")
            else:
                print(f"  [Auto-push] Commit failed: {result.stderr[:100]}")
    except Exception as e:
        print(f"  [Auto-push] Failed: {e}")

    print(f"\n{'=' * 70}")
    print("  V13 Pipeline Complete")
    print(f"{'=' * 70}")
    if output_usd:
        print(f"  Output: {output_usd}")
        print(f"  Test:")
        print(f"    ./isaaclab.sh -p ~/v13-simready-pipeline/scripts/environments/teleoperation/teleop_se3_agent_cinematic.py --asset {output_usd} --device cpu")
    print(f"  Debug log: ~/SimReady_Debug/{dbg.run_id}_{asset_name}.json")
    print(f"  GitHub: https://github.com/therobotsimguy/v13-simready-pipeline")
    print(f"{'=' * 70}")


# ═══════════════════════════════════════════════════════════════════
# CLI ENTRY POINT
# ═══════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="V9 SimReady Agent Pipeline — agent-driven USD physics application"
    )
    ap.add_argument("--input", required=True, help="Path to input USD file")
    ap.add_argument("--dynamic", action="store_true",
                    help="Dynamic body mode (for trolleys / draggable shells)")
    ap.add_argument("--max-retries", type=int, default=2,
                    help="Max audit-fix-retry attempts (default: 2)")
    args = ap.parse_args()
    asyncio.run(run_pipeline(args.input, dynamic=args.dynamic, max_retries=args.max_retries))


if __name__ == "__main__":
    main()

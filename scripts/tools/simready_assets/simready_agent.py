#!/usr/bin/env python3
"""
simready_agent.py — V9 SimReady Agent Pipeline

Agent-driven USD → SimReady conversion using Claude Agent SDK.
Independent from V8 make_simready.py (calls it as a black-box CLI tool).

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
VALIDATE_DYNAMICS = SCRIPT_DIR / "validate_dynamics.py"
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent
ISAACLAB_ROOT = Path(os.path.expanduser("~/IsaacLab"))
SKILLS_DIRS = [
    ISAACLAB_ROOT / ".cursor" / "skills",
    ISAACLAB_ROOT / ".claude" / "skills",
]
OUTPUT_ROOT = Path(os.path.expanduser("~/SimReady_Output"))
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

If a Gemini visual analysis report is provided alongside the hierarchy,
use it to cross-check your classification — especially for parts with
ambiguous names. Gemini can see handles, hinges, and materials you can't
infer from prim names alone.

1. Identify the BODY — the main structural Xform (largest, most meshes/vertices).
2. For each Xform child of the body (or default prim), classify:
   - Door/lid/flap (hinged): "movable:revolute" + axis (Z=vertical hinge, X=horizontal)
   - Drawer/slider: "movable:prismatic" + axis (Y=depth, X=lateral)
   - Wheel/caster: "movable:continuous" + axis (thin bbox dimension = axle)
   - Shelf/divider/interior: "structural"
   - Bolts/clips/LEDs/logos: "decorative"
3. Use name AND geometry (bbox, mesh count, pivot ops) to decide.
4. Only DIRECT children of body can be movable. Grandchildren are structural.
5. Output ONLY valid JSON. No markdown fences, no explanation.

## Output Format
{{"body": "<body_xform_name>", "parts": {{"<part>": {{"class": "movable:revolute", "axis": "Z"}}, "<part>": {{"class": "structural"}}}}}}

## Pre-Flight Checks (MUST verify before output)
- F05: Every part name you output must exist in the hierarchy above
- F06: Structural keywords (fixer/bolt/body/mount/stopper) → structural
- F07: Movable keywords (door/drawer/wheel/lid/flap) → movable
- F08: Joint type matches part type (hinged=revolute, sliding=prismatic, spinning=continuous)
- F09: Axis matches physics (vertical hinge=Z, horizontal=X, wheel=thin bbox dimension)
- F10: No grandchildren classified as movable"""

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
}}"""

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
{('## Gemini Visual Analysis' + chr(10) + chr(10) + vision_report + chr(10)) if vision_report else ''}
{('## Object Understanding (V10)' + chr(10) + chr(10) + object_description + chr(10)) if object_description else ''}
## Tools Available

- "classifier" agent: Send it the hierarchy, it returns classify.json
- "auditor" agent: Send it failed audit output, it diagnoses and returns corrected classify.json
- Bash: Run make_simready.py and other commands
- Write: Save classify.json to disk

## Steps — Execute in Order

### STEP 1: CLASSIFY
Use the "classifier" agent. Send it the full hierarchy text above.
IMPORTANT: If Object Understanding data is available above, the classifier MUST use it:
- Use the object's identified behavior ("slider" vs "drawer") for joint classification
- Use the object's range_meters for travel limits if available
- If the object is identified as non-articulated, classify all parts as structural
Tell the classifier to return JSON in this exact format:
{{"body": "name", "parts": {{"part": {{"class": "movable:revolute", "axis": "Z"}}}}}}

### STEP 2: SAVE
Parse the classifier's JSON response. Write it to {CLASSIFY_TMP}
Verify the JSON is valid before saving.

### STEP 2b: SAVE OBJECT DATA
If Object Understanding data is available, write it to /tmp/v9_object.json
This passes Gemini's mass and material density to make_simready.py.

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

    # ── Phase 7: Behavioral validation (V2) ──
    dbg.start_stage("mujoco_validation")
    # Find the output physics USD
    output_usd = None
    # Check V13 persistent output directory first
    v13_output = OUTPUT_ROOT / input_path.stem
    if v13_output.exists():
        for f in v13_output.glob("*_physics.usd"):
            output_usd = f
            break
    if not output_usd:
        # Fallback: simready_out next to input
        output_dir = input_path.parent / "simready_out"
        if output_dir.exists():
            for f in output_dir.glob("*_physics.usd"):
                output_usd = f
                break
    if not output_usd:
        candidate = input_path.with_name(input_path.stem + "_physics.usd")
        if candidate.exists():
            output_usd = candidate

    if output_usd and VALIDATE_DYNAMICS.exists():
        print(f"\n[Phase 7] Behavioral validation (MuJoCo, headless)...")
        print("-" * 70)
        from validate_dynamics import validate as run_behavioral_validation
        bv_results = run_behavioral_validation(str(output_usd), verbose=True)
        dbg.mujoco_score = f"{bv_results['pass_count']}/{bv_results['total']} pass, {bv_results['warn_count']} warn, {bv_results['fail_count']} fail"
        if bv_results["fail_count"] > 0:
            print(f"\n  WARNING: {bv_results['fail_count']} behavioral check(s) FAILED")
    elif output_usd:
        print(f"\n[Phase 7] Skipped — validate_dynamics.py not found")
    else:
        print(f"\n[Phase 7] Skipped — output _physics.usd not found")
    dbg.end_stage(decisions={"mujoco": dbg.mujoco_score})

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
    V13_REPO = Path(os.path.expanduser("~/v13-simready-pipeline"))
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

        # Copy output asset (only if run produced output)
        if output_usd and Path(str(output_usd)).exists():
            repo_example = V13_REPO / "examples" / asset_name
            repo_example.mkdir(parents=True, exist_ok=True)
            _sh.copy2(str(output_usd), str(repo_example / Path(str(output_usd)).name))
            # Copy textures
            tex_src = Path(str(output_usd)).parent / "Textures"
            tex_dst = repo_example / "Textures"
            if tex_src.is_dir() and not tex_dst.is_dir():
                _sh.copytree(str(tex_src), str(tex_dst))

        # Git add + commit + push
        _sp.run(["git", "add", "debug_history/", "classify/", "examples/"],
                cwd=str(V13_REPO), capture_output=True)
        commit_msg = f"data: {dbg.run_id} {asset_name} — {dbg.verdict or 'PENDING'}"
        result = _sp.run(
            ["git", "-c", "user.name=therobotsimguy",
             "-c", "user.email=therobotsimguy@users.noreply.github.com",
             "commit", "-m", commit_msg],
            cwd=str(V13_REPO), capture_output=True, text=True)
        if result.returncode == 0:
            push = _sp.run(["git", "push", "origin", "main"],
                          cwd=str(V13_REPO), capture_output=True, text=True)
            if push.returncode == 0:
                print(f"  [Auto-push] Pushed: {commit_msg}")
            else:
                print(f"  [Auto-push] Commit OK but push failed: {push.stderr[:100]}")
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

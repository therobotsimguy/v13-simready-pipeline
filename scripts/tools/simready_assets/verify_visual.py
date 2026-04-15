#!/usr/bin/env python3
"""
verify_visual.py — Post-build visual verification.

Renders the physics USD at rest (q=0) and max extension (q=max),
sends both to Gemini to check: "Does this look physically correct?"

Catches F07 (missing movable), F13 (wrong position), F15 (wheel clips),
F16 (hinge wrong side), F25 (invisible cloak), F37 (slider range).

Usage:
    from verify_visual import verify_post_build
    result = verify_post_build("/path/to/asset_physics.usd")
"""

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf


SCRIPT_DIR = Path(__file__).parent.resolve()
RENDER_SCRIPT = SCRIPT_DIR / "render_views.py"
API_KEYS_PATH = SCRIPT_DIR / ".." / "api_keys.json"


def _set_joint_positions(stage, q_fraction):
    """Set all joints to a fraction of their limit range (0.0=rest, 1.0=max).

    Moves the movable Xform transforms to simulate joint positions,
    since Blender doesn't run PhysX.
    """
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue

        jtype = prim.GetTypeName()
        lo_attr = prim.GetAttribute("physics:lowerLimit")
        hi_attr = prim.GetAttribute("physics:upperLimit")
        axis_attr = prim.GetAttribute("physics:axis")

        if not lo_attr or not hi_attr:
            continue

        lo = lo_attr.Get() or 0.0
        hi = hi_attr.Get() or 0.0
        axis = axis_attr.Get() if axis_attr else "Y"

        # Compute target joint value
        # Use the limit with larger magnitude (the max extension direction)
        if abs(lo) > abs(hi):
            q_target = lo * q_fraction
        else:
            q_target = hi * q_fraction

        # Find the movable body (body1)
        body1_rel = prim.GetRelationship("physics:body1")
        if not body1_rel:
            continue
        targets = body1_rel.GetTargets()
        if not targets:
            continue

        movable_prim = stage.GetPrimAtPath(targets[0])
        if not movable_prim:
            continue

        xf = UsdGeom.Xformable(movable_prim)
        if not xf:
            continue

        if "Revolute" in jtype:
            # Rotate around axis
            import math
            angle_deg = math.degrees(q_target)
            ops = xf.GetOrderedXformOps()
            # Add a rotation op
            if axis == "Z":
                rot_op = xf.AddRotateZOp(opSuffix="joint_sim")
                rot_op.Set(angle_deg)
            elif axis == "X":
                rot_op = xf.AddRotateXOp(opSuffix="joint_sim")
                rot_op.Set(angle_deg)
            elif axis == "Y":
                rot_op = xf.AddRotateYOp(opSuffix="joint_sim")
                rot_op.Set(angle_deg)

        elif "Prismatic" in jtype:
            # Translate along axis
            axis_vec = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}.get(axis, Gf.Vec3d(0, 1, 0))
            offset = axis_vec * q_target
            translate_op = xf.AddTranslateOp(opSuffix="joint_sim")
            translate_op.Set(offset)


def _render_usd(usd_path, output_dir, label=""):
    """Render 4 views using Blender headless. Returns list of PNG paths."""
    cmd = ["blender", "--background", "--python", str(RENDER_SCRIPT),
           "--", str(usd_path), str(output_dir)]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if result.returncode != 0:
        return []
    views = []
    for name in ("front", "back", "left", "right"):
        path = os.path.join(output_dir, f"{name}.png")
        if os.path.exists(path):
            # Rename with label
            labeled = os.path.join(output_dir, f"{label}_{name}.png" if label else f"{name}.png")
            if label:
                os.rename(path, labeled)
                views.append(labeled)
            else:
                views.append(path)
    return views


def _ask_gemini(image_paths, asset_description, verbose=True):
    """Send rest + max-extension images to Gemini for visual verification."""
    from google import genai
    from google.genai import types

    # Load API key
    api_key = os.environ.get("GOOGLE_API_KEY")
    if not api_key:
        keys_path = API_KEYS_PATH.resolve()
        if keys_path.exists():
            with open(keys_path) as f:
                keys = json.load(f)
            for name in ("google", "gemini"):
                if name in keys:
                    api_key = keys[name].get("api_key")
                    break
    if not api_key:
        return {"error": "No Gemini API key"}

    # Load model
    model_name = "gemini-2.5-pro"
    keys_path = API_KEYS_PATH.resolve()
    if keys_path.exists():
        with open(keys_path) as f:
            keys = json.load(f)
        for name in ("google", "gemini"):
            if name in keys:
                model_name = keys[name].get("model", model_name)
                break

    client = genai.Client(api_key=api_key)

    contents = []
    for img_path in image_paths:
        with open(img_path, "rb") as f:
            img_data = f.read()
        label = Path(img_path).stem
        contents.append(types.Part.from_text(text=f"[{label}]"))
        contents.append(types.Part.from_bytes(data=img_data, mime_type="image/png"))

    contents.append(types.Part.from_text(text=f"""
POST-BUILD VISUAL VERIFICATION for a SimReady physics asset.

{asset_description}

You are shown the asset at REST position (joints at q=0) and at MAX EXTENSION
(joints at their limit — doors fully open, drawers fully pulled out, sliders
at max range).

Check for these specific issues:

1. **DETACHMENT**: Do any parts visually separate from the body when extended?
   (rails pulling out of tracks, brackets floating in space)

2. **RANGE**: Do movable parts reach their full expected range?
   (caliper should go 0-15 on ruler, doors should open ~120°, drawers should
   extend most of their depth)

3. **WRONG DIRECTION**: Do parts move the wrong way?
   (door opening into the body, drawer sliding backward)

4. **MISSING PARTS**: Are there parts that LOOK movable but don't move between
   rest and max images? (a visible door that stays in the same position)

5. **COLLISION ARTIFACTS**: Do parts clip through each other or through the body?

6. **POSITION ERRORS**: Are parts in physically impossible positions at max extension?

Output JSON:
{{
  "overall": "PASS" or "FAIL",
  "issues": [
    {{"type": "detachment|range|direction|missing|collision|position",
      "part": "name",
      "description": "what's wrong",
      "severity": "critical|warning"}}
  ],
  "confidence": 0.0-1.0
}}

If everything looks correct, output {{"overall": "PASS", "issues": [], "confidence": 0.95}}
"""))

    if verbose:
        print(f"  Sending {len(image_paths)} images to {model_name}...")

    response = client.models.generate_content(
        model=model_name,
        contents=contents,
        config={"temperature": 0.1},
    )

    text = response.text.strip()
    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()

    try:
        result = json.loads(text)
    except json.JSONDecodeError:
        result = {"overall": "UNKNOWN", "raw_response": text, "parse_error": True}

    return result


def verify_post_build(physics_usd_path, verbose=True):
    """Full post-build visual verification. Returns structured result."""
    physics_usd_path = str(Path(physics_usd_path).resolve())

    if verbose:
        print(f"\n  Post-Build Visual Verification")
        print(f"  Input: {physics_usd_path}")
        print(f"  {'─' * 50}")

    with tempfile.TemporaryDirectory(prefix="v9_postbuild_") as tmpdir:
        # Step 1: Render at rest (q=0)
        if verbose:
            print("\n  [1/4] Rendering at rest (q=0)...")
        rest_views = _render_usd(physics_usd_path, tmpdir, label="rest")

        # Step 2: Create a temp USD with joints at max extension
        if verbose:
            print("  [2/4] Setting joints to max extension...")
        max_usd = os.path.join(tmpdir, "max_extension.usd")
        shutil.copy2(physics_usd_path, max_usd)
        stage = Usd.Stage.Open(max_usd)
        _set_joint_positions(stage, q_fraction=0.9)  # 90% of max to avoid edge issues
        stage.GetRootLayer().Save()
        del stage

        # Step 3: Render at max extension
        if verbose:
            print("  [3/4] Rendering at max extension...")
        max_dir = os.path.join(tmpdir, "max_views")
        os.makedirs(max_dir, exist_ok=True)
        max_views = _render_usd(max_usd, max_dir, label="max")

        all_views = rest_views + max_views
        if not all_views:
            if verbose:
                print("  ERROR: No views rendered")
            return {"overall": "ERROR", "issues": [], "error": "Rendering failed"}

        if verbose:
            print(f"  Rendered {len(rest_views)} rest + {len(max_views)} max views")

        # Step 4: Ask Gemini
        if verbose:
            print("\n  [4/4] Gemini visual verification...")

        # Build description from the USD
        desc_lines = [f"Asset: {Path(physics_usd_path).stem}"]
        check_stage = Usd.Stage.Open(physics_usd_path)
        for prim in check_stage.Traverse():
            if prim.IsA(UsdPhysics.Joint):
                jtype = prim.GetTypeName().replace("Physics", "")
                lo = prim.GetAttribute("physics:lowerLimit").Get()
                hi = prim.GetAttribute("physics:upperLimit").Get()
                axis = prim.GetAttribute("physics:axis").Get()
                body1 = prim.GetRelationship("physics:body1").GetTargets()
                part_name = body1[0].name if body1 else "?"
                desc_lines.append(f"  Joint: {part_name} ({jtype}, axis={axis}, limits=[{lo:.3f}, {hi:.3f}])")
        del check_stage

        description = "\n".join(desc_lines)
        result = _ask_gemini(all_views, description, verbose=verbose)

        if verbose:
            overall = result.get("overall", "?")
            n_issues = len(result.get("issues", []))
            conf = result.get("confidence", "?")
            print(f"\n  VISUAL VERDICT: {overall} ({n_issues} issues, confidence={conf})")
            for issue in result.get("issues", []):
                sev = issue.get("severity", "?")
                typ = issue.get("type", "?")
                desc = issue.get("description", "?")
                print(f"    [{sev}] {typ}: {desc}")

    return result


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Post-build visual verification")
    ap.add_argument("--input", required=True, help="Path to _physics.usd")
    args = ap.parse_args()
    result = verify_post_build(args.input)
    print(json.dumps(result, indent=2))

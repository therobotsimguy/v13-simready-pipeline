#!/usr/bin/env python3
"""
gemini_vision.py — Gemini 2.5 Pro visual analysis for SimReady assets.

Renders 4 views of a USD asset via Blender, sends to Gemini for
visual part identification, material detection, and classification
cross-checking.

Usage (called by simready_agent.py):
    from gemini_vision import analyze_asset_visually
    report = analyze_asset_visually("/path/to/asset.usd")
"""

import json
import os
import subprocess
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
RENDER_SCRIPT = SCRIPT_DIR / "render_views.py"
API_KEYS_PATH = SCRIPT_DIR / ".." / "api_keys.json"


def _load_gemini_key():
    """Load Gemini API key from api_keys.json or environment."""
    key = os.environ.get("GOOGLE_API_KEY")
    if key:
        return key
    keys_path = API_KEYS_PATH.resolve()
    if keys_path.exists():
        with open(keys_path) as f:
            keys = json.load(f)
        for name in ("google", "gemini"):
            if name in keys:
                return keys[name].get("api_key")
    return None


def _load_gemini_model():
    """Load Gemini model from api_keys.json or default."""
    keys_path = API_KEYS_PATH.resolve()
    if keys_path.exists():
        with open(keys_path) as f:
            keys = json.load(f)
        for name in ("google", "gemini"):
            if name in keys:
                return keys[name].get("model", "gemini-2.5-pro")
    return "gemini-2.5-pro"


def render_views(usd_path: str, output_dir: str, verbose: bool = True) -> list:
    """Render 4 views of USD asset using Blender headless. Returns list of PNG paths."""
    if not RENDER_SCRIPT.exists():
        raise FileNotFoundError(f"render_views.py not found at {RENDER_SCRIPT}")

    cmd = [
        "blender", "--background", "--python", str(RENDER_SCRIPT),
        "--", str(usd_path), str(output_dir)
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)

    if result.returncode != 0:
        if verbose:
            print(f"  Blender stderr: {result.stderr[-500:]}")
        raise RuntimeError(f"Blender render failed: {result.returncode}")

    views = []
    for name in ("front", "back", "left", "right"):
        path = os.path.join(output_dir, f"{name}.png")
        if os.path.exists(path):
            views.append(path)

    if verbose:
        print(f"  Rendered {len(views)} views to {output_dir}")
    return views


def analyze_with_gemini(image_paths: list, hierarchy_text: str,
                        verbose: bool = True) -> dict:
    """Send rendered views + hierarchy to Gemini for visual analysis."""
    from google import genai
    from google.genai import types

    api_key = _load_gemini_key()
    if not api_key:
        raise ValueError("No Gemini API key found. Set GOOGLE_API_KEY or add to api_keys.json")

    model_name = _load_gemini_model()
    client = genai.Client(api_key=api_key)

    # Build multi-modal content
    contents = []

    # Add images
    for img_path in image_paths:
        with open(img_path, "rb") as f:
            img_data = f.read()
        view_name = Path(img_path).stem
        contents.append(types.Part.from_text(text=f"[{view_name} view]"))
        contents.append(types.Part.from_bytes(data=img_data, mime_type="image/png"))

    # Add hierarchy text
    contents.append(types.Part.from_text(text=f"""
Analyze this furniture asset for robotic simulation (SimReady).

USD HIERARCHY:
{hierarchy_text}

Based on the images and hierarchy, identify:

1. MOVABLE PARTS: List every part that can move independently (doors, drawers,
   wheels, lids, flaps). For each, state:
   - Name (match to hierarchy Xform names)
   - Type: door (revolute), drawer (prismatic), wheel (continuous)
   - Axis: Z for vertical hinges, X for horizontal hinges, Y for drawer depth
   - Hinge side (for doors): left or right edge
   - Handle visible? yes/no

2. MATERIALS: For each visible surface, identify the material type:
   - metal/steel/chrome, plastic, glass, wood, rubber
   - This maps to friction coefficients for robot gripper interaction

3. CLASSIFICATION ISSUES: Flag anything suspicious:
   - Parts that look movable but aren't in the hierarchy as Xforms
   - Parts that look structural but have Xform + pivot (false positive risk)
   - Ambiguous names that could be misclassified (e.g., "Group_014")

4. SCALE CHECK: Does the asset look proportionally correct?
   - Standard fridge: ~180cm tall, ~90cm wide, ~70cm deep
   - Doors and drawers proportional to the body?

Output as JSON:
{{
  "movable_parts": [
    {{"name": "...", "type": "door|drawer|wheel", "axis": "X|Y|Z",
      "hinge_side": "left|right|null", "handle_visible": true}}
  ],
  "materials": {{"surface_description": "material_type"}},
  "issues": ["list of potential problems"],
  "scale_ok": true,
  "confidence": 0.0-1.0
}}
"""))

    if verbose:
        print(f"  Sending {len(image_paths)} images + hierarchy to {model_name}...")

    response = client.models.generate_content(
        model=model_name,
        contents=contents,
        config={"temperature": 0.1},
    )

    # Parse response
    text = response.text.strip()
    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()

    try:
        result = json.loads(text)
    except json.JSONDecodeError:
        result = {"raw_response": text, "parse_error": True}

    if verbose:
        n_parts = len(result.get("movable_parts", []))
        n_issues = len(result.get("issues", []))
        conf = result.get("confidence", "?")
        print(f"  Gemini found: {n_parts} movable parts, {n_issues} issues, confidence={conf}")

    return result


def analyze_asset_visually(usd_path: str, hierarchy_text: str = "",
                           verbose: bool = True) -> dict:
    """Full visual analysis: render + Gemini. Returns structured report."""
    if verbose:
        print(f"\n  V3 Visual Analysis")
        print(f"  Input: {usd_path}")
        print(f"  {'─' * 50}")

    with tempfile.TemporaryDirectory(prefix="v9_vision_") as tmpdir:
        # Step 1: Render
        if verbose:
            print("\n  [1/2] Rendering 4 views (Blender headless)...")
        try:
            views = render_views(usd_path, tmpdir, verbose=verbose)
        except Exception as e:
            if verbose:
                print(f"  ERROR: Rendering failed: {e}")
            return {"error": str(e), "movable_parts": [], "issues": []}

        if not views:
            if verbose:
                print("  ERROR: No views rendered")
            return {"error": "No views rendered", "movable_parts": [], "issues": []}

        # Step 2: Gemini analysis
        if verbose:
            print("\n  [2/2] Gemini visual analysis...")
        try:
            result = analyze_with_gemini(views, hierarchy_text, verbose=verbose)
        except Exception as e:
            if verbose:
                print(f"  ERROR: Gemini analysis failed: {e}")
            return {"error": str(e), "movable_parts": [], "issues": []}

    return result


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Gemini visual analysis of USD assets")
    ap.add_argument("--input", required=True, help="Path to USD file")
    args = ap.parse_args()

    # Read hierarchy for context
    from simready_agent import read_usd_hierarchy
    hierarchy = read_usd_hierarchy(args.input)

    result = analyze_asset_visually(args.input, hierarchy_text=hierarchy)
    print(json.dumps(result, indent=2))

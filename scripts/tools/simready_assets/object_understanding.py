#!/usr/bin/env python3
"""
object_understanding.py — V10 Object Understanding Layer

Asks Gemini: "What IS this object?" before classification.
Produces a structured description that drives:
  - Joint type and limits (slider vs drawer, bidirectional vs one-way)
  - Mass (material density × mesh volume, not bbox guess)
  - Friction (material-specific, not generic 0.5)
  - Behavior expectations (full range of motion, not just geometry heuristics)

Usage:
    from object_understanding import understand_object
    description = understand_object(usd_path, hierarchy_text, rendered_views)
"""

import json
import os
from pathlib import Path

API_KEYS_PATH = Path(__file__).parent.resolve() / ".." / "api_keys.json"

# Material density table (kg/m³) — used when Gemini identifies material
MATERIAL_DENSITIES = {
    "stainless_steel": 7800,
    "steel": 7800,
    "carbon_steel": 7850,
    "aluminum": 2700,
    "aluminium": 2700,
    "chrome": 7150,
    "iron": 7870,
    "brass": 8500,
    "copper": 8960,
    "titanium": 4500,
    "plastic": 1200,
    "abs_plastic": 1050,
    "nylon": 1150,
    "polycarbonate": 1200,
    "wood": 600,
    "plywood": 550,
    "mdf": 750,
    "oak": 750,
    "pine": 500,
    "glass": 2500,
    "rubber": 1100,
    "silicone": 1100,
    "ceramic": 2300,
    "concrete": 2400,
    "foam": 30,
    "cardboard": 200,
    "paper": 700,
}


def _load_gemini():
    """Load Gemini client and model name."""
    from google import genai

    api_key = os.environ.get("GOOGLE_API_KEY")
    model_name = "gemini-2.5-pro"

    keys_path = API_KEYS_PATH.resolve()
    if keys_path.exists():
        with open(keys_path) as f:
            keys = json.load(f)
        for name in ("google", "gemini"):
            if name in keys:
                api_key = api_key or keys[name].get("api_key")
                model_name = keys[name].get("model", model_name)

    if not api_key:
        raise ValueError("No Gemini API key")

    client = genai.Client(api_key=api_key)
    return client, model_name


def understand_object(usd_path, hierarchy_text="", rendered_views=None, verbose=True):
    """Ask Gemini what this object IS, not just what parts it has.

    Returns a structured description that drives classification and physics:
    {
        "object_name": "vernier caliper",
        "object_type": "measurement_tool",
        "material": "stainless_steel",
        "material_density_kg_m3": 7800,
        "estimated_mass_kg": 0.15,
        "is_articulated": true,
        "movable_parts": [
            {
                "name": "depthblade",
                "behavior": "slider",
                "motion": "bidirectional linear along ruler",
                "range_description": "0 to 15cm on ruler scale",
                "range_meters": 0.15,
                "joint_type": "prismatic",
                "axis": "Y",
                "limits_bidirectional": true
            }
        ],
        "special_notes": "Sliding jaw must reach full ruler range 0-15cm",
        "is_graspable": true,
        "grip_location": "body/handle area"
    }
    """
    from google.genai import types

    client, model_name = _load_gemini()

    contents = []

    # Add rendered views if available
    if rendered_views:
        for img_path in rendered_views:
            if os.path.exists(img_path):
                with open(img_path, "rb") as f:
                    img_data = f.read()
                view_name = Path(img_path).stem
                contents.append(types.Part.from_text(text=f"[{view_name}]"))
                contents.append(types.Part.from_bytes(data=img_data, mime_type="image/png"))

    contents.append(types.Part.from_text(text=f"""
You are an expert at identifying physical objects for robotic simulation.

USD HIERARCHY:
{hierarchy_text}

TASK: Identify what this object IS, what it's made of, and how it behaves.
This is NOT about listing USD parts — it's about understanding the OBJECT.

Answer these questions:

1. WHAT IS IT? Give the specific name (e.g., "vernier caliper", "surgical mallet",
   "double-door refrigerator", "instrument trolley with caster wheels").

2. WHAT IS IT MADE OF? Identify the primary material from visual appearance
   and object type. Be specific: "stainless steel" not just "metal".
   Common surgical instruments are stainless steel (~7800 kg/m³).
   Furniture is typically wood/MDF (~600-750 kg/m³) with metal hardware.

3. HOW MUCH DOES IT WEIGH? Estimate based on what this object typically
   weighs in the real world. A surgical caliper: ~150g. A mallet: ~300g.
   A fridge door: ~20kg. A trolley: ~10kg.

4. IS IT ARTICULATED? Does it have parts that move independently?
   - If YES: describe EACH movable part, what motion it makes (rotation,
     sliding, spinning), what range of motion (e.g., "0-15cm", "0-120°"),
     and whether the motion is ONE-DIRECTIONAL (drawer) or BIDIRECTIONAL (slider/caliper).
   - If NO: is it a graspable tool (pick it up) or a static fixture?

5. SPECIAL PHYSICS NOTES: Anything that would affect simulation:
   - "Sliding jaw must reach full ruler range"
   - "Forceps tips are a single fused mesh, cannot articulate"
   - "Caster wheels have both swivel and roll axes"
   - "Drawer has rail mechanism that must maintain overlap"

Output ONLY valid JSON:
{{
    "object_name": "specific name",
    "object_type": "furniture|tool|instrument|container|fixture",
    "material": "specific_material (use underscore, lowercase)",
    "material_density_kg_m3": 7800,
    "estimated_mass_kg": 0.15,
    "is_articulated": true,
    "movable_parts": [
        {{
            "name": "match to USD Xform name if possible",
            "behavior": "door|drawer|slider|wheel|lever|button|static",
            "motion": "describe the motion in plain English",
            "range_description": "human-readable range",
            "range_meters": 0.15,
            "joint_type": "revolute|prismatic|continuous",
            "axis": "X|Y|Z",
            "limits_bidirectional": false
        }}
    ],
    "special_notes": "anything important for physics",
    "is_graspable": true,
    "grip_location": "where to grip it"
}}

For non-articulated objects, set movable_parts to empty list [].
"""))

    if verbose:
        print(f"  Asking Gemini: 'What IS this object?'...")

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
        result = {"error": "JSON parse failed", "raw": text[:500]}

    # Enrich with material density lookup if Gemini's density seems off
    material = result.get("material", "").lower().replace(" ", "_")
    if material in MATERIAL_DENSITIES:
        known_density = MATERIAL_DENSITIES[material]
        gemini_density = result.get("material_density_kg_m3", 0)
        if abs(gemini_density - known_density) > known_density * 0.3:
            result["material_density_kg_m3"] = known_density
            result["_density_corrected"] = True

    if verbose:
        name = result.get("object_name", "?")
        mat = result.get("material", "?")
        mass = result.get("estimated_mass_kg", "?")
        n_parts = len(result.get("movable_parts", []))
        print(f"  Object: {name}")
        print(f"  Material: {mat} ({result.get('material_density_kg_m3', '?')} kg/m³)")
        print(f"  Mass: {mass} kg")
        print(f"  Articulated: {result.get('is_articulated', '?')} ({n_parts} movable parts)")
        for p in result.get("movable_parts", []):
            bidir = " [BIDIRECTIONAL]" if p.get("limits_bidirectional") else ""
            print(f"    {p.get('name','?')} → {p.get('behavior','?')} {p.get('joint_type','?')} "
                  f"axis={p.get('axis','?')} range={p.get('range_description','?')}{bidir}")
        notes = result.get("special_notes", "")
        if notes:
            print(f"  Notes: {notes}")

    return result


def density_for_material(material_name):
    """Look up density from material name. Returns kg/m³ or 500 (default)."""
    key = material_name.lower().replace(" ", "_")
    return MATERIAL_DENSITIES.get(key, 500)


if __name__ == "__main__":
    import argparse
    import sys
    sys.path.insert(0, str(Path(__file__).parent))

    ap = argparse.ArgumentParser(description="V10 Object Understanding")
    ap.add_argument("--input", required=True, help="Path to USD file")
    args = ap.parse_args()

    from simready_agent import read_usd_hierarchy
    from gemini_vision import render_views
    import tempfile

    hierarchy = read_usd_hierarchy(args.input)

    with tempfile.TemporaryDirectory(prefix="v10_understand_") as tmpdir:
        views = render_views(args.input, tmpdir, verbose=False)
        result = understand_object(args.input, hierarchy_text=hierarchy,
                                   rendered_views=views, verbose=True)

    print(json.dumps(result, indent=2))

---
name: blender-3d-generation
description: >-
  LLM-driven 3D asset and scene generation using Blender Python API. Covers
  agentic generation architectures (SceneCraft, ShapeCraft, Proc3D), scene graph
  construction, visual feedback loops, procedural shape representations, and
  spatial skill libraries. Use when generating 3D geometry in Blender via code,
  designing agent-based 3D pipelines, or debugging Blender script generation.
  Derived from 11 research papers (2024-2026).
---

# Blender 3D Generation Skill

## When to Use
- Generating 3D assets/scenes via LLM-written Blender Python scripts
- Designing agentic pipelines that create geometry programmatically
- Setting up visual feedback loops for iterative refinement
- Choosing a representation (scene graph, GPS, PCG) for LLM-driven 3D

## Architecture Patterns

### Pattern 1: Scene Graph + Constraint Optimization (SceneCraft)
```
Text Query
  -> LLM Decomposer (split into sub-scenes)
  -> LLM Planner (build relational scene graph)
  -> LLM Coder (write Python constraints + Blender code)
  -> Blender Render
  -> LLM+V Reviewer (visual feedback)
  -> iterate (inner loop: refine constraints; outer loop: learn skill library)
```

**Key innovations:**
- **Scene graph as intermediate representation**: Assets linked by spatial relations (Proximity, Alignment, Symmetry, Parallelism, etc.)
- **Constraint-based layout**: Each relation has a scoring function F_r that returns [0,1]
- **Self-improving skill library**: Common constraint functions are extracted and reused
- **Results**: 45.1% CLIP score improvement over BlenderGPT; 40.9% on real-world queries

**11 Spatial Relation Types:**
1. Proximity, 2. Direction, 3. Alignment, 4. Symmetry, 5. Overlap
6. Parallelism, 7. Perpendicularity, 8. Hierarchy, 9. Rotation
10. Repetition, 11. Scaling

### Pattern 2: Graph-based Procedural Shape (ShapeCraft)
```
Text Description
  -> Parser agent (hierarchical decomposition -> GPS graph)
  -> Coder agent (generate Blender code per node, multi-path sampling)
  -> Evaluator agent (render + VLM feedback per component)
  -> Procedural Execution (assemble components)
  -> BRDF Painting (component-aware texture)
```

**GPS Representation:**
- Flat graph: virtual root v_0, component nodes {v_i} as direct children
- Each node has: geometric description, positional description, bounding volume, code snippet
- Enables parallel generation of components

**Multi-path sampling:**
- M=3 paths, T=3 iterations each
- Higher temperature for exploration
- Best path selected by Evaluator score (threshold s_tau = 9/10)

**Results**: IoGT 0.471, Hausdorff 0.415, CLIP 27.27 -- best across all metrics vs LLM baselines

### Pattern 3: Procedural Compact Graph (Proc3D)
```
Text Prompt
  -> LLM generates PCG (compact graph representation)
  -> Interpreter converts to Blender/Unity3D Python
  -> 3D Mesh rendered
  -> User edits via sliders/checkboxes OR text commands
  -> LLM updates PCG parameters
```

**PCG Representation:**
- Each line = node with operation + parameters
- 4-10x reduction in token count vs raw Blender code
- 89% compile rate (vs 0% for raw Blender geometry code, 60% for others)
- **400x speedup** for edits (10ms parameter update vs 30-500s regeneration)
- Exposes parameters: booleans (ArmsOn, WheelsOn), continuous (BaseHeight, BackAngle)

**PCG vs Other Representations:**
| Representation | Compile Rate | Avg Tokens | Gen Time |
|----------------|-------------|-----------|----------|
| Blender Geo. Node | 0% | 6048 | 62s |
| Infinigen | 5% | 3403 | 50s |
| Blender Code | 30% | 2789 | 43s |
| LLaMA-Mesh | 45% | 3189 | 25s |
| **PCG (Proc3D)** | **89%** | **702** | **9s** |

## Visual Feedback Loop Design

### The Critic-Revise Pattern (All Papers)
```python
for iteration in range(max_iterations):
    img = Blender.Render(scene, code)
    feedback, score = LLM_V_Reviewer(img, description)
    if score >= threshold:
        break
    code = LLM_Coder.revise(code, feedback, scene_graph)
```

**Best practices from literature:**
- Render from **3+ camera angles** (front, 3/4 view left, 3/4 view right, top-down)
- Use **multimodal LLM** (GPT-4V, Qwen-VL-Max) as reviewer
- **Inner loop** (2-5 iterations): refine constraints for current scene
- **Outer loop** (batch processing): extract common patterns into reusable skill library
- **Early stopping**: If score >= 9/10, stop iterating (saves 40% compute)

### Common VLM Feedback Types
1. **Spatial errors**: "The chair is floating above the table" -> adjust Z position
2. **Scale errors**: "The lamp is too large relative to the desk" -> reduce scale
3. **Orientation errors**: "The books should be vertical, not horizontal" -> rotate 90
4. **Missing elements**: "I don't see the handle on the drawer" -> add component
5. **Constraint violations**: "The two chairs are not parallel" -> adjust rotation

## Blender Python API Patterns for LLM Generation

### Spatial Utility Functions (SceneCraft's Skill Library)
```python
# Core layout functions
import_obj(name) -> Layout
scale_group(objects, factor)
shift(objects, shift_loc: dict)
rotate_objects_z_axis(objects, angle_degrees)

# Measurement functions
find_highest_vertex_point(objects) -> dict
find_lowest_vertex_point(objects) -> dict
calculate_shortest_distance(vertices1, vertices2) -> float

# Constraint scoring functions (return 0.0 to 1.0)
proximity_score(obj1, obj2, min_distance=1.0, max_distance=5.0)
alignment_score(assets, axis='x'|'y'|'z')
direction_score(obj1, obj2)  # how directly obj1 faces obj2
parallelism_score(assets)
orientation_similarity(orient1, orient2)
check_vertex_overlap(vertices1, vertices2, threshold=0.01)

# Layout optimization
constraint_solving(assets, constraints, max_iterations=100)
evaluate_constraints(assets, constraints) -> float
```

### Layout Dataclass
```python
@dataclass
class Layout:
    location: Tuple[float, float, float]
    min: Tuple[float, float, float]
    max: Tuple[float, float, float]
    orientation: Tuple[float, float, float]  # Euler (pitch, yaw, roll)
```

## Failure Modes in LLM-Driven 3D Generation

| Failure | Cause | Mitigation |
|---------|-------|-----------|
| **Ambiguous prompts** | Parser can't decompose "colorful table" | Use explicit component descriptions |
| **Brief prompts** | Evaluator has no visual signal to correct | Require minimum description length |
| **Creative prompts** | "A fruit that looks like an apple" confuses system | Restrict to concrete objects |
| **Hallucinated geometry** | LLM generates impossible Blender operations | Evaluator + render verification |
| **Spatial inconsistency** | CoT reasoning loses 3D coherence | Use GPS/PCG as shared memory |
| **Scale drift** | Components generated independently lose relative scale | Bounding volume constraints in GPS |
| **Organic geometry** | Procedural methods struggle with tails, wings | Import native 3D models as external components |
| **Low compile rate** | Raw Blender code syntax errors | Use PCG (89% compile) or GPS (100% compile) |

## Key Design Principles

1. **Decompose first**: Break complex shapes into component sub-tasks before generating code
2. **Intermediate representation**: Use scene graph / GPS / PCG, not raw code -- LLMs reason better about structure than syntax
3. **Visual verification mandatory**: Never trust LLM-generated 3D without rendering and checking
4. **Library learning**: Extract reusable constraint functions from successful generations
5. **Multi-path exploration**: Generate M=3 candidates, evaluate all, pick best -- single-path is fragile
6. **Component-aware texturing**: Apply BRDF per-component, not globally -- better material accuracy
7. **Parametric over static**: PCG/GPS enable editing; raw meshes require full regeneration

---

## Implementation Patterns

Concrete patterns from the papers, adapted for SimReady articulated asset generation in Blender.

### Pattern 1: Hierarchical Shape Decomposition (ShapeCraft Parser)

Break a complex object description into a component tree before generating any geometry.
This is the single most impactful pattern — without it, LLMs try to generate the entire
object in one shot and fail on anything beyond simple primitives.

```python
import json

DECOMPOSE_PROMPT = """Given this object description, decompose it into a flat list of 
geometric components. Each component should be a simple shape that can be modeled 
independently. Output JSON.

Object: {description}

Output format:
{{
  "root": "object name",
  "components": [
    {{
      "id": "component_name",
      "geometric_description": "A rectangular box with...",
      "positional_description": "Centered at the base, extending upward...",
      "bounding_volume": {{
        "center": [x, y, z],
        "size": [w, h, d]
      }}
    }}
  ]
}}

Rules:
- Each component should be a single geometric primitive or simple combination
- Positional descriptions are relative to the root object center
- Bounding volumes are in meters
- Movable parts (doors, drawers, wheels) MUST be separate components
- Structural sub-parts (shelves, dividers) are children of the body component
"""

def decompose_object(description, llm_client):
    """Decompose object into component tree using LLM.
    
    Returns list of components with geometry and position descriptions.
    """
    response = llm_client.generate(DECOMPOSE_PROMPT.format(description=description))
    components = json.loads(response)
    
    # Validate: at least a body + one movable part for articulated objects
    has_body = any('body' in c['id'] or 'frame' in c['id'] or 'base' in c['id'] 
                   for c in components['components'])
    if not has_body:
        # Insert body as first component
        components['components'].insert(0, {
            'id': 'main_body',
            'geometric_description': f'The main structural body of the {description}',
            'positional_description': 'Centered at origin',
            'bounding_volume': {'center': [0, 0, 0], 'size': [1, 1, 1]}
        })
    
    return components
```

**When to use:** First step when generating any articulated asset from text/image. The decomposition maps directly to USD hierarchy: each component becomes an Xform, body vs movable classification feeds into `simready-behaviors`.

### Pattern 2: Procedural Compact Graph for Articulated Furniture (Proc3D)

Generate parametric Blender objects using a compact graph notation that LLMs can
reliably produce (89% compile rate vs 30% for raw Blender code).

```python
# PCG format for a cabinet with doors and drawers
# Each line: operation(parameters)
# This compiles to Blender Python via an interpreter

PCG_CABINET_EXAMPLE = """
model Cabinet
  # Body
  primitive Box(1.2, 0.5, 0.9)  # width, depth, height
  transform Position(0, 0, 0.45)  # center at half-height
  
  # Parameters (exposed as sliders/checkboxes)
  param DoorsOn: bool = true
  param DrawersOn: bool = true
  param NumDrawers: int = 2
  param DoorWidth: float = 0.58
  param DrawerHeight: float = 0.2
  param ShelfOn: bool = true
  
  # Left door (if DoorsOn)
  part LeftDoor
    primitive Box(DoorWidth, 0.02, 0.88)
    transform Position(-0.3, -0.24, 0.45)
    joint Revolute(axis=Z, parent=Body, limits=[-120, 0])
  
  # Right door
  part RightDoor
    primitive Box(DoorWidth, 0.02, 0.88)
    transform Position(0.3, -0.24, 0.45)
    joint Revolute(axis=Z, parent=Body, limits=[0, 120])
  
  # Drawers (repeated NumDrawers times)
  repeat i in range(NumDrawers)
    part Drawer_{i}
      primitive Box(0.5, 0.45, DrawerHeight)
      transform Position(0, 0, 0.1 + i * (DrawerHeight + 0.02))
      joint Prismatic(axis=Y, parent=Body, limits=[0, 0.4])
  
  # Internal shelf
  part Shelf
    primitive Box(1.16, 0.48, 0.02)
    transform Position(0, 0, 0.5)
    joint Fixed(parent=Body)
  
  output USD
"""

def pcg_to_blender_python(pcg_text):
    """Convert PCG notation to executable Blender Python script.
    
    Key mappings:
      primitive Box(w,d,h) -> bpy.ops.mesh.primitive_cube_add(size=1); scale=(w/2,d/2,h/2)
      primitive Cylinder(r,h) -> bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=h)
      transform Position(x,y,z) -> obj.location = (x,y,z)
      joint Revolute(...) -> [stored as metadata for make_simready.py]
      param name: type = default -> [exposed as custom property]
    """
    lines = []
    lines.append("import bpy")
    lines.append("import mathutils")
    lines.append("")
    lines.append("# Clear scene")
    lines.append("bpy.ops.object.select_all(action='SELECT')")
    lines.append("bpy.ops.object.delete()")
    lines.append("")
    
    # Parse PCG and generate Blender operations
    current_part = None
    for line in pcg_text.strip().split('\n'):
        line = line.strip()
        if line.startswith('primitive Box'):
            dims = _parse_args(line)
            w, d, h = float(dims[0]), float(dims[1]), float(dims[2])
            name = current_part or 'Body'
            lines.append(f"bpy.ops.mesh.primitive_cube_add(size=1)")
            lines.append(f"obj = bpy.context.active_object")
            lines.append(f"obj.name = '{name}'")
            lines.append(f"obj.scale = ({w/2}, {d/2}, {h/2})")
            lines.append(f"bpy.ops.object.transform_apply(scale=True)")
        elif line.startswith('transform Position'):
            coords = _parse_args(line)
            x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
            lines.append(f"obj.location = ({x}, {y}, {z})")
        elif line.startswith('part '):
            current_part = line.split('part ')[1].strip()
    
    return '\n'.join(lines)

def _parse_args(line):
    """Extract arguments from PCG line like 'primitive Box(1.2, 0.5, 0.9)'"""
    start = line.index('(') + 1
    end = line.index(')')
    return [a.strip() for a in line[start:end].split(',')]
```

**When to use:** When generating furniture or cabinet-style objects. The PCG format:
- Compiles reliably (89% vs 30% raw Blender code)
- Exposes parameters for editing without regeneration
- Maps directly to SimReady hierarchy (parts with joints)
- Token-efficient (702 tokens avg vs 2789 for raw Blender)

### Pattern 3: Visual Feedback Loop with Gemini (SceneCraft + Your Pipeline)

Integrate visual verification into the generation pipeline. Adapted from SceneCraft's
critic-revise loop for your `render_views.py` + `gemini_vision.py` setup.

```python
import subprocess
import json

def generate_with_feedback(description, blender_script_path, max_iterations=3,
                            score_threshold=8):
    """Generate a Blender asset with visual feedback loop.
    
    Uses render_views.py to render and gemini_vision.py to critique.
    
    Args:
        description: text description of the object
        blender_script_path: path to generated Blender Python script
        max_iterations: max critic-revise cycles
        score_threshold: stop if Gemini scores >= this (out of 10)
    
    Returns:
        final_script_path: path to the refined Blender script
        final_score: Gemini's quality score
    """
    current_script = blender_script_path
    
    for iteration in range(max_iterations):
        # Step 1: Run Blender script to produce USD
        usd_path = current_script.replace('.py', '.usd')
        subprocess.run([
            'blender', '--background', '--python', current_script,
            '--', '--output', usd_path
        ], check=True)
        
        # Step 2: Render multi-view images
        render_dir = f'/tmp/render_feedback_{iteration}'
        subprocess.run([
            'python3', 'scripts/tools/simready_assets/render_views.py',
            '--input', usd_path,
            '--output', render_dir,
            '--views', 'front,back,left,right,top,three_quarter'
        ], check=True)
        
        # Step 3: Gemini visual critique
        # Camera angles from ShapeCraft: 3 preset angles + top-down = 4 minimum
        critique = gemini_critique(render_dir, description)
        
        if critique['score'] >= score_threshold:
            print(f"  Iteration {iteration}: score {critique['score']}/10 — PASS")
            return current_script, critique['score']
        
        print(f"  Iteration {iteration}: score {critique['score']}/10 — revising")
        print(f"  Issues: {critique['issues']}")
        
        # Step 4: Revise script based on feedback
        current_script = revise_script(current_script, critique, description)
    
    return current_script, critique['score']

def gemini_critique(render_dir, description):
    """Ask Gemini to score and critique rendered views.
    
    Returns: {'score': int 0-10, 'issues': [str], 'suggestions': [str]}
    """
    CRITIQUE_PROMPT = f"""You are evaluating a 3D model rendered from multiple angles.
    
The model should be: {description}

Score 0-10 on these criteria:
1. Shape accuracy (does it match the description?)
2. Proportions (are parts correctly sized relative to each other?)  
3. Completeness (are all described parts present?)
4. Physical plausibility (would this work as a real object?)
5. Articulation readiness (are movable parts separate from the body?)

For each issue found, describe:
- What is wrong
- Which component is affected  
- How to fix it in the Blender script

Output JSON: {{"score": N, "issues": [...], "suggestions": [...]}}
"""
    # Call gemini_vision.py with the rendered images
    # ... (uses your existing gemini_vision.py infrastructure)
    pass

REVISE_PROMPT = """The Blender script below generated a 3D model with these issues:

Issues: {issues}
Suggestions: {suggestions}

Current script:
```python
{current_script}
```

Fix the issues. Output ONLY the corrected Python script, no explanation.
Key rules:
- Do NOT change parts that are working correctly
- Apply transforms (bpy.ops.object.transform_apply) after scaling
- Ensure movable parts (doors, drawers) are separate objects, not joined to body
- Keep object names meaningful (door_left, drawer_01, etc.)
"""
```

**When to use:** After initial Blender script generation and before passing to `make_simready.py`. The feedback loop catches:
- Missing components (Gemini sees "no handle on the door")
- Wrong proportions (Gemini sees "drawer is too tall for the cabinet")
- Merged geometry (Gemini sees "door and body are one piece" — must be separate for articulation)

### Pattern 4: Constraint-Based Scene Layout (SceneCraft)

When placing multiple assets in a scene or positioning parts relative to each other.

```python
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Callable

@dataclass
class Layout:
    name: str
    location: Tuple[float, float, float]
    size: Tuple[float, float, float]  # bounding box (w, d, h)
    orientation: Tuple[float, float, float]  # Euler (pitch, yaw, roll) in degrees

def proximity_score(a: Layout, b: Layout, min_dist=0.1, max_dist=5.0) -> float:
    """Score how close two objects are. 1.0 = within min_dist, 0.0 = beyond max_dist."""
    dist = np.linalg.norm(np.array(a.location) - np.array(b.location))
    if dist <= min_dist: return 1.0
    if dist >= max_dist: return 0.0
    return 1.0 - (dist - min_dist) / (max_dist - min_dist)

def alignment_score(assets: List[Layout], axis='x') -> float:
    """Score how well assets align along an axis. 1.0 = perfectly aligned."""
    idx = {'x': 0, 'y': 1, 'z': 2}[axis]
    coords = [a.location[idx] for a in assets]
    variance = np.var(coords)
    return 1.0 / (1.0 + variance)

def parallelism_score(a: Layout, b: Layout) -> float:
    """Score whether two objects face the same direction."""
    def to_forward(orient):
        yaw = np.radians(orient[1])
        return np.array([np.cos(yaw), np.sin(yaw), 0])
    fa, fb = to_forward(a.orientation), to_forward(b.orientation)
    return (np.dot(fa, fb) + 1) / 2  # Map [-1,1] to [0,1]

def no_overlap_score(a: Layout, b: Layout, margin=0.05) -> float:
    """Score 1.0 if bounding boxes don't overlap, 0.0 if they do."""
    for i in range(3):
        a_min = a.location[i] - a.size[i]/2
        a_max = a.location[i] + a.size[i]/2
        b_min = b.location[i] - b.size[i]/2
        b_max = b.location[i] + b.size[i]/2
        if a_max + margin < b_min or b_max + margin < a_min:
            return 1.0  # Separated on this axis
    return 0.0  # Overlapping on all axes

def on_ground_score(a: Layout) -> float:
    """Score 1.0 if object is sitting on ground plane (z=0)."""
    bottom = a.location[2] - a.size[2]/2
    return max(0, 1.0 - abs(bottom) * 10)  # Penalty for floating or buried

def constraint_solve(assets: List[Layout], 
                      constraints: List[Tuple[Callable, List[str], float]],
                      max_iterations=200, step=0.05) -> List[Layout]:
    """Optimize asset positions to satisfy spatial constraints.
    
    Args:
        assets: list of Layout objects to position
        constraints: list of (score_fn, [asset_names], weight) tuples
        max_iterations: optimization steps
        step: random perturbation magnitude (meters)
        
    Returns:
        optimized list of Layout objects
    """
    asset_dict = {a.name: a for a in assets}
    best_score = evaluate_all(asset_dict, constraints)
    best_state = {k: (v.location, v.orientation) for k, v in asset_dict.items()}
    
    for _ in range(max_iterations):
        # Randomly perturb one asset
        name = np.random.choice(list(asset_dict.keys()))
        asset = asset_dict[name]
        old_loc = asset.location
        old_orient = asset.orientation
        
        # Random position + orientation perturbation
        dx, dy = np.random.uniform(-step, step, 2)
        dyaw = np.random.uniform(-5, 5)  # degrees
        asset.location = (old_loc[0]+dx, old_loc[1]+dy, old_loc[2])
        asset.orientation = (old_orient[0], old_orient[1]+dyaw, old_orient[2])
        
        score = evaluate_all(asset_dict, constraints)
        if score > best_score:
            best_score = score
            best_state = {k: (v.location, v.orientation) for k, v in asset_dict.items()}
        else:
            # Revert
            asset.location = old_loc
            asset.orientation = old_orient
    
    return list(asset_dict.values())

def evaluate_all(assets: dict, constraints) -> float:
    """Sum weighted constraint scores."""
    total = 0
    for fn, names, weight in constraints:
        involved = [assets[n] for n in names if n in assets]
        if len(involved) >= 1:
            total += weight * fn(*involved) if len(involved) > 1 else weight * fn(involved[0])
    return total
```

**When to use:** When your pipeline needs to:
- Place generated objects in a scene for Isaac Sim
- Position parts relative to each other (handles on doors, wheels on body)
- Validate spatial relationships after Blender generation
- Layout multiple assets for a sim environment (kitchen scene, warehouse)

### Pattern 5: Component-Aware Blender Script Generation

Generate Blender Python that produces clean, SimReady-compatible geometry.
Each component is a separate Blender object, correctly named for the pipeline.

```python
BLENDER_GENERATION_PROMPT = """Generate a Blender Python script to model: {description}

CRITICAL RULES for SimReady compatibility:
1. Each movable part MUST be a separate Blender object (not joined to body)
2. Name objects clearly: main_body, door_left, door_right, drawer_01, wheel_FL, etc.
3. Set object origins to their pivot point:
   - Doors: origin at hinge edge center
   - Drawers: origin at back-center of drawer
   - Wheels: origin at axle center
4. Apply all transforms: bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
5. Use meters (1 Blender unit = 1 meter)
6. Export as USD: bpy.ops.wm.usd_export(filepath=output_path)
7. Ensure manifold meshes (no holes, no zero-area faces)

Object structure (match this hierarchy):
{component_list}

Script template:
```python
import bpy
import math

# Clear scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# === MAIN BODY ===
# [Generate body geometry here]
# Set name: body_obj.name = "main_body"

# === MOVABLE PARTS ===
# [Generate each movable part as separate object]
# Set origin to pivot point
# Apply transforms

# === EXPORT ===
bpy.ops.wm.usd_export(
    filepath="{output_path}",
    export_materials=True,
    export_meshes=True,
    export_normals=True,
    export_textures=True,
)
```
"""

# Common Blender operations for SimReady assets
BLENDER_PRIMITIVES = {
    'box': 'bpy.ops.mesh.primitive_cube_add(size=1)',
    'cylinder': 'bpy.ops.mesh.primitive_cylinder_add(radius=1, depth=1, vertices=32)',
    'sphere': 'bpy.ops.mesh.primitive_uv_sphere_add(radius=1, segments=32, ring_count=16)',
    'plane': 'bpy.ops.mesh.primitive_plane_add(size=1)',
    'torus': 'bpy.ops.mesh.primitive_torus_add(major_radius=1, minor_radius=0.25)',
}

# Origin placement patterns for articulated parts
ORIGIN_PATTERNS = """
# Door origin at hinge edge (left hinge):
door_obj.select_set(True)
bpy.context.view_layer.objects.active = door_obj
# Move 3D cursor to hinge position
bpy.context.scene.cursor.location = (hinge_x, hinge_y, hinge_z)
bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

# Drawer origin at back-center:
drawer_obj.select_set(True)
bpy.context.view_layer.objects.active = drawer_obj
bpy.context.scene.cursor.location = (center_x, back_y, center_z)
bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

# Wheel origin at axle center:
wheel_obj.select_set(True)
bpy.context.view_layer.objects.active = wheel_obj
bpy.context.scene.cursor.location = (axle_x, axle_y, axle_z)
bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
"""

# Boolean operations for cavities (drawer slots, door openings)
BOOLEAN_PATTERNS = """
# Cut a rectangular opening in body for a door:
bpy.ops.mesh.primitive_cube_add(size=1)
cutter = bpy.context.active_object
cutter.name = "cutter_temp"
cutter.scale = (opening_width/2, opening_depth/2, opening_height/2)
cutter.location = (opening_x, opening_y, opening_z)
bpy.ops.object.transform_apply(scale=True)

# Apply boolean difference
body_obj.select_set(True)
bpy.context.view_layer.objects.active = body_obj
mod = body_obj.modifiers.new(name="Boolean", type='BOOLEAN')
mod.operation = 'DIFFERENCE'
mod.object = cutter
bpy.ops.object.modifier_apply(modifier="Boolean")

# Delete the cutter
bpy.data.objects.remove(cutter, do_unlink=True)
"""
```

**When to use:** When prompting an LLM (Claude, GPT-4) to write Blender scripts for your pipeline. The prompt template enforces SimReady naming conventions, separate objects per part, and correct origin placement — the three things LLMs most commonly get wrong.

### Pattern 6: Self-Improving Spatial Skill Library (SceneCraft)

Build a library of reusable Blender functions that grows over time.

```python
import os
import json

class SpatialSkillLibrary:
    """A growing library of constraint functions learned from successful generations.
    
    From SceneCraft: After each successful generation, extract common patterns
    into reusable functions. Over time, the library covers most spatial relationships.
    """
    
    def __init__(self, library_dir='scripts/tools/simready_assets/blender_skills/'):
        self.library_dir = library_dir
        os.makedirs(library_dir, exist_ok=True)
        self.skills = self._load_skills()
    
    def _load_skills(self):
        """Load all skill functions from library directory."""
        skills = {}
        for fname in os.listdir(self.library_dir):
            if fname.endswith('.py'):
                with open(os.path.join(self.library_dir, fname)) as f:
                    skills[fname[:-3]] = f.read()
        return skills
    
    def add_skill(self, name, code, description):
        """Add a new skill function to the library."""
        header = f'"""{description}"""\n\n'
        path = os.path.join(self.library_dir, f'{name}.py')
        with open(path, 'w') as f:
            f.write(header + code)
        self.skills[name] = header + code
    
    def get_relevant_skills(self, description, max_skills=5):
        """Find skills relevant to a generation task.
        
        Simple keyword matching — could be upgraded to embedding similarity.
        """
        keywords = description.lower().split()
        scored = []
        for name, code in self.skills.items():
            # Score by keyword overlap
            code_lower = code.lower()
            score = sum(1 for kw in keywords if kw in code_lower or kw in name)
            if score > 0:
                scored.append((name, score, code))
        
        scored.sort(key=lambda x: x[1], reverse=True)
        return [(name, code) for name, _, code in scored[:max_skills]]
    
    def extract_from_success(self, script, description, llm_client):
        """After a successful generation, extract reusable patterns.
        
        From SceneCraft's outer-loop library learning:
        1. Ask LLM to identify reusable constraint/utility functions
        2. Generalize them (replace hardcoded values with parameters)
        3. Add to library
        """
        EXTRACT_PROMPT = f"""This Blender script successfully generated: {description}

```python
{script}
```

Identify any functions or code patterns that could be reused for other objects.
For each pattern:
1. Extract it as a standalone function with parameters
2. Give it a descriptive name
3. Write a one-line description

Output JSON: [{{"name": "...", "description": "...", "code": "..."}}]
Only extract patterns that would be useful for OTHER objects, not object-specific code.
"""
        response = llm_client.generate(EXTRACT_PROMPT)
        patterns = json.loads(response)
        
        for pattern in patterns:
            self.add_skill(pattern['name'], pattern['code'], pattern['description'])
        
        return len(patterns)
```

**When to use:** Over many asset generations, the library accumulates patterns like:
- `add_handle(obj, position, width, depth)` — add a handle to any object
- `hollow_box(w, d, h, thickness)` — create a cabinet-style hollow box
- `add_hinged_door(body, side, width, height)` — door with correct origin placement
- `wheel_with_bracket(radius, width, bracket_height)` — caster wheel assembly

After ~20-30 assets, the library covers most common operations and generation becomes faster and more reliable.

### Pattern 7: USD Export Validation from Blender

Verify the Blender output is SimReady-compatible before passing to `make_simready.py`.

```python
import bpy

def validate_blender_for_simready():
    """Run before USD export to catch common issues.
    
    Returns: list of (severity, message) tuples
    """
    issues = []
    
    # Check 1: Separate objects for movable parts
    all_objects = [o for o in bpy.data.objects if o.type == 'MESH']
    if len(all_objects) < 2:
        issues.append(('ERROR', 'Only one mesh object — movable parts must be separate objects'))
    
    # Check 2: Naming conventions
    movable_keywords = ['door', 'drawer', 'wheel', 'lid', 'flap', 'knob', 'handle']
    has_body = any('body' in o.name.lower() or 'base' in o.name.lower() or 'frame' in o.name.lower()
                   for o in all_objects)
    if not has_body:
        issues.append(('WARN', 'No object named *body*, *base*, or *frame* — pipeline may misidentify main body'))
    
    # Check 3: Applied transforms
    for obj in all_objects:
        if any(abs(s - 1.0) > 0.001 for s in obj.scale):
            issues.append(('ERROR', f'{obj.name}: unapplied scale {tuple(obj.scale)} — apply transforms first'))
        if any(abs(r) > 0.001 for r in obj.rotation_euler):
            issues.append(('WARN', f'{obj.name}: unapplied rotation — consider applying'))
    
    # Check 4: Manifold meshes
    for obj in all_objects:
        mesh = obj.data
        # Check for zero-area faces
        mesh.calc_loop_triangles()
        for tri in mesh.loop_triangles:
            if tri.area < 1e-8:
                issues.append(('WARN', f'{obj.name}: has zero-area faces — may cause collision issues'))
                break
        
        # Check for non-manifold edges (holes in mesh)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='DESELECT')
        bpy.ops.mesh.select_non_manifold()
        bpy.ops.object.mode_set(mode='OBJECT')
        selected = sum(1 for v in mesh.vertices if v.select)
        if selected > 0:
            issues.append(('WARN', f'{obj.name}: {selected} non-manifold vertices — mesh has holes'))
    
    # Check 5: Scale sanity (should be in meters)
    for obj in all_objects:
        bbox = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
        dims = [max(v[i] for v in bbox) - min(v[i] for v in bbox) for i in range(3)]
        max_dim = max(dims)
        if max_dim > 10:
            issues.append(('ERROR', f'{obj.name}: largest dimension is {max_dim:.1f}m — likely in cm, not meters'))
        elif max_dim < 0.01:
            issues.append(('WARN', f'{obj.name}: largest dimension is {max_dim:.4f}m — suspiciously small'))
    
    # Check 6: Origin placement for movable parts
    for obj in all_objects:
        name_lower = obj.name.lower()
        if any(kw in name_lower for kw in ['door', 'lid', 'flap']):
            # Door origin should be at edge (hinge), not center
            bbox_center = sum((obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box), mathutils.Vector()) / 8
            origin_to_center = (obj.matrix_world.translation - bbox_center).length
            bbox_diagonal = (mathutils.Vector(obj.dimensions)).length / 2
            if origin_to_center < bbox_diagonal * 0.2:
                issues.append(('WARN', f'{obj.name}: origin is near bbox center — should be at hinge edge for doors'))
    
    return issues

# Usage: run in Blender before export
# issues = validate_blender_for_simready()
# for severity, msg in issues:
#     print(f"[{severity}] {msg}")
```

**When to use:** As a pre-export check in your Blender generation scripts. Catches the most common issues that would cause `make_simready.py` to fail or produce broken physics.

## Reference Papers
Located at: `scripts/tools/simready_assets/reference_library/papers_blender_generation/`
- SceneCraft (01), EZBlender (02), Visual Feedback Agent (03)
- From Idea to Co-Creation (05), Agentic 3D Scene (06), Agricultural Sim (07)
- 3D Mini-Map (08), ShapeCraft (09), SAGE (10), Proc3D (11), MUSES (12)

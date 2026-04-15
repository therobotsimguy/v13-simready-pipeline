#!/usr/bin/env python3
"""
render_views.py — Render 8 views of a USD asset using Blender headless.

8 views: 4 cardinal + 2 vertical + 2 diagonal (45° corners)
Covers all 6 faces plus joint/hinge details from angles.

Usage (called by v12_pipeline.py, not directly):
  blender --background --python render_views.py -- /path/to/asset.usd /path/to/output_dir

Produces: front.png, back.png, left.png, right.png, top.png, bottom.png,
          corner_fl.png, corner_fr.png (1024x1024 each)
"""

import bpy
import sys
import os
import math
import mathutils

# Parse args after "--"
argv = sys.argv[sys.argv.index("--") + 1:]
usd_path = argv[0]
out_dir = argv[1] if len(argv) > 1 else "/tmp/v12_views"
os.makedirs(out_dir, exist_ok=True)

# Clear default scene
bpy.ops.wm.read_factory_settings(use_empty=True)

# Import USD
bpy.ops.wm.usd_import(filepath=usd_path)

# Compute scene bounds
objects = [o for o in bpy.context.scene.objects if o.type == 'MESH']
if not objects:
    print("ERROR: No mesh objects found in USD")
    sys.exit(1)

min_co = mathutils.Vector((float('inf'),) * 3)
max_co = mathutils.Vector((float('-inf'),) * 3)
for obj in objects:
    for corner in obj.bound_box:
        world_co = obj.matrix_world @ mathutils.Vector(corner)
        for i in range(3):
            min_co[i] = min(min_co[i], world_co[i])
            max_co[i] = max(max_co[i], world_co[i])

center = (min_co + max_co) / 2
size = max(max_co[i] - min_co[i] for i in range(3))
dist = size * 2.2

# Lighting
bpy.ops.object.light_add(type='SUN', location=(5, 5, 10))
sun = bpy.context.object
sun.data.energy = 3.0

bpy.ops.object.light_add(type='AREA', location=(0, 0, -5))
fill = bpy.context.object
fill.data.energy = 50.0
fill.data.size = 10.0

# Render settings
bpy.context.scene.render.engine = 'BLENDER_EEVEE_NEXT'
bpy.context.scene.render.resolution_x = 1024
bpy.context.scene.render.resolution_y = 1024
bpy.context.scene.render.image_settings.file_format = 'PNG'
bpy.context.scene.render.film_transparent = True

# World background
world = bpy.data.worlds.new("World")
bpy.context.scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes["Background"]
bg.inputs[0].default_value = (0.15, 0.15, 0.18, 1.0)

# 8 camera views:
#   4 cardinal: front, back, left, right (slightly elevated)
#   2 vertical: top (looking down), bottom (looking up)
#   2 diagonal: front-left 45°, front-right 45° (catches hinges, handles)
elev = size * 0.2  # slight elevation for cardinal views
diag = dist * 0.707  # 45° = dist * cos(45°)

views = {
    # Cardinal (Z-up, Y-front convention)
    "front":     (center.x, center.y - dist, center.z + elev),
    "back":      (center.x, center.y + dist, center.z + elev),
    "left":      (center.x - dist, center.y, center.z + elev),
    "right":     (center.x + dist, center.y, center.z + elev),
    # Vertical
    "top":       (center.x, center.y, center.z + dist),
    "bottom":    (center.x, center.y, center.z - dist),
    # Diagonal (45° from front, elevated 30°)
    "corner_fl": (center.x - diag, center.y - diag, center.z + dist * 0.5),
    "corner_fr": (center.x + diag, center.y - diag, center.z + dist * 0.5),
}

rendered = []
for name, loc in views.items():
    bpy.ops.object.camera_add(location=loc)
    cam = bpy.context.object
    cam.data.lens = 50
    cam.data.clip_end = dist * 5

    # Point camera at center
    direction = mathutils.Vector(center) - mathutils.Vector(loc)
    rot_quat = direction.to_track_quat('-Z', 'Y')
    cam.rotation_euler = rot_quat.to_euler()

    bpy.context.scene.camera = cam
    filepath = os.path.join(out_dir, f"{name}.png")
    bpy.context.scene.render.filepath = filepath
    bpy.ops.render.render(write_still=True)
    rendered.append(filepath)

    bpy.data.objects.remove(cam)

print(f"RENDERED: {len(rendered)} views to {out_dir}")
for r in rendered:
    print(f"  {r}")

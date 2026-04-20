#!/usr/bin/env python3
"""Load a SimReady USD asset into a fresh Isaac Sim stage with ground +
lighting + physics + auto-play. Isolates asset behavior from the teleop/
Franka context — if the asset crashes here, it's the asset, not teleop.

Usage:
  cd ~/IsaacLab
  ./isaaclab.sh -p scripts/tools/simready_v13/scripts/tools/simready_assets/load_with_ground.py \\
      --usd ~/SimReady_Output/simready/SurgicalChair_A01_01/SurgicalChair_A01_01_physics.usd

What you should see:
  - Asset spawns slightly above the ground at (0,0,0.5)
  - Falls under gravity, settles on the ground
  - Ground plane is a 10×10 m light-grey flat quad at z=0
  - Scene has a dome light so you can actually see the asset

Flags:
  --no-play            Don't auto-start; lets you inspect at rest before sim
  --spawn-z <float>    Override spawn Z height (default 0.5m)
"""
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--usd", required=True, help="path to physics USD")
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--spawn-z", type=float, default=0.5,
                    help="height to drop the asset from (m)")
parser.add_argument("--no-play", action="store_true",
                    help="don't auto-start simulation")
args = parser.parse_args()

usd_path = os.path.abspath(os.path.expanduser(args.usd))
if not os.path.isfile(usd_path):
    raise FileNotFoundError(f"USD not found: {usd_path}")

from isaacsim import SimulationApp

app = SimulationApp({"headless": False, "width": args.width, "height": args.height})

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdLux, UsdShade

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

UsdPhysics.Scene.Define(stage, "/World/physicsScene")

ground_xform = UsdGeom.Xform.Define(stage, "/World/ground")
ground = UsdGeom.Cube.Define(stage, "/World/ground/plate")
ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
ground.AddScaleOp().Set(Gf.Vec3f(10.0, 10.0, 0.05))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
UsdPhysics.MeshCollisionAPI.Apply(ground.GetPrim())
ground.GetPrim().GetAttribute("primvars:displayColor").Set([Gf.Vec3f(0.6, 0.6, 0.6)]) \
    if ground.GetPrim().HasAttribute("primvars:displayColor") \
    else ground.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.6)])

dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1500.0)
distant = UsdLux.DistantLight.Define(stage, "/World/sun")
distant.CreateIntensityAttr(3000.0)
distant.AddRotateXYZOp().Set(Gf.Vec3f(-45, 15, 0))

asset_xform = UsdGeom.Xform.Define(stage, "/World/asset")
asset_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(args.spawn_z)))
asset_xform.GetPrim().GetReferences().AddReference(usd_path)

app.update()

try:
    import omni.kit.viewport.utility as vp_util

    viewport = vp_util.get_active_viewport()
    if viewport:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        cs = ViewportCameraState(viewport.viewport_api)
        cs.set_position_world(Gf.Vec3d(2.5, -2.5, 1.5), True)
        cs.set_target_world(Gf.Vec3d(0.0, 0.0, 0.3), True)
except Exception as e:
    print(f"Camera setup skipped: {e}")

if not args.no_play:
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print("Simulation playing. Watch terminal for PhysX errors.")
else:
    print("Stage loaded. Press Play to start sim.")

print(f"Asset:  {usd_path}")
print(f"Spawn Z: {args.spawn_z} m  |  Ground: 10x10 m plate at z=0")

while app.is_running():
    app.update()

app.close()

#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Inspect a SimReady USD in an Isaac Lab scene — same ground + lighting as
teleop_se3_agent_cinematic.py, but no Franka robot and no teleop input.

The asset spawns at the origin, falls under gravity, settles on the ground.
Use this to isolate asset behavior: if the asset crashes or explodes here,
it's the asset (not teleop / not the Franka interaction).

Usage:
    cd ~/IsaacLab
    ./isaaclab.sh -p scripts/environments/teleoperation/inspect_asset.py \\
        --asset ~/SimReady_Output/simready/SurgicalChair_A01_01/SurgicalChair_A01_01_physics.usd

Flags mirror teleop's asset flags:
    --asset_pos X Y Z       spawn position (default 0 0 0.3)
    --asset_rot W X Y Z     spawn quaternion (default identity)
    --asset_scale FLOAT     extra scale multiplier
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Inspect a SimReady asset without a robot.")
parser.add_argument("--asset", type=str, required=True,
                    help="Absolute path to the SimReady physics USD.")
parser.add_argument("--asset_pos", type=float, nargs=3, default=[0.0, 0.0, 0.05],
                    help="Spawn position (m). Default z=0.05 so authored "
                         "body bottoms (at z=0) are 5cm above the ground "
                         "plane — avoids F61 initial-penetration = infinite "
                         "depenetration velocity = asset flies.")
parser.add_argument("--asset_rot", type=float, nargs=4, default=[1.0, 0.0, 0.0, 0.0],
                    help="Spawn rotation (wxyz quaternion).")
parser.add_argument("--asset_scale", type=float, default=None,
                    help="Extra scale multiplier (e.g. 5.0 for small assets).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
sim_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import AssetBaseCfg, ArticulationCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from pxr import Usd as _Usd, UsdGeom as _UsdGeom, UsdPhysics as _UsdPhysics  # noqa: E402


asset_path = os.path.abspath(os.path.expanduser(args_cli.asset))
if not os.path.isfile(asset_path):
    raise FileNotFoundError(f"asset not found: {asset_path}")

# Mirror teleop's scale + kinematic-root detection.
_tmp = _Usd.Stage.Open(asset_path)
_mpu = _UsdGeom.GetStageMetersPerUnit(_tmp)
_s = _mpu if abs(_mpu - 1.0) > 0.01 else 1.0
if args_cli.asset_scale:
    _s *= args_cli.asset_scale
_scale = (_s, _s, _s) if abs(_s - 1.0) > 0.001 else None
_dp = _tmp.GetDefaultPrim()
_dynamic_root = False
if _dp:
    for _c in _dp.GetChildren():
        if _c.HasAPI(_UsdPhysics.RigidBodyAPI):
            _k = _c.GetAttribute("physics:kinematicEnabled")
            _kv = _k.Get() if _k and _k.HasValue() else False
            if not _kv:
                _dynamic_root = True
            break
del _tmp


if _dynamic_root:
    _asset_cfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Asset",
        spawn=UsdFileCfg(usd_path=asset_path, scale=_scale),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(args_cli.asset_pos),
            rot=tuple(args_cli.asset_rot),
        ),
        actuators={
            "all_joints": ImplicitActuatorCfg(
                joint_names_expr=[".*"], stiffness=0.0, damping=2.0,
            ),
        },
    )
    print("[Asset] Spawn mode: ArticulationCfg (dynamic root — gravity simulated)")
else:
    _asset_cfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Asset",
        spawn=UsdFileCfg(usd_path=asset_path, scale=_scale),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=tuple(args_cli.asset_pos),
            rot=tuple(args_cli.asset_rot),
        ),
    )
    print("[Asset] Spawn mode: AssetBaseCfg (kinematic fixture)")


@configclass
class InspectSceneCfg(InteractiveSceneCfg):
    """Bare Isaac Lab scene: textured ground, dome light, distant sun, asset."""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(20.0, 20.0)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.95)),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(1.0, 1.0, 0.98)),
    )
    asset = _asset_cfg


def main():
    # Force CPU device to avoid PxSceneFlag::eENABLE_DIRECT_GPU_API errors
    # from Isaac Lab's implicit actuators trying host-side addForce/addTorque.
    # Cap depenetration velocity (F61) so initial collider overlap with the
    # ground doesn't fire an infinite push-out impulse and yeet the asset
    # upward at spawn. 5 m/s is Isaac Sim's recommended debug-load cap.
    sim_cfg = SimulationCfg(dt=1.0 / 120.0, device="cpu")
    if hasattr(sim_cfg, "physx") and sim_cfg.physx is not None:
        for attr in ("max_depenetration_velocity", "max_position_iteration_count"):
            if hasattr(sim_cfg.physx, attr) and attr == "max_depenetration_velocity":
                setattr(sim_cfg.physx, attr, 5.0)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.5, -2.5, 1.5], target=[0.0, 0.0, 0.4])

    scene_cfg = InspectSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    # Start physics immediately so the user doesn't have to click Play.
    sim.play()
    print(f"\n[Inspect] Asset loaded: {asset_path}")
    print(f"[Inspect] Position: {args_cli.asset_pos}  Rotation (wxyz): {args_cli.asset_rot}")
    print(f"[Inspect] Camera: eye (2.5, -2.5, 1.5), target (0, 0, 0.4)")
    print("[Inspect] Physics playing. Close window or Ctrl+C to exit.")
    print("[Inspect] Shift+drag in viewport to push rigid bodies.\n")

    while sim_app.is_running():
        sim.step()
        scene.update(sim.get_physics_dt())

    sim_app.close()


if __name__ == "__main__":
    main()

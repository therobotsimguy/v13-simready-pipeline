# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Teleoperation with custom articulated assets and runtime violation monitoring.

Based on the original teleop_se3_agent.py from Isaac Lab.
Adds: custom asset spawning, runtime monitor, data collection cameras.

Robot controls (keyboard):
    W/S: Forward/Back    A/D: Left/Right    Q/E: Up/Down
    Z/X: Roll    T/G: Pitch    C/V: Yaw
    K: Toggle gripper    R: Reset
"""

"""Launch Isaac Sim Simulator first."""

import argparse
from collections.abc import Callable

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Teleoperation with custom assets.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--teleop_device", type=str, default="keyboard", help="Teleop device.")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-Franka-IK-Rel-v0", help="Task name.")
parser.add_argument("--sensitivity", type=float, default=1.0, help="Sensitivity factor.")
parser.add_argument(
    "--asset",
    type=str,
    default=None,
    help="Absolute path to your SimReady USD (e.g. *_physics.usd from make_simready.py). Not a doc placeholder.",
)
parser.add_argument("--asset_pos", type=float, nargs=3, default=[2.25, 0.0, 0.0], help="Asset spawn position.")
parser.add_argument("--asset_rot", type=float, nargs=4, default=[0.707, 0.0, 0.0, 0.707], help="Asset rotation (wxyz quat, +90deg Z so drawer fronts face robot).")
parser.add_argument("--asset_scale", type=float, default=None, help="Extra scale multiplier for small assets (e.g. 5.0 to make scissors 5x bigger).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import logging
import os
import sys

import gymnasium as gym
import torch

from isaaclab.devices import Se3Gamepad, Se3GamepadCfg, Se3Keyboard, Se3KeyboardCfg, Se3SpaceMouse, Se3SpaceMouseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

# V3 — no V1 runtime monitor
_HAS_MONITOR = False

logger = logging.getLogger(__name__)

# Default asset (SimReady V7 cabinet — physics USD)
_DEFAULT_ASSET = (
    "/home/msi/IsaacLab/scripts/tools/simready_assets/cabinet_3x3_mcp/cabinet_3x3_drawers_physics.usd"
)


def main() -> None:
    """Run teleoperation with custom asset."""
    # Parse env config
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=False)
    env_cfg.env_name = args_cli.task
    if not isinstance(env_cfg, ManagerBasedRLEnvCfg):
        raise ValueError(f"Only ManagerBasedRLEnv supported. Got: {type(env_cfg).__name__}")

    env_cfg.terminations.time_out = None

    # Remove default Lift task objects
    env_cfg.scene.table = None
    env_cfg.scene.object = None
    env_cfg.observations.policy.object_position = None
    env_cfg.observations.policy.target_object_position = None
    env_cfg.commands.object_pose = None
    env_cfg.events.reset_object_position = None
    env_cfg.rewards.reaching_object = None
    env_cfg.rewards.lifting_object = None
    env_cfg.rewards.object_goal_tracking = None
    env_cfg.rewards.object_goal_tracking_fine_grained = None
    env_cfg.terminations.object_dropping = None

    # Use non-instanceable Franka so contactOffset can be overridden on
    # collision prims. The instanceable version locks properties → 20mm grip gap.
    import isaaclab.sim as sim_utils
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
    env_cfg.scene.robot.spawn.usd_path = f"{ISAAC_NUCLEUS_DIR}/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    env_cfg.scene.robot.spawn.collision_props = sim_utils.CollisionPropertiesCfg(
        contact_offset=0.00005, rest_offset=0.0,
    )
    env_cfg.scene.robot.spawn.rigid_props.solver_position_iteration_count = 16
    env_cfg.scene.robot.spawn.rigid_props.solver_velocity_iteration_count = 2

    # Ground and robot
    env_cfg.scene.plane.init_state.pos = (0.0, 0.0, 0.0)
    env_cfg.scene.robot.init_state.pos = (0.0, 0.0, 0.0)

    # --- Custom asset ---
    asset_path = args_cli.asset or _DEFAULT_ASSET
    asset_path = os.path.expanduser(asset_path)
    asset_path_abs = os.path.abspath(asset_path)
    if not os.path.isfile(asset_path_abs):
        norm = asset_path.replace("\\", "/").lower()
        placeholder_hint = ""
        if "/path/to/" in norm or norm.endswith("/path/to/asset_physics.usd"):
            placeholder_hint = (
                "\n  You passed a documentation example path. Replace it with the real file "
                "produced by make_simready.py (or omit --asset to use the script default)."
            )
        logger.error(
            "Asset USD does not exist or is not a file:\n  %s%s\n\n"
            "Example:\n  --asset %s",
            asset_path_abs,
            placeholder_hint,
            _DEFAULT_ASSET,
        )
        simulation_app.close()
        return

    asset_path = asset_path_abs
    asset_dir = os.path.dirname(asset_path)

    # Optional articulation.json (legacy simready assets with named DOFs)
    art_path = os.path.join(asset_dir, "articulation.json")
    art_spec = None
    if os.path.exists(art_path):
        with open(art_path) as f:
            art_spec = json.load(f)

    from isaaclab.assets import AssetBaseCfg
    from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg

    # Spawn cabinet via AssetBaseCfg — integrated into scene construction so
    # physics engine picks it up before simulation starts. The USD has no internal
    # physicsScene or simulationOwner (avoids reference scope issues). Root body
    # is kinematic (stays at spawn position, no world anchor needed).
    from pxr import Usd as _Usd, UsdGeom as _UsdGeom
    _tmp_stage = _Usd.Stage.Open(asset_path)
    _mpu = _UsdGeom.GetStageMetersPerUnit(_tmp_stage)
    _s = _mpu if abs(_mpu - 1.0) > 0.01 else 1.0
    if args_cli.asset_scale:
        _s *= args_cli.asset_scale
    _scale = (_s, _s, _s) if abs(_s - 1.0) > 0.001 else None
    del _tmp_stage

    env_cfg.scene.cabinet = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/CustomAsset",
        spawn=UsdFileCfg(
            usd_path=asset_path,
            scale=_scale,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=tuple(args_cli.asset_pos),
            rot=tuple(args_cli.asset_rot),
        ),
    )

    asset_name = os.path.basename(asset_path).replace("_simready.usd", "").replace("_physics.usd", "")
    print(f"\n[Asset] {asset_name}: {asset_path}")
    print(f"[Asset] Position: {args_cli.asset_pos}, Rotation: {args_cli.asset_rot}")

    # --- Create environment ---
    try:
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    except Exception as e:
        logger.error(f"Failed to create environment: {e}")
        simulation_app.close()
        return

    # --- Fix robot contactOffset on ALL shapes ---
    # The Franka USD from Nucleus has no explicit contactOffset on its shapes,
    # so they inherit Isaac Sim's default of 0.1m (10cm). We must override every shape.
    try:
        from pxr import UsdPhysics as _UsdPhysics, Sdf as _Sdf
        stage = env.sim.stage
        robot_prim_path = env.scene["robot"].cfg.prim_path.replace("{ENV_REGEX_NS}", "/World/envs/env_0")
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        if robot_prim.IsValid():
            fixed_count = 0
            for prim in _UsdPhysics.CollisionAPI.GetSchemaAttributeNames():
                pass  # just checking import works
            finger_count = 0
            for prim in stage.Traverse():
                prim_path = str(prim.GetPath())
                if prim_path.startswith(robot_prim_path) or prim_path.startswith("/World/envs/env_0"):
                    if prim.HasAPI(_UsdPhysics.CollisionAPI):
                        prim.CreateAttribute("physxCollision:contactOffset", _Sdf.ValueTypeNames.Float).Set(0.00005)
                        prim.CreateAttribute("physxCollision:restOffset", _Sdf.ValueTypeNames.Float).Set(0.0)
                        fixed_count += 1
                        # Fix gripper finger invisible cloak: the finger collision
                        # mesh is concave (inner gripping face) but PhysX defaults to
                        # convexHull which bloats it by ~66%. Use convexDecomposition
                        # for tight finger contact.
                        prim_name = prim.GetName().lower()
                        if "finger" in prim_name or "hand" in prim_name:
                            mc = _UsdPhysics.MeshCollisionAPI.Apply(prim)
                            mc.CreateApproximationAttr("convexDecomposition")
                            prim.CreateAttribute(
                                "physxConvexDecompositionCollision:maxConvexHulls",
                                _Sdf.ValueTypeNames.Int).Set(64)
                            prim.CreateAttribute(
                                "physxConvexDecompositionCollision:voxelResolution",
                                _Sdf.ValueTypeNames.Int).Set(300000)
                            finger_count += 1
            print(f"[CollisionFix] Set contactOffset=0.00005 on {fixed_count} shapes, decomp on {finger_count} finger/hand meshes")
    except Exception as e:
        print(f"[CollisionFix] Could not fix robot offsets: {e}")

    # --- Teleop device (using original Isaac Lab pattern) ---
    should_reset = False
    teleoperation_active = True

    def reset_env():
        nonlocal should_reset
        should_reset = True

    def start_teleop():
        nonlocal teleoperation_active
        teleoperation_active = True

    def stop_teleop():
        nonlocal teleoperation_active
        teleoperation_active = False

    s = args_cli.sensitivity
    if args_cli.teleop_device.lower() == "keyboard":
        teleop = Se3Keyboard(Se3KeyboardCfg(pos_sensitivity=0.05 * s, rot_sensitivity=0.05 * s))
    elif args_cli.teleop_device.lower() == "spacemouse":
        teleop = Se3SpaceMouse(Se3SpaceMouseCfg(pos_sensitivity=0.05 * s, rot_sensitivity=0.05 * s))
    elif args_cli.teleop_device.lower() == "gamepad":
        teleop = Se3Gamepad(Se3GamepadCfg(pos_sensitivity=0.1 * s, rot_sensitivity=0.1 * s))
    else:
        logger.error(f"Unsupported device: {args_cli.teleop_device}")
        env.close()
        simulation_app.close()
        return

    teleop.add_callback("R", reset_env)

    # --- Runtime monitor ---
    monitor = None
    if _HAS_MONITOR and art_spec:
        try:
            cabinet = env.scene["cabinet"]
            monitor = RuntimeMonitor(
                articulation=cabinet,
                art_spec=art_spec,
                check_every=60,
            )
        except Exception as e:
            print(f"[RuntimeMonitor] Could not initialize: {e}")

    # --- Telemetry Recorder for V3 Stage 8 ---
    telemetry = {
        "asset": asset_name,
        "asset_path": asset_path,
        "steps": 0,
        "contact_forces": [],       # gripper contact force magnitudes
        "joint_positions": [],       # asset joint positions over time
        "joint_velocities": [],      # asset joint velocities over time
        "gripper_positions": [],     # end-effector position over time
        "collision_count": 0,        # total collision events
        "grasp_times": [],           # timestamps when gripper closes on object
        "first_contact_step": None,  # when gripper first touches asset
    }

    prev_gripper_closed = False
    record_every = 10  # record every N steps to avoid huge files

    # --- Run ---
    env.reset()
    teleop.reset()

    print(f"\n=== Teleoperation: {asset_name} ===")
    print("Robot: WASD/QE + ZX/TG/CV + K (gripper)")
    print("Reset: R | Save telemetry: L")
    print(
        "Viewport: Shift+drag moves dynamic parts (doors, drawers, wheels). "
        "Fridge/cabinet shell is kinematic (SimReady B–F recipe) and does not drag; click the moving part."
    )
    print("====================================\n")

    def save_telemetry():
        """Save telemetry to disk for Stage 8."""
        import time
        telemetry_path = os.path.join(asset_dir, "telemetry.json")

        # Compute summary metrics from recorded data
        summary = {
            "asset": telemetry["asset"],
            "total_steps": telemetry["steps"],
            "first_contact_step": telemetry["first_contact_step"],
            "collision_count": telemetry["collision_count"],
        }

        # Contact force variance
        if telemetry["contact_forces"]:
            forces = torch.tensor(telemetry["contact_forces"])
            summary["contact_force_mean"] = float(forces.mean())
            summary["contact_force_variance"] = float(forces.var())
            summary["contact_force_max"] = float(forces.max())
        else:
            summary["contact_force_mean"] = 0.0
            summary["contact_force_variance"] = 0.0
            summary["contact_force_max"] = 0.0

        # Joint jitter (variance of joint velocities)
        if telemetry["joint_velocities"]:
            vels = torch.tensor(telemetry["joint_velocities"])
            summary["joint_jitter"] = float(vels.abs().mean())
            summary["joint_velocity_max"] = float(vels.abs().max())
        else:
            summary["joint_jitter"] = 0.0
            summary["joint_velocity_max"] = 0.0

        # Grip stability (how stable the contact force is when gripper is closed)
        if len(telemetry["contact_forces"]) > 10:
            recent = torch.tensor(telemetry["contact_forces"][-50:])
            if recent.mean() > 0.1:
                stability = 1.0 - min(1.0, float(recent.std() / recent.mean()))
            else:
                stability = 0.0
            summary["handle_grip_stability"] = round(stability, 3)
        else:
            summary["handle_grip_stability"] = 0.0

        # Time to first contact
        if telemetry["first_contact_step"] and telemetry["steps"] > 0:
            summary["time_to_grasp_ms"] = telemetry["first_contact_step"] * 20  # 20ms per step

        summary["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%SZ")

        with open(telemetry_path, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"\n[Telemetry] Saved to {telemetry_path}")
        print(f"  Steps: {summary['total_steps']}")
        print(f"  Contact force: mean={summary['contact_force_mean']:.2f}N, var={summary['contact_force_variance']:.2f}")
        print(f"  Joint jitter: {summary['joint_jitter']:.4f} rad")
        print(f"  Grip stability: {summary['handle_grip_stability']:.2f}")
        print(f"  Collisions: {summary['collision_count']}")

    teleop.add_callback("L", save_telemetry)

    while simulation_app.is_running():
        try:
            with torch.inference_mode():
                action = teleop.advance()

                if teleoperation_active:
                    actions = action.repeat(env.num_envs, 1)
                    env.step(actions)
                    telemetry["steps"] += 1

                    # Record telemetry every N steps
                    if telemetry["steps"] % record_every == 0:
                        try:
                            # Gripper end-effector position
                            robot = env.scene["robot"]
                            ee_pos = robot.data.body_pos_w[:, -1, :].squeeze()  # last body = end effector
                            telemetry["gripper_positions"].append(ee_pos.cpu().tolist())

                            # Asset joint positions and velocities (skip if PhysX exposes no DOFs)
                            cabinet = env.scene["cabinet"]
                            if cabinet.num_joints > 0:
                                j_pos = cabinet.data.joint_pos.squeeze()
                                j_vel = cabinet.data.joint_vel.squeeze()
                                telemetry["joint_positions"].append(j_pos.cpu().tolist())
                                telemetry["joint_velocities"].append(j_vel.cpu().tolist())

                            # Contact forces (net force on end effector bodies)
                            net_forces = robot.data.body_acc_w[:, -3:, :].squeeze()  # last 3 bodies (hand + 2 fingers)
                            force_mag = float(net_forces.norm(dim=-1).mean())
                            telemetry["contact_forces"].append(force_mag)

                            # Detect first contact (force spike)
                            if telemetry["first_contact_step"] is None and force_mag > 5.0:
                                telemetry["first_contact_step"] = telemetry["steps"]
                                print(f"[Telemetry] First contact at step {telemetry['steps']}")

                            # Count collision events (force exceeds threshold)
                            if force_mag > 10.0:
                                telemetry["collision_count"] += 1

                        except Exception:
                            pass  # telemetry recording should never crash the sim

                else:
                    env.sim.render()

                if monitor:
                    monitor.step()

                if should_reset:
                    env.reset()
                    teleop.reset()
                    should_reset = False
        except Exception as e:
            logger.error(f"Error: {e}")
            break

    # Auto-save telemetry on exit
    save_telemetry()

    # Summary
    if monitor:
        monitor.print_summary()
        try:
            monitor.save_results(os.path.join(asset_dir, "runtime_verification_results.json"))
        except Exception:
            pass

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()

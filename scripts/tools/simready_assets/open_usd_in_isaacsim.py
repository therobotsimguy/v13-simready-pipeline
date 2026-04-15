#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Open a USD in Isaac Sim with the **same minimal Kit session** as ``load_in_isaacsim.py``.

Do **not** use :class:`isaaclab.app.AppLauncher` here: that loads the full Isaac Lab stack and
the viewport/session can look like a different app than a plain Sim open.

.. code-block:: bash

    cd ~/IsaacLab
    ./isaaclab.sh -p scripts/tools/simready_assets/open_usd_in_isaacsim.py \\
        --usd scripts/tools/simready_assets/cabinet_2_simready_out/teak_outdoor_sideboard_physics.usd

The V7 ``cabinet_2`` run produces a **teak outdoor sideboard** (wide sideboard with doors/drawers
from ``stage_c.json``), not the separate Blender-MCP primitive “cabinet” experiments.
"""

import argparse
import os

parser = argparse.ArgumentParser(description="Open a USD in Isaac Sim (minimal Kit, same as load_in_isaacsim).")
parser.add_argument(
    "--usd",
    type=str,
    required=True,
    help="Path to the .usd file (absolute or relative to cwd).",
)
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
args = parser.parse_args()

usd_path = os.path.abspath(os.path.expanduser(args.usd))
if not os.path.isfile(usd_path):
    raise FileNotFoundError(f"USD not found: {usd_path}")

from isaacsim import SimulationApp

app = SimulationApp({"headless": False, "width": args.width, "height": args.height})

import omni.usd
from pxr import Gf, UsdGeom

print(f"Opening: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
app.update()

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

try:
    import omni.kit.viewport.utility as vp_util

    viewport = vp_util.get_active_viewport()
    if viewport:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        camera_state = ViewportCameraState(viewport.viewport_api)
        camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.8), True)
        camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, 0.5), True)
except Exception as e:
    print(f"Camera setup skipped: {e}")

app.update()
print("Stage loaded. Close the window to exit.")

while app.is_running():
    app.update()

app.close()

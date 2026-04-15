#!/usr/bin/env python3
"""
export_urdf.py — Export SimReady physics USD to URDF + OBJ meshes.

Produces a URDF file and mesh directory that works with MuJoCo, PyBullet,
Drake, Pinocchio, and any URDF-compatible simulator.

Usage:
  python3 export_urdf.py --input /path/to/asset_physics.usd
  python3 export_urdf.py --input /path/to/asset_physics.usd --output-dir /path/to/output

Output:
  {output_dir}/
  ├── {asset_name}.urdf
  └── {asset_name}_meshes/
      ├── mesh1.obj
      ├── mesh2.obj
      └── ...
"""

import argparse
import glob
import os
import shutil
import sys
from pathlib import Path


def export_urdf(usd_path: str, output_dir: str = None, verbose: bool = True) -> str:
    """Export physics USD to URDF + meshes. Returns URDF path."""
    from nvidia.srl.from_usd.to_urdf import UsdToUrdf

    usd_path = str(Path(usd_path).resolve())
    asset_name = Path(usd_path).stem.replace("_physics", "")

    # Default output: next to the USD
    if not output_dir:
        output_dir = str(Path(usd_path).parent)

    os.makedirs(output_dir, exist_ok=True)

    # Convert
    if verbose:
        print(f"\n  URDF Export")
        print(f"  Input:  {usd_path}")
        print(f"  Output: {output_dir}/")
        print(f"  {'─' * 50}")
        print(f"  Converting USD → URDF...")

    # Export to temp first, then organize
    import tempfile
    with tempfile.TemporaryDirectory(prefix="urdf_export_") as tmpdir:
        tmp_urdf = os.path.join(tmpdir, "robot.urdf")
        converter = UsdToUrdf.init_from_file(usd_path)
        converter.save_to_file(tmp_urdf)

        # Organize meshes into named directory
        meshes_dir = os.path.join(output_dir, f"{asset_name}_meshes")
        os.makedirs(meshes_dir, exist_ok=True)

        # Copy OBJ + MTL files
        tmp_meshes = os.path.join(tmpdir, "meshes")
        mesh_count = 0
        if os.path.exists(tmp_meshes):
            for f in glob.glob(os.path.join(tmp_meshes, "*.obj")):
                shutil.copy2(f, meshes_dir)
                mesh_count += 1
            for f in glob.glob(os.path.join(tmp_meshes, "*.mtl")):
                shutil.copy2(f, meshes_dir)

        # Update URDF mesh paths to point to the named meshes directory
        with open(tmp_urdf) as f:
            urdf_text = f.read()
        urdf_text = urdf_text.replace('filename="meshes/', f'filename="{asset_name}_meshes/')

        # Write final URDF
        urdf_path = os.path.join(output_dir, f"{asset_name}.urdf")
        with open(urdf_path, 'w') as f:
            f.write(urdf_text)

    if verbose:
        # Parse joint info for summary
        import xml.etree.ElementTree as ET
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        n_links = len(root.findall("link"))
        n_joints = len(root.findall("joint"))

        print(f"  URDF: {urdf_path}")
        print(f"  Meshes: {mesh_count} OBJ files in {asset_name}_meshes/")
        print(f"  Links: {n_links}, Joints: {n_joints}")
        print(f"\n  Compatible with: MuJoCo, PyBullet, Drake, Pinocchio, ROS")

        for j in root.findall("joint"):
            jname = j.attrib["name"]
            jtype = j.attrib["type"]
            limit = j.find("limit")
            if limit is not None:
                lo = limit.attrib.get("lower", "0")
                hi = limit.attrib.get("upper", "0")
                print(f"    {jtype:10s} {jname}  [{lo}, {hi}]")

    return urdf_path


def main():
    ap = argparse.ArgumentParser(description="Export SimReady USD to URDF + meshes")
    ap.add_argument("--input", required=True, help="Path to _physics.usd")
    ap.add_argument("--output-dir", default=None, help="Output directory (default: next to USD)")
    args = ap.parse_args()
    export_urdf(args.input, output_dir=args.output_dir)


if __name__ == "__main__":
    main()

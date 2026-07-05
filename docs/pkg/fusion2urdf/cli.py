"""fusion2urdf CLI.

The Fusion 360 add-in exports a self-contained bundle::

    <robot>_export/
        robot.json     intermediate robot description
        meshes/*.stl   one binary STL per link (mm units)

This CLI turns that bundle (or any robot.json) into the final artifacts:

    fusion2urdf build <export_dir> --targets urdf,ros2,usd -o out/
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .intermediate import load_robot_json
from .mjcf_writer import build_mjcf
from .ros2_package import generate_ros2_package
from .urdf_writer import build_urdf
from .usd_export import (
    convert_mjcf_to_usd,
    convert_urdf_to_usd,
    write_asset_transformer_files,
    write_isaac_import_script,
)

ALL_TARGETS = ("urdf", "ros2", "mjcf", "usd", "usd-newton")


def _find_robot_json(path: Path) -> Path:
    if path.is_file():
        return path
    candidate = path / "robot.json"
    if candidate.exists():
        return candidate
    raise SystemExit(f"error: no robot.json found in {path}")


def cmd_build(args: argparse.Namespace) -> int:
    export_dir = Path(args.export_dir)
    robot_json = _find_robot_json(export_dir)
    meshes_dir = robot_json.parent / "meshes"
    robot = load_robot_json(robot_json)
    robot.validate()

    targets = [t.strip() for t in args.targets.split(",") if t.strip()]
    unknown = set(targets) - set(ALL_TARGETS)
    if unknown:
        raise SystemExit(f"error: unknown targets {sorted(unknown)}; valid: {ALL_TARGETS}")

    out_root = Path(args.output or (robot_json.parent / "out"))
    out_root.mkdir(parents=True, exist_ok=True)
    produced: list[str] = []

    urdf_dir = out_root / "urdf"
    urdf_file = urdf_dir / f"{robot.name}.urdf"
    if "urdf" in targets or "usd" in targets:
        urdf_dir.mkdir(parents=True, exist_ok=True)
        urdf_text = build_urdf(robot, mesh_uri_template="meshes/{name}.stl")
        urdf_file.write_text(urdf_text, encoding="utf-8")
        _copy_meshes(robot, meshes_dir, urdf_dir / "meshes")
        write_isaac_import_script(
            robot.name,
            urdf_dir,
            fix_base=args.fix_base,
            robot_type=args.robot_type,
        )
        produced.append(str(urdf_file))

    if "ros2" in targets:
        pkg = generate_ros2_package(
            robot,
            out_root / "ros2",
            meshes_dir=meshes_dir,
            package_name=args.package_name,
        )
        produced.append(str(pkg))

    mjcf_dir = out_root / "mjcf"
    mjcf_file = mjcf_dir / f"{robot.name}.xml"
    if "mjcf" in targets or "usd-newton" in targets:
        mjcf_dir.mkdir(parents=True, exist_ok=True)
        mjcf_file.write_text(
            build_mjcf(robot, floating_base=not args.fix_base), encoding="utf-8"
        )
        _copy_meshes(robot, meshes_dir, mjcf_dir / "meshes")
        produced.append(str(mjcf_file))

    if "usd" in targets:
        usd_dir = out_root / "usd"
        try:
            usd_file = convert_urdf_to_usd(urdf_file, usd_dir)
            write_asset_transformer_files(robot.name, usd_dir)
            produced.append(str(usd_file))
        except ImportError as exc:
            print(f"warning: USD target skipped: {exc}", file=sys.stderr)

    if "usd-newton" in targets:
        usd_dir = out_root / "usd_newton"
        try:
            usd_file = convert_mjcf_to_usd(mjcf_file, usd_dir)
            write_asset_transformer_files(robot.name, usd_dir)
            produced.append(str(usd_file))
        except ImportError as exc:
            print(f"warning: usd-newton target skipped: {exc}", file=sys.stderr)

    for p in produced:
        print(p)
    return 0


def _copy_meshes(robot, meshes_dir: Path, dest: Path) -> None:
    import shutil

    if not meshes_dir.exists():
        return
    dest.mkdir(parents=True, exist_ok=True)
    for link in robot.links:
        if not link.mesh:
            continue
        src = meshes_dir / link.mesh
        if src.exists():
            shutil.copy2(src, dest / src.name)


def cmd_validate(args: argparse.Namespace) -> int:
    robot_json = _find_robot_json(Path(args.export_dir))
    robot = load_robot_json(robot_json)
    robot.validate()
    print(f"OK: {robot.name} ({len(robot.links)} links, {len(robot.joints)} joints)")
    return 0


def cmd_info(args: argparse.Namespace) -> int:
    robot_json = _find_robot_json(Path(args.export_dir))
    robot = load_robot_json(robot_json)
    info = {
        "name": robot.name,
        "base_link": robot.base_link,
        "links": [l.name for l in robot.links],
        "joints": [
            {"name": j.name, "type": j.type, "parent": j.parent, "child": j.child}
            for j in robot.joints
        ],
        "total_mass_kg": round(
            sum(l.inertial.mass for l in robot.links if l.inertial), 6
        ),
    }
    print(json.dumps(info, indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="fusion2urdf",
        description="Convert a Fusion 360 export bundle to URDF / ROS 2 / OpenUSD",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_build = sub.add_parser("build", help="generate output artifacts")
    p_build.add_argument("export_dir", help="export bundle dir or robot.json path")
    p_build.add_argument(
        "--targets",
        default="urdf,ros2,mjcf,usd,usd-newton",
        help=f"comma-separated targets: {','.join(ALL_TARGETS)} (default: all)",
    )
    p_build.add_argument("-o", "--output", help="output root dir (default: <export>/out)")
    p_build.add_argument("--package-name", help="ROS 2 package name override")
    p_build.add_argument(
        "--fix-base",
        action="store_true",
        default=True,
        help="fix base to world in the Isaac import script (default: true)",
    )
    p_build.add_argument(
        "--no-fix-base", dest="fix_base", action="store_false",
        help="mobile robots: do not fix the base",
    )
    p_build.add_argument(
        "--robot-type",
        default="Manipulator",
        help="Isaac robot schema type (Manipulator, Wheeled, Quadruped, ...)",
    )
    p_build.set_defaults(func=cmd_build)

    p_val = sub.add_parser("validate", help="validate a robot.json")
    p_val.add_argument("export_dir")
    p_val.set_defaults(func=cmd_validate)

    p_info = sub.add_parser("info", help="print robot summary as JSON")
    p_info.add_argument("export_dir")
    p_info.set_defaults(func=cmd_info)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())

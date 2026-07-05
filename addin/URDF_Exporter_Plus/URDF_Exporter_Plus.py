# Author: Johnny Nunez (builds on syuntoku14, SpaceMaster85, Lentin Joseph)
# Description: Export the active design to URDF / ROS 2 package / OpenUSD bundle
"""URDF Exporter Plus - Fusion 360 script entry point.

Exports a self-contained bundle:

    <robot>_export/
        robot.json          intermediate robot description (schema v1)
        meshes/*.stl        one binary STL per link (mm)
        out/urdf/           plain URDF for Isaac Sim / urdf-usd-converter
        out/ros2/           ROS 2 <robot>_description package (Jazzy, gz-sim)

USD generation needs pip deps, so it runs on the desktop afterwards:

    pip install fusion2urdf[usd]
    fusion2urdf build <robot>_export --targets usd
"""

import os
import sys
import traceback

import adsk
import adsk.core
import adsk.fusion

# Make the vendored fusion2urdf core importable inside Fusion's Python.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fusion2urdf.intermediate import save_robot_json  # noqa: E402
from fusion2urdf.ros2_package import generate_ros2_package  # noqa: E402
from fusion2urdf.urdf_writer import build_urdf  # noqa: E402
from fusion2urdf.usd_export import write_isaac_import_script  # noqa: E402
from fusion_helpers import extractor, stl_export  # noqa: E402

TITLE = "URDF Exporter Plus"


def run(context):  # noqa: ARG001 - Fusion API signature
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design.", TITLE)
            return

        welcome = (
            "Export the active design to URDF, a ROS 2 description package "
            "and an Isaac Sim-ready bundle.\n\n"
            "Requirements:\n"
            "- one component named 'base_link'\n"
            "- joints of type Rigid, Revolute or Slider\n"
            "- revolute/slider joints need both limits (or none for continuous)\n\n"
            "Press OK to choose the output folder."
        )
        if ui.messageBox(
            welcome, TITLE, adsk.core.MessageBoxButtonTypes.OKCancelButtonType
        ) != adsk.core.DialogResults.DialogOK:
            return

        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = "Choose export folder"
        if folder_dialog.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        base_dir = folder_dialog.folder

        # 1. Extract the robot model from the design.
        robot = extractor.extract_robot(design)

        export_dir = os.path.join(base_dir, f"{robot.name}_export")
        os.makedirs(export_dir, exist_ok=True)

        # 2. Intermediate JSON + meshes.
        save_robot_json(robot, os.path.join(export_dir, "robot.json"))
        stl_export.export_link_meshes(app, export_dir)

        # 3. Plain URDF (Isaac Sim / urdf-usd-converter ready).
        out_urdf = os.path.join(export_dir, "out", "urdf")
        os.makedirs(out_urdf, exist_ok=True)
        urdf_text = build_urdf(robot, mesh_uri_template="meshes/{name}.stl")
        with open(
            os.path.join(out_urdf, f"{robot.name}.urdf"), "w", encoding="utf-8"
        ) as f:
            f.write(urdf_text)
        _copy_meshes(export_dir, out_urdf, robot)
        write_isaac_import_script(robot.name, out_urdf)

        # 4. ROS 2 description package.
        generate_ros2_package(
            robot,
            os.path.join(export_dir, "out", "ros2"),
            meshes_dir=os.path.join(export_dir, "meshes"),
        )

        ui.messageBox(
            "Export finished:\n\n"
            f"{export_dir}\n\n"
            f"- out/urdf/{robot.name}.urdf  (Isaac Sim ready)\n"
            f"- out/ros2/{robot.name}_description  (ROS 2 Jazzy package)\n\n"
            "For OpenUSD run on your desktop:\n"
            f"  fusion2urdf build \"{export_dir}\" --targets usd",
            TITLE,
        )

    except Exception:  # noqa: BLE001 - surface everything to the user
        if ui:
            ui.messageBox(f"Failed:\n{traceback.format_exc()}", TITLE)


def _copy_meshes(export_dir: str, dest_dir: str, robot) -> None:
    import shutil

    src_dir = os.path.join(export_dir, "meshes")
    mesh_dest = os.path.join(dest_dir, "meshes")
    os.makedirs(mesh_dest, exist_ok=True)
    for link in robot.links:
        if not link.mesh:
            continue
        src = os.path.join(src_dir, link.mesh)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(mesh_dest, link.mesh))

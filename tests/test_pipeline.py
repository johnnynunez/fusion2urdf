"""End-to-end pipeline tests: CLI build, ROS 2 package, xacro expansion,
URDF parsing with yourdfpy, and real USD conversion."""

import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from fusion2urdf.cli import main as cli_main


def test_cli_validate(export_bundle):
    assert cli_main(["validate", str(export_bundle)]) == 0


def test_cli_build_urdf_and_ros2(export_bundle, capsys):
    rc = cli_main(["build", str(export_bundle), "--targets", "urdf,ros2"])
    assert rc == 0

    out = export_bundle / "out"
    urdf = out / "urdf" / "twodof.urdf"
    assert urdf.exists()
    assert (out / "urdf" / "meshes" / "link1.stl").exists()
    assert (out / "urdf" / "isaac_sim_import.py").exists()

    pkg = out / "ros2" / "twodof_description"
    for rel in (
        "package.xml",
        "CMakeLists.txt",
        "urdf/twodof.xacro",
        "urdf/materials.xacro",
        "urdf/twodof.ros2control.xacro",
        "launch/display.launch.py",
        "launch/gazebo.launch.py",
        "rviz/display.rviz",
        "config/ros_gz_bridge.yaml",
        "meshes/base_link.stl",
    ):
        assert (pkg / rel).exists(), f"missing {rel}"

    # package.xml is valid XML with the right name
    tree = ET.parse(pkg / "package.xml")
    assert tree.findtext("name") == "twodof_description"

    # launch files compile as python
    for launch in ("display.launch.py", "gazebo.launch.py"):
        compile((pkg / "launch" / launch).read_text(), launch, "exec")


def test_urdf_parses_with_yourdfpy(export_bundle):
    yourdfpy = pytest.importorskip("yourdfpy")
    cli_main(["build", str(export_bundle), "--targets", "urdf"])
    urdf_file = export_bundle / "out" / "urdf" / "twodof.urdf"
    robot = yourdfpy.URDF.load(str(urdf_file), load_meshes=True)
    assert robot.num_actuated_joints == 2
    assert set(robot.link_map) == {"base_link", "link1", "link2"}
    # FK at zero config: link2 frame must be at world z=0.3
    import numpy as np

    T = robot.get_transform("link2", "base_link")
    assert np.allclose(T[:3, 3], [0.0, 0.0, 0.3], atol=1e-9)


def test_xacro_expands(export_bundle, tmp_path):
    xacro = pytest.importorskip("xacro")
    cli_main(["build", str(export_bundle), "--targets", "ros2"])
    xacro_file = (
        export_bundle / "out" / "ros2" / "twodof_description" / "urdf" / "twodof.xacro"
    )
    # Resolve $(find ...) without a ROS install: point it at the package dir.
    pkg_dir = xacro_file.parent.parent
    text = xacro_file.read_text().replace(
        "$(find twodof_description)", str(pkg_dir)
    )
    patched = tmp_path / "patched.xacro"
    patched.write_text(text)
    doc = xacro.process_file(str(patched))
    root = ET.fromstring(doc.toxml())
    assert len(root.findall("link")) == 3
    assert len(root.findall("joint")) == 2
    assert {m.get("name") for m in root.findall("material")} >= {"grey", "orange"}


def test_usd_conversion(export_bundle):
    pytest.importorskip("urdf_usd_converter")
    from pxr import Usd, UsdPhysics

    rc = cli_main(["build", str(export_bundle), "--targets", "urdf,usd"])
    assert rc == 0

    usd_dir = export_bundle / "out" / "usd"
    usd_files = list(usd_dir.rglob("*.usd*"))
    assert usd_files, f"no USD produced in {usd_dir}"

    main_layer = usd_dir / "twodof.usda"
    if not main_layer.exists():
        main_layer = usd_files[0]
    stage = Usd.Stage.Open(str(main_layer))
    assert stage is not None

    prim_names = {p.GetName() for p in stage.Traverse()}
    assert {"base_link", "link1", "link2"} <= prim_names

    # physics joints present
    joints = [
        p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)
    ]
    assert len(joints) == 2

    # Asset Transformer companion files emitted
    assert (usd_dir / "asset_transformer_profile.json").exists()
    assert (usd_dir / "run_asset_transformer.py").exists()
    import json

    profile = json.loads((usd_dir / "asset_transformer_profile.json").read_text())
    assert profile["version"] == "1.0"
    assert any(r["type"].endswith("RobotSchemaRule") for r in profile["rules"])
    compile(
        (usd_dir / "run_asset_transformer.py").read_text(),
        "run_asset_transformer.py",
        "exec",
    )


def test_usd_newton_conversion(export_bundle):
    """MJCF -> USD via newton-physics mujoco-usd-converter, asserting the
    Newton/MuJoCo (Mjc*) physics attributes made it into the stage."""
    pytest.importorskip("mujoco_usd_converter")
    from pxr import Usd, UsdPhysics

    rc = cli_main(["build", str(export_bundle), "--targets", "mjcf,usd-newton"])
    assert rc == 0

    usd_dir = export_bundle / "out" / "usd_newton"
    main_layer = usd_dir / "twodof.usda"
    if not main_layer.exists():
        candidates = list(usd_dir.glob("*.usda"))
        assert candidates, f"no USD produced in {usd_dir}"
        main_layer = candidates[0]

    stage = Usd.Stage.Open(str(main_layer))
    assert stage is not None

    prim_names = {p.GetName() for p in stage.Traverse()}
    assert {"base_link", "link1", "link2"} <= prim_names

    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    assert len(joints) == 2

    # Mjc* (MuJoCo/Newton schema) attributes present somewhere in the stage
    has_mjc = any(
        attr.GetName().startswith("mjc:")
        for p in stage.Traverse()
        for attr in p.GetAttributes()
    )
    assert has_mjc, "expected mjc:* attributes from mujoco-usd-converter"

    assert (usd_dir / "asset_transformer_profile.json").exists()


ISAAC_MODELS = Path(
    "/home/spark/Projects/isaac/IsaacSim/source/extensions/"
    "isaacsim.asset.transformer/isaacsim/asset/transformer/models.py"
)


@pytest.mark.skipif(not ISAAC_MODELS.exists(), reason="Isaac Sim source not present")
def test_transformer_profile_parses_with_real_ruleprofile(export_bundle):
    """Validate the emitted profile against Isaac Sim's actual RuleProfile
    parser (models.py is pure Python; loaded standalone, no Kit needed)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("at_models", ISAAC_MODELS)
    models = importlib.util.module_from_spec(spec)
    sys.modules["at_models"] = models  # dataclasses resolves annotations here
    try:
        spec.loader.exec_module(models)
    finally:
        sys.modules.pop("at_models", None)

    from fusion2urdf.usd_export import write_asset_transformer_files

    profile_path, script_path = write_asset_transformer_files(
        "twodof", export_bundle / "at"
    )
    profile = models.RuleProfile.from_json(profile_path.read_text())
    assert profile.profile_name == "fusion2urdf Isaac Sim Structure"
    assert len(profile.rules) == 10
    assert all(
        r.type.startswith("isaacsim.asset.transformer.rules.") for r in profile.rules
    )
    # round-trip through the real serializer
    again = models.RuleProfile.from_json(profile.to_json())
    assert [r.name for r in again.rules] == [r.name for r in profile.rules]
    compile(script_path.read_text(), "run_asset_transformer.py", "exec")


def test_cli_entrypoint_subprocess(export_bundle):
    """Run the installed console script end to end."""
    exe = shutil.which("fusion2urdf")
    if exe is None:
        pytest.skip("fusion2urdf entry point not on PATH")
    result = subprocess.run(
        [exe, "info", str(export_bundle)],
        capture_output=True, text=True, check=True,
    )
    assert '"name": "twodof"' in result.stdout

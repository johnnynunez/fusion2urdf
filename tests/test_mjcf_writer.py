"""MJCF writer tests: structure and validation with the real MuJoCo parser."""

import xml.etree.ElementTree as ET

import pytest

from fusion2urdf.cli import main as cli_main
from fusion2urdf.mjcf_writer import build_mjcf


def _parse(text: str) -> ET.Element:
    return ET.fromstring(text)


def test_mjcf_structure(two_dof_robot):
    root = _parse(build_mjcf(two_dof_robot))
    assert root.tag == "mujoco"
    assert root.get("model") == "twodof"
    compiler = root.find("compiler")
    assert compiler.get("angle") == "radian"
    assert compiler.get("meshdir") == "meshes"
    assert len(root.findall("asset/mesh")) == 3


def test_mjcf_body_tree_and_positions(two_dof_robot):
    root = _parse(build_mjcf(two_dof_robot))
    base = root.find("worldbody/body")
    assert base.get("name") == "base_link"
    link1 = base.find("body")
    assert link1.get("name") == "link1"
    assert [float(v) for v in link1.get("pos").split()] == pytest.approx([0, 0, 0.1])
    link2 = link1.find("body")
    assert link2.get("name") == "link2"
    # relative to link1 (world 0.3 - 0.1)
    assert [float(v) for v in link2.get("pos").split()] == pytest.approx([0, 0, 0.2])


def test_mjcf_joints(two_dof_robot):
    root = _parse(build_mjcf(two_dof_robot))
    j1 = root.find("worldbody/body/body/joint")
    assert j1.get("name") == "joint1"
    assert j1.get("type") == "hinge"
    assert [float(v) for v in j1.get("axis").split()] == pytest.approx([0, 0, 1])
    lo, hi = (float(v) for v in j1.get("range").split())
    assert lo == pytest.approx(-3.141593, abs=1e-5)
    assert hi == pytest.approx(3.141593, abs=1e-5)


def test_mjcf_fullinertia_order(two_dof_robot):
    # model stores [xx yy zz xy yz xz]; MJCF wants [xx yy zz xy xz yz]
    link1 = two_dof_robot.link_map()["link1"]
    link1.inertial.inertia = [1.0, 2.0, 3.0, 0.004, 0.005, 0.006]
    root = _parse(build_mjcf(two_dof_robot))
    inertial = root.find("worldbody/body/body/inertial")
    vals = [float(v) for v in inertial.get("fullinertia").split()]
    assert vals == pytest.approx([1.0, 2.0, 3.0, 0.004, 0.006, 0.005])


def test_mjcf_floating_base(two_dof_robot):
    root = _parse(build_mjcf(two_dof_robot, floating_base=True))
    base = root.find("worldbody/body")
    assert base.find("freejoint") is not None
    # fixed base: no freejoint
    root = _parse(build_mjcf(two_dof_robot, floating_base=False))
    assert root.find("worldbody/body").find("freejoint") is None


def test_mjcf_loads_in_real_mujoco(export_bundle):
    """The generated MJCF must compile in MuJoCo itself (real physics check)."""
    mujoco = pytest.importorskip("mujoco")
    cli_main(["build", str(export_bundle), "--targets", "mjcf"])
    mjcf_file = export_bundle / "out" / "mjcf" / "twodof.xml"
    model = mujoco.MjModel.from_xml_path(str(mjcf_file))
    assert model.njnt == 2
    assert model.nbody == 4  # world + 3 links
    # masses survived the trip
    import numpy as np

    body_masses = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): model.body_mass[i]
        for i in range(model.nbody)
    }
    assert body_masses["base_link"] == pytest.approx(1.0)
    assert body_masses["link1"] == pytest.approx(0.5)
    assert body_masses["link2"] == pytest.approx(0.25)

    # FK at qpos=0: link2 frame at world z=0.3
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    link2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link2")
    assert np.allclose(data.xpos[link2_id], [0, 0, 0.3], atol=1e-9)

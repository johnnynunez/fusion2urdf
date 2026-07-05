"""URDF writer tests: structure, kinematics math, and parser validation."""

import xml.etree.ElementTree as ET

import pytest

from fusion2urdf.urdf_writer import (
    build_materials_xacro,
    build_ros2_control_xacro,
    build_urdf,
    build_xacro_main,
)


def _parse(urdf_text: str) -> ET.Element:
    return ET.fromstring(urdf_text)


def test_urdf_is_valid_xml(two_dof_robot):
    root = _parse(build_urdf(two_dof_robot))
    assert root.tag == "robot"
    assert root.get("name") == "twodof"


def test_urdf_link_and_joint_count(two_dof_robot):
    root = _parse(build_urdf(two_dof_robot))
    assert len(root.findall("link")) == 3
    assert len(root.findall("joint")) == 2
    assert len(root.findall("material")) == 2


def test_joint_origin_is_relative_to_parent(two_dof_robot):
    root = _parse(build_urdf(two_dof_robot))
    joints = {j.get("name"): j for j in root.findall("joint")}
    # joint1 anchor at world z=0.1, parent (base) at origin -> origin z=0.1
    xyz1 = joints["joint1"].find("origin").get("xyz").split()
    assert [float(v) for v in xyz1] == pytest.approx([0.0, 0.0, 0.1])
    # joint2 anchor at world z=0.3, parent link1 frame at z=0.1 -> dz=0.2
    xyz2 = joints["joint2"].find("origin").get("xyz").split()
    assert [float(v) for v in xyz2] == pytest.approx([0.0, 0.0, 0.2])


def test_visual_origin_compensates_world_mesh(two_dof_robot):
    root = _parse(build_urdf(two_dof_robot))
    links = {l.get("name"): l for l in root.findall("link")}
    # link1 frame at world z=0.1; mesh baked in world coords -> shift -0.1
    xyz = links["link1"].find("visual/origin").get("xyz").split()
    assert [float(v) for v in xyz] == pytest.approx([0.0, 0.0, -0.1])


def test_inertial_com_is_link_local(two_dof_robot):
    root = _parse(build_urdf(two_dof_robot))
    links = {l.get("name"): l for l in root.findall("link")}
    # link1 COM at world z=0.2, frame at z=0.1 -> local z=0.1
    xyz = links["link1"].find("inertial/origin").get("xyz").split()
    assert [float(v) for v in xyz] == pytest.approx([0.0, 0.0, 0.1])
    mass = float(links["link1"].find("inertial/mass").get("value"))
    assert mass == pytest.approx(0.5)


def test_limits_written(two_dof_robot):
    root = _parse(build_urdf(two_dof_robot))
    j1 = next(j for j in root.findall("joint") if j.get("name") == "joint1")
    limit = j1.find("limit")
    assert float(limit.get("lower")) == pytest.approx(-3.141593, abs=1e-5)
    assert float(limit.get("upper")) == pytest.approx(3.141593, abs=1e-5)
    assert float(limit.get("effort")) > 0
    assert float(limit.get("velocity")) > 0


def test_mesh_uri_template(two_dof_robot):
    urdf = build_urdf(
        two_dof_robot,
        mesh_uri_template="package://twodof_description/meshes/{name}.stl",
    )
    root = _parse(urdf)
    mesh = root.find("link/visual/geometry/mesh")
    assert mesh.get("filename").startswith("package://twodof_description/")
    assert mesh.get("scale") == "0.001 0.001 0.001"


def test_fixed_joint_has_no_axis_or_limit(two_dof_robot):
    from fusion2urdf.model import Inertial, Joint, Link

    two_dof_robot.links.append(
        Link(
            name="tool",
            world_xyz=[0, 0, 0.45],
            inertial=Inertial(mass=0.01, center_of_mass=[0, 0, 0.45],
                              inertia=[1e-6] * 3 + [0.0] * 3),
        )
    )
    two_dof_robot.joints.append(
        Joint(name="tool_joint", type="fixed", parent="link2", child="tool",
              world_xyz=[0, 0, 0.45])
    )
    root = _parse(build_urdf(two_dof_robot))
    tj = next(j for j in root.findall("joint") if j.get("name") == "tool_joint")
    assert tj.find("axis") is None
    assert tj.find("limit") is None


def test_xacro_main_includes_materials(two_dof_robot):
    text = build_xacro_main(two_dof_robot, "twodof_description")
    assert 'xmlns:xacro="http://www.ros.org/wiki/xacro"' in text
    assert "$(find twodof_description)/urdf/materials.xacro" in text
    root = ET.fromstring(
        text.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', "")
        .replace("xacro:include", "include")
    )
    assert len(root.findall("link")) == 3


def test_materials_xacro(two_dof_robot):
    root = ET.fromstring(
        build_materials_xacro(two_dof_robot)
        .replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', "")
    )
    names = {m.get("name") for m in root.findall("material")}
    assert names == {"grey", "orange"}


def test_ros2_control_xacro(two_dof_robot):
    root = ET.fromstring(
        build_ros2_control_xacro(two_dof_robot)
        .replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', "")
    )
    joints = root.findall("ros2_control/joint")
    assert {j.get("name") for j in joints} == {"joint1", "joint2"}
    for j in joints:
        assert j.find("command_interface").get("name") == "position"

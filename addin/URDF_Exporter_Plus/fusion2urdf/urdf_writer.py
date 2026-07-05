"""URDF / xacro writers.

Two flavors are produced from the same Robot model:

* ``ros2``  - xacro-based ``*_description`` package layout with
  ``package://<pkg>/meshes/...`` mesh URIs (rviz2, Gazebo Sim, ros2_control).
* ``isaac`` - a single plain ``robot.urdf`` with mesh paths relative to the
  URDF file. Isaac Sim's ``URDFImporter`` and NVIDIA's
  ``urdf-usd-converter`` both resolve relative paths without needing a ROS
  workspace, so this file is directly importable.

Joint origins follow the classic fusion2urdf math: every link frame sits at
its parent joint's world anchor (base_link at the world origin), so a joint
origin is ``child_anchor - parent_anchor`` and mesh/COM data (captured in
world coordinates) is shifted by the link's world position.
"""

from __future__ import annotations

from typing import Dict, List
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

from .math3d import round_list, vec_sub
from .model import Joint, Link, Robot

_ND = 6  # rounding digits


def _fmt(values) -> str:
    out = []
    for v in values:
        r = round(float(v), _ND)
        if r == int(r):
            out.append(str(int(r)))
        else:
            out.append(repr(r))
    return " ".join(out)


def _pretty(elem: Element) -> str:
    raw = tostring(elem, "unicode")
    return minidom.parseString(raw).toprettyxml(indent="  ")


def link_world_positions(robot: Robot) -> Dict[str, List[float]]:
    """World position of every link frame: base at origin, others at their
    parent joint anchor."""
    pos: Dict[str, List[float]] = {robot.base_link: [0.0, 0.0, 0.0]}
    for j in robot.joints:
        pos[j.child] = list(j.world_xyz)
    return pos


def _link_element(
    link: Link,
    world_xyz: List[float],
    mesh_uri: str | None,
    with_material: bool,
) -> Element:
    el = Element("link", {"name": link.name})

    if link.inertial:
        inertial = SubElement(el, "inertial")
        com_local = vec_sub(link.inertial.center_of_mass, world_xyz)
        SubElement(inertial, "origin", {"xyz": _fmt(com_local), "rpy": "0 0 0"})
        SubElement(inertial, "mass", {"value": repr(round(link.inertial.mass, 9))})
        ixx, iyy, izz, ixy, iyz, ixz = round_list(link.inertial.inertia, 9)
        SubElement(
            inertial,
            "inertia",
            {
                "ixx": repr(ixx), "iyy": repr(iyy), "izz": repr(izz),
                "ixy": repr(ixy), "iyz": repr(iyz), "ixz": repr(ixz),
            },
        )

    if mesh_uri:
        vis_origin = {"xyz": _fmt([-v for v in world_xyz]), "rpy": "0 0 0"}
        mesh_attrib = {"filename": mesh_uri, "scale": _fmt(link.mesh_scale)}

        visual = SubElement(el, "visual")
        SubElement(visual, "origin", dict(vis_origin))
        geom_v = SubElement(visual, "geometry")
        SubElement(geom_v, "mesh", dict(mesh_attrib))
        if with_material and link.material:
            SubElement(visual, "material", {"name": link.material})

        collision = SubElement(el, "collision")
        SubElement(collision, "origin", dict(vis_origin))
        geom_c = SubElement(collision, "geometry")
        SubElement(geom_c, "mesh", dict(mesh_attrib))

    return el


def _joint_element(
    joint: Joint, positions: Dict[str, List[float]]
) -> Element:
    el = Element("joint", {"name": joint.name, "type": joint.type})
    origin = vec_sub(positions[joint.child], positions[joint.parent])
    SubElement(el, "origin", {"xyz": _fmt(origin), "rpy": "0 0 0"})
    SubElement(el, "parent", {"link": joint.parent})
    SubElement(el, "child", {"link": joint.child})
    if joint.type in ("revolute", "continuous", "prismatic", "planar"):
        SubElement(el, "axis", {"xyz": _fmt(joint.axis)})
    if joint.type in ("revolute", "prismatic"):
        SubElement(
            el,
            "limit",
            {
                "lower": repr(round(joint.lower, _ND)),
                "upper": repr(round(joint.upper, _ND)),
                "effort": repr(round(joint.effort, _ND)),
                "velocity": repr(round(joint.velocity, _ND)),
            },
        )
    return el


def build_urdf(
    robot: Robot,
    mesh_uri_template: str = "meshes/{name}.stl",
    with_materials: bool = True,
) -> str:
    """Render the Robot model to a self-contained plain URDF string.

    ``mesh_uri_template`` receives ``{name}`` (the link name); use e.g.
    ``package://my_robot_description/meshes/{name}.stl`` for ROS or
    ``meshes/{name}.stl`` for Isaac Sim / urdf-usd-converter.
    """
    robot.validate()
    positions = link_world_positions(robot)

    root = Element("robot", {"name": robot.name})

    if with_materials:
        for mat in robot.materials.values():
            m = SubElement(root, "material", {"name": mat.name})
            SubElement(m, "color", {"rgba": _fmt(mat.rgba)})

    for link in robot.links:
        mesh_uri = (
            mesh_uri_template.format(name=link.name) if link.mesh else None
        )
        root.append(
            _link_element(link, positions[link.name], mesh_uri, with_materials)
        )

    for joint in robot.joints:
        root.append(_joint_element(joint, positions))

    return _pretty(root)


def build_xacro_main(robot: Robot, package_name: str) -> str:
    """Top-level .xacro that includes materials and wraps the URDF body."""
    body = build_urdf(
        robot,
        mesh_uri_template=(
            "package://" + package_name + "/meshes/{name}.stl"
        ),
        with_materials=False,
    )
    # Strip the XML declaration and <robot> open tag; re-wrap with xacro ns.
    lines = body.splitlines()
    assert lines[0].startswith("<?xml")
    assert lines[1].startswith("<robot")
    inner = "\n".join(lines[2:-1])
    header = (
        '<?xml version="1.0" ?>\n'
        f'<robot name="{robot.name}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n\n'
        f'  <xacro:include filename="$(find {package_name})/urdf/materials.xacro" />\n'
    )
    return header + "\n" + inner + "\n</robot>\n"


def build_materials_xacro(robot: Robot) -> str:
    root = Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    mats = dict(robot.materials)
    if not mats:
        from .model import Material

        mats["silver"] = Material(name="silver", rgba=[0.7, 0.7, 0.7, 1.0])
    for mat in mats.values():
        m = SubElement(root, "material", {"name": mat.name})
        SubElement(m, "color", {"rgba": _fmt(mat.rgba)})
    return _pretty(root)


def build_ros2_control_xacro(
    robot: Robot, plugin: str = "gz_ros2_control/GazeboSimSystem"
) -> str:
    """ros2_control hardware description for the actuated joints."""
    root = Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    r2c = SubElement(root, "ros2_control", {"name": "GazeboSimSystem", "type": "system"})
    hw = SubElement(r2c, "hardware")
    SubElement(hw, "plugin").text = plugin
    for j in robot.joints:
        if j.type in ("fixed", "floating"):
            continue
        je = SubElement(r2c, "joint", {"name": j.name})
        ci = SubElement(je, "command_interface", {"name": "position"})
        if j.type in ("revolute", "prismatic"):
            mn = SubElement(ci, "param", {"name": "min"})
            mn.text = repr(round(j.lower, _ND))
            mx = SubElement(ci, "param", {"name": "max"})
            mx.text = repr(round(j.upper, _ND))
        SubElement(je, "state_interface", {"name": "position"})
        SubElement(je, "state_interface", {"name": "velocity"})
        SubElement(je, "state_interface", {"name": "effort"})
    return _pretty(root)

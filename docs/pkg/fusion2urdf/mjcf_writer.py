"""MJCF (MuJoCo XML) writer.

Renders the same Robot model to MJCF so assets can go through NVIDIA's
``mujoco-usd-converter`` (newton-physics) and carry MuJoCo/Newton physics
attributes (Mjc*) into USD, complementing the URDF -> UsdPhysics path.

Frame conventions map 1:1 from the shared model:

* body frame     = parent joint anchor (base at world origin)
* body pos       = child_anchor - parent_anchor  (same as URDF joint origin)
* joint          = declared in the child body at its frame origin
* geom/mesh pos  = -world_xyz (meshes are baked in world coordinates)
* inertial pos   = COM local to the body frame

MuJoCo's ``fullinertia`` order is [ixx iyy izz ixy ixz iyz]; the model stores
[ixx iyy izz ixy iyz ixz] (Fusion / URDF order), so the last two swap.
"""

from __future__ import annotations

from typing import Dict, List
from xml.etree.ElementTree import Element, SubElement

from .model import Robot
from .urdf_writer import _fmt, _pretty, link_world_positions

_MJCF_JOINT = {"revolute": "hinge", "continuous": "hinge", "prismatic": "slide"}


def build_mjcf(
    robot: Robot,
    mesh_dir: str = "meshes",
    floating_base: bool = False,
) -> str:
    """Render the Robot model to an MJCF XML string.

    ``floating_base=True`` adds a ``<freejoint/>`` to the base body (mobile
    robots); otherwise the base is welded to the world.
    """
    robot.validate()
    positions = link_world_positions(robot)
    links = robot.link_map()

    children: Dict[str, List] = {}
    for j in robot.joints:
        children.setdefault(j.parent, []).append(j)

    root = Element("mujoco", {"model": robot.name})
    SubElement(
        root,
        "compiler",
        {
            "angle": "radian",
            "meshdir": mesh_dir,
            "autolimits": "true",
            "balanceinertia": "true",
        },
    )

    asset = SubElement(root, "asset")
    for link in robot.links:
        if link.mesh:
            SubElement(
                asset,
                "mesh",
                {
                    "name": link.name,
                    "file": link.mesh,
                    "scale": _fmt(link.mesh_scale),
                },
            )

    worldbody = SubElement(root, "worldbody")

    def emit_body(parent_el: Element, link_name: str, joint=None) -> None:
        link = links[link_name]
        world = positions[link_name]
        parent_world = positions[joint.parent] if joint else [0.0, 0.0, 0.0]
        pos = [w - p for w, p in zip(world, parent_world)]

        body = SubElement(parent_el, "body", {"name": link_name, "pos": _fmt(pos)})

        if joint is None and floating_base:
            SubElement(body, "freejoint", {"name": f"{link_name}_free"})
        elif joint is not None and joint.type in _MJCF_JOINT:
            attrs = {
                "name": joint.name,
                "type": _MJCF_JOINT[joint.type],
                "axis": _fmt(joint.axis),
            }
            if joint.type != "continuous":
                attrs["range"] = _fmt([joint.lower, joint.upper])
            SubElement(body, "joint", attrs)
        # fixed joints: no joint element -> welded to parent

        if link.inertial:
            i = link.inertial.inertia
            SubElement(
                body,
                "inertial",
                {
                    "pos": _fmt(
                        [c - w for c, w in zip(link.inertial.center_of_mass, world)]
                    ),
                    "mass": repr(round(link.inertial.mass, 9)),
                    # model order [xx yy zz xy yz xz] -> MJCF [xx yy zz xy xz yz]
                    "fullinertia": _fmt([i[0], i[1], i[2], i[3], i[5], i[4]]),
                },
            )

        if link.mesh:
            geom = {"type": "mesh", "mesh": link.name, "pos": _fmt([-v for v in world])}
            mat = robot.materials.get(link.material or "")
            if mat:
                geom["rgba"] = _fmt(mat.rgba)
            SubElement(body, "geom", geom)

        for child_joint in children.get(link_name, []):
            emit_body(body, child_joint.child, child_joint)

    emit_body(worldbody, robot.base_link)
    return _pretty(root)

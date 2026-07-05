"""Shared fixtures: a synthetic 2-DOF arm with real STL meshes."""

from __future__ import annotations

import math
import struct
from pathlib import Path

import pytest

from fusion2urdf.model import Inertial, Joint, Link, Material, Robot


def write_box_stl(path: Path, sx: float, sy: float, sz: float,
                  cx: float = 0.0, cy: float = 0.0, cz: float = 0.0) -> None:
    """Write a binary STL box (dimensions/center in mm, matching Fusion's
    mm STL export convention)."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = [
        (cx - hx, cy - hy, cz - hz), (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz), (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz), (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz), (cx - hx, cy + hy, cz + hz),
    ]
    quads = [
        (0, 3, 2, 1, (0, 0, -1)), (4, 5, 6, 7, (0, 0, 1)),
        (0, 1, 5, 4, (0, -1, 0)), (2, 3, 7, 6, (0, 1, 0)),
        (0, 4, 7, 3, (-1, 0, 0)), (1, 2, 6, 5, (1, 0, 0)),
    ]
    tris = []
    for a, b, c, d, n in quads:
        tris.append((n, v[a], v[b], v[c]))
        tris.append((n, v[a], v[c], v[d]))
    with open(path, "wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", len(tris)))
        for n, p1, p2, p3 in tris:
            f.write(struct.pack("<3f", *n))
            for p in (p1, p2, p3):
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


def box_inertia(mass: float, sx: float, sy: float, sz: float) -> list[float]:
    """Solid box inertia about its COM (SI units)."""
    return [
        mass / 12.0 * (sy ** 2 + sz ** 2),
        mass / 12.0 * (sx ** 2 + sz ** 2),
        mass / 12.0 * (sx ** 2 + sy ** 2),
        0.0, 0.0, 0.0,
    ]


@pytest.fixture
def two_dof_robot() -> Robot:
    """2-DOF arm. base(0.1m cube at origin), link1 (0.2m long box, joint at
    z=0.1), link2 (0.15m long box, joint at z=0.3)."""
    base = Link(
        name="base_link",
        world_xyz=[0.0, 0.0, 0.0],
        inertial=Inertial(
            mass=1.0,
            center_of_mass=[0.0, 0.0, 0.05],
            inertia=box_inertia(1.0, 0.1, 0.1, 0.1),
        ),
        mesh="base_link.stl",
        material="grey",
    )
    link1 = Link(
        name="link1",
        world_xyz=[0.0, 0.0, 0.1],
        inertial=Inertial(
            mass=0.5,
            center_of_mass=[0.0, 0.0, 0.2],  # world coords
            inertia=box_inertia(0.5, 0.04, 0.04, 0.2),
        ),
        mesh="link1.stl",
        material="orange",
    )
    link2 = Link(
        name="link2",
        world_xyz=[0.0, 0.0, 0.3],
        inertial=Inertial(
            mass=0.25,
            center_of_mass=[0.0, 0.0, 0.375],
            inertia=box_inertia(0.25, 0.03, 0.03, 0.15),
        ),
        mesh="link2.stl",
        material="orange",
    )
    j1 = Joint(
        name="joint1", type="revolute", parent="base_link", child="link1",
        world_xyz=[0.0, 0.0, 0.1], axis=[0.0, 0.0, 1.0],
        lower=-math.pi, upper=math.pi,
    )
    j2 = Joint(
        name="joint2", type="revolute", parent="link1", child="link2",
        world_xyz=[0.0, 0.0, 0.3], axis=[0.0, 1.0, 0.0],
        lower=-math.pi / 2, upper=math.pi / 2,
    )
    return Robot(
        name="twodof",
        links=[base, link1, link2],
        joints=[j1, j2],
        materials={
            "grey": Material(name="grey", rgba=[0.5, 0.5, 0.5, 1.0]),
            "orange": Material(name="orange", rgba=[1.0, 0.42, 0.04, 1.0]),
        },
    )


@pytest.fixture
def export_bundle(tmp_path: Path, two_dof_robot: Robot) -> Path:
    """A full export bundle: robot.json + real STL meshes (mm units)."""
    from fusion2urdf.intermediate import save_robot_json

    bundle = tmp_path / "twodof_export"
    meshes = bundle / "meshes"
    meshes.mkdir(parents=True)
    save_robot_json(two_dof_robot, bundle / "robot.json")
    # Meshes baked in world coordinates (mm), like Fusion exports them.
    write_box_stl(meshes / "base_link.stl", 100, 100, 100, cz=50)
    write_box_stl(meshes / "link1.stl", 40, 40, 200, cz=200)
    write_box_stl(meshes / "link2.stl", 30, 30, 150, cz=375)
    return bundle

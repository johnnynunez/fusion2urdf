"""Robot description data model (stdlib only).

This is the intermediate representation shared by the Fusion 360 add-in and
the desktop CLI. All values use SI units and URDF conventions:

* lengths in meters, angles in radians, mass in kg, inertia in kg*m^2
* every link frame is world-aligned at the design pose (rpy supported but the
  Fusion extractor emits translation-only frames, the proven fusion2urdf
  approach: meshes are baked in world coordinates and shifted by the link
  origin)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

JOINT_TYPES = ("fixed", "revolute", "continuous", "prismatic", "floating", "planar")


@dataclass
class Origin:
    xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def to_dict(self) -> dict:
        return {"xyz": list(self.xyz), "rpy": list(self.rpy)}

    @classmethod
    def from_dict(cls, d: dict) -> "Origin":
        return cls(xyz=list(d.get("xyz", [0, 0, 0])), rpy=list(d.get("rpy", [0, 0, 0])))


@dataclass
class Inertial:
    mass: float
    center_of_mass: List[float]
    # [ixx, iyy, izz, ixy, iyz, ixz] about the COM, world-aligned axes
    inertia: List[float]

    def to_dict(self) -> dict:
        return {
            "mass": self.mass,
            "center_of_mass": list(self.center_of_mass),
            "inertia": list(self.inertia),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Inertial":
        return cls(
            mass=float(d["mass"]),
            center_of_mass=list(d["center_of_mass"]),
            inertia=list(d["inertia"]),
        )


@dataclass
class Material:
    name: str
    rgba: List[float] = field(default_factory=lambda: [0.7, 0.7, 0.7, 1.0])

    def to_dict(self) -> dict:
        return {"name": self.name, "rgba": list(self.rgba)}

    @classmethod
    def from_dict(cls, d: dict) -> "Material":
        return cls(name=d["name"], rgba=list(d.get("rgba", [0.7, 0.7, 0.7, 1.0])))


@dataclass
class Link:
    name: str
    # Link frame position in world coordinates at the design pose.
    world_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    inertial: Optional[Inertial] = None
    # Mesh filename relative to the meshes/ dir (e.g. "base_link.stl").
    mesh: Optional[str] = None
    # Scale applied to the mesh (Fusion STL exports are in mm -> 0.001).
    mesh_scale: List[float] = field(default_factory=lambda: [0.001, 0.001, 0.001])
    material: Optional[str] = None

    @property
    def visual_origin(self) -> Origin:
        """Mesh is baked in world coords; shift it back by the link origin."""
        return Origin(xyz=[-v for v in self.world_xyz])

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "world_xyz": list(self.world_xyz),
            "inertial": self.inertial.to_dict() if self.inertial else None,
            "mesh": self.mesh,
            "mesh_scale": list(self.mesh_scale),
            "material": self.material,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Link":
        return cls(
            name=d["name"],
            world_xyz=list(d.get("world_xyz", [0, 0, 0])),
            inertial=Inertial.from_dict(d["inertial"]) if d.get("inertial") else None,
            mesh=d.get("mesh"),
            mesh_scale=list(d.get("mesh_scale", [0.001, 0.001, 0.001])),
            material=d.get("material"),
        )


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    # Joint anchor position in world coordinates at the design pose.
    world_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Axis in the child link frame (world-aligned frames -> world axis).
    axis: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 100.0
    velocity: float = 100.0

    def __post_init__(self) -> None:
        if self.type not in JOINT_TYPES:
            raise ValueError(f"unknown joint type {self.type!r} for joint {self.name!r}")

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "type": self.type,
            "parent": self.parent,
            "child": self.child,
            "world_xyz": list(self.world_xyz),
            "axis": list(self.axis),
            "lower": self.lower,
            "upper": self.upper,
            "effort": self.effort,
            "velocity": self.velocity,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Joint":
        return cls(
            name=d["name"],
            type=d["type"],
            parent=d["parent"],
            child=d["child"],
            world_xyz=list(d.get("world_xyz", [0, 0, 0])),
            axis=list(d.get("axis", [0, 0, 1])),
            lower=float(d.get("lower", 0.0)),
            upper=float(d.get("upper", 0.0)),
            effort=float(d.get("effort", 100.0)),
            velocity=float(d.get("velocity", 100.0)),
        )


@dataclass
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)
    materials: Dict[str, Material] = field(default_factory=dict)
    base_link: str = "base_link"

    def link_map(self) -> Dict[str, Link]:
        return {l.name: l for l in self.links}

    def validate(self) -> None:
        names = [l.name for l in self.links]
        if len(names) != len(set(names)):
            raise ValueError("duplicate link names")
        if self.base_link not in names:
            raise ValueError(
                f"base link {self.base_link!r} not found; "
                "name one Fusion component 'base_link'"
            )
        link_set = set(names)
        children = set()
        for j in self.joints:
            if j.parent not in link_set:
                raise ValueError(f"joint {j.name!r}: unknown parent link {j.parent!r}")
            if j.child not in link_set:
                raise ValueError(f"joint {j.name!r}: unknown child link {j.child!r}")
            if j.child in children:
                raise ValueError(f"link {j.child!r} has more than one parent joint")
            children.add(j.child)
        orphans = link_set - children - {self.base_link}
        if orphans:
            raise ValueError(
                f"links not connected by any joint: {sorted(orphans)}"
            )

    def to_dict(self) -> dict:
        return {
            "schema": 1,
            "name": self.name,
            "base_link": self.base_link,
            "links": [l.to_dict() for l in self.links],
            "joints": [j.to_dict() for j in self.joints],
            "materials": {k: v.to_dict() for k, v in self.materials.items()},
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Robot":
        robot = cls(
            name=d["name"],
            base_link=d.get("base_link", "base_link"),
            links=[Link.from_dict(x) for x in d.get("links", [])],
            joints=[Joint.from_dict(x) for x in d.get("joints", [])],
            materials={
                k: Material.from_dict(v) for k, v in d.get("materials", {}).items()
            },
        )
        return robot

"""Fusion 360 design extractor.

Builds a fusion2urdf Robot model from the active Fusion 360 design. Runs
inside Fusion's embedded Python (imports ``adsk``). The math and conventions
follow the field-proven syuntoku14 / SpaceMaster85 exporters, updated for
current Fusion API behavior:

* joints are collected from the root component and all sub-components
* joint origins are transformed to world coordinates through the occurrence
  chain (nested assemblies supported)
* light-bulb visibility controls what gets exported
* internal Fusion units are cm / kg -> converted to m / kg / kg*m^2
"""

from __future__ import annotations

import re
import traceback

import adsk
import adsk.core
import adsk.fusion

from fusion2urdf.math3d import inertia_about_com
from fusion2urdf.model import Inertial, Joint, Link, Material, Robot

CM_TO_M = 0.01
KGCM2_TO_KGM2 = 1e-4

# URDF names for adsk.fusion.JointTypes enum indices (0..6).
FUSION_JOINT_TYPES = (
    "fixed",       # RigidJointType
    "revolute",    # RevoluteJointType
    "prismatic",   # SliderJointType
    "cylindrical",  # unsupported
    "pin_slot",    # unsupported
    "planar",      # unsupported
    "ball",        # unsupported
)

_SANITIZE = re.compile(r"[ :()<>]")


def clean_name(name: str) -> str:
    return _SANITIZE.sub("_", name)


def is_base_link(name: str) -> bool:
    return "base_link" in name


def top_level_occurrence(occ):
    """Walk up assembly contexts to the top-level occurrence of a joint side."""
    while occ.assemblyContext is not None:
        occ = occ.assemblyContext
    return occ


def occurrence_link_name(occ) -> str:
    if is_base_link(occ.name) or is_base_link(occ.component.name):
        return "base_link"
    return clean_name(occ.name)


def joint_origin_world(joint) -> list:
    """Joint anchor in world coordinates (cm), robust to nested assemblies.

    Transforms the joint geometry origin (component context) through the
    occurrence transform chain, per the Autodesk forum reference used by all
    fusion2urdf forks.
    """

    def chain_matrix(occ):
        mat = adsk.core.Matrix3D.create()
        occ = adsk.fusion.Occurrence.cast(occ)
        if not occ:
            return mat
        occs = []
        cur = occ
        while cur is not None:
            occs.append(cur)
            cur = cur.assemblyContext
        for o in occs:
            mat.transformBy(o.transform2 if hasattr(o, "transform2") else o.transform)
        return mat

    geometry = joint.geometryOrOriginTwo
    if isinstance(geometry, adsk.fusion.JointOrigin):
        origin = geometry.geometry.origin.copy()
    else:
        origin = geometry.origin.copy()

    occ = joint.occurrenceTwo
    if occ is not None and occ.assemblyContext is not None:
        origin.transformBy(chain_matrix(occ))
    return list(origin.asArray())


def collect_joints(root) -> list:
    """All light-bulb-enabled joints from root and nested components."""
    seen = set()
    joints = []
    containers = [root] + [occ.component for occ in root.allOccurrences]
    for comp in containers:
        for joint in comp.joints:
            token = joint.entityToken
            if token in seen:
                continue
            seen.add(token)
            joints.append(joint)
    return joints


def extract_joints(root) -> list[Joint]:
    result = []
    for joint in collect_joints(root):
        if not joint.isLightBulbOn:
            continue
        if joint.occurrenceOne is None or joint.occurrenceTwo is None:
            continue
        if not joint.occurrenceOne.isLightBulbOn:
            continue

        motion = joint.jointMotion
        jtype = FUSION_JOINT_TYPES[motion.jointType]
        if jtype not in ("fixed", "revolute", "prismatic"):
            raise ValueError(
                f"joint '{joint.name}': type '{jtype}' is not supported "
                "(use Rigid, Revolute or Slider)"
            )

        axis = [0.0, 0.0, 1.0]
        lower = upper = 0.0

        if jtype == "revolute":
            axis = [round(v, 6) for v in motion.rotationAxisVector.asArray()]
            limits = motion.rotationLimits
            if limits.isMaximumValueEnabled and limits.isMinimumValueEnabled:
                upper = round(limits.maximumValue, 6)  # rad
                lower = round(limits.minimumValue, 6)
            elif limits.isMaximumValueEnabled != limits.isMinimumValueEnabled:
                raise ValueError(
                    f"joint '{joint.name}': set BOTH rotation limits or neither"
                )
            else:
                jtype = "continuous"
        elif jtype == "prismatic":
            axis = [round(v, 6) for v in motion.slideDirectionVector.asArray()]
            limits = motion.slideLimits
            if limits.isMaximumValueEnabled and limits.isMinimumValueEnabled:
                upper = round(limits.maximumValue * CM_TO_M, 6)
                lower = round(limits.minimumValue * CM_TO_M, 6)
            else:
                raise ValueError(
                    f"joint '{joint.name}': prismatic joints need both slide limits"
                )

        parent_occ = top_level_occurrence(joint.occurrenceTwo)
        child_occ = top_level_occurrence(joint.occurrenceOne)

        world_xyz = [round(v * CM_TO_M, 6) for v in joint_origin_world(joint)]

        result.append(
            Joint(
                name=clean_name(joint.name),
                type=jtype,
                parent=occurrence_link_name(parent_occ),
                child=occurrence_link_name(child_occ),
                world_xyz=world_xyz,
                axis=axis,
                lower=lower,
                upper=upper,
            )
        )
    return result


def extract_inertials(root) -> dict[str, Inertial]:
    """Physical properties per top-level occurrence (very high accuracy)."""
    inertials = {}
    for occ in root.occurrences:
        if not occ.isLightBulbOn:
            continue
        props = occ.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        )
        mass = props.mass  # kg
        com = [v * CM_TO_M for v in props.centerOfMass.asArray()]
        (_, xx, yy, zz, xy, yz, xz) = props.getXYZMomentsOfInertia()
        inertia_world = [v * KGCM2_TO_KGM2 for v in (xx, yy, zz, xy, yz, xz)]
        inertia_com = [
            round(v, 9) for v in inertia_about_com(inertia_world, com, mass)
        ]
        inertials[occurrence_link_name(occ)] = Inertial(
            mass=round(mass, 9),
            center_of_mass=[round(v, 6) for v in com],
            inertia=inertia_com,
        )
    return inertials


def _first_color(occ):
    """Depth-first search for an appearance color on an occurrence."""
    def color_of(appearance):
        if not appearance:
            return None
        for prop in appearance.appearanceProperties:
            if isinstance(prop, adsk.core.ColorProperty) and prop.value:
                return appearance.name, prop.value
        return None

    found = color_of(occ.appearance)
    if found:
        return found
    for body in occ.bRepBodies:
        found = color_of(body.appearance)
        if found:
            return found
    try:
        if occ.component.material:
            found = color_of(occ.component.material.appearance)
            if found:
                return found
    except Exception:
        pass
    for child in occ.childOccurrences:
        found = _first_color(child)
        if found:
            return found
    return None


def extract_materials(root) -> tuple[dict[str, str], dict[str, Material]]:
    """Returns (link->material_name, material_name->Material)."""
    link_materials: dict[str, str] = {}
    materials: dict[str, Material] = {
        "silver_default": Material(name="silver_default", rgba=[0.7, 0.7, 0.7, 1.0])
    }
    for occ in root.occurrences:
        if not occ.isLightBulbOn:
            continue
        link = occurrence_link_name(occ)
        mat_name = "silver_default"
        try:
            found = _first_color(occ)
            if found:
                raw_name, color = found
                mat_name = clean_name(
                    re.sub(r"\s+", " ", re.sub(r"[^A-Za-z0-9 ]", "", raw_name)).strip()
                ).lower() or "silver_default"
                materials[mat_name] = Material(
                    name=mat_name,
                    rgba=[
                        round(color.red / 255.0, 4),
                        round(color.green / 255.0, 4),
                        round(color.blue / 255.0, 4),
                        round(color.opacity / 255.0, 4),
                    ],
                )
        except Exception:
            print("material extraction failed:\n" + traceback.format_exc())
        link_materials[link] = mat_name
    return link_materials, materials


def extract_robot(design) -> Robot:
    """Build the full Robot model from the active design."""
    root = design.rootComponent
    robot_name = clean_name(root.name.split()[0]).lower()

    joints = extract_joints(root)
    inertials = extract_inertials(root)
    link_materials, materials = extract_materials(root)

    link_positions = {name: [0.0, 0.0, 0.0] for name in inertials}
    for j in joints:
        link_positions[j.child] = list(j.world_xyz)

    links = []
    for name, inertial in inertials.items():
        links.append(
            Link(
                name=name,
                world_xyz=link_positions.get(name, [0.0, 0.0, 0.0]),
                inertial=inertial,
                mesh=f"{name}.stl",
                material=link_materials.get(name),
            )
        )

    robot = Robot(name=robot_name, links=links, joints=joints, materials=materials)
    robot.validate()
    return robot

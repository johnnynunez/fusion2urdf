"""Small 3D math helpers (stdlib only).

Conventions
-----------
* Lengths in meters, mass in kg, inertia in kg*m^2.
* Rotation matrices are row-major 3x3 nested lists.
* RPY is the URDF fixed-axis XYZ convention (roll about X, pitch about Y,
  yaw about Z, applied in that order in the fixed frame).
"""

from __future__ import annotations

import math
from typing import List, Sequence

Vec3 = List[float]
Mat3 = List[List[float]]


def identity3() -> Mat3:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def mat_vec(a: Mat3, v: Sequence[float]) -> Vec3:
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(a: Mat3) -> Mat3:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rpy_to_matrix(rpy: Sequence[float]) -> Mat3:
    """URDF rpy -> rotation matrix R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matrix_to_rpy(m: Mat3) -> Vec3:
    """Rotation matrix -> URDF rpy (inverse of rpy_to_matrix)."""
    sp = -m[2][0]
    sp = max(-1.0, min(1.0, sp))
    p = math.asin(sp)
    if abs(abs(sp) - 1.0) < 1e-9:  # gimbal lock
        r = math.atan2(-m[0][1], m[1][1])
        y = 0.0
    else:
        r = math.atan2(m[2][1], m[2][2])
        y = math.atan2(m[1][0], m[0][0])
    return [r, p, y]


def vec_sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [a[i] - b[i] for i in range(3)]


def vec_norm(a: Sequence[float]) -> float:
    return math.sqrt(sum(x * x for x in a))


def vec_normalize(a: Sequence[float]) -> Vec3:
    n = vec_norm(a)
    if n < 1e-12:
        raise ValueError("cannot normalize zero-length vector")
    return [x / n for x in a]


def inertia_about_com(
    inertia_world: Sequence[float],
    center_of_mass: Sequence[float],
    mass: float,
) -> Vec3:
    """Translate an inertia tensor about the origin to one about the COM.

    Parameters
    ----------
    inertia_world:
        [ixx, iyy, izz, ixy, iyz, ixz] tensor components expressed about the
        world origin (Fusion's ``getXYZMomentsOfInertia`` convention, i.e.
        off-diagonal entries already carry the tensor sign).
    center_of_mass:
        [x, y, z] of the COM in the same frame.
    mass:
        mass in kg.

    Returns
    -------
    [ixx, iyy, izz, ixy, iyz, ixz] about the COM (parallel axis theorem,
    inverse direction). This matches the math used and field-validated by the
    syuntoku14 / SpaceMaster85 exporters.
    """
    x, y, z = center_of_mass
    translation = [
        y ** 2 + z ** 2,
        x ** 2 + z ** 2,
        x ** 2 + y ** 2,
        -x * y,
        -y * z,
        -x * z,
    ]
    return [i - mass * t for i, t in zip(inertia_world, translation)]


def symmetric_eigenvalues(inertia: Sequence[float]) -> Vec3:
    """Eigenvalues of a symmetric 3x3 tensor [ixx, iyy, izz, ixy, iyz, ixz].

    Closed-form trigonometric solution (no numpy). Returns the principal
    moments sorted ascending. Used to check inertia plausibility: physical
    principal moments are positive and satisfy the triangle inequality
    regardless of the frame the tensor was expressed in.
    """
    ixx, iyy, izz, ixy, iyz, ixz = (float(v) for v in inertia)
    p1 = ixy * ixy + iyz * iyz + ixz * ixz
    if p1 < 1e-30:
        return sorted([ixx, iyy, izz])
    q = (ixx + iyy + izz) / 3.0
    p2 = (ixx - q) ** 2 + (iyy - q) ** 2 + (izz - q) ** 2 + 2.0 * p1
    p = math.sqrt(p2 / 6.0)
    b11, b22, b33 = (ixx - q) / p, (iyy - q) / p, (izz - q) / p
    b12, b23, b13 = ixy / p, iyz / p, ixz / p
    det_b = (
        b11 * (b22 * b33 - b23 * b23)
        - b12 * (b12 * b33 - b23 * b13)
        + b13 * (b12 * b23 - b22 * b13)
    )
    r = max(-1.0, min(1.0, det_b / 2.0))
    phi = math.acos(r) / 3.0
    e1 = q + 2.0 * p * math.cos(phi)
    e3 = q + 2.0 * p * math.cos(phi + 2.0 * math.pi / 3.0)
    return sorted([e1, 3.0 * q - e1 - e3, e3])


def round_list(values: Sequence[float], ndigits: int = 6) -> Vec3:
    return [round(v, ndigits) for v in values]

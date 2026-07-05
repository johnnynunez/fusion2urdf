"""Mass properties from triangle meshes (stdlib only).

Computes volume, center of mass and the inertia tensor of a closed triangle
mesh via signed-tetrahedron integration (Eberly, "Polyhedral Mass
Properties"). Used to estimate link inertials from geometry when the source
format carries no physics data (e.g. STEP imports in the web app), assuming
uniform density.

Supports binary and ASCII STL. All outputs follow the model conventions:
meters, kg, inertia [ixx, iyy, izz, ixy, iyz, ixz] about the COM with
world-aligned axes.
"""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Iterable, List, Tuple

from .model import Inertial, Robot

Triangle = Tuple[Tuple[float, float, float], ...]


def read_stl(data: bytes) -> List[Triangle]:
    """Parse STL bytes (binary or ASCII) into a triangle list."""
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if 84 + 50 * count == len(data):
            tris = []
            for i in range(count):
                v = struct.unpack_from("<12f", data, 84 + 50 * i)
                tris.append((v[3:6], v[6:9], v[9:12]))
            return tris
    # ASCII fallback
    tris, verts = [], []
    for line in data.decode("ascii", errors="ignore").splitlines():
        parts = line.split()
        if parts[:1] == ["vertex"]:
            verts.append(tuple(float(x) for x in parts[1:4]))
            if len(verts) == 3:
                tris.append(tuple(verts))
                verts = []
    if not tris:
        raise ValueError("not a valid STL file")
    return tris


def mass_properties(
    triangles: Iterable[Triangle],
    density: float = 1000.0,
    scale: float = 1.0,
) -> Inertial:
    """Integrate mass properties of a closed mesh with uniform density.

    ``scale`` converts mesh units to meters (Fusion STLs are mm -> 0.001);
    ``density`` is in kg/m^3. Handles inward-facing normals by sign.
    """
    intg = [0.0] * 10  # 1, x, y, z, x^2, y^2, z^2, xy, yz, zx

    def sub(w0: float, w1: float, w2: float):
        t0 = w0 + w1
        f1 = t0 + w2
        t1 = w0 * w0
        t2 = t1 + w1 * t0
        f2 = t2 + w2 * f1
        f3 = w0 * t1 + w1 * t2 + w2 * f2
        return f1, f2, f3, f2 + w0 * (f1 + w0), f2 + w1 * (f1 + w1), f2 + w2 * (f1 + w2)

    for tri in triangles:
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = (
            tuple(c * scale for c in v) for v in tri
        )
        a1, b1, c1 = x1 - x0, y1 - y0, z1 - z0
        a2, b2, c2 = x2 - x0, y2 - y0, z2 - z0
        d0, d1, d2 = b1 * c2 - b2 * c1, a2 * c1 - a1 * c2, a1 * b2 - a2 * b1

        fx1, fx2, fx3, gx0, gx1, gx2 = sub(x0, x1, x2)
        fy1, fy2, fy3, gy0, gy1, gy2 = sub(y0, y1, y2)
        fz1, fz2, fz3, gz0, gz1, gz2 = sub(z0, z1, z2)

        intg[0] += d0 * fx1
        intg[1] += d0 * fx2
        intg[2] += d1 * fy2
        intg[3] += d2 * fz2
        intg[4] += d0 * fx3
        intg[5] += d1 * fy3
        intg[6] += d2 * fz3
        intg[7] += d0 * (y0 * gx0 + y1 * gx1 + y2 * gx2)
        intg[8] += d1 * (z0 * gy0 + z1 * gy1 + z2 * gy2)
        intg[9] += d2 * (x0 * gz0 + x1 * gz1 + x2 * gz2)

    for i, k in enumerate((6, 24, 24, 24, 60, 60, 60, 120, 120, 120)):
        intg[i] /= k

    volume = intg[0]
    if abs(volume) < 1e-12:
        raise ValueError("mesh has zero volume (open or degenerate)")
    sign = 1.0 if volume > 0 else -1.0  # inward normals flip every integral
    volume *= sign

    cx, cy, cz = (sign * intg[i] / volume for i in (1, 2, 3))
    mass = density * volume
    ixx = sign * density * (intg[5] + intg[6]) - mass * (cy * cy + cz * cz)
    iyy = sign * density * (intg[4] + intg[6]) - mass * (cx * cx + cz * cz)
    izz = sign * density * (intg[4] + intg[5]) - mass * (cx * cx + cy * cy)
    ixy = -(sign * density * intg[7] - mass * cx * cy)
    iyz = -(sign * density * intg[8] - mass * cy * cz)
    ixz = -(sign * density * intg[9] - mass * cz * cx)

    return Inertial(
        mass=mass,
        center_of_mass=[cx, cy, cz],
        inertia=[ixx, iyy, izz, ixy, iyz, ixz],
    )


def ensure_inertials(robot: Robot, meshes_dir: str | Path) -> List[str]:
    """Fill missing link inertials from mesh geometry (uniform density).

    Links with ``inertial=None``, a mesh and a ``density`` get their mass
    properties computed from the STL. Returns the names of computed links.
    """
    meshes_dir = Path(meshes_dir)
    computed = []
    for link in robot.links:
        if link.inertial is not None or not link.mesh or not link.density:
            continue
        tris = read_stl((meshes_dir / link.mesh).read_bytes())
        link.inertial = mass_properties(
            tris, density=link.density, scale=link.mesh_scale[0]
        )
        computed.append(link.name)
    return computed

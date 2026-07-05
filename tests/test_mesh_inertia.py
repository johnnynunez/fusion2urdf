"""Mesh mass-properties tests, validated against closed-form solids."""

import math
import struct

import pytest

from fusion2urdf.mesh_inertia import mass_properties, read_stl
from fusion2urdf.model import Link, Robot


def box_triangles(sx, sy, sz, offset=(0, 0, 0)):
    """12 triangles of an axis-aligned box centered at offset."""
    ox, oy, oz = offset
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = [
        (ox + x, oy + y, oz + z)
        for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)
    ]
    # outward-facing quads (indices into v), split into triangles
    quads = [
        (0, 1, 3, 2), (4, 6, 7, 5),  # x- x+
        (0, 4, 5, 1), (2, 3, 7, 6),  # y- y+
        (0, 2, 6, 4), (1, 5, 7, 3),  # z- z+
    ]
    tris = []
    for a, b, c, d in quads:
        tris.append((v[a], v[b], v[c]))
        tris.append((v[a], v[c], v[d]))
    return tris


def test_box_mass_properties():
    """1x2x3 m box, density 1000: closed-form mass/COM/inertia."""
    props = mass_properties(box_triangles(1, 2, 3, offset=(0.5, 0, 1)), density=1000)
    m = 1000 * 1 * 2 * 3
    assert props.mass == pytest.approx(m)
    assert props.center_of_mass == pytest.approx([0.5, 0, 1])
    assert props.inertia[0] == pytest.approx(m / 12 * (2**2 + 3**2))  # ixx
    assert props.inertia[1] == pytest.approx(m / 12 * (1**2 + 3**2))  # iyy
    assert props.inertia[2] == pytest.approx(m / 12 * (1**2 + 2**2))  # izz
    assert props.inertia[3:] == pytest.approx([0, 0, 0], abs=1e-9)


def test_box_inverted_normals():
    """Inward-facing normals must not flip the sign of the results."""
    tris = [(a, c, b) for a, b, c in box_triangles(1, 1, 1)]
    props = mass_properties(tris, density=1000)
    assert props.mass == pytest.approx(1000)
    assert props.inertia[0] == pytest.approx(1000 / 6)


def test_mm_scale():
    """A 100 mm cube with scale=0.001 == a 0.1 m cube."""
    props = mass_properties(box_triangles(100, 100, 100), density=1000, scale=0.001)
    assert props.mass == pytest.approx(1000 * 0.1**3)


def test_read_stl_binary_and_ascii():
    tris = box_triangles(1, 1, 1)
    binary = b"\0" * 80 + struct.pack("<I", len(tris))
    for t in tris:
        binary += struct.pack("<12f", 0, 0, 0, *t[0], *t[1], *t[2]) + b"\0\0"
    assert len(read_stl(binary)) == 12

    ascii_stl = "solid box\n" + "".join(
        "facet normal 0 0 0\nouter loop\n"
        + "".join(f"vertex {x} {y} {z}\n" for x, y, z in t)
        + "endloop\nendfacet\n"
        for t in tris
    ) + "endsolid box\n"
    assert len(read_stl(ascii_stl.encode())) == 12

    with pytest.raises(ValueError):
        read_stl(b"garbage")


def test_degenerate_mesh_rejected():
    p = (1.0, 2.0, 3.0)
    with pytest.raises(ValueError, match="zero volume"):
        mass_properties([(p, p, p)] * 4, density=1000)


def test_density_roundtrip_via_model():
    link = Link(name="l", mesh="l.stl", density=2700.0)
    d = link.to_dict()
    assert d["density"] == 2700.0
    assert Link.from_dict(d).density == 2700.0
    # absent density stays None (backwards compatible)
    assert Link.from_dict({"name": "x"}).density is None

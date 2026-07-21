"""Tests for the sim-ready inertial plausibility checks."""

from __future__ import annotations

import math

import pytest

from fusion2urdf.math3d import symmetric_eigenvalues
from fusion2urdf.model import Inertial, Link, Robot


def test_symmetric_eigenvalues_diagonal():
    assert symmetric_eigenvalues([3.0, 1.0, 2.0, 0.0, 0.0, 0.0]) == [1.0, 2.0, 3.0]


def test_symmetric_eigenvalues_rotated():
    # Tensor of diag(1, 2, 3) rotated 45 deg about Z:
    # ixx = iyy = 1.5, ixy = -0.5 (tensor element), izz = 3.
    moments = symmetric_eigenvalues([1.5, 1.5, 3.0, -0.5, 0.0, 0.0])
    assert moments == pytest.approx([1.0, 2.0, 3.0])


def test_symmetric_eigenvalues_degenerate():
    moments = symmetric_eigenvalues([2.0, 2.0, 2.0, 0.0, 0.0, 0.0])
    assert moments == pytest.approx([2.0, 2.0, 2.0])


def _one_link_robot(inertial: Inertial | None) -> Robot:
    return Robot(name="r", links=[Link(name="base_link", inertial=inertial)])


def test_validate_inertials_clean(two_dof_robot):
    assert two_dof_robot.validate_inertials() == []


def test_validate_inertials_missing_inertial():
    warnings = _one_link_robot(None).validate_inertials()
    assert len(warnings) == 1
    assert "no inertial" in warnings[0]


def test_validate_inertials_zero_mass():
    inertial = Inertial(mass=0.0, center_of_mass=[0, 0, 0], inertia=[0, 0, 0, 0, 0, 0])
    warnings = _one_link_robot(inertial).validate_inertials()
    assert any("zero mass" in w for w in warnings)


def test_validate_inertials_negative_mass_raises():
    inertial = Inertial(mass=-1.0, center_of_mass=[0, 0, 0], inertia=[1, 1, 1, 0, 0, 0])
    with pytest.raises(ValueError, match="negative or non-finite mass"):
        _one_link_robot(inertial).validate_inertials()


def test_validate_inertials_nan_mass_raises():
    inertial = Inertial(mass=math.nan, center_of_mass=[0, 0, 0], inertia=[1, 1, 1, 0, 0, 0])
    with pytest.raises(ValueError):
        _one_link_robot(inertial).validate_inertials()


def test_validate_inertials_triangle_inequality():
    # izz greater than ixx + iyy is not producible by any mass distribution.
    inertial = Inertial(
        mass=1.0, center_of_mass=[0, 0, 0], inertia=[0.1, 0.1, 0.5, 0, 0, 0]
    )
    warnings = _one_link_robot(inertial).validate_inertials()
    assert any("triangle inequality" in w for w in warnings)


def test_validate_inertials_triangle_inequality_frame_independent():
    # The same non-physical tensor expressed in a rotated frame (45 deg about
    # X) must still be caught: the raw diagonal [0.1, 0.3, 0.3] passes a naive
    # diagonal check, the principal moments [0.1, 0.1, 0.5] do not.
    inertial = Inertial(
        mass=1.0, center_of_mass=[0, 0, 0], inertia=[0.1, 0.3, 0.3, 0, -0.2, 0]
    )
    warnings = _one_link_robot(inertial).validate_inertials()
    assert any("triangle inequality" in w for w in warnings)


def test_validate_inertials_eigendecomposition_noise_ok():
    # Tiny negative moments from float noise must not raise.
    inertial = Inertial(
        mass=1.0,
        center_of_mass=[0, 0, 0],
        inertia=[-4.2e-22, 0.0015, 0.0015, 0, 0, 0],
    )
    warnings = _one_link_robot(inertial).validate_inertials()
    assert not any("negative" in w for w in warnings)


def test_validate_inertials_implausible_total_mass():
    inertial = Inertial(
        mass=250000.0, center_of_mass=[0, 0, 0], inertia=[1, 1, 1, 0, 0, 0]
    )
    warnings = _one_link_robot(inertial).validate_inertials()
    assert any("implausible" in w for w in warnings)


def test_validate_usd_physics_without_pxr_or_stage(tmp_path):
    from fusion2urdf.usd_export import validate_usd_physics

    result = validate_usd_physics(tmp_path / "missing.usda")
    assert len(result) == 1
    assert "skipped" in result[0]


def test_validate_usd_physics_full_quartet(tmp_path, two_dof_robot):
    pxr = pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics

    from fusion2urdf.usd_export import validate_usd_physics

    path = tmp_path / "robot.usda"
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    body = UsdGeom.Xform.Define(stage, "/robot/base").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    mass_api = UsdPhysics.MassAPI.Apply(body)
    mass_api.GetMassAttr().Set(1.75)  # sum of the fixture masses
    mass_api.GetCenterOfMassAttr().Set((0.0, 0.0, 0.1))
    mass_api.GetDiagonalInertiaAttr().Set((0.01, 0.01, 0.01))
    stage.Save()

    assert validate_usd_physics(path, two_dof_robot) == []

    # Dropping the center of mass must be reported.
    body.RemoveProperty("physics:centerOfMass")
    stage.Save()
    warnings = validate_usd_physics(path, two_dof_robot)
    assert any("physics:centerOfMass" in w for w in warnings)

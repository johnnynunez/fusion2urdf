"""Core model + math tests."""

import math

import pytest

from fusion2urdf.math3d import (
    inertia_about_com,
    matrix_to_rpy,
    rpy_to_matrix,
)
from fusion2urdf.model import Joint, Robot


def test_inertia_about_com_point_mass():
    # 1 kg point mass at (0, 0, 1): world inertia about origin has
    # ixx = iyy = m*z^2 = 1, izz = 0. About the COM everything vanishes.
    world = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    com = [0.0, 0.0, 1.0]
    result = inertia_about_com(world, com, 1.0)
    assert result == pytest.approx([0.0] * 6, abs=1e-12)


def test_inertia_about_com_offdiagonal_sign():
    # Point mass at (1, 1, 0): world tensor ixy component is -m*x*y = -1
    # (tensor convention). Parallel-axis back to COM must cancel it exactly.
    world = [1.0, 1.0, 2.0, -1.0, 0.0, 0.0]
    result = inertia_about_com(world, [1.0, 1.0, 0.0], 1.0)
    assert result == pytest.approx([0.0] * 6, abs=1e-12)


def test_rpy_roundtrip():
    for rpy in ([0.1, -0.4, 2.0], [0, 0, 0], [math.pi / 2, 0.2, -1.0]):
        m = rpy_to_matrix(rpy)
        back = matrix_to_rpy(m)
        m2 = rpy_to_matrix(back)
        for i in range(3):
            assert m[i] == pytest.approx(m2[i], abs=1e-9)


def test_robot_validate_ok(two_dof_robot):
    two_dof_robot.validate()  # should not raise


def test_robot_validate_missing_base(two_dof_robot):
    two_dof_robot.base_link = "nonexistent"
    with pytest.raises(ValueError, match="base link"):
        two_dof_robot.validate()


def test_robot_validate_orphan_link(two_dof_robot):
    two_dof_robot.joints.pop()  # link2 becomes an orphan
    with pytest.raises(ValueError, match="not connected"):
        two_dof_robot.validate()


def test_robot_validate_duplicate_parent(two_dof_robot):
    two_dof_robot.joints.append(
        Joint(name="dup", type="fixed", parent="base_link", child="link2")
    )
    with pytest.raises(ValueError, match="more than one parent"):
        two_dof_robot.validate()


def test_joint_type_checked():
    with pytest.raises(ValueError, match="unknown joint type"):
        Joint(name="bad", type="hinge", parent="a", child="b")


def test_json_roundtrip(two_dof_robot):
    data = two_dof_robot.to_dict()
    back = Robot.from_dict(data)
    assert back.to_dict() == data

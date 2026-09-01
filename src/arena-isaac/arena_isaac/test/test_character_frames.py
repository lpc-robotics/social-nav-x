import math

import pytest

from pedestrian.simulator.logic.people.character_frames import (
    CHARACTER_FORWARD_AXIS,
    character_to_ros_quaternion,
    ros_to_character_quaternion,
)


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_error(left, right):
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


def _rotate(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # Unit-quaternion vector rotation without requiring NumPy or Isaac Sim.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


@pytest.mark.parametrize(
    "yaw",
    (-math.pi, -2.3, -math.pi / 2.0, 0.0, math.pi / 2.0, 2.3, math.pi),
)
def test_character_native_minus_y_points_along_ros_heading(yaw):
    rendered = ros_to_character_quaternion(_yaw_quaternion(yaw))
    visual_forward = _rotate(rendered, CHARACTER_FORWARD_AXIS)

    assert visual_forward[0] == pytest.approx(math.cos(yaw), abs=1.0e-12)
    assert visual_forward[1] == pytest.approx(math.sin(yaw), abs=1.0e-12)
    assert visual_forward[2] == pytest.approx(0.0, abs=1.0e-12)


@pytest.mark.parametrize("yaw", (-math.pi, -1.7, 0.0, 0.8, math.pi))
def test_ros_character_round_trip_preserves_logical_yaw(yaw):
    logical = _yaw_quaternion(yaw)
    recovered = character_to_ros_quaternion(
        ros_to_character_quaternion(logical)
    )

    assert _angle_error(_yaw(recovered), yaw) <= 1.0e-12


def test_character_yaw_has_positive_ninety_degree_asset_offset():
    rendered = ros_to_character_quaternion(_yaw_quaternion(0.0))

    assert _angle_error(_yaw(rendered), math.pi / 2.0) <= 1.0e-12


@pytest.mark.parametrize(
    "invalid",
    ((), (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, math.nan, 1.0)),
)
def test_invalid_quaternion_is_rejected(invalid):
    with pytest.raises(ValueError):
        ros_to_character_quaternion(invalid)

"""Checks for low-object retention, floor removal, and cloud binary layout."""

import importlib.util
import math
from pathlib import Path
import struct

import numpy as np
import pytest
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2, PointField


spec = importlib.util.spec_from_file_location(
    'leader_obstacle_scan',
    Path(__file__).resolve().parents[1] / 'scripts' / 'leader_obstacle_scan.py')
projection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(projection)


def test_low_obstacle_survives_floor_self_and_invalid_filtering():
    xyz = np.array([
        [0.6, 0.0, -0.0155],  # floor under the elevated LiDAR
        [0.7, 0.0, 0.005],    # floor noise, below height threshold
        [0.1, 0.0, 0.09],     # own chassis
        [0.8, 0.0, 0.035],    # top of a 5 cm obstacle, relative to base_link
        [1.5, 0.0, 0.33],     # taller object behind it
        [0.5, 0.0, 0.6],      # outside the robot's collision height
        [np.nan, 0.0, 0.1],
        [np.inf, 0.0, 0.1],
    ])
    scan = projection.obstacle_ranges(xyz)
    assert list(scan[np.isfinite(scan)]) == pytest.approx([0.8])
    assert np.isinf(projection.obstacle_ranges(xyz[[0, 1, 2, 5, 6, 7]])).all()


def test_oblique_returns_use_horizontal_range_in_all_directions():
    # Input is in the elevated sensor frame. The four targets are all low,
    # including the rear hemisphere that the RGB-D camera cannot observe.
    transform = Transform()
    transform.translation.z = 0.33
    transform.rotation.z = math.sin(math.pi / 4)
    transform.rotation.w = math.cos(math.pi / 4)
    xyz = np.array([[0.8, 0, -0.295], [-0.8, 0, -0.295],
                    [0, 0.8, -0.295], [0, -0.8, -0.295]])
    base = projection.transform_xyz(xyz, transform)
    assert base[:, 2] == pytest.approx([0.035] * 4)
    scan = projection.obstacle_ranges(base)
    finite = scan[np.isfinite(scan)]
    assert len(finite) == 5  # rear direction represented at both -pi and +pi
    assert finite == pytest.approx([0.8] * 5)


@pytest.mark.parametrize('big_endian', [False, True])
def test_organized_point_cloud_supports_padding_and_endianness(big_endian):
    msg = PointCloud2()
    msg.width, msg.height = 1, 2
    msg.point_step, msg.row_step = 20, 28
    msg.is_bigendian = big_endian
    msg.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in [('z', 12), ('x', 4), ('y', 8)]]
    data = bytearray(56)
    order = '>' if big_endian else '<'
    struct.pack_into(order + 'fff', data, 4, 0.8, 0.2, -0.295)
    struct.pack_into(order + 'fff', data, 32, -1.2, 0.4, -0.25)
    msg.data = bytes(data)
    assert projection.cloud_xyz(msg) == pytest.approx(
        np.array([[0.8, 0.2, -0.295], [-1.2, 0.4, -0.25]]))
    msg.data = bytes(data[:30])
    with pytest.raises(ValueError, match='truncated'):
        projection.cloud_xyz(msg)
    msg.width = 0
    with pytest.raises(ValueError, match='Empty'):
        projection.cloud_xyz(msg)

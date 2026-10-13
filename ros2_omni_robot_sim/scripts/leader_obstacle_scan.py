#!/usr/bin/env python3
"""Project the leader's multilayer LiDAR into the scan consumed by D* and MPC.

The my_maze floor is planar. Height is measured relative to base_link; points
below 2.5 cm are discarded, including the floor below the chassis origin.
Transforms and timestamps come from the actual sensor messages, not a second
copy of the camera mast dimensions. No synthetic free scan is sent on failure.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def cloud_xyz(msg):
    """Read XYZ float fields, including padded rows and big-endian clouds."""
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32
           or fields[name].count != 1 for name in ('x', 'y', 'z')):
        raise ValueError('LiDAR cloud needs scalar FLOAT32 x, y, z fields')
    if msg.width == 0 or msg.height == 0:
        raise ValueError('Empty LiDAR cloud dimensions')
    offsets = [fields[name].offset for name in ('x', 'y', 'z')]
    if (min(offsets) < 0 or max(offsets) + 4 > msg.point_step
            or msg.row_step < msg.width * msg.point_step
            or len(msg.data) < (msg.height - 1) * msg.row_step + msg.width * msg.point_step):
        raise ValueError('Invalid LiDAR point/row strides or truncated cloud')
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [('>f4' if msg.is_bigendian else '<f4')] * 3,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    return np.column_stack([points[name].ravel() for name in ('x', 'y', 'z')])


def transform_xyz(xyz, transform):
    """Apply the sensor-to-base rigid transform to finite point rows."""
    q = transform.rotation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid sensor transform quaternion')
    x, y, z, w = (q.x / norm, q.y / norm, q.z / norm, q.w / norm)
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    t = transform.translation
    return xyz @ rotation.T + np.array([t.x, t.y, t.z])


def obstacle_ranges(xyz, *, bins=240, min_height=0.025, max_height=0.40,
                    min_range=0.16, max_range=10.0):
    """Keep the nearest obstacle in each azimuth, excluding floor and self.

    Output is horizontal distance in base_link, not the oblique beam length.
    An infinite range means no retained return; it is not proof of free volume.
    """
    ranges = np.full(bins, np.inf, dtype=np.float64)
    valid = (np.isfinite(xyz).all(axis=1)
             & (xyz[:, 2] >= min_height) & (xyz[:, 2] <= max_height))
    points = xyz[valid]
    distance = np.hypot(points[:, 0], points[:, 1])
    valid_range = (distance >= min_range) & (distance < max_range)
    points, distance = points[valid_range], distance[valid_range]
    angles = np.arctan2(points[:, 1], points[:, 0])
    indices = np.rint((angles + math.pi) * (bins - 1) / (2 * math.pi)).astype(int)
    np.minimum.at(ranges, np.clip(indices, 0, bins - 1), distance)
    # -pi and +pi represent the same direction.
    ranges[0] = ranges[-1] = min(ranges[0], ranges[-1])
    return ranges


class LeaderObstacleScan(Node):
    def __init__(self):
        super().__init__('leader_obstacle_scan')
        for name, value in (
            ('robot_name', 'robot3'), ('min_obstacle_height', 0.025),
            ('max_obstacle_height', 0.40), ('self_radius', 0.16),
            ('max_range', 10.0), ('max_cloud_age', 0.30),
        ):
            self.declare_parameter(name, value)
        self.robot = str(self.get_parameter('robot_name').value)
        self.base_frame = f'{self.robot}/base_link'
        self.sensor_frame = f'{self.robot}/leader_lidar_link'
        self.min_height = float(self.get_parameter('min_obstacle_height').value)
        self.max_height = float(self.get_parameter('max_obstacle_height').value)
        self.self_radius = float(self.get_parameter('self_radius').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.max_age = float(self.get_parameter('max_cloud_age').value)
        if not (0 < self.min_height < self.max_height
                and 0 < self.self_radius < self.max_range and self.max_age > 0):
            raise ValueError('Invalid obstacle height, range, or cloud age limits')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            LaserScan, f'/{self.robot}/scan', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, '/leader/lidar/points', self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f'{self.robot}: multilayer LiDAR -> /{self.robot}/scan; '
            f'height {self.min_height:.3f}..{self.max_height:.2f} m above base_link')

    def on_cloud(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if abs(age) > self.max_age:
            self.get_logger().warning('Dropping stale LiDAR cloud', throttle_duration_sec=5.0)
            return
        if msg.header.frame_id != self.sensor_frame:
            self.get_logger().error(
                f'Unexpected LiDAR frame: {msg.header.frame_id}', throttle_duration_sec=5.0)
            return
        try:
            transform = self.buffer.lookup_transform(self.base_frame, msg.header.frame_id, stamp)
            xyz = cloud_xyz(msg)
            # Invalid rays must not contaminate the rigid transform calculation.
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            points = transform_xyz(xyz, transform.transform)
            ranges = obstacle_ranges(
                points, min_height=self.min_height, max_height=self.max_height,
                min_range=self.self_radius, max_range=self.max_range)
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'LiDAR projection waiting: {error}', throttle_duration_sec=5.0)
            return
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / (len(ranges) - 1)
        scan.scan_time = 0.2
        scan.range_min = self.self_radius
        scan.range_max = self.max_range
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderObstacleScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

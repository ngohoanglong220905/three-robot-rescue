#!/usr/bin/env python3
"""Conservative paired LiDAR clusters: possible legs, never body orientation.

This geometric fallback is not a trained leg classifier. Non-human objects can
also form pairs; candidates request clearance, not a conversation label.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class LeaderLegCandidates(Node):
    def __init__(self):
        super().__init__('leader_leg_candidates')
        self.declare_parameter('leader_robot', 'robot3')
        leader = self.get_parameter('leader_robot').value
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PoseArray, '/leader/people/leg_candidates', 1)
        self.create_subscription(LaserScan, f'/{leader}/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseArray, '/formation/robot_states', self.on_team, 1)
        self.team = []
        self.team_stamp = -math.inf
        self.previous = []
        self.last_stamp = -math.inf
        self.create_timer(0.25, self.expire)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def on_team(self, msg):
        if msg.header.frame_id == 'map' and len(msg.poses) == 3:
            self.team = [np.array([p.position.x, p.position.y]) for p in msg.poses]
            self.team_stamp = self.stamp(msg)

    def expire(self):
        now = self.now()
        if not 0 <= now - self.last_stamp < 0.8:
            self.previous = []
            msg = PoseArray()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)

    def on_scan(self, msg):
        stamp = self.stamp(msg)
        now = self.now()
        if (not -0.1 <= now - stamp < 0.4 or not 0 <= now - self.team_stamp < 1.0
                or not msg.header.frame_id or msg.angle_increment <= 0):
            return
        if stamp < self.last_stamp:
            self.previous = []
        self.last_stamp = stamp
        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            return
        clusters, cluster = [], []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not max(0.25, msg.range_min) < distance < min(6.0, msg.range_max):
                if cluster:
                    clusters.append(cluster)
                cluster = []
                continue
            angle = msg.angle_min + i * msg.angle_increment
            p = np.array([distance * math.cos(angle), distance * math.sin(angle)])
            if cluster and np.linalg.norm(p - cluster[-1]) > 0.12:
                clusters.append(cluster)
                cluster = []
            cluster.append(p)
        if cluster:
            clusters.append(cluster)
        if len(clusters) > 1 and np.linalg.norm(clusters[0][0] - clusters[-1][-1]) < 0.12:
            clusters[0] = clusters.pop() + clusters[0]
        centers = [np.mean(c, axis=0) for c in clusters if 2 <= len(c) <= 18
                   and 0.04 <= np.linalg.norm(c[-1] - c[0]) <= 0.23]
        pairs = sorted((np.linalg.norm(a - b), i, j) for i, a in enumerate(centers)
                       for j, b in enumerate(centers) if i < j
                       and 0.12 < np.linalg.norm(a - b) < 0.60
                       and abs(np.linalg.norm(a) - np.linalg.norm(b)) < 0.35)
        q = tf.rotation
        v, w = np.array([q.x, q.y, q.z]), q.w
        if abs(float(v @ v) + w * w - 1) > 0.01:
            return
        translation = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        output = PoseArray()
        output.header.frame_id, output.header.stamp = 'map', msg.header.stamp
        used, matched, current = set(), set(), []
        for _, i, j in pairs:
            if i in used or j in used:
                continue
            used.update((i, j))
            local = np.append((centers[i] + centers[j]) / 2, 0)
            p = (local + 2 * np.cross(v, np.cross(v, local) + w * local) + translation)[:2]
            if any(np.linalg.norm(p - robot) < 0.45 for robot in self.team):
                continue
            choices = [(np.linalg.norm(p - old[0]), k) for k, old in enumerate(self.previous)
                       if k not in matched and 0 <= stamp - old[1] < 0.5]
            distance, key = min(choices, default=(math.inf, -1))
            hits = self.previous[key][2] + 1 if distance < 0.35 else 1
            if distance < 0.35:
                matched.add(key)
            current.append((p, stamp, hits))
            if hits >= 3:
                pose = Pose()
                pose.position.x, pose.position.y = map(float, p)
                pose.orientation.w = 1.0  # Explicitly unknown; no heading inference.
                output.poses.append(pose)
        self.previous = current
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderLegCandidates()
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

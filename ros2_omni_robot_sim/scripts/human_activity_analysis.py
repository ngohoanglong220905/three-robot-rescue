#!/usr/bin/env python3
"""Ten-second RGB-D body history; geometric activity hypotheses, not speech.

No Gazebo actor poses, animation names or authored social groups are inputs.
"""
from collections import deque
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from body_pose_estimation import PAIRS


class HumanActivityAnalysis(Node):
    def __init__(self):
        super().__init__('human_activity_analysis')
        self.declare_parameter('analysis_window', 10.0)
        self.window = float(self.get_parameter('analysis_window').value)
        if not 5 <= self.window <= 30:
            raise ValueError('analysis_window must be between 5 and 30 seconds')
        self.tracks = {}
        self.next_id = 0
        self.last_stamp = -math.inf
        self.last_time = 0.0
        self.create_subscription(String, '/leader/people/body_observations', self.on_bodies, 1)
        self.pub = self.create_publisher(String, '/leader/people/activity', 1)
        self.poses = self.create_publisher(PoseArray, '/leader/people/social_body_poses', 1)
        self.markers = self.create_publisher(MarkerArray, '/leader/people/activity_markers', 1)
        self.create_timer(0.5, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_bodies(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = data['stamp']['sec'] + data['stamp']['nanosec'] / 1e9
            if (data.get('version') != 1 or data['frame_id'] != 'map'
                    or not -0.1 <= self.now() - stamp < 1.0 or stamp <= self.last_stamp):
                return
            self.last_stamp = stamp
            used = set()
            for body in data['people']:
                p = np.asarray(body['position'], dtype=float)
                if p.shape != (3,) or not np.all(np.isfinite(p)):
                    continue
                joints = {int(k): np.asarray(v, dtype=float) for k, v in body['keypoints'].items()
                          if len(v) == 4 and np.all(np.isfinite(v))}
                choices = [(np.linalg.norm(t['p'][:2] - p[:2]), key) for key, t in self.tracks.items()
                           if key not in used and stamp - t['last'] < 1.5]
                distance, key = min(choices, default=(math.inf, -1))
                if distance > 0.7:
                    key = self.next_id
                    self.next_id += 1
                    self.tracks[key] = dict(p=p.copy(), first=stamp, last=stamp, history=deque())
                t = self.tracks[key]
                t['p'], t['last'] = p, stamp
                heading = body.get('heading')
                quality = float(body.get('heading_quality', 0.0))
                if heading is not None and not math.isfinite(float(heading)):
                    heading, quality = None, 0.0
                t['history'].append(dict(stamp=stamp, p=p, joints=joints, heading=heading,
                                         quality=quality, visibility=body['visibility']))
                while stamp - t['history'][0]['stamp'] > self.window + 0.3:
                    t['history'].popleft()
                used.add(key)
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f'Invalid body observation: {exc}', throttle_duration_sec=5.0)

    def summarize(self, key, track, now):
        history = list(track['history'])
        times = np.array([s['stamp'] for s in history])
        points = np.array([s['p'][:2] for s in history])
        elapsed = min(self.window, max(0.0, now - track['first']))
        coverage = min(1.0, float(np.clip(np.diff(times), 0, 0.75).sum()) / self.window)
        ready = elapsed >= self.window and coverage >= 0.75 and len(history) >= 12 and now - track['last'] < 1.0
        centered = times - times.mean()
        velocity = ((centered[:, None] * (points - points.mean(0))).sum(0)
                    / max(float(centered @ centered), 1e-9))
        speed = float(np.linalg.norm(velocity))
        spread = float(np.linalg.norm(np.percentile(points, 90, axis=0) - np.percentile(points, 10, axis=0)))
        amplitude = 0.0
        for joint in (3, 4, 6, 7):
            relative = []
            for s in history:
                js = s['joints']
                if joint in js and 2 in js and 5 in js:
                    relative.append(js[joint][:3] - (js[2][:3] + js[5][:3]) / 2)
            if len(relative) >= 8:
                amplitude = max(amplitude, float(np.linalg.norm(
                    np.percentile(relative, 90, axis=0) - np.percentile(relative, 10, axis=0))))
        head_motion, torso_motion = [], []
        for sample in history:
            js = sample['joints']
            if 0 in js and 1 in js:
                head_motion.append(js[0][:3] - js[1][:3])
            if all(j in js for j in (2, 5, 8, 11)):
                torso_motion.append((js[2][:3] + js[5][:3] - js[8][:3] - js[11][:3]) / 2)

        def extent(series):
            return (float(np.linalg.norm(np.percentile(series, 90, axis=0)
                                         - np.percentile(series, 10, axis=0))) if len(series) >= 8 else 0.0)

        head_amplitude, torso_amplitude = extent(head_motion), extent(torso_motion)
        angles = [s['heading'] for s in history if s['heading'] is not None and s['quality'] >= 0.65]
        heading = None
        if len(angles) >= max(6, len(history) // 2):
            direction = np.mean([[math.cos(a), math.sin(a)] for a in angles], axis=0)
            if np.linalg.norm(direction) >= 0.85 and history[-1]['quality'] >= 0.65:
                heading = float(math.atan2(direction[1], direction[0]))
        stationary = speed < 0.15 and spread < 0.45
        gesturing = ready and stationary and (amplitude >= 0.15 or head_amplitude >= 0.09 or torso_amplitude >= 0.10)
        state = ('observing' if not ready else
                 'moving' if not stationary else
                 'upper_body_motion' if gesturing else 'standing_or_still')
        if history[-1]['visibility'] in ('legs_only', 'insufficient'):
            heading = None
            state = 'partial_body_orientation_unknown' if ready else 'observing_partial_body'
        return dict(id=key, position=track['p'].tolist(), stamp=track['last'], window_seconds=self.window,
                    observed_seconds=elapsed, coverage=coverage, window_ready=ready,
                    state=state, heading=heading, speed=speed, gesture_amplitude=amplitude,
                    head_motion_amplitude=head_amplitude, torso_motion_amplitude=torso_amplitude,
                    gesturing=gesturing, partners=[], source='camera_body_history')

    def tick(self):
        now = self.now()
        if now < self.last_time:
            self.tracks.clear()
            self.last_stamp = -math.inf
        self.last_time = now
        self.tracks = {k: t for k, t in self.tracks.items() if now - t['last'] < 2.0}
        people = [self.summarize(k, t, now) for k, t in self.tracks.items()]
        for i, a in enumerate(people):
            for b in people[i + 1:]:
                if not (a['window_ready'] and b['window_ready'] and a['heading'] is not None
                        and b['heading'] is not None and a['speed'] < 0.15 and b['speed'] < 0.15):
                    continue
                v = np.asarray(b['position'][:2]) - a['position'][:2]
                distance = np.linalg.norm(v)
                if not 0.65 < distance < 3.0:
                    continue
                v /= distance
                da = np.array([math.cos(a['heading']), math.sin(a['heading'])])
                db = np.array([math.cos(b['heading']), math.sin(b['heading'])])
                if da @ v > 0.5 and db @ -v > 0.5 and (a['gesturing'] or b['gesturing']):
                    a['partners'].append(b['id'])
                    b['partners'].append(a['id'])
                    a['state'] = b['state'] = 'conversation_candidate'
        msg = PoseArray()
        msg.header.frame_id = 'map'
        usable = [p for p in people if p['window_ready'] and p['heading'] is not None and now - p['stamp'] < 0.8]
        stamp_ns = int(min((p['stamp'] for p in usable), default=now) * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        for p in usable:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p['position']
            pose.orientation.z, pose.orientation.w = math.sin(p['heading'] / 2), math.cos(p['heading'] / 2)
            msg.poses.append(pose)
        self.poses.publish(msg)
        self.pub.publish(String(data=json.dumps(dict(version=1, frame_id='map', stamp=now,
                                                     tracks=people), allow_nan=False)))
        markers = [Marker(action=Marker.DELETEALL)]
        for p in people:
            skeleton = Marker(header=msg.header, ns='camera_body_skeleton', id=p['id'],
                              type=Marker.LINE_LIST, action=Marker.ADD)
            skeleton.pose.orientation.w = 1.0
            skeleton.scale.x = 0.025
            skeleton.color.r, skeleton.color.g, skeleton.color.b, skeleton.color.a = 0.25, 0.75, 1.0, 1.0
            joints = self.tracks[p['id']]['history'][-1]['joints']
            for a, b in PAIRS:
                if a in joints and b in joints:
                    for j in (a, b):
                        skeleton.points.append(Point(x=float(joints[j][0]), y=float(joints[j][1]), z=float(joints[j][2])))
            label = Marker(header=msg.header, ns='camera_body_activity', id=p['id'],
                           type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.position.x, label.pose.position.y = p['position'][:2]
            label.pose.position.z = 2.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = (f"track {p['id']}: {p['state']}\n"
                          f"{p['observed_seconds']:.1f}/{self.window:.0f}s, coverage {p['coverage']:.0%}")
            markers.extend((skeleton, label))
        self.markers.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = HumanActivityAnalysis()
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

#!/usr/bin/env python3
"""Camera-derived social keep-outs shared by D* Lite and all three MPC nodes.

Inputs are camera head/body poses, TV / multicolour targets and the physical
map. This node never reads the authored scene, actor names or human Gazebo poses.
"""
from collections import deque
import copy
import itertools
import math
import json

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from camera_people_tracking import CameraPeople
from social_search import SocialSearch, VIEWING_KINDS
from color_attention import ColorAttention


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def hull(points):
    return cv2.convexHull(np.asarray(points, dtype=np.float32)).reshape(-1, 2).astype(float)


def region_mask(xs, ys, polygon, margin):
    """Convex polygon / segment / point including a metric boundary buffer."""
    inside = np.ones(xs.shape, dtype=bool) if len(polygon) >= 3 else np.zeros(xs.shape, dtype=bool)
    cross_positive, cross_negative = np.ones(xs.shape, bool), np.ones(xs.shape, bool)
    distance = np.full(xs.shape, np.inf)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        v = b - a
        dx, dy = xs - a[0], ys - a[1]
        cross = v[0] * dy - v[1] * dx
        cross_positive &= cross >= -1e-9
        cross_negative &= cross <= 1e-9
        t = np.clip((dx * v[0] + dy * v[1]) / max(float(v @ v), 1e-12), 0, 1)
        distance = np.minimum(distance, (dx - t * v[0]) ** 2 + (dy - t * v[1]) ** 2)
    inside &= cross_positive | cross_negative
    return inside | (distance <= margin * margin)


class SocialNavigation(Node):
    def __init__(self):
        super().__init__('social_navigation')
        defaults = dict(head_max_age=8.0, object_memory=60.0, zone_memory=60.0,
                        group_margin=0.35, attention_margin=0.25,
                        allow_inferred_focus=False, robot_clearance=0.20,
                        prediction_horizon=1.5, auto_observe=True, observe_cooldown=30.0,
                        leader_robot='robot3', require_activity_window=False,
                        protect_uncertain_groups=False, provisional_memory=20.0,
                        nearby_group_distance=2.5, provisional_margin=0.45,
                        viewing_length=3.2, nearby_tv_distance=4.5, color_attention_enabled=True,
                        color_person_memory=75.0, color_object_memory=75.0,
                        color_zone_memory=90.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        numeric = [k for k, value in defaults.items() if type(value) is float]
        if any(not math.isfinite(self.cfg[k]) or self.cfg[k] <= 0 for k in numeric):
            raise ValueError('Social navigation durations / margins must be positive')
        if self.cfg['prediction_horizon'] > 2.0:
            raise ValueError('Limit uncertain motion prediction to <= 2 seconds')
        if not 0.8 <= self.cfg['viewing_length'] <= 5.0:
            raise ValueError('viewing_length must be between 0.8 and 5.0 metres')
        if not 0.65 <= self.cfg['nearby_group_distance'] <= 3.5:
            raise ValueError('nearby_group_distance must be between 0.65 and 3.5 metres')
        if not 0.8 <= self.cfg['nearby_tv_distance'] <= 6.0:
            raise ValueError('nearby_tv_distance must be between 0.8 and 6.0 metres')
        self.leader_index = ['robot1', 'robot2', 'robot3'].index(self.cfg['leader_robot'])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/social_navigation/map', qos)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/social_navigation/keepout', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/social_navigation/markers', qos)
        self.status_pub = self.create_publisher(String, '/social_navigation/status', 1)
        self.search_pub = self.create_publisher(String, '/social_navigation/search', 10)
        self.search = SocialSearch(self.emit_search, self.cfg['observe_cooldown'], self.cfg['viewing_length'])
        self.create_subscription(String, '/social_navigation/search_status', self.on_search_status, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(PoseArray, '/leader/people/head_poses', self.on_heads, 1)
        self.create_subscription(PoseArray, '/leader/people/poses', self.on_people, 1)
        self.create_subscription(PoseArray, '/leader/people/leg_candidates', self.on_legs, 1)
        self.create_subscription(PoseArray, '/leader/people/social_body_poses', self.on_body_heads, 1)
        self.create_subscription(String, '/leader/people/activity', self.on_activity, 1)
        self.create_subscription(String, '/leader/people/body_observations', self.on_raw_bodies, 1)
        self.create_subscription(PoseArray, '/leader/people/attention_targets', self.on_targets, 1)
        self.create_subscription(String, '/leader/people/color_targets', self.on_color_targets, 1)
        self.create_subscription(PoseArray, '/formation/robot_states', self.on_team, 1)
        self.create_subscription(String, '/social_navigation/observation_status', self.on_observation_status, 1)
        self.create_service(Trigger, '/social_navigation/reset_memory', self.reset_memory)
        self.map_msg = None
        self.heads, self.objects, self.zones = {}, {}, {}
        self.body_heads, self.activity_ready = {}, []
        self.raw_body_heads = {}
        self.color_attention = self.new_attention_memory()
        self.last_raw_body_stamp = -math.inf
        self.last_body_stamp = -math.inf
        self.legs = CameraPeople()
        self.people = CameraPeople()
        self.person_regions = {}
        self.observed_places = []
        self.last_look = -math.inf
        self.observation_active = False
        self.pending_look = []
        self.last_person_update = -math.inf
        self.next_id = 0
        self.team = []
        self.team_stamp = -math.inf
        self.last_data = None
        self.previous_mask = None
        self.escape_mask = None
        self.last_time = 0.0
        self.last_head_stamp = self.last_object_stamp = -math.inf
        self.create_timer(0.2, self.tick)
        self.get_logger().info('Social rules: one person looking towards a nearby peer -> keepout; '
                               'one measured heading + nearby camera TV / multicolour target -> viewing. '
                               'Known viewing regions end camera checks immediately.')
        if self.cfg['protect_uncertain_groups'] or self.cfg['allow_inferred_focus']:
            self.get_logger().warn('Legacy proximity-only / inferred-focus regions are disabled; '
                                   'measured orientation and target rules take priority.')

    def new_attention_memory(self):
        return ColorAttention(self.cfg['color_person_memory'], self.cfg['color_object_memory'],
                              self.cfg['nearby_group_distance'], self.cfg['nearby_tv_distance'])

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def stamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def valid_message(self, msg):
        return msg.header.frame_id == 'map' and -0.1 <= self.now() - self.stamp(msg) <= 1.0

    def observations(self, msg, tracks, oriented):
        if not self.valid_message(msg):
            return
        stamp = self.stamp(msg)
        used = set()
        for pose in msg.poses:
            p = np.array([pose.position.x, pose.position.y])
            if not np.all(np.isfinite(p)):
                continue
            angle = yaw(pose.orientation)
            if not math.isfinite(angle):
                continue
            d = np.array([math.cos(angle), math.sin(angle)])
            eligible = [(np.linalg.norm(t['p'] - p), key) for key, t in tracks.items()
                        if key not in used and stamp - t['stamp'] <
                        (self.cfg['head_max_age'] if oriented else self.cfg['object_memory'])]
            distance, key = min(eligible, default=(math.inf, -1))
            if distance > (0.65 if oriented else 0.9):
                key = self.next_id
                self.next_id += 1
                tracks[key] = dict(p=p, d=d, stamp=stamp, hits=1, since=stamp)
            else:
                t = tracks[key]
                stable = float(t['d'] @ d) > math.cos(math.radians(25))
                t['hits'] = t['hits'] + 1 if stable or not oriented else 1
                if oriented and not stable:
                    t['since'] = stamp
                t['p'] = 0.65 * t['p'] + 0.35 * p
                direction = 0.5 * t['d'] + 0.5 * d
                t['d'] = direction / max(np.linalg.norm(direction), 1e-9)
                t['stamp'] = stamp
            used.add(key)

    def on_heads(self, msg):
        if self.stamp(msg) <= self.last_head_stamp or not self.valid_message(msg):
            return
        self.last_head_stamp = self.stamp(msg)
        self.observations(msg, self.heads, True)

    def on_people(self, msg):
        if self.valid_message(msg):
            self.people.observe([(p.position.x, p.position.y) for p in msg.poses], self.stamp(msg))

    def on_legs(self, msg):
        if self.valid_message(msg):
            self.legs.observe([(p.position.x, p.position.y) for p in msg.poses], self.stamp(msg))

    def on_body_heads(self, msg):
        if self.valid_message(msg) and self.stamp(msg) > self.last_body_stamp:
            self.last_body_stamp = self.stamp(msg)
            self.observations(msg, self.body_heads, True)

    def on_raw_bodies(self, msg):
        """Early torso evidence may protect a possible viewing area, not label gaze."""
        try:
            data = json.loads(msg.data)
            stamp = data['stamp']['sec'] + data['stamp']['nanosec'] / 1e9
            if (data.get('version') != 1 or data.get('frame_id') != 'map'
                    or not -0.1 <= self.now() - stamp < 1.0 or stamp <= self.last_raw_body_stamp):
                return
            poses = PoseArray()
            poses.header.frame_id = 'map'
            poses.header.stamp.sec = data['stamp']['sec']
            poses.header.stamp.nanosec = data['stamp']['nanosec']
            for body in data['people']:
                heading = body.get('heading')
                # The estimator already checks torso baselines/head agreement.
                # Missing arms/legs must not discard an otherwise valid heading.
                if heading is None or not math.isfinite(heading) or body['heading_quality'] < 0.65:
                    continue
                position = body['position']
                if len(position) != 3 or not np.all(np.isfinite(position)):
                    continue
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, position)
                pose.orientation.z, pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
                poses.poses.append(pose)
            self.last_raw_body_stamp = stamp
            self.observations(poses, self.raw_body_heads, True)
        except (ValueError, KeyError, TypeError):
            return

    def on_activity(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('frame_id') != 'map' or not -0.1 <= self.now() - data['stamp'] < 1.0:
                return
            self.activity_ready = [(np.asarray(t['position'][:2]), t['stamp']) for t in data['tracks']
                                   if t['window_ready'] and np.all(np.isfinite(t['position']))]
        except (ValueError, KeyError, TypeError):
            return

    def request_observation(self, now):
        if (not self.cfg['auto_observe'] or self.observation_active or self.search.current is not None
                or now - self.last_look < self.cfg['observe_cooldown']
                or now - self.team_stamp > 1.0 or len(self.team) != 3):
            return
        self.observed_places = [(p, stamp) for p, stamp in self.observed_places if now - stamp < 60.0]
        leader = np.asarray(self.team[self.leader_index])
        resolved_ids = {ident for key in self.zones for ident in (key[1:-1] if key[0] in VIEWING_KINDS else key[1:])}
        resolved_positions = [p['p'] for ident, p in self.color_attention.people.items() if ident in resolved_ids]
        unseen = [t['p'] for t in itertools.chain(self.people.tracks.values(), self.legs.tracks.values())
                  if now - t['stamp'] < 0.8 and t['hits'] >= 3 and t['stamp'] - t['since'] >= 0.4
                  and 0.8 < np.linalg.norm(t['p'] - leader) < 4.5
                  and not any(np.linalg.norm(t['p'] - p) < 0.65 for p in resolved_positions)
                  and not any(np.linalg.norm(t['p'] - p) < 0.9 for p, _ in self.observed_places)]
        if not unseen:
            return
        # A brief directed look must be cancellable as soon as the person's
        # social region is known, just like a TV search (same correlated ID).
        point = min(unseen, key=lambda p: np.linalg.norm(p - leader))
        if self.search.inspect_person(point, leader, now):
            self.last_look = now
            self.pending_look = [point.copy()]

    def emit_search(self, data):
        self.search_pub.publish(String(data=json.dumps(data)))
        if data['command'] == 'start':
            self.get_logger().info('Camera search: ' + data['kind'], throttle_duration_sec=5.0)

    def on_search_status(self, msg):
        try:
            data = json.loads(msg.data)
            owned = self.search.current is not None and data.get('id') == self.search.current['id']
            self.search.feedback(data, self.now())
            if owned and data.get('state') == 'scanning':
                self.observation_active = True
                self.last_look = self.now()
            elif owned and data.get('state') in ('completed', 'aborted', 'cancelled'):
                self.observation_active = False
                self.last_look = self.now()
            elif owned and data.get('state') == 'rejected' and self.search.current is None:
                self.last_look = self.now() - self.cfg['observe_cooldown'] + 5.0
                self.pending_look = []
        except (ValueError, TypeError, AttributeError):
            return

    def on_observation_status(self, msg):
        now = self.now()
        if msg.data.startswith('scanning:'):
            self.observation_active = True
            self.last_look = now
        elif msg.data.startswith('rejected:'):
            # A yield/recovery may briefly prevent a sweep. Retry later rather
            # than marking the person as observed when no sweep took place.
            self.pending_look = []
            self.last_look = now - self.cfg['observe_cooldown'] + min(5.0, self.cfg['observe_cooldown'])
        elif msg.data.startswith(('completed', 'aborted:', 'cancelled:')):
            self.observation_active = False
            self.last_look = now
            if msg.data.startswith('completed'):
                self.observed_places.extend((t['p'].copy(), now) for t in self.color_attention.people.values()
                                            if now - t['direction_stamp'] < 2.0)
            self.pending_look = []

    def on_targets(self, msg):
        if self.stamp(msg) <= self.last_object_stamp or not self.valid_message(msg):
            return
        self.last_object_stamp = self.stamp(msg)
        self.observations(msg, self.objects, False)

    def on_color_targets(self, msg):
        if not self.cfg['color_attention_enabled']:
            return
        try:
            data = json.loads(msg.data)
            stamp = data['stamp']['sec'] + data['stamp']['nanosec'] / 1e9
            if (data.get('version') != 1 or data.get('frame_id') != 'map'
                    or not -0.1 <= self.now() - stamp < 1.0):
                return
            objects = []
            for target in data['objects']:
                point = np.asarray(target['position'], dtype=float)
                if point.shape != (3,) or not np.all(np.isfinite(point)):
                    continue
                # Don't turn a nearby robot or a detected person's clothes
                # into the shared object. The RGB detector also masks people.
                if any(np.linalg.norm(point[:2] - t['p']) < 0.55
                       for t in self.people.tracks.values() if self.now() - t['stamp'] < 1.5):
                    continue
                if self.now() - self.team_stamp < 1.0 and any(np.linalg.norm(point[:2] - p) < 0.55 for p in self.team):
                    continue
                objects.append(target)
            self.color_attention.observe_objects(objects, stamp)
        except (ValueError, KeyError, TypeError):
            return

    def on_team(self, msg):
        if self.valid_message(msg) and len(msg.poses) == 3:
            self.team = [(p.position.x, p.position.y) for p in msg.poses]
            self.team_stamp = self.stamp(msg)

    def on_map(self, msg):
        if msg.header.frame_id != 'map' or msg.info.resolution <= 0:
            self.get_logger().error('Social map requires frame map and a positive resolution.')
            return
        if len(msg.data) != msg.info.width * msg.info.height:
            return
        self.map_msg = msg
        self.raw = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.res = msg.info.resolution
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        theta = yaw(msg.info.origin.orientation)
        self.rotation = np.array([[math.cos(theta), -math.sin(theta)],
                                  [math.sin(theta), math.cos(theta)]])
        yy, xx = np.indices(self.raw.shape)
        local = np.stack(((xx + 0.5) * self.res, (yy + 0.5) * self.res), axis=-1)
        world = local @ self.rotation.T + self.origin
        self.xs, self.ys = world[..., 0], world[..., 1]
        r = math.ceil(self.cfg['robot_clearance'] / self.res)
        self.footprint = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self.physical = (self.raw >= 65) | (self.raw < 0)
        self.physical_inflated = cv2.dilate(self.physical.astype(np.uint8), self.footprint) > 0
        self.last_data = self.previous_mask = self.escape_mask = None
        self.tick()

    def cell(self, point):
        local = self.rotation.T @ (np.asarray(point) - self.origin)
        x, y = np.floor(local / self.res).astype(int)
        return (int(y), int(x)) if 0 <= x < self.raw.shape[1] and 0 <= y < self.raw.shape[0] else None

    def visible(self, start, end):
        distance = float(np.linalg.norm(end - start))
        if distance < 0.8:
            middle = self.cell((start + end) / 2)
            return middle is not None and not self.physical[middle]
        # Ignore the person's own body/seat and the target object's surface.
        for length in np.arange(0.65, distance - 0.35, max(0.025, self.res / 2)):
            cell = self.cell(start + (end - start) * length / distance)
            if cell is None or self.physical[cell]:
                return False
        return True

    def carve_escape(self, mask, person_mask, changed, now):
        """If a newly observed zone covers a robot, leave a short exit to its edge.

        Carving affects only virtual social cells. Physical collision cells are
        never cleared. Hold the same exits until the whole team has left.
        """
        near = cv2.dilate(mask.astype(np.uint8), self.footprint) > 0
        occupied = self.physical_inflated | (cv2.dilate(person_mask.astype(np.uint8), self.footprint) > 0)
        starts = [self.cell(p) for p in self.team] if now - self.team_stamp < 1.0 else []
        starts = [p for p in starts if p is not None and near[p]]
        if not starts:
            self.escape_mask = np.zeros(mask.shape, bool)
        elif changed or self.escape_mask is None or not self.escape_mask.any():
            clear = np.zeros(mask.shape, np.uint8)
            rows, cols = mask.shape
            for start in starts:
                queue, parents = deque([start]), {start: None}
                finish = None
                while queue:
                    p = queue.popleft()
                    if not near[p] and not occupied[p]:
                        finish = p
                        break
                    y, x = p
                    for nxt in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                        ny, nx = nxt
                        if (0 <= ny < rows and 0 <= nx < cols and nxt not in parents
                                and not occupied[nxt]):
                            parents[nxt] = p
                            queue.append(nxt)
                if finish is None:
                    self.get_logger().warn('New social zone covers a robot; no physical exit found.',
                                           throttle_duration_sec=5.0)
                while finish is not None:
                    cv2.circle(clear, (finish[1], finish[0]), math.ceil(0.25 / self.res), 1, -1)
                    finish = parents[finish]
            self.escape_mask = clear.astype(bool)
        return mask & ~self.escape_mask

    def tick(self):
        now = self.now()
        if now < self.last_time:
            self.clear_memory()
        self.last_time = now
        if self.map_msg is None:
            return
        self.heads = {k: t for k, t in self.heads.items() if now - t['stamp'] < max(8.0, self.cfg['head_max_age'])}
        self.body_heads = {k: t for k, t in self.body_heads.items() if now - t['stamp'] < self.cfg['head_max_age']}
        self.raw_body_heads = {k: t for k, t in self.raw_body_heads.items() if now - t['stamp'] < 3.0}
        self.objects = {k: t for k, t in self.objects.items() if now - t['stamp'] < self.cfg['object_memory']}
        self.people.prune(now)
        self.legs.prune(now)
        self.color_attention.prune(now)
        if self.cfg['color_attention_enabled']:
            self.color_attention.observe_tv_objects(self.objects, now)
        self.color_attention.observe_people(self.people.tracks,
                                             (self.heads, self.raw_body_heads, self.body_heads), now)
        for key in list(self.zones):
            if self.color_attention.contradicted(key, self.visible):
                del self.zones[key]
        social_regions = self.color_attention.regions(
            self.visible, max(0.40, self.cfg['attention_margin']), self.cfg['group_margin'],
            self.cfg['color_attention_enabled'])
        social_regions = {key: zone for key, zone in social_regions.items()
                          if now - zone['stamp'] < (self.cfg['color_zone_memory']
                                                   if key[0] in VIEWING_KINDS else self.cfg['zone_memory'])}
        viewing_sets = {key[1:-1]: key for key in social_regions if key[0] in VIEWING_KINDS}
        for key in list(self.zones):
            if key[0] in VIEWING_KINDS and any(
                    set(key[1:-1]) <= set(members) and new_key != key
                    for members, new_key in viewing_sets.items()):
                del self.zones[key]
        head_count = sum(p['d'] is not None for p in self.color_attention.people.values())
        candidates = {key: (zone['polygon'], zone['margin'], zone['source'])
                      for key, zone in social_regions.items()}
        self.color_attention.invalidated.clear()
        for key, (polygon, margin, source) in candidates.items():
            old = self.zones.get(key)
            stamp = social_regions[key]['stamp']
            # Freeze small measurement jitter so the planner does not stop/replan
            # at every image. Replace a boundary only after a meaningful change.
            if old is None or len(old['polygon']) != len(polygon) or max(
                    min(np.linalg.norm(p - q) for q in old['polygon']) for p in polygon) > 0.25:
                self.zones[key] = dict(polygon=polygon, margin=margin, source=source, stamp=stamp)
            else:
                old['stamp'], old['source'] = stamp, source
        self.zones = {k: v for k, v in self.zones.items() if now - v['stamp'] <
                      (self.cfg['color_zone_memory'] if k[0] in VIEWING_KINDS else
                       self.cfg['zone_memory'])}
        if now - self.last_person_update >= 1.0:
            fresh = self.people.regions(now, self.cfg['prediction_horizon'])
            for key, region in self.legs.regions(now, self.cfg['prediction_horizon']).items():
                if any(np.linalg.norm(region['polygon'][0] - t['p']) < 0.75 for t in self.people.tracks.values()):
                    continue
                region['source'] = 'possible paired legs; orientation unknown'
                fresh[('possible_legs', key[1])] = region
            for key, zone in fresh.items():
                old = self.person_regions.get(key)
                if (old is not None and old['polygon'].shape == zone['polygon'].shape
                        and old['margin'] == zone['margin']
                        and np.max(np.linalg.norm(old['polygon'] - zone['polygon'], axis=1)) < 0.20):
                    zone['polygon'] = old['polygon']
            self.person_regions = fresh
            self.last_person_update = now
        mask = np.zeros(self.raw.shape, dtype=bool)
        for zone in self.zones.values():
            mask |= region_mask(self.xs, self.ys, zone['polygon'], zone['margin'])
        person_mask = np.zeros(self.raw.shape, dtype=bool)
        for zone in self.person_regions.values():
            person_mask |= region_mask(self.xs, self.ys, zone['polygon'], zone['margin'])
        changed = self.previous_mask is None or not np.array_equal(mask, self.previous_mask)
        self.previous_mask = mask.copy()
        # An exit from a newly formed social region must never erase measured
        # people/legs. Preserve these cells for both D* Lite and every MPC.
        mask = self.carve_escape(mask, person_mask, changed, now) | person_mask
        fused = self.raw.copy()
        fused[mask] = 100
        data = fused.tobytes()
        if data != self.last_data:
            self.last_data = data
            msg = copy.deepcopy(self.map_msg)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data = fused.ravel().tolist()
            self.map_pub.publish(msg)
            overlay = copy.deepcopy(msg)
            overlay.data = np.where(mask, 100, -1).astype(np.int8).ravel().tolist()
            self.mask_pub.publish(overlay)
            self.get_logger().info(f'Social map updated: {len(self.zones)} group regions; '
                                   'camera observations + conservative LiDAR candidates.')
        header = copy.deepcopy(self.map_msg.header)
        header.stamp = self.get_clock().now().to_msg()
        self.publish_markers(header)
        if len(self.team) == 3 and now - self.team_stamp < 1.0:
            # Keepouts already accepted into the map remain sufficient while
            # the camera turns away; never reacquire a whole body to release a scan.
            self.search.update(self.color_attention, self.objects, self.people.tracks, self.zones,
                               self.visible, np.asarray(self.team[self.leader_index]), now,
                               available=not self.observation_active and now - self.last_look >= 5.0,
                               enabled=self.cfg['auto_observe'] and self.cfg['color_attention_enabled'])
        elif self.search.current is not None:
            self.search.progress(self.color_attention, self.people.tracks, self.zones, self.visible, now)
        self.request_observation(now)
        provisional_count = 0
        color_count = sum(k[0] == 'color_attention' for k in self.zones)
        tv_count = sum(k[0] == 'viewing_tv' for k in self.zones)
        colour_targets = sum(t['source'] == 'colour' for t in self.color_attention.objects.values())
        status = (
            f'usable_orientations={head_count}, provisional_regions={provisional_count}, '
            f'confirmed_regions={len(self.zones) - provisional_count - color_count - tv_count}, '
            f'color_regions={color_count}, tv_regions={tv_count}, color_targets={colour_targets}, '
            f'remembered_people={len(self.color_attention.people)}, remembered_viewers={head_count}, '
            f'tv_tracks={len(self.objects)}, person_tracks={len(self.person_regions)}, '
            f'search={self.search.status}; '
            + ('no group region: need a direction towards a nearby peer, or one heading towards a nearby TV/colour target' if not self.zones else
               'social keepout active on D* Lite and all three MPC maps'))
        self.status_pub.publish(String(data=status))
        self.get_logger().info(status, throttle_duration_sec=10.0)

    def publish_markers(self, header):
        markers = [Marker(action=Marker.DELETEALL)]
        for index, (key, zone) in enumerate(itertools.chain(self.zones.items(), self.person_regions.items())):
            polygon = zone['polygon']
            color = (0.9, 0.35, 0.75) if key[0] in VIEWING_KINDS else (1.0, 0.6, 0.05)
            if key[0] == 'person':
                color = (0.25, 0.85, 1.0)
            outline = Marker(header=header, ns='camera_social_regions', id=2 * index,
                             type=Marker.LINE_STRIP, action=Marker.ADD)
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.07
            outline.color.r, outline.color.g, outline.color.b = color
            outline.color.a = 0.95
            # A ring/capsule makes a single-person margin visible as well.
            display_polygon = hull([p + zone['margin'] * np.array([math.cos(a), math.sin(a)])
                                    for p in polygon for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)])
            for x, y in np.vstack((display_polygon, display_polygon[:1])):
                outline.points.append(Point(x=float(x), y=float(y), z=0.08))
            label = Marker(header=header, ns='camera_social_regions', id=2 * index + 1,
                           type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.orientation.w = 1.0
            label.pose.position.x, label.pose.position.y = map(float, polygon.mean(axis=0))
            label.pose.position.z = 0.35
            label.scale.z = 0.16
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f"{key[0]}\n{zone['source']}\n+ {zone['margin']:.2f} m buffer"
            markers.extend((outline, label))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def clear_memory(self):
        self.search.reset(self.now())
        self.heads.clear()
        self.body_heads.clear()
        self.raw_body_heads.clear()
        self.color_attention = self.new_attention_memory()
        self.last_raw_body_stamp = -math.inf
        self.activity_ready = []
        self.last_body_stamp = -math.inf
        self.legs = CameraPeople()
        self.objects.clear()
        self.zones.clear()
        self.people = CameraPeople()
        self.person_regions.clear()
        self.observed_places.clear()
        self.pending_look = []
        self.observation_active = False
        self.last_look = self.last_person_update = -math.inf
        self.team = []
        self.last_head_stamp = self.last_object_stamp = -math.inf
        self.last_data = self.previous_mask = self.escape_mask = None

    def reset_memory(self, request, response):
        self.clear_memory()
        self.tick()
        response.success, response.message = True, 'Cleared camera observations and social regions.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SocialNavigation()
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

"""A leader on a D* route, followed by two robots on its measured trail.

Follower progress is bounded by the measured progress of its predecessor.
The initial trail connects the three measured poses in convoy order; later
points come exclusively from the leader's measured world pose.
"""

import bisect
import math

from .adaptive_structure import interpolate, swept_separation
from .peer_safety import peer_motion_free


class Polyline:
    def __init__(self, points):
        self.points, self.distances = [], []
        for point in points:
            self.append(point)

    def append(self, point):
        point = tuple(point[:2])
        distance = 0.0 if not self.points else math.dist(self.points[-1], point)
        if self.points and distance < 1e-6:
            return
        self.distances.append((self.distances[-1] if self.distances else 0.0) + distance)
        self.points.append(point)

    @property
    def length(self):
        return self.distances[-1]

    def sample(self, distance):
        if len(self.points) == 1:
            return (*self.points[0], 0.0)
        distance = max(0.0, min(self.length, distance))
        k = max(1, min(len(self.points) - 1, bisect.bisect_left(self.distances, distance)))
        a, b = self.points[k - 1], self.points[k]
        ratio = (distance - self.distances[k - 1]) / (self.distances[k] - self.distances[k - 1])
        return (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]),
                math.atan2(b[1] - a[1], b[0] - a[0]))

    def project(self, point, previous, behind=0.20, ahead=0.65):
        if len(self.points) == 1:
            return 0.0, math.dist(point[:2], self.points[0])
        lower, upper = max(0.0, previous - behind), min(self.length, previous + ahead)
        first = max(1, bisect.bisect_left(self.distances, lower))
        last = min(len(self.points) - 1, bisect.bisect_right(self.distances, upper))
        best = (float('inf'), previous, float('inf'))
        for k in range(first, last + 1):
            a, b = self.points[k - 1], self.points[k]
            length = self.distances[k] - self.distances[k - 1]
            ratio = ((point[0] - a[0]) * (b[0] - a[0])
                     + (point[1] - a[1]) * (b[1] - a[1])) / (length * length)
            s = max(lower, min(upper, self.distances[k - 1] + max(0.0, min(1.0, ratio)) * length))
            projected = self.sample(s)
            error = math.dist(point[:2], projected[:2])
            score = error * error + 0.001 * (s - previous) ** 2
            if score < best[0]:
                best = (score, s, error)
        return best[1], best[2]


class LeaderFollower:
    def __init__(self, route, trail, initial_progress, order, segment_free, clearance,
                 gap=0.70, speed=0.16, narrow_speed=0.08):
        self.route = Polyline(route)
        self.trail = Polyline(trail)
        self.order = tuple(order)
        self.leader = order[0]
        self.gap, self.speed, self.narrow_speed = gap, speed, narrow_speed
        self.segment_free, self.clearance = segment_free, clearance
        self.progress = list(initial_progress)
        self.progress[self.leader] = 0.0
        self.reference_progress = self.progress.copy()
        self.errors = [0.0] * 3
        self.mode = 'single file'
        self.failure_reason = ''
        self.leader_recorded_progress = 0.0

    def observe_leader(self, poses):
        """Update the shared measured trail without imposing follower slots."""
        leader_pose = poses[self.leader]
        self.progress[self.leader], self.errors[self.leader] = self.route.project(
            leader_pose, self.progress[self.leader])
        last = self.trail.points[-1]
        moved = math.dist(last, leader_pose[:2])
        if moved > 0.40:
            self.failure_reason = 'leader pose jumped'
            return False
        if moved >= 0.015 and self.progress[self.leader] > self.leader_recorded_progress + 0.005:
            # A teleport or a newly blocked segment needs a fresh route and
            # trail connector; never draw a straight trail through a wall.
            if not self.segment_free(last, leader_pose):
                self.failure_reason = 'leader trail lost clearance or pose jumped'
                return False
            self.trail.append(leader_pose)
            self.leader_recorded_progress = self.progress[self.leader]
        if self.errors[self.leader] > 0.30:
            self.failure_reason = 'leader deviated more than 0.30 m from its route'
            return False
        return True

    def observe(self, poses):
        if not self.observe_leader(poses):
            return False
        for index in self.order[1:]:
            self.progress[index], self.errors[index] = self.trail.project(poses[index], self.progress[index])
        if max(self.errors) > 0.30:
            self.failure_reason = 'robot deviated more than 0.30 m from its route'
            return False
        return True

    def rows(self, poses, dt, horizon, mpc_dt):
        """Return atomic time-major references, all indices in robot-name order."""
        leader, middle, tail = self.order
        # Every follower stays behind a position the predecessor has reached.
        predecessor_s = {middle: self.trail.length, tail: self.progress[middle]}
        limits = [0.0] * 3
        limits[leader] = self.route.length
        for index in (middle, tail):
            limits[index] = max(self.progress[index], predecessor_s[index] - self.gap)

        speeds = [0.0] * 3
        narrow = False
        for index in self.order:
            line = self.route if index == leader else self.trail
            s = self.progress[index]
            clearance = min(self.clearance(line.sample(min(line.length, s + offset)))
                            for offset in (0.0, 0.20, 0.40))
            turn = abs(math.atan2(math.sin(line.sample(min(line.length, s + 0.30))[2]
                                           - line.sample(s)[2]),
                                  math.cos(line.sample(min(line.length, s + 0.30))[2]
                                           - line.sample(s)[2])))
            is_narrow = clearance < 0.32 or turn > 0.45
            narrow |= is_narrow
            speeds[index] = self.narrow_speed if is_narrow else self.speed
            # Each robot closes its own tracking error without freezing a
            # lagging follower's target together with the leader's target.
            speeds[index] *= max(0.0, min(1.0, (0.14 - self.errors[index]) / 0.08))

        gaps = (self.trail.length - self.progress[middle],
                self.progress[middle] - self.progress[tail])
        wait_factor = max(0.0, min(1.0, (self.gap + 0.35 - max(gaps)) / 0.25))
        speeds[leader] *= wait_factor
        if self.errors[middle] > 0.14 or self.errors[tail] > 0.14:
            speeds[leader] = 0.0
        self.mode = 'column / narrow passage' if narrow else 'single file'
        if speeds[leader] < 1e-4 and self.progress[leader] < self.route.length - 0.05:
            self.mode += ' / leader waits'

        for index in self.order:
            base = max(self.progress[index], self.reference_progress[index])
            self.reference_progress[index] = min(limits[index], self.progress[index] + 0.10,
                                                  base + max(0.0, dt) * speeds[index])
        rows = []
        for step in range(horizon + 1):
            row = [None] * 3
            for index in self.order:
                line = self.route if index == leader else self.trail
                s = min(limits[index], self.progress[index] + 0.22,
                        self.reference_progress[index] + speeds[index] * step * mpc_dt)
                row[index] = line.sample(s)
            # Do not send targets that geometrically overlap at hairpin turns.
            # Hold the rear robot's actual pose until its predecessor clears.
            for rank in (1, 2):
                index = self.order[rank]
                if any(math.dist(row[index][:2], row[j][:2]) < 0.40
                       or swept_separation(poses[index], row[index], poses[j], poses[j])
                       < min(0.40, math.dist(poses[index][:2], poses[j][:2])) - 1e-9
                       for j in self.order[:rank]):
                    # Arc-length spacing can look adequate on a hairpin even
                    # when two measured robot centers are already too close.
                    # Let the predecessor clear before the rear robot advances.
                    row[index] = tuple(poses[index][:3])
            rows.append(tuple(row))
        return rows

    def display_paths(self):
        paths = [None] * 3
        paths[self.leader] = self.route.points
        for index in self.order[1:]:
            first = max(0, bisect.bisect_left(self.trail.distances, self.progress[index]) - 1)
            paths[index] = self.trail.points[first:]
        return paths

    def remaining_segments(self):
        leader, middle, tail = self.order
        for line, s in ((self.route, self.progress[leader]), (self.trail, self.progress[tail])):
            first = max(0, bisect.bisect_left(line.distances, s) - 1)
            yield from zip(line.points[first:], line.points[first + 1:])


class ClearanceRecovery:
    """Slow, sequential movements away from the preferred clearance boundary."""
    def __init__(self, rows, segment_free, speed=0.05):
        self.rows = rows
        self.segment_free = segment_free
        self.times = [0.0]
        for a, b in zip(rows, rows[1:]):
            distance = max(math.dist(p[:2], q[:2]) for p, q in zip(a, b))
            self.times.append(self.times[-1] + max(0.1, distance / speed))

    def sample(self, seconds):
        if seconds >= self.times[-1]:
            return self.rows[-1]
        k = max(1, bisect.bisect_right(self.times, max(0.0, seconds)))
        ratio = (seconds - self.times[k - 1]) / (self.times[k] - self.times[k - 1])
        return tuple(interpolate(a, b, ratio) for a, b in zip(self.rows[k - 1], self.rows[k]))

    def transition_free(self, before, after):
        return (all(self.segment_free(a, b) for a, b in zip(before, after))
                and all(peer_motion_free(before[i], after[i], before[j], after[j])
                        for i, j in ((0, 1), (0, 2), (1, 2))))

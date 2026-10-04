"""Moving triangle/column references with independent follower catch-up paths."""

import math

from .adaptive_leader import AdaptiveLeaderFormation
from .formation_motion import team_motion_free
from .leader_follower import Polyline
from .peer_safety import peer_motion_free


def checked_team_row(before, proposed, order, segment_free):
    """Prefer a safe leader step, then accept concurrent follower steps."""
    leader = order[0]
    # The first MPC step also checks measured, stationary peers. Do not promise
    # a leader step that requires a follower to have moved first.
    leader_clear = (segment_free(before[leader], proposed[leader])
                    and all(peer_motion_free(before[leader], proposed[leader],
                                             before[i], before[i], preferred=0.38)
                            for i in order[1:]))
    candidate = list(proposed)
    if not leader_clear:
        candidate[leader] = (*before[leader][:2], proposed[leader][2])
    if team_motion_free(before, candidate, segment_free):
        return tuple(candidate)
    row = tuple((*a[:2], b[2]) for a, b in zip(before, proposed))
    for index in order:
        if index == leader and not leader_clear:
            continue
        candidate = list(row)
        candidate[index] = proposed[index]
        if team_motion_free(before, candidate, segment_free):
            row = tuple(candidate)
    return row


class FollowerTrack:
    """Cache checked joining routes; never stop another robot for tracking lag."""
    def __init__(self, segment_free, route_between):
        self.segment_free, self.route_between = segment_free, route_between
        self.line, self.goal, self.progress = None, None, 0.0
        self.next_route = 0.0

    def prepare(self, actual, goal, peers, elapsed, hint=None):
        # Straight joins and measured breadcrumb segments are inexpensive.
        if hint and all(self.segment_free(a, b) for a, b in zip(hint, hint[1:])):
            path = hint
        elif (self.segment_free(actual, goal)
              and all(peer_motion_free(actual, goal, p, p, preferred=0.38) for p in peers)):
            path = [actual, goal]
        else:
            if self.line is not None:
                s, error = self.line.project(actual, self.progress, ahead=0.80)
                remaining = [actual, self.line.sample(s)]
                remaining += [point for point, distance in
                              zip(self.line.points, self.line.distances) if distance > s]
                if (self.line.length > 1e-6 and error < 0.20
                        and math.dist(self.goal[:2], goal[:2]) < 0.10
                        and all(self.segment_free(a, b) for a, b in zip(remaining, remaining[1:]))
                        and all(peer_motion_free(a, b, p, p, preferred=0.38)
                                for a, b in zip(remaining, remaining[1:])
                                for p in peers)):
                    self.progress = s
                    return
            if elapsed < self.next_route:
                self.line = Polyline([actual])
                self.progress = 0.0
                return
            self.next_route = elapsed + 0.35
            path = self.route_between(actual, goal, peers)
            if not path or not all(self.segment_free(a, b) for a, b in zip(path, path[1:])):
                path = [actual]
        self.line, self.goal, self.progress = Polyline(path), tuple(goal), 0.0

    def references(self, actual, goals, speed, dt, horizon, mpc_dt, cap_to_slots):
        line = self.line or Polyline([actual])
        previous = self.progress
        result = []
        for k in range(horizon + 1):
            distance = min(line.length, self.progress + speed * (dt + k * mpc_dt))
            if cap_to_slots:
                limit, _ = line.project(goals[k], previous, behind=line.length, ahead=line.length)
                distance = max(previous, min(distance, limit))
            # A row must include each corner before the following segment.
            # Interpolating directly across two route segments may cut a wall.
            distance = next((s for s in line.distances if previous + 1e-6 < s < distance), distance)
            result.append((*line.sample(distance)[:2], goals[k][2]))
            previous = distance
        return result


class MovingAdaptiveFormation(AdaptiveLeaderFormation):
    """Switch target shape immediately and form it while the leader moves."""
    def __init__(self, *args, follower_speed=0.22, allow_triangle=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.follower_speed = follower_speed
        self.allow_triangle = allow_triangle
        self.tracks = {i: FollowerTrack(self.segment_free, self.route_between)
                       for i in self.order[1:]}
        self.leader_reference = 0.0

    def observe(self, poses):
        self.poses = tuple(tuple(p[:3]) for p in poses)
        if not self.column.observe_leader(poses):
            self.failure_reason = self.column.failure_reason
            return False
        # A follower joining a new shape may be far from its old slot. Route
        # its own catch-up movement instead of invalidating the leader's path.
        for index in self.order[1:]:
            self.column.progress[index], self.column.errors[index] = self.column.trail.project(
                poses[index], self.column.progress[index], ahead=0.80)
        return True

    def wide_safe(self, width, entering=False):
        if not super().wide_safe(width, entering=entering):
            return False
        if entering and self.poses is not None:
            # A leader outside a doorway is insufficient: both followers must
            # have room and checked joining segments before expansion starts.
            if any(self.clearance(p) < self.planning_radius + 0.05 for p in self.poses):
                return False
            joins = [self.configuration(0.0, width, side) for side in (self.side, -self.side)]
            if not any(all(self.segment_free(self.poses[i], target[i]) for i in self.order[1:])
                       for target in joins):
                return False
        return True

    def begin_transition(self, state, width):
        if state == 'triangle':
            # Use the cheaper left/right assignment; live sweeps still prevent
            # either follower from crossing its teammate.
            sides = [side for side in (self.side, -self.side)
                     if all(self.segment_free(self.poses[i], self.configuration(0.0, width, side)[i])
                            for i in self.order[1:])]
            self.side = min(sides or (self.side,), key=lambda side: sum(
                math.dist(self.poses[i][:2], self.configuration(0.0, width, side)[i][:2])
                for i in self.order[1:]))
        self.state, self.transition_state, self.width = state, state, width
        self.transition = None
        self.open_candidate = None
        self.mode = state + ' / followers joining'
        for track in self.tracks.values():
            track.line = None
            track.next_route = 0.0
        return True

    def decide(self):
        width = self.widths[0]
        if not self.allow_triangle:
            if self.state != 'column':
                self.begin_transition('column', 0.0)
            return
        if self.state == 'initializing':
            wide = self.wide_safe(width, entering=True)
            self.begin_transition('triangle' if wide else 'column', width if wide else 0.0)
            return
        if self.state == 'triangle':
            if not self.wide_safe(width):
                # No timer and no stopped assembly before a narrow passage.
                self.begin_transition('column', 0.0)
            return
        if not self.wide_safe(width, entering=True):
            self.open_candidate = None
            return
        if self.open_candidate is None:
            self.open_candidate, self.open_since = width, self.elapsed
        if self.elapsed - self.open_since + 1e-9 >= self.open_hold:
            self.begin_transition('triangle', width)

    def rows(self, poses, dt, horizon, mpc_dt):
        self.poses = tuple(tuple(p[:3]) for p in poses)
        dt = max(0.0, min(0.2, dt))
        self.elapsed += dt
        self.decide()
        if self.failure_reason:
            return [self.poses] * (horizon + 1)

        column, leader = self.column, self.leader
        s, line = column.progress[leader], column.route
        clearance = min(self.clearance(line.sample(s + offset)) for offset in (0.0, 0.20, 0.40))
        speed = self.narrow_speed if clearance < 0.32 else self.speed
        # Only the leader's own path error can reduce its nominal reference
        # speed. Follower lag, gap, yaw and assembly no longer stop it.
        speed *= max(0.0, min(1.0, (0.20 - column.errors[leader]) / 0.12))
        self.leader_reference = min(line.length, s + 0.08,
                                    max(s, self.leader_reference) + dt * speed)
        previous, leader_rows = s, []
        for k in range(horizon + 1):
            distance = min(line.length, s + 0.24, self.leader_reference + k * speed * mpc_dt)
            distance = next((d for d in line.distances if previous + 1e-6 < d < distance), distance)
            leader_rows.append(line.sample(distance))
            previous = distance

        ideal = []
        for point in leader_rows:
            if self.state == 'triangle':
                distance, _ = line.project(point, s, behind=0.0, ahead=0.40)
                row = list(self.configuration(max(0.0, distance - s), self.width))
            else:
                row = list(self.poses)
                limits = {self.middle: column.trail.length,
                          self.tail: column.progress[self.middle]}
                for index in self.order[1:]:
                    row[index] = column.trail.sample(max(column.progress[index], limits[index] - self.gap))
            row[leader] = point
            ideal.append(tuple(row))

        proposed = [[None] * 3 for _ in range(horizon + 1)]
        for k, point in enumerate(leader_rows):
            proposed[k][leader] = point
        for index in self.order[1:]:
            goals = [row[index] for row in ideal]
            hint = None
            if self.state == 'column':
                end, _ = column.trail.project(goals[-1], column.progress[index],
                                             behind=0.0, ahead=column.trail.length)
                start = min(end, column.progress[index])
                hint = [self.poses[index], column.trail.sample(start)]
                hint += [p for p, d in zip(column.trail.points, column.trail.distances) if start < d < end]
                hint.append(goals[-1])
            track = self.tracks[index]
            peers = [self.poses[j] for j in range(3) if j != index]
            track.prepare(self.poses[index], goals[-1], peers, self.elapsed, hint)
            joining = math.dist(self.poses[index][:2], goals[0][:2]) > 0.12
            follower_speed = self.transition_speed if joining else self.follower_speed
            if min(self.clearance(track.line.sample(track.progress + offset))
                   for offset in (0.0, 0.20)) < 0.32:
                follower_speed = min(follower_speed, self.narrow_speed)
            references = track.references(self.poses[index], goals, follower_speed, dt,
                                          horizon, mpc_dt, self.state == 'triangle')
            for k, point in enumerate(references):
                proposed[k][index] = point

        result, before = [], self.poses
        for row in proposed:
            before = checked_team_row(before, tuple(row), self.order, self.segment_free)
            result.append(before)
        self.leader_reference, _ = line.project(result[0][leader], s, behind=0.0, ahead=0.40)
        self.mode = ('triangle / tracking' if self.state == 'triangle' else 'column / tracking')
        if clearance < 0.32:
            self.mode += ' / narrow passage'
        if s < line.length - 0.05 and math.dist(result[0][leader][:2], self.poses[leader][:2]) < 1e-5:
            self.mode += ' / leader path blocked'
        self.last_rows = result
        return result

    def display_paths(self):
        paths = [None] * 3
        paths[self.leader] = self.column.route.points
        for index in self.order[1:]:
            paths[index] = (self.tracks[index].line.points if self.tracks[index].line is not None
                            else [self.poses[index][:2]] if self.poses else self.column.trail.points)
        return paths

"""Moving triangle/column references with independent follower catch-up paths."""

import bisect
import math

from .adaptive_leader import AdaptiveLeaderFormation
from .formation_motion import team_motion_free
from .leader_follower import Polyline
from .peer_safety import peer_motion_free


def checked_advance(line, previous, distance, segment_free, actual=None):
    """Cross small trail samples in one step when the swept chord is free."""
    start = line.sample(previous) if actual is None else actual
    if segment_free(start, line.sample(distance)):
        return distance
    # Retain a corner if skipping it would cut into an inflated obstacle.
    first = bisect.bisect_right(line.distances, previous + 1e-6)
    last = bisect.bisect_left(line.distances, distance)
    for k in range(last - 1, first - 1, -1):
        if segment_free(start, line.points[k]):
            return line.distances[k]
    return previous


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
    """Cache checked joining routes and sample them by distance, not vertex count."""
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
            # Measured breadcrumbs may be only 1.5 cm apart. Treating every
            # sample as a mandatory stop starves MPC of forward velocity.
            distance = checked_advance(line, previous, distance, self.segment_free,
                                       actual if k == 0 else None)
            result.append((*line.sample(distance)[:2], goals[k][2]))
            previous = distance
        return result


class MovingAdaptiveFormation(AdaptiveLeaderFormation):
    """Switch target shape immediately and form it while the leader moves."""
    def __init__(self, *args, follower_speed=0.22, narrow_follower_speed=0.12,
                 allow_triangle=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.follower_speed = follower_speed
        self.narrow_follower_speed = narrow_follower_speed
        self.allow_triangle = allow_triangle
        self.tracks = {i: FollowerTrack(self.segment_free, self.route_between)
                       for i in self.order[1:]}
        self.leader_reference = 0.0
        self.column_terminal_end = None
        self.column_narrow = False

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
        self.column_terminal_end = None
        self.column_narrow = False
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

    def column_spacing(self):
        """Limit convoy stretching using measured adjacent arc gaps."""
        column = self.column
        predecessors = (column.trail.length, column.progress[self.middle])
        factors = []
        for index, predecessor in zip(self.order[1:], predecessors):
            excess = max(0.0, predecessor - column.progress[index] - self.gap)
            # Off-trail joining distance also takes time to recover. A small
            # tracking error is normal; larger errors slow the predecessor.
            excess = max(excess, column.errors[index])
            factors.append(max(0.0, min(1.0, (0.38 - excess) / 0.30)))
        return factors

    def column_goals(self, leader_rows, leader_distances):
        """Use one moving leader anchor for both followers, with peer bounds."""
        column, trail = self.column, self.column.trail
        s, route = column.progress[self.leader], column.route
        endpoint = route.sample(route.length)
        if (self.column_terminal_end is None and route.length - s < 0.05
                and math.dist(self.poses[self.leader][:2], endpoint[:2]) < 0.05
                and self.segment_free(trail.points[-1], endpoint)):
            # Latch final slots once: the tail must not wait for the middle's
            # remaining tracking error to disappear before getting its goal.
            self.column_terminal_end = trail.length + math.dist(trail.points[-1], endpoint[:2])

        # Preview may move inside the desired gap, but never assumes that the
        # preceding robot has already vacated its measured position. XY sweeps
        # below still handle hairpins where arc spacing alone is insufficient.
        preview_gap = min(self.gap, max(0.46, self.gap - 0.20))
        predecessors = {self.middle: trail.length,
                        self.tail: column.progress[self.middle]}
        result = []
        for point, distance in zip(leader_rows, leader_distances):
            anchor = (self.column_terminal_end if self.column_terminal_end is not None
                      else trail.length + max(0.0, distance - s))
            row = list(self.poses)
            row[self.leader] = point
            for rank, index in enumerate(self.order[1:], 1):
                target = max(column.progress[index], min(
                    anchor - rank * self.gap, predecessors[index] - preview_gap))
                # Followers only use recorded, collision-checked trail. Future
                # leader motion shifts their slots along that existing trail.
                row[index] = self._spine(min(trail.length, target))
            result.append(tuple(row))
        return result

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
        spacing_factors = (1.0, 1.0)
        if self.state == 'column':
            spacing_factors = self.column_spacing()
            team_clearance = min(
                [clearance] + [self.clearance(self.poses[i]) for i in self.order[1:]]
                + [self.clearance(column.trail.sample(column.progress[i] + offset))
                   for i in self.order[1:] for offset in (0.0, 0.20, 0.40)])
            if team_clearance < 0.32:
                self.column_narrow = True
            elif team_clearance >= 0.36:
                self.column_narrow = False
            # Keep passage speed until the rear has cleared it, even if the
            # leader itself is already outside. Reserve follower catch-up speed.
            if self.column_narrow:
                speed = min(speed, self.narrow_speed)
            speed = min(speed, self.follower_speed, self.transition_speed)
            speed *= min(spacing_factors)
        speed *= max(0.0, min(1.0, (0.20 - column.errors[leader]) / 0.12))
        allowance = min(0.08, 0.5 * speed) if self.state == 'column' else 0.08
        self.leader_reference = min(line.length, s + allowance,
                                    max(s, self.leader_reference) + dt * speed)
        previous, leader_rows, leader_distances = s, [], []
        for k in range(horizon + 1):
            distance = min(line.length, s + 0.24, self.leader_reference + k * speed * mpc_dt)
            distance = checked_advance(line, previous, distance, self.segment_free,
                                       self.poses[leader] if k == 0 else None)
            leader_rows.append(line.sample(distance))
            leader_distances.append(distance)
            previous = distance

        ideal = []
        if self.state == 'triangle':
            for point, distance in zip(leader_rows, leader_distances):
                row = list(self.configuration(max(0.0, distance - s), self.width))
                row[leader] = point
                ideal.append(tuple(row))
        else:
            ideal = self.column_goals(leader_rows, leader_distances)

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
                limit = self.narrow_follower_speed if self.state == 'column' else self.narrow_speed
                follower_speed = min(follower_speed, limit)
            if self.state == 'column' and index == self.middle:
                follower_speed *= spacing_factors[1]
            references = track.references(self.poses[index], goals, follower_speed, dt,
                                          horizon, mpc_dt, True)
            for k, point in enumerate(references):
                proposed[k][index] = point

        result, before = [], self.poses
        for row in proposed:
            before = checked_team_row(before, tuple(row), self.order, self.segment_free)
            result.append(before)
        self.leader_reference, _ = line.project(result[0][leader], s, behind=0.0, ahead=0.40)
        self.mode = ('triangle / tracking' if self.state == 'triangle' else 'column / tracking')
        if clearance < 0.32 or (self.state == 'column' and self.column_narrow):
            self.mode += ' / narrow passage'
        if self.state == 'column' and min(spacing_factors) < 0.95:
            self.mode += ' / closing gaps'
        if self.state == 'column' and self.column_terminal_end is not None:
            self.mode += ' / final slots'
        if (s < line.length - 0.05 and speed > 1e-5
                and math.dist(result[0][leader][:2], self.poses[leader][:2]) < 1e-5):
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

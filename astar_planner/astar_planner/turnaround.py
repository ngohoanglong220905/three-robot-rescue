"""Turn the leader onto its route while followers join along checked paths."""

import math

from .adaptive_structure import swept_separation, wrap
from .formation_motion import GatedTeamMotion, assemble_followers, team_motion_free
from .leader_follower import Polyline
from .peer_safety import peer_motion_free


def leader_route_blockers(poses, route, order, lookahead=1.20):
    """Find measured followers intersecting the leader's upcoming footprint."""
    if len(route) < 2:
        return {}
    line = Polyline(route)
    end = min(line.length, lookahead)
    points = [line.sample(0.0)]
    points += [p for p, d in zip(line.points, line.distances) if 0.0 < d < end]
    points.append(line.sample(end))
    blockers = {}
    for index in order[1:]:
        if any(not peer_motion_free(a, b, poses[index], poses[index], preferred=0.40)
               for a, b in zip(points, points[1:])):
            distance, _ = line.project(poses[index], 0.0, behind=0.0, ahead=end)
            blockers[index] = distance
    return blockers


def team_heading(poses, order):
    leader, a, b = order
    dx = poses[leader][0] - 0.5 * (poses[a][0] + poses[b][0])
    dy = poses[leader][1] - 0.5 * (poses[a][1] + poses[b][1])
    return math.atan2(dy, dx) if math.hypot(dx, dy) > 0.20 else poses[leader][2]


def needs_turnaround(poses, route, order):
    if len(route) < 2:
        return False
    line = Polyline(route)
    if line.length < 0.25:
        return False
    point = line.sample(min(0.45, line.length))
    dx, dy = point[0] - poses[order[0]][0], point[1] - poses[order[0]][1]
    if math.hypot(dx, dy) < 0.12:
        return False
    incoming = line.sample(min(0.06, line.length))[2]
    outgoing = math.atan2(dy, dx)
    future_heading = line.sample(min(0.45, line.length))[2]
    return (abs(wrap(outgoing - team_heading(poses, order))) > math.radians(105.0)
            or abs(wrap(incoming - poses[order[0]][2])) > math.radians(45.0)
            or abs(wrap(future_heading - incoming)) > math.radians(50.0))


class RollingTurnMotion:
    """Independent measured progress; release the leader as soon as it is aligned."""
    def __init__(self, actual, lines, targets, delays, order, segment_free, clearance,
                 speed=0.16, follower_speed=0.18, narrow_speed=0.08, alignment=0.20,
                 require_alignment=True, leader_completion=False, follow_indices=()):
        self.lines, self.targets, self.delays = lines, targets, delays
        self.order, self.leader = tuple(order), order[0]
        self.segment_free, self.clearance = segment_free, clearance
        self.speed, self.follower_speed, self.narrow_speed = speed, follower_speed, narrow_speed
        self.alignment = alignment
        self.require_alignment, self.leader_completion = require_alignment, leader_completion
        self.follow_indices = dict(follow_indices)
        self.progress, self.reference_progress = [0.0] * 3, [0.0] * 3
        self.finished, self.failure_reason = False, ''
        self.mode = 'turn / aligning leader; followers tracking'

    def checked_row(self, before, proposed):
        if team_motion_free(before, proposed, self.segment_free):
            return proposed
        # Keep yaw commands active when translation is held. Prefer the leader
        # when it can pass safely, otherwise let followers clear its way.
        row = tuple((*a[:2], b[2]) for a, b in zip(before, proposed))
        for index in self.order:
            candidate = list(row)
            candidate[index] = proposed[index]
            if team_motion_free(before, candidate, self.segment_free):
                row = tuple(candidate)
        return row

    def rows(self, actual, dt, horizon, mpc_dt):
        errors = []
        for index, line in enumerate(self.lines):
            self.progress[index], error = line.project(actual[index], self.progress[index])
            errors.append(error)
        if max(errors) > 0.30:
            self.failure_reason = 'flexible turn tracking error exceeds 0.30 m'
            return [tuple(actual)] * (horizon + 1)
        leader = self.leader
        heading = self.lines[leader].sample(self.progress[leader] + 0.06)[2]
        aligned = (not self.require_alignment
                   or abs(wrap(actual[leader][2] - heading)) <= self.alignment)
        speeds = [0.0] * 3
        active_flags = [False] * 3
        for index, line in enumerate(self.lines):
            active = (aligned if index == leader else
                      self.progress[leader] + 1e-6 >= self.delays[index])
            active_flags[index] = active
            speed = self.speed if index == leader else self.follower_speed
            if min(self.clearance(line.sample(self.progress[index] + offset))
                   for offset in (0.0, 0.20)) < 0.32:
                speed = min(speed, self.narrow_speed)
            speeds[index] = speed * max(0.0, min(1.0, (0.14 - errors[index]) / 0.08)) if active else 0.0
            self.reference_progress[index] = min(
                line.length, self.progress[index] + 0.06,
                max(self.progress[index], self.reference_progress[index]) + max(0.0, dt) * speeds[index])

        result, before = [], tuple(actual)
        preview_progress = self.progress.copy()
        for k in range(horizon + 1):
            proposed = []
            for index, line in enumerate(self.lines):
                distance = min(line.length, self.progress[index] + 0.22,
                               self.reference_progress[index] + speeds[index] * k * mpc_dt)
                if index in self.follow_indices:
                    # A non-obstructing follower tracks behind measured leader
                    # progress while its teammate yields, rather than parking.
                    gap = self.follow_indices[index]
                    behind = self.progress[leader] - gap
                    spine = self.lines[leader].sample(max(0.0, behind))
                    slot = (spine[0] + min(0.0, behind) * math.cos(spine[2]),
                            spine[1] + min(0.0, behind) * math.sin(spine[2]))
                    limit, _ = line.project(slot, self.progress[index],
                                            behind=line.length, ahead=line.length)
                    distance = min(distance, max(self.progress[index], limit))
                # Expose a corner before the following segment so reference
                # interpolation cannot shortcut the inside of a tight bend.
                distance = next((s for s in line.distances
                                 if preview_progress[index] + 1e-6 < s < distance), distance)
                if not active_flags[index]:
                    pose = (*actual[index][:2], heading if index == leader else self.targets[index][2])
                else:
                    point = line.sample(distance)
                    pose = (*point[:2], heading if index == leader
                            else self.targets[index][2])
                proposed.append(pose)
            row = self.checked_row(before, tuple(proposed))
            result.append(row)
            for index, line in enumerate(self.lines):
                preview_progress[index], _ = line.project(row[index], preview_progress[index])
            before = row
        for index, line in enumerate(self.lines):
            self.reference_progress[index], _ = line.project(result[0][index], self.progress[index])
        self.mode = ('turn / leader moving; followers tracking' if aligned else
                     'turn / aligning leader; followers tracking')
        self.finished = all(math.dist(p[:2], q[:2]) < 0.05
                            for p, q in zip(actual, self.targets))
        if self.leader_completion:
            # Resume ordinary following as soon as the leader has passed the
            # obstruction; do not wait for perfect bay arrival or reassembly.
            self.finished = math.dist(actual[leader][:2], self.targets[leader][:2]) < 0.05
            moving = math.dist(result[0][leader][:2], actual[leader][:2]) > 1e-5
            self.mode = ('yield / leader passing; followers clear corridor' if moving else
                         'yield / follower clears leader corridor')
        return result

    def display_paths(self):
        return [line.points for line in self.lines]

    def remaining_segments(self):
        for index, line in enumerate(self.lines):
            points = [line.sample(self.progress[index])[:2]]
            points += [p for p, s in zip(line.points, line.distances) if s > self.progress[index] + 1e-6]
            yield from zip(points, points[1:])


class TurnaroundPlanner:
    def __init__(self, segment_free, clearance, route_between, order,
                 gap=0.70, half_width=0.50, planning_radius=0.18,
                 speed=0.16, follower_speed=0.18, narrow_speed=0.08,
                 angular_speed=1.0, alignment=0.20):
        self.segment_free, self.clearance, self.route_between = segment_free, clearance, route_between
        self.order, self.gap, self.half_width = tuple(order), gap, half_width
        self.planning_radius = planning_radius
        self.speed, self.follower_speed, self.narrow_speed = speed, follower_speed, narrow_speed
        self.angular_speed, self.alignment = angular_speed, alignment

    def yield_followers(self, actual, route):
        """Move only obstructing followers to the nearest reachable passing bay."""
        blockers = leader_route_blockers(actual, route, self.order)
        if not blockers:
            return None
        line = Polyline(route)
        advance = min(line.length, max(0.65, max(blockers.values()) + 0.50))
        prefix = [line.points[0]]
        prefix += [p for p, s in zip(line.points, line.distances) if 0.0 < s < advance]
        prefix.append(line.sample(advance)[:2])
        corridor = Polyline(prefix)
        targets, lines = list(actual), [Polyline([p[:2]]) for p in actual]
        leader = self.order[0]
        lines[leader], targets[leader] = corridor, corridor.sample(corridor.length)
        for index in self.order[1:]:
            if index not in blockers:
                continue
            heading = line.sample(blockers[index] + 0.02)[2]
            candidates = []
            for lateral in (0.16, 0.24, 0.34, 0.45, 0.55, 0.70, 0.85):
                for sign in (1.0, -1.0):
                    for forward in (0.0, -0.15, 0.15):
                        point = (actual[index][0] + forward * math.cos(heading)
                                 - sign * lateral * math.sin(heading),
                                 actual[index][1] + forward * math.sin(heading)
                                 + sign * lateral * math.cos(heading))
                        if self.clearance(point) < self.planning_radius + 0.025:
                            continue
                        if any(swept_separation(a, b, point, point) < 0.42
                               for a, b in zip(corridor.points, corridor.points[1:])):
                            continue
                        peers = [actual[j] for j in range(3) if j != index]
                        if any(math.dist(point, p[:2]) < 0.40 for p in peers):
                            continue
                        # Include earlier bay assignments in the static endpoint
                        # check, while routing around measured current peers.
                        if any(math.dist(point, targets[j][:2]) < 0.40
                               for j in self.order[1:] if j != index):
                            continue
                        candidates.append((math.dist(actual[index][:2], point), point))
            for _, point in sorted(candidates):
                peers = [actual[j] for j in range(3) if j != index]
                if (self.segment_free(actual[index], point)
                        and all(peer_motion_free(actual[index], point, p, p, preferred=0.38)
                                for p in peers)):
                    path = [actual[index], point]
                else:
                    path = self.route_between(actual[index], point, peers)
                if not path or not all(self.segment_free(a, b) for a, b in zip(path, path[1:])):
                    continue
                lines[index] = Polyline(path)
                targets[index] = (*point, heading)
                break
            else:
                return None
        following = {}
        for rank, index in enumerate(self.order[1:], 1):
            if index in blockers:
                continue
            distance = advance - rank * self.gap
            point = corridor.sample(max(0.0, distance))
            goal = (point[0] + min(0.0, distance) * math.cos(point[2]),
                    point[1] + min(0.0, distance) * math.sin(point[2]), point[2])
            if (self.clearance(goal) < self.planning_radius
                    or any(math.dist(goal[:2], targets[j][:2]) < 0.40
                           for j in range(3) if j != index)):
                continue
            if self.segment_free(actual[index], goal):
                path = [actual[index], goal]
            else:
                peers = [actual[j] for j in self.order[1:] if j != index]
                path = self.route_between(actual[index], goal, peers)
            if path and all(self.segment_free(a, b) for a, b in zip(path, path[1:])):
                lines[index], targets[index] = Polyline(path), goal
                following[index] = rank * self.gap
        return RollingTurnMotion(
            actual, lines, tuple(targets), [0.0] * 3, self.order,
            self.segment_free, self.clearance, self.speed, self.follower_speed,
            self.narrow_speed, self.alignment,
            require_alignment=False, leader_completion=True, follow_indices=following)

    def rolling(self, actual, line, advance):
        leader, first, second = self.order
        prefix = [line.points[0]] + [p for p, s in zip(line.points, line.distances) if 0.0 < s < advance]
        prefix.append(line.sample(advance)[:2])
        leader_line = Polyline(prefix)
        end = leader_line.sample(advance)
        for width in (self.half_width, 0.0):
            choices = []
            for side in (1.0, -1.0) if width else (1.0,):
                targets = list(actual)
                targets[leader] = end
                for rank, index in enumerate((first, second), 1):
                    if width:
                        lateral = side * width * (1.0 if rank == 1 else -1.0)
                        targets[index] = (end[0] - self.gap * math.cos(end[2]) - lateral * math.sin(end[2]),
                                          end[1] - self.gap * math.sin(end[2]) + lateral * math.cos(end[2]), end[2])
                    else:
                        distance = advance - rank * self.gap
                        point = leader_line.sample(max(0.0, distance))
                        targets[index] = (point[0] + min(0.0, distance) * math.cos(point[2]),
                                          point[1] + min(0.0, distance) * math.sin(point[2]), point[2])
                if not team_motion_free(targets, targets, self.segment_free):
                    continue
                lines = [None] * 3
                lines[leader] = leader_line
                for index in (first, second):
                    # The leader will leave; its current pose is checked by the
                    # shared sweeps and live MPC, rather than permanently masked.
                    peers = [actual[j] for j in (first, second) if j != index]
                    if (self.segment_free(actual[index], targets[index])
                            and all(peer_motion_free(actual[index], targets[index], p, p,
                                                     preferred=0.38) for p in peers)):
                        # Avoid a cell-center detour when a follower can hold
                        # its position or join along a clear straight segment.
                        path = [actual[index], targets[index]]
                    else:
                        path = self.route_between(actual[index], targets[index], peers)
                    if not path:
                        break
                    lines[index] = Polyline(path)
                else:
                    wait = min(self.gap, 0.5 * advance)
                    for waits in ((0.0, 0.0), (wait, 0.0), (0.0, wait), (wait, wait)):
                        delays = [0.0] * 3
                        delays[first], delays[second] = waits
                        duration = self.rolling_duration(actual, lines, delays)
                        if duration is not None:
                            choices.append((duration, lines, tuple(targets), delays))
            if choices:
                _, lines, targets, delays = min(choices, key=lambda item: item[0])
                return RollingTurnMotion(actual, lines, targets, delays, self.order,
                                         self.segment_free, self.clearance, self.speed,
                                         self.follower_speed, self.narrow_speed, self.alignment)
        return None

    def rolling_duration(self, actual, lines, delays):
        """Check every corner of a concurrent nominal turn before accepting it."""
        leader = self.order[0]
        rotation = abs(wrap(lines[leader].sample(0.0)[2] - actual[leader][2])) / self.angular_speed
        starts, speeds = [0.0] * 3, [self.follower_speed] * 3
        starts[leader], speeds[leader] = rotation, self.speed
        for index in self.order[1:]:
            if delays[index] > 0.0:
                starts[index] = rotation + delays[index] / self.speed
        events = {0.0}
        for index, line in enumerate(lines):
            events.update(starts[index] + s / speeds[index] for s in line.distances)
        before = tuple(actual)
        for seconds in sorted(events):
            row = tuple(line.sample(max(0.0, seconds - starts[index]) * speeds[index])
                        for index, line in enumerate(lines))
            if not team_motion_free(before, row, self.segment_free):
                return None
            before = row
        return max(events)

    def build(self, actual, route):
        line = Polyline(route)
        leader, first, second = self.order
        advance = min(line.length, max(1.40, 2.0 * self.gap))
        rolling = self.rolling(actual, line, advance)
        if rolling is not None:
            return rolling
        steps = max(1, math.ceil(advance / 0.08))
        distances = sorted(set([advance * k / steps for k in range(steps + 1)]
                               + [s for s in line.distances if 0.0 < s < advance]))
        # Keep every D* polyline corner; regular sampling must not cut it off.
        leader_path = [line.sample(s) for s in distances]
        outgoing = line.sample(min(0.40, line.length))
        heading = math.atan2(outgoing[1] - actual[leader][1], outgoing[0] - actual[leader][0])
        candidates = []
        bays = {1: [], -1: []}
        for sign in (1, -1):
            for longitudinal in (0.35, 0.65, 0.95):
                center = line.sample(min(longitudinal, advance))
                for lateral in (0.55, 0.70, 0.85):
                    point = (center[0] - sign * lateral * math.sin(heading),
                             center[1] + sign * lateral * math.cos(heading))
                    if self.clearance(point) < self.planning_radius + 0.025:
                        continue
                    if any(swept_separation(a, b, point, point) < 0.44
                           for a, b in zip(leader_path, leader_path[1:])):
                        continue
                    bays[sign].append(point)
        for side in (1, -1):
            for a in bays[side]:
                for b in bays[-side]:
                    if math.dist(a, b) < 0.50:
                        continue
                    travel = math.dist(actual[first][:2], a) + math.dist(actual[second][:2], b)
                    if max(math.dist(actual[first][:2], a), math.dist(actual[second][:2], b)) > 1.8:
                        continue
                    parked = list(actual)
                    parked[first], parked[second] = (*a, actual[first][2]), (*b, actual[second][2])
                    candidates.append((travel, tuple(parked)))

        # Rank geometric bays before spending D* work on stationary-peer routes.
        for _, parked in sorted(candidates, key=lambda item: item[0])[:8]:
            yielding = assemble_followers(actual, parked, self.order, self.segment_free, self.route_between)
            if yielding is None:
                continue
            turned = list(parked)
            turned[leader] = (*actual[leader][:2], heading)
            turned = tuple(turned)
            passing = [turned]
            for point in leader_path[1:]:
                row = list(turned)
                row[leader] = point
                if not team_motion_free(passing[-1], row, self.segment_free):
                    break
                passing.append(tuple(row))
            else:
                joined = self.rejoin(passing[-1], line, advance)
                if joined is None:
                    continue
                mode, rows = joined
                stages = [
                    ('turnaround / followers yield', yielding, ()),
                    ('turnaround / leader turns', [parked, turned], (leader,)),
                    ('turnaround / leader passes', passing, ()),
                    ('turnaround / reforming ' + mode, rows, ()),
                ]
                return GatedTeamMotion(stages, speed=self.follower_speed,
                                       angular_speed=min(0.85, self.angular_speed))
        return None

    def rejoin(self, actual, line, advance):
        leader, first, second = self.order
        x, y, yaw = actual[leader]
        # Triangle is always preferred; a column is the fallback when the rear
        # slots or their collision-free approach do not fit the available room.
        for width in (self.half_width, 0.0):
            alternatives = []
            for side in (1, -1):
                target = list(actual)
                for rank, index in enumerate((first, second), 1):
                    if width:
                        sign = side if rank == 1 else -side
                        target[index] = (x - self.gap * math.cos(yaw) - sign * width * math.sin(yaw),
                                         y - self.gap * math.sin(yaw) + sign * width * math.cos(yaw), yaw)
                    else:
                        distance = advance - rank * self.gap
                        if distance >= 0.0:
                            target[index] = line.sample(distance)
                        else:
                            sx, sy, syaw = line.sample(0.0)
                            target[index] = (sx + distance * math.cos(syaw),
                                             sy + distance * math.sin(syaw), syaw)
                if not team_motion_free(target, target, self.segment_free):
                    continue
                rows = assemble_followers(actual, target, self.order, self.segment_free,
                                          self.route_between, max_distance=2.2)
                if rows is not None:
                    travel = sum(max(math.dist(a[:2], b[:2]) for a, b in zip(p, q))
                                 for p, q in zip(rows, rows[1:]))
                    alternatives.append((travel, rows))
            if alternatives:
                return ('triangle' if width else 'column', min(alternatives, key=lambda item: item[0])[1])
        return None

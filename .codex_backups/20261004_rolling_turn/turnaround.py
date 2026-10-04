"""Park followers to either side, turn/pass the leader, then rejoin behind it."""

import math

from .adaptive_structure import swept_separation, wrap
from .formation_motion import GatedTeamMotion, assemble_followers, team_motion_free
from .leader_follower import Polyline


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
    return abs(wrap(math.atan2(dy, dx) - team_heading(poses, order))) > math.radians(105.0)


class TurnaroundPlanner:
    def __init__(self, segment_free, clearance, route_between, order,
                 gap=0.70, half_width=0.50, planning_radius=0.18):
        self.segment_free, self.clearance, self.route_between = segment_free, clearance, route_between
        self.order, self.gap, self.half_width = tuple(order), gap, half_width
        self.planning_radius = planning_radius

    def build(self, actual, route):
        line = Polyline(route)
        leader, first, second = self.order
        advance = min(line.length, max(1.40, 2.0 * self.gap))
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
                return GatedTeamMotion(stages)
        return None

    def rejoin(self, actual, line, advance):
        leader, first, second = self.order
        x, y, yaw = actual[leader]
        # Triangle is always preferred; a column is the fallback when the rear
        # slots or their collision-free approach do not fit the available room.
        for width in (self.half_width, 0.8 * self.half_width, 0.6 * self.half_width, 0.0):
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
                    travel = sum(math.dist(a[:2], b[:2]) for p, q in zip(rows, rows[1:]) for a, b in zip(p, q))
                    alternatives.append((travel, rows))
            if alternatives:
                return ('triangle' if width else 'column', min(alternatives, key=lambda item: item[0])[1])
        return None

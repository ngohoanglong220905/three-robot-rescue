"""Swept team checks and measured-arrival gates for formation manoeuvres."""

import bisect
import itertools
import math

from .adaptive_structure import interpolate, wrap
from .leader_follower import Polyline
from .peer_safety import peer_motion_free


def team_motion_free(before, after, segment_free):
    if not all(segment_free(a, b) for a, b in zip(before, after)):
        return False
    for i, j in ((0, 1), (0, 2), (1, 2)):
        if not peer_motion_free(before[i], after[i], before[j], after[j], preferred=0.38):
            return False
    return True


def assemble_followers(actual, target, order, segment_free, route_between, max_distance=1.8):
    """Leader holds; route followers together or sequentially around teammates."""
    if math.dist(actual[order[0]][:2], target[order[0]][:2]) > 1e-6:
        return None
    if max(math.dist(a[:2], b[:2]) for a, b in zip(actual, target)) > max_distance:
        return None
    if team_motion_free(actual, target, segment_free):
        return [tuple(actual), tuple(target)]
    choices = []
    # Try simultaneous routed movements before making one follower wait. Use
    # every corner from both routes as an event, so interpolation never cuts
    # across a wall. At equal speed the shorter route finishes first and holds.
    routes = {}
    for index in order[1:]:
        if math.dist(actual[index][:2], target[index][:2]) < 0.015:
            routes[index] = Polyline([actual[index][:2]])
            continue
        peers = [p for j, p in enumerate(actual) if j != index]
        route = route_between(actual[index], target[index], peers)
        if not route:
            break
        routes[index] = Polyline(route)
    else:
        events = sorted({distance for line in routes.values() for distance in line.distances})
        rows = [tuple(actual)]
        for distance in events[1:]:
            after = list(actual)
            for index, line in routes.items():
                after[index] = (*line.sample(distance)[:2], target[index][2])
            if not team_motion_free(rows[-1], after, segment_free):
                break
            rows.append(tuple(after))
        else:
            total = sum(line.length for line in routes.values())
            if total <= 5.0 and team_motion_free(rows[-1], target, segment_free):
                choices.append(rows + [tuple(target)])
    for moving in itertools.permutations(order[1:]):
        rows, current, total = [tuple(actual)], list(actual), 0.0
        for index in moving:
            if math.dist(current[index][:2], target[index][:2]) < 0.015:
                continue
            peers = [p for j, p in enumerate(current) if j != index]
            route = route_between(current[index], target[index], peers)
            if route is None:
                break
            valid = True
            for point in route[1:]:
                after = list(current)
                after[index] = (*point[:2], target[index][2])
                if not team_motion_free(current, after, segment_free):
                    valid = False
                    break
                total += math.dist(current[index][:2], after[index][:2])
                rows.append(tuple(after))
                current = after
            if not valid or total > 5.0:
                break
        else:
            if team_motion_free(current, target, segment_free):
                choices.append(rows + [tuple(target)])

    def duration(rows):
        return sum(max(math.dist(a[:2], b[:2]) for a, b in zip(p, q))
                   for p, q in zip(rows, rows[1:]))

    return min(choices, key=duration) if choices else None


class GatedTeamMotion:
    """Only expose one stage to MPC; advance after measured arrival, including yaw."""
    def __init__(self, stages, speed=0.08, angular_speed=0.45):
        # Stage: (mode, rows, indices whose final yaw must also be reached).
        self.stages = stages
        self.timelines = []
        for _, rows, _ in stages:
            times = [0.0]
            for before, after in zip(rows, rows[1:]):
                move = max(math.dist(a[:2], b[:2]) for a, b in zip(before, after))
                turn = max(abs(wrap(b[2] - a[2])) for a, b in zip(before, after))
                times.append(times[-1] + max(move / speed, turn / angular_speed, 0.05))
            self.timelines.append(times)
        self.stage_index, self.phase = 0, 0.0
        self.finished = False
        self.mode = stages[0][0]

    def sample(self, seconds):
        rows, times = self.stages[self.stage_index][1], self.timelines[self.stage_index]
        if seconds >= times[-1]:
            return rows[-1]
        k = max(1, bisect.bisect_right(times, max(0.0, seconds)))
        ratio = (seconds - times[k - 1]) / (times[k] - times[k - 1])
        return tuple(interpolate(a, b, ratio) for a, b in zip(rows[k - 1], rows[k]))

    def rows(self, actual, dt, horizon, mpc_dt):
        target = self.sample(self.phase)
        xy_error = max(math.dist(a[:2], b[:2]) for a, b in zip(actual, target))
        yaw_indices = self.stages[self.stage_index][2]
        yaw_error = max((abs(wrap(actual[i][2] - target[i][2])) for i in yaw_indices), default=0.0)
        rate = max(0.0, min(1.0, (0.10 - xy_error) / 0.07, (0.40 - yaw_error) / 0.25))
        end_time = self.timelines[self.stage_index][-1]
        self.phase = min(end_time, self.phase + max(0.0, dt) * rate)
        result = [self.sample(self.phase + rate * k * mpc_dt) for k in range(horizon + 1)]
        end = self.stages[self.stage_index][1][-1]
        reached = max(math.dist(a[:2], b[:2]) for a, b in zip(actual, end)) < 0.04
        aligned = all(abs(wrap(actual[i][2] - end[i][2])) < 0.12 for i in yaw_indices)
        if self.phase >= end_time and reached and aligned:
            if self.stage_index == len(self.stages) - 1:
                self.finished = True
            else:
                self.stage_index += 1
                self.phase = 0.0
                self.mode = self.stages[self.stage_index][0]
        return result

    def display_paths(self):
        rows = [row for _, stage, _ in self.stages[self.stage_index:] for row in stage]
        return [[row[i] for row in rows] for i in range(3)]

    def remaining_segments(self):
        for index in range(self.stage_index, len(self.stages)):
            rows = self.stages[index][1]
            if index == self.stage_index:
                first = max(0, bisect.bisect_left(self.timelines[index], self.phase) - 1)
                rows = rows[first:]
            for before, after in zip(rows, rows[1:]):
                yield from zip(before, after)

"""Collision-checked virtual structure with one time parameter for the team.

Rows always contain robot1, robot2, robot3 in that order. Robot2 anchors the
structure; the flank robots fold into a convoy along the anchor's path.
This module has no ROS dependencies.
"""

import bisect
import itertools
import math


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def interpolate(a, b, fraction):
    return (
        a[0] + fraction * (b[0] - a[0]),
        a[1] + fraction * (b[1] - a[1]),
        wrap(a[2] + fraction * wrap(b[2] - a[2])),
    )


def swept_separation(a0, a1, b0, b1):
    """Minimum center distance during two simultaneous straight movements."""
    rx, ry = a0[0] - b0[0], a0[1] - b0[1]
    dx = a1[0] - a0[0] - b1[0] + b0[0]
    dy = a1[1] - a0[1] - b1[1] + b0[1]
    norm = dx * dx + dy * dy
    fraction = max(0.0, min(1.0, -(rx * dx + ry * dy) / norm)) if norm > 1e-12 else 0.0
    return math.hypot(rx + fraction * dx, ry + fraction * dy)


class AdaptiveStructure:
    def __init__(self, segment_free, route_between, width=0.8, gap=0.65,
                 separation=0.44, speed=0.16, angular_speed=0.65):
        self.segment_free = segment_free
        self.route_between = route_between
        self.width = width
        self.gap = gap
        self.separation = separation
        self.speed = speed
        self.angular_speed = angular_speed
        if min(width, gap, separation, speed, angular_speed) <= 0.0:
            raise ValueError('Formation dimensions and speeds must be positive')
        self.rows = []
        self.times = []
        self.widths = []

    def transition_free(self, before, after):
        return (
            all(self.segment_free(a, b) for a, b in zip(before, after))
            and all(
                swept_separation(before[i], after[i], before[j], after[j])
                >= self.separation
                for i, j in ((0, 1), (0, 2), (1, 2))
            )
        )

    def assemble(self, actual, target):
        if self.transition_free(actual, target):
            return [actual, target]
        # A direct change may cross the center robot. Route one robot at a time
        # while treating the other two positions as occupied. Try each ordering.
        for order in itertools.permutations(range(3)):
            rows = [actual]
            current = list(actual)
            for index in order:
                peers = [p for i, p in enumerate(current) if i != index]
                route = self.route_between(current[index], target[index], peers)
                if route is None:
                    break
                valid = True
                for point in route[1:]:
                    updated = list(current)
                    updated[index] = (point[0], point[1], target[index][2])
                    if not self.transition_free(current, updated):
                        valid = False
                        break
                    rows.append(tuple(updated))
                    current = updated
                if not valid:
                    break
            else:
                if self.transition_free(current, target):
                    return rows + [target]
        return None

    def build(self, points, headings, cumulative, actual, sample_path):
        """Dynamic programming over width candidates along the D* path.

        Validate swept motions, including relative motion between robots.
        Backtracking lets a narrow section cause folding *before* it is reached.
        Negative path distances extend the initial tangent; followers never
        collapse onto the leader at distance zero.
        """
        normal = (-math.sin(headings[0]), math.cos(headings[0]))
        projection = [
            (actual[i][0] - actual[1][0]) * normal[0]
            + (actual[i][1] - actual[1][1]) * normal[1]
            for i in (0, 2)
        ]
        sign = 1.0 if projection[0] >= projection[1] else -1.0
        signs = (sign, 0.0, -sign)
        candidates = [self.width * k / 8.0 for k in range(9)]

        def configuration(distance, width):
            compact = 1.0 - width / self.width
            lags = (self.gap * compact, 0.0, 2.0 * self.gap * compact)
            row = []
            for i in range(3):
                s = distance - lags[i]
                if s < 0.0:
                    yaw = headings[0]
                    x = points[0][0] + s * math.cos(yaw)
                    y = points[0][1] + s * math.sin(yaw)
                else:
                    x, y, yaw = sample_path(points, headings, cumulative, s)
                row.append((x - signs[i] * width * math.sin(yaw),
                            y + signs[i] * width * math.cos(yaw), yaw))
            return tuple(row)

        layers = []
        for distance in cumulative:
            layer = {}
            for k, width in enumerate(candidates):
                row = configuration(distance, width)
                if self.transition_free(row, row):
                    layer[k] = row
            if not layer:
                return False
            layers.append(layer)

        def movement(a, b):
            return max(math.hypot(p[0] - q[0], p[1] - q[1]) for p, q in zip(a, b))

        # Solve backwards first. Initial assembly must select a width that can
        # actually reach the end, including any imminent narrow passage.
        costs = {k: 0.0 for k in layers[-1]}
        choices = [{} for _ in layers]
        for step in range(len(layers) - 2, -1, -1):
            next_costs = {}
            distance = cumulative[step + 1] - cumulative[step]
            for k, row in layers[step].items():
                for following, cost in costs.items():
                    if abs(k - following) > 1:
                        continue
                    after = layers[step + 1][following]
                    if not self.transition_free(row, after):
                        continue
                    trial = (cost + movement(row, after)
                             + 0.6 * distance * (1.0 - candidates[k] / self.width)
                             + 0.05 * abs(candidates[k] - candidates[following]))
                    if trial < next_costs.get(k, float('inf')):
                        next_costs[k] = trial
                        choices[step][k] = following
            if not next_costs:
                return False
            costs = next_costs

        starts = {k: [actual, layers[0][k]] for k in costs
                  if self.transition_free(actual, layers[0][k])}
        if not starts:
            for k in sorted(costs, key=lambda k: costs[k] + movement(actual, layers[0][k])):
                assembly = self.assemble(actual, layers[0][k])
                if assembly:
                    starts[k] = assembly
                    break
        if not starts:
            return False
        first = min(starts, key=lambda k: costs[k] + sum(
            movement(a, b) for a, b in zip(starts[k], starts[k][1:])))
        selected = [first]
        for step in range(len(layers) - 1):
            selected.append(choices[step][selected[-1]])
        rows = list(starts[first])
        rows.extend(layers[i][selected[i]] for i in range(1, len(layers)))
        widths = [candidates[k] for k in selected]
        # After the anchor arrives, restore the widest reachable formation.
        # This also permits reopening on a short goal approach, where a path
        # length cost alone would otherwise prefer to remain in a convoy.
        for k in range(selected[-1] + 1, len(candidates)):
            target = layers[-1].get(k)
            if target is None or not self.transition_free(rows[-1], target):
                break
            rows.append(target)
            widths.append(candidates[k])
        self.rows = [tuple(row) for row in rows]
        self.widths = widths
        self.times = [0.0]
        for before, after in zip(self.rows, self.rows[1:]):
            rotation = max(abs(wrap(a[2] - b[2])) for a, b in zip(before, after))
            duration = max(movement(before, after) / self.speed,
                           rotation / self.angular_speed, 1e-3)
            self.times.append(self.times[-1] + duration)
        return True

    def sample(self, seconds):
        if seconds <= 0.0:
            return self.rows[0]
        if seconds >= self.times[-1]:
            return self.rows[-1]
        index = bisect.bisect_right(self.times, seconds)
        fraction = ((seconds - self.times[index - 1])
                    / (self.times[index] - self.times[index - 1]))
        return tuple(interpolate(a, b, fraction)
                     for a, b in zip(self.rows[index - 1], self.rows[index]))

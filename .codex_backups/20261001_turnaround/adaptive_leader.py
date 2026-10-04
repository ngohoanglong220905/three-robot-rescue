"""Leader-first triangle/column references with checked, stationary transitions.

The leader keeps its D* route. In a column the followers use its measured
trail. In a triangle they follow opposite sides of that trail at the same lag.
Space ahead, space behind the leader, and swept transitions are all checked.
"""

import itertools
import math

from .adaptive_structure import swept_separation, wrap
from .leader_follower import ClearanceRecovery, LeaderFollower


class AdaptiveLeaderFormation:
    def __init__(self, route, trail, progress, order, hard_segment_free, clearance,
                 segment_free, route_between, gap=0.70, speed=0.16, narrow_speed=0.08,
                 half_width=0.50, lookahead=1.20, planning_radius=0.18):
        self.column = LeaderFollower(route, trail, progress, order, hard_segment_free,
                                     clearance, gap, speed, narrow_speed)
        self.order = tuple(order)
        self.leader, self.middle, self.tail = order
        self.segment_free, self.clearance = segment_free, clearance
        self.route_between = route_between
        self.gap, self.speed, self.narrow_speed = gap, speed, narrow_speed
        self.widths = tuple(sorted(set((half_width, 0.8 * half_width,
                                       0.6 * half_width)), reverse=True))
        self.lookahead, self.planning_radius = lookahead, planning_radius
        self.state, self.mode = 'column', 'column'
        self.width = 0.0
        self.side = 1.0
        self.transition = None
        self.transition_phase = 0.0
        self.transition_state = 'column'
        self.elapsed, self.next_decision = 0.0, 0.0
        self.open_candidate, self.open_since = None, 0.0
        self.failure_reason = ''
        self.poses = None
        self.last_rows = []
        self.triangle_progress = 0.0

    def _spine_point(self, distance):
        trail = self.column.trail
        if distance <= trail.length:
            return trail.sample(max(0.0, distance))
        return self.column.route.sample(self.column.progress[self.leader] + distance - trail.length)

    def _spine(self, distance):
        point = self._spine_point(distance)
        # A finite tangent reduces yaw noise from centimetre-scale pose samples.
        before, after = self._spine_point(distance - 0.12), self._spine_point(distance + 0.12)
        yaw = (math.atan2(after[1] - before[1], after[0] - before[0])
               if math.dist(before[:2], after[:2]) > 1e-5 else point[2])
        return point[0], point[1], yaw

    def configuration(self, forward, width, side=None):
        side = self.side if side is None else side
        available = self.column.route.length - self.column.progress[self.leader]
        forward = max(0.0, min(available, forward))
        row = [None] * 3
        row[self.leader] = self.column.route.sample(self.column.progress[self.leader] + forward)
        for rank, index in enumerate(self.order[1:], 1):
            lag = self.gap if width else rank * self.gap
            x, y, yaw = self._spine(self.column.trail.length + forward - lag)
            lateral = (side if rank == 1 else -side) * width
            row[index] = (x - lateral * math.sin(yaw), y + lateral * math.cos(yaw), yaw)
        return tuple(row)

    def transition_free(self, before, after):
        if not all(self.segment_free(a, b) for a, b in zip(before, after)):
            return False
        for i, j in ((0, 1), (0, 2), (1, 2)):
            initial = math.dist(before[i][:2], before[j][:2])
            if initial < 0.34 - 1e-9:
                return False
            # Permit separation from a measured close pair, never approach it.
            if swept_separation(before[i], after[i], before[j], after[j]) < min(0.38, initial) - 1e-9:
                return False
        return True

    def wide_safe(self, width, entering=False):
        # Enough recorded path must remain behind the leader to fold again.
        if self.column.trail.length < 2.0 * self.gap + 0.05:
            return False
        margin = self.planning_radius + (0.05 if entering else 0.015)
        count = max(1, math.ceil(self.lookahead / 0.12))
        previous = None
        first_heading = None
        for k in range(count + 1):
            row = self.configuration(k * self.lookahead / count, width)
            if any(self.clearance(p) < margin for p in row):
                return False
            if any(math.dist(row[i][:2], row[j][:2]) < 0.40 for i, j in ((0, 1), (0, 2), (1, 2))):
                return False
            if not self.transition_free(previous or row, row):
                return False
            rear_x = 0.5 * (row[self.middle][0] + row[self.tail][0])
            rear_y = 0.5 * (row[self.middle][1] + row[self.tail][1])
            dx, dy = row[self.leader][0] - rear_x, row[self.leader][1] - rear_y
            yaw = row[self.middle][2]
            if (dx * math.cos(yaw) + dy * math.sin(yaw) < 0.40
                    or abs(-dx * math.sin(yaw) + dy * math.cos(yaw)) > 0.25):
                return False
            # A wide endpoint alone is insufficient: preserve a swept folding
            # manoeuvre at each forecast position, including behind the leader.
            folded = self.configuration(k * self.lookahead / count, 0.0)
            if not self.transition_free(row, folded):
                return False
            # Tight turns use a column even if the two endpoints happen to fit.
            heading = self._spine(self.column.trail.length + k * self.lookahead / count)[2]
            if first_heading is None:
                first_heading = heading
            if abs(wrap(heading - first_heading)) > 0.50:
                return False
            previous = row
        return True

    def assemble(self, actual, target):
        """Hold the leader; try direct swept motion, then sequential D* moves."""
        if max(math.dist(a[:2], b[:2]) for a, b in zip(actual, target)) > 1.60:
            return None
        if self.transition_free(actual, target):
            return [tuple(actual), tuple(target)]
        choices = []
        for order in itertools.permutations(self.order[1:]):
            rows, current, total = [tuple(actual)], list(actual), 0.0
            for index in order:
                if math.dist(current[index][:2], target[index][:2]) < 0.015:
                    continue
                peers = [p for j, p in enumerate(current) if j != index]
                route = self.route_between(current[index], target[index], peers)
                if route is None:
                    break
                valid = True
                for point in route[1:]:
                    after = list(current)
                    after[index] = (*point[:2], target[index][2])
                    if not self.transition_free(current, after):
                        valid = False
                        break
                    total += math.dist(current[index][:2], after[index][:2])
                    rows.append(tuple(after))
                    current = after
                if not valid or total > 4.0:
                    break
            else:
                if self.transition_free(current, target):
                    choices.append((total, rows + [tuple(target)]))
        return min(choices, key=lambda item: item[0])[1] if choices else None

    def begin_transition(self, state, width):
        choices = []
        # Assign left/right to minimize travel without swapping in motion.
        sides = (self.side, -self.side) if state == 'triangle' and self.state == 'column' else (self.side,)
        for side in sides:
            target = list(self.configuration(0.0, width, side))
            target[self.leader] = tuple(self.poses[self.leader])
            rows = self.assemble(self.poses, tuple(target))
            if rows is not None:
                cost = sum(math.dist(a[:2], b[:2]) for p, q in zip(rows, rows[1:])
                           for a, b in zip(p, q))
                choices.append((cost, side, rows))
        if not choices:
            return False
        _, self.side, rows = min(choices, key=lambda item: item[0])
        self.transition = ClearanceRecovery(rows, self.segment_free, speed=self.narrow_speed)
        self.transition_phase = 0.0
        self.transition_state, self.width = state, width
        self.state, self.mode = 'transition', 'forming ' + state
        self.open_candidate = None
        return True

    def observe(self, poses):
        self.poses = tuple(tuple(p[:3]) for p in poses)
        if self.failure_reason:
            return False
        ok = (self.column.observe(poses) if self.state == 'column'
              else self.column.observe_leader(poses))
        if not ok:
            self.failure_reason = self.column.failure_reason
            return False
        if self.state == 'triangle':
            target = self.configuration(0.0, self.width)
            if max(math.dist(p[:2], q[:2]) for p, q in zip(poses, target)) > 0.35:
                self.failure_reason = 'triangle tracking error exceeds 0.35 m'
                return False
        return True

    def decide(self):
        if self.elapsed < self.next_decision or self.state == 'transition':
            return
        self.next_decision = self.elapsed + 0.5
        if self.state == 'triangle' and not self.wide_safe(self.width):
            # A smaller triangle can be retained if all swept motions fit.
            smaller = [w for w in self.widths if w < self.width - 0.01 and self.wide_safe(w, entering=True)]
            for width in smaller:
                if self.begin_transition('triangle', width):
                    return
            if not self.begin_transition('column', 0.0):
                self.failure_reason = 'no safe fold from measured positions; rebuilding formation routes'
            return

        safe = [w for w in self.widths if self.wide_safe(w, entering=True)]
        selected = safe[0] if safe else None
        if selected is None or (self.state == 'triangle' and selected <= self.width + 0.01):
            self.open_candidate = None
            return
        if selected != self.open_candidate:
            self.open_candidate, self.open_since = selected, self.elapsed
            return
        # Expansion needs stable open space; contraction reacts immediately.
        if self.elapsed - self.open_since >= 1.0:
            for width in safe:
                if self.state == 'triangle' and width <= self.width + 0.01:
                    continue
                if self.begin_transition('triangle', width):
                    return
            self.next_decision = self.elapsed + 2.0
            self.open_candidate = None

    def transition_rows(self, dt, horizon, mpc_dt):
        target = self.transition.sample(self.transition_phase)
        error = max(math.dist(p[:2], q[:2]) for p, q in zip(self.poses, target))
        rate = max(0.0, min(1.0, (0.10 - error) / 0.07))
        self.transition_phase = min(self.transition.times[-1], self.transition_phase + dt * rate)
        result = [self.transition.sample(self.transition_phase + rate * k * mpc_dt)
                  for k in range(horizon + 1)]
        if self.transition_phase >= self.transition.times[-1] and error < 0.04:
            self.state = self.transition_state
            self.transition = None
            self.next_decision = self.elapsed + 1.0
            self.triangle_progress = self.column.progress[self.leader]
            if self.state == 'column':
                for rank, index in enumerate(self.order[1:], 1):
                    expected = max(0.0, self.column.trail.length - rank * self.gap)
                    s, _ = self.column.trail.project(self.poses[index], expected)
                    self.column.progress[index] = s
                self.column.reference_progress = self.column.progress.copy()
            self.mode = self.state
        return result

    def triangle_rows(self, dt, horizon, mpc_dt):
        target = self.configuration(0.0, self.width)
        error = max(math.dist(p[:2], q[:2]) for p, q in zip(self.poses, target))
        rate = max(0.0, min(1.0, (0.14 - error) / 0.08))
        speed = self.speed if self.width == self.widths[0] else self.narrow_speed
        progress = self.column.progress[self.leader]
        self.triangle_progress = min(self.column.route.length, progress + 0.08,
                                     max(progress, self.triangle_progress) + dt * speed * rate)
        rows = [self.configuration(min(0.22, self.triangle_progress - progress
                                       + speed * rate * k * mpc_dt), self.width)
                for k in range(horizon + 1)]
        if not all(self.transition_free(a, b) for a, b in zip(rows, rows[1:])):
            self.next_decision = 0.0
            return [self.poses] * (horizon + 1)
        self.mode = f'triangle / width {2.0 * self.width:.2f} m'
        if rate < 0.01:
            self.mode += ' / leader waits'
        return rows

    def rows(self, poses, dt, horizon, mpc_dt):
        self.poses = tuple(tuple(p[:3]) for p in poses)
        dt = max(0.0, min(0.2, dt))
        self.elapsed += dt
        self.decide()
        if self.failure_reason:
            result = [self.poses] * (horizon + 1)
        elif self.state == 'transition':
            result = self.transition_rows(dt, horizon, mpc_dt)
        elif self.state == 'triangle':
            result = self.triangle_rows(dt, horizon, mpc_dt)
        else:
            result = self.column.rows(poses, dt, horizon, mpc_dt)
            self.mode = self.column.mode
        self.last_rows = result
        return result

    def display_paths(self):
        if self.state == 'transition':
            return [[row[i] for row in self.transition.rows] for i in range(3)]
        if self.state == 'column':
            return self.column.display_paths()
        paths = [[row[i] for row in self.last_rows] for i in range(3)]
        paths[self.leader] = self.column.route.points
        return paths

    def remaining_segments(self):
        if self.state == 'column':
            yield from self.column.remaining_segments()
        else:
            line = self.column.route
            s = self.column.progress[self.leader]
            first = max(0, next((k for k, d in enumerate(line.distances) if d >= s), len(line.points) - 1) - 1)
            yield from zip(line.points[first:], line.points[first + 1:])
        rows = self.last_rows
        if self.transition is not None:
            first = max(0, next((k for k, t in enumerate(self.transition.times)
                                 if t >= self.transition_phase), len(self.transition.rows) - 1) - 1)
            rows = self.transition.rows[first:]
        for a, b in zip(rows, rows[1:]):
            yield from zip(a, b)

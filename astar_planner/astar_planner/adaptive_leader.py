"""Leader-first triangle/column references with checked, stationary transitions.

The leader keeps its D* route. In a column the followers use its measured
trail. In a triangle they occupy virtual slots behind its route direction.
Space ahead, space behind the leader, and swept transitions are all checked.
"""

import math

from .formation_motion import assemble_followers, team_motion_free
from .leader_follower import ClearanceRecovery, LeaderFollower


class AdaptiveLeaderFormation:
    def __init__(self, route, trail, progress, order, hard_segment_free, clearance,
                 segment_free, route_between, gap=0.70, speed=0.16, narrow_speed=0.08,
                 half_width=0.50, lookahead=1.20, planning_radius=0.18,
                 transition_speed=0.18, open_hold=0.60):
        self.column = LeaderFollower(route, trail, progress, order, hard_segment_free,
                                     clearance, gap, speed, narrow_speed)
        self.order = tuple(order)
        self.leader, self.middle, self.tail = order
        self.segment_free, self.clearance = segment_free, clearance
        self.route_between = route_between
        self.gap, self.speed, self.narrow_speed = gap, speed, narrow_speed
        # Use two shapes: the normal triangle and a single-file column.
        # Intermediate widths cause repeated stopped assemblies at a doorway.
        self.widths = (half_width,)
        self.transition_speed, self.open_hold = transition_speed, open_hold
        if transition_speed <= 0.0 or open_hold < 0.0:
            raise ValueError('Require positive transition speed and nonnegative open hold')
        self.lookahead, self.planning_radius = lookahead, planning_radius
        self.state, self.mode = 'initializing', 'preparing triangle'
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
        distance = self.column.progress[self.leader] + forward
        leader = self.column.route.sample(distance)
        if width:
            # Smooth the heading over a short route chord. The triangle is
            # tied to the leader's virtual frame, not to a zigzag seed trail.
            before = self.column.route.sample(max(0.0, distance - 0.12))
            after = self.column.route.sample(distance + 0.30)
            yaw = (math.atan2(after[1] - before[1], after[0] - before[0])
                   if math.dist(before[:2], after[:2]) > 0.03 else
                   (self.poses[self.leader][2] if self.poses is not None else leader[2]))
            # The leader faces its actual route tangent. Slot smoothing may
            # turn the virtual rear frame early, but must not turn the leader
            # away from the segment it is still traversing.
            row[self.leader] = leader
            x, y = leader[0] - self.gap * math.cos(yaw), leader[1] - self.gap * math.sin(yaw)
        else:
            row[self.leader] = leader
        for rank, index in enumerate(self.order[1:], 1):
            if not width:
                x, y, yaw = self._spine(self.column.trail.length + forward - rank * self.gap)
            lateral = (side if rank == 1 else -side) * width
            row[index] = (x - lateral * math.sin(yaw), y + lateral * math.cos(yaw), yaw)
        return tuple(row)

    def transition_free(self, before, after):
        return team_motion_free(before, after, self.segment_free)

    def wide_safe(self, width, entering=False):
        margin = self.planning_radius + (0.05 if entering else 0.015)
        # Rear slots lag the leader by gap. Extend the preview by that lag so
        # a doorway within lookahead of the leader is checked at full width
        # before the leader enters it, not only when the followers get there.
        distance = self.lookahead + self.gap
        count = max(1, math.ceil(distance / 0.12))
        previous = None
        for k in range(count + 1):
            row = self.configuration(k * distance / count, width)
            if any(self.clearance(p) < margin for p in row):
                return False
            if any(math.dist(row[i][:2], row[j][:2]) < 0.40 for i, j in ((0, 1), (0, 2), (1, 2))):
                return False
            if not self.transition_free(previous or row, row):
                return False
            # A bend alone does not justify a column. The swept triangle must
            # actually lose obstacle or peer clearance before it is contracted.
            previous = row
        return True

    def assemble(self, actual, target):
        """Hold the leader; try direct swept motion, then sequential D* moves."""
        return assemble_followers(actual, target, self.order, self.segment_free,
                                  self.route_between, max_distance=1.60)

    def begin_transition(self, state, width):
        choices = []
        # Assign left/right to minimize travel without swapping in motion.
        sides = (self.side, -self.side) if state == 'triangle' and self.state != 'triangle' else (self.side,)
        for side in sides:
            target = list(self.configuration(0.0, width, side))
            target[self.leader] = tuple(self.poses[self.leader])
            rows = self.assemble(self.poses, tuple(target))
            if rows is not None:
                # Concurrent follower motion takes the longest robot movement
                # in each interval, rather than the sum of both movements.
                cost = sum(max(math.dist(a[:2], b[:2]) for a, b in zip(p, q))
                           for p, q in zip(rows, rows[1:]))
                choices.append((cost, side, rows))
        if not choices:
            return False
        _, self.side, rows = min(choices, key=lambda item: item[0])
        self.transition = ClearanceRecovery(rows, self.segment_free, speed=self.transition_speed)
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
        width = self.widths[0]
        expanding = self.state == 'transition' and self.transition_state == 'triangle'
        # Contraction is checked on every control tick, including while an
        # expansion is under way. It must bypass the open-space debounce.
        if (self.state == 'triangle' or expanding) and not self.wide_safe(width):
            if not self.begin_transition('column', 0.0):
                self.failure_reason = 'no safe fold from measured positions; rebuilding formation routes'
            return
        if self.state == 'transition':
            return
        if self.state == 'initializing':
            if self.wide_safe(width, entering=True) and self.begin_transition('triangle', width):
                return
            if not self.begin_transition('column', 0.0):
                # The seeded column starts at measured poses, so it can create
                # longitudinal room when a stopped assembly does not fit.
                self.state, self.mode = 'column', 'column / insufficient triangle clearance'
            return
        if self.state == 'triangle':
            self.open_candidate = None
            return
        if self.elapsed + 1e-9 < self.next_decision:
            return
        self.next_decision = self.elapsed + 0.25
        if not self.wide_safe(width, entering=True):
            self.open_candidate = None
            return
        if width != self.open_candidate:
            self.open_candidate, self.open_since = width, self.elapsed
            return
        # Keep the column until the entire team and the preview fit the normal
        # triangle with extra clearance for a stable, checked expansion.
        if self.elapsed - self.open_since + 1e-9 >= self.open_hold:
            if self.begin_transition('triangle', width):
                return
            self.next_decision = self.elapsed + 0.5
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
            self.next_decision = self.elapsed
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
        preview = self.configuration(0.04, self.width)
        motion_gain = max(1.0, max(math.dist(a[:2], b[:2]) for a, b in zip(target, preview)) / 0.04)
        # Rotation of the virtual frame can move the rear slots much faster
        # than the leader. Reserve MPC speed for correcting tracking error.
        speed = min(speed, 0.22 / motion_gain)
        progress = self.column.progress[self.leader]
        self.triangle_progress = min(self.column.route.length, progress + min(0.05, 0.035 / motion_gain),
                                     max(progress, self.triangle_progress) + dt * speed * rate)
        rows, last_forward = [], 0.0
        for k in range(horizon + 1):
            forward = min(0.18, self.triangle_progress - progress + speed * rate * k * mpc_dt)
            before = rows[-1] if rows else target
            allowance = 0.22 * mpc_dt if rows else 0.035
            if max(math.dist(a[:2], b[:2]) for a, b in zip(before, self.configuration(forward, self.width))) > allowance:
                low, high = last_forward, forward
                for _ in range(8):
                    middle = 0.5 * (low + high)
                    movement = max(math.dist(a[:2], b[:2]) for a, b in
                                   zip(before, self.configuration(middle, self.width)))
                    if movement <= allowance:
                        low = middle
                    else:
                        high = middle
                forward = low
            rows.append(self.configuration(forward, self.width))
            last_forward = forward
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
        if self.state in ('column', 'initializing'):
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
            yield from line.remaining_segments(s)
        rows = self.last_rows
        if self.transition is not None:
            first = max(0, next((k for k, t in enumerate(self.transition.times)
                                 if t >= self.transition_phase), len(self.transition.rows) - 1) - 1)
            rows = self.transition.rows[first:]
        for a, b in zip(rows, rows[1:]):
            yield from zip(a, b)

    def remaining_leader_route(self):
        line, s = self.column.route, self.column.progress[self.leader]
        return [line.sample(s)[:2]] + [p for p, d in zip(line.points, line.distances) if d > s + 0.01]

"""Finite, feedback-driven camera sweeps expressed as ordinary MPC references.

This module does not publish velocity commands. The coordinator retains control
of recovery and the MPC retains its obstacle and inter-robot collision checks.
"""
import math


def angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


class ObservationSweep:
    AUTOMATIC_TIMEOUT = 8.0

    def __init__(self, actual, leader, now, speed, dwell, bearing=None, analysis_time=0.0,
                 search_kind=None):
        self.anchors = tuple(tuple(p) for p in actual)
        self.leader = leader
        self.speed = speed
        self.dwell = dwell
        self.analysis_time = analysis_time
        initial = actual[leader][2]
        self.search_kind = search_kind
        self.focus = None
        self.focus_count = 0
        if search_kind is not None:
            # A viewer's measured heading needs one directed glance, not a
            # multi-angle scan. Only a screen with no known viewer checks sides.
            offsets = [0, -60, 60] if search_kind == 'find_viewers' else [0]
            self.targets = [angle(bearing + math.radians(offset)) for offset in offsets]
            self.kind = search_kind
            self.dwell = min(0.8, max(0.4, dwell))
        elif bearing is None:
            # Every step is < pi, including the wrap across +/- pi.
            self.targets = [angle(initial + k * math.pi / 3) for k in range(7)]
            self.kind = 'full'
        else:
            self.targets = [angle(bearing)]
            self.kind = 'look'
            self.dwell = min(0.8, max(0.4, dwell))
        travel = sum(abs(angle(b - a)) for a, b in
                     zip([initial] + self.targets[:-1], self.targets))
        self.deadline = now + min(80.0, travel / speed + len(self.targets) * (dwell + 1.5) + analysis_time + 3.0)
        if self.kind != 'full':
            self.deadline = now + self.AUTOMATIC_TIMEOUT
        self.started = now
        self.settled_since = None
        self.index = 0
        self.finished = False
        self.reason = ''
        self.progress_key = None
        self.progress_time = now
        self.best_error = self.best_drift = math.inf

    def focus_on(self, bearing):
        """Glance at one newly found person without extending the short deadline."""
        if self.search_kind != 'find_viewers' or self.focus is not None or self.focus_count >= 1:
            return False
        self.focus = angle(bearing)
        self.focus_count += 1
        self.settled_since = None
        return True

    def rows(self, actual, now, horizon, step):
        if now < self.started or now >= self.deadline:
            self.finished, self.reason = True, 'aborted: observation timeout or clock reset'
            return None
        drift = max(math.dist(p[:2], anchor[:2]) for p, anchor in zip(actual, self.anchors))
        if drift > 0.18:
            self.finished, self.reason = True, 'aborted: robot moved away from observation anchor'
            return None
        target = self.targets[self.index] if self.focus is None else self.focus
        current = actual[self.leader][2]
        error = angle(target - current)
        progress_key = (self.index, self.focus)
        if self.progress_key != progress_key:
            self.progress_key, self.progress_time = progress_key, now
            self.best_error, self.best_drift = abs(error), drift
        if abs(error) < self.best_error - 0.04 or drift < self.best_drift - 0.01:
            self.progress_time = now
            self.best_error, self.best_drift = min(self.best_error, abs(error)), min(self.best_drift, drift)
        stall_timeout = 6.0 if self.kind == 'full' else 3.0
        if (abs(error) > 0.12 or drift > 0.07) and now - self.progress_time > stall_timeout:
            self.finished, self.reason = True, 'aborted: no safe rotation/settling progress'
            return None
        # Dwell only once the measured heading and all positions have settled.
        if abs(error) <= 0.12 and drift <= 0.07:
            if self.settled_since is None:
                self.settled_since = now
            # Social geometry uses short, stable head/torso evidence. Reserve
            # the long activity-analysis dwell for the explicitly requested sweep.
            required = (max(self.dwell, self.analysis_time)
                        if self.kind == 'full' and self.index == 0 else self.dwell)
            if now - self.settled_since >= required:
                if self.focus is not None:
                    self.focus = None
                else:
                    self.index += 1
                self.settled_since = None
                if self.index == len(self.targets):
                    self.finished = True
                    self.reason = 'completed: search exhausted; evidence unresolved' if self.search_kind else 'completed'
                    return None
                target = self.targets[self.index]
                error = angle(target - current)
        else:
            self.settled_since = None
        rows = []
        for k in range(horizon + 1):
            row = list(self.anchors)
            turn = math.copysign(min(abs(error), self.speed * k * step), error)
            row[self.leader] = (*self.anchors[self.leader][:2], angle(current + turn))
            rows.append(tuple(row))
        return rows

"""Short, sequential escapes from measured peer buffers, checked against the map."""

import math

from .formation_motion import GatedTeamMotion
from .peer_safety import PHYSICAL_SEPARATION, peer_motion_free, minimum_team_distance


class SeparationRecoveryPlanner:
    def __init__(self, segment_free, clearance, order, physical=PHYSICAL_SEPARATION,
                 trigger=0.38, target=0.46, obstacle_margin=0.18):
        self.segment_free, self.clearance, self.order = segment_free, clearance, tuple(order)
        self.physical, self.trigger, self.target = physical, trigger, target
        self.obstacle_margin = obstacle_margin
        self.failure_reason = ''

    def build(self, actual):
        if minimum_team_distance(actual) < self.physical:
            self.failure_reason = 'Measured peer spacing is below the footprint limit'
            return None
        rows = [tuple(actual)]
        current = list(actual)
        # Move a follower before moving the leader; leave the other two holding.
        priority = (*self.order[1:], self.order[0])
        for _ in range(4):
            close = [(i, j) for i, j in ((0, 1), (0, 2), (1, 2))
                     if math.dist(current[i][:2], current[j][:2]) < self.trigger]
            if not close:
                return GatedTeamMotion([('peer separation recovery', rows, ())], speed=0.05)
            choices = []
            involved = {i for pair in close for i in pair}
            for rank, index in enumerate(priority):
                if index not in involved:
                    continue
                start = current[index]
                peers = [p for j, p in enumerate(current) if j != index]
                for distance in (0.08, 0.12, 0.16, 0.22, 0.30):
                    for k in range(32):
                        angle = k * 2.0 * math.pi / 32
                        point = (start[0] + distance * math.cos(angle),
                                 start[1] + distance * math.sin(angle), start[2])
                        if self.clearance(point) < self.obstacle_margin + 0.015:
                            continue
                        if not self.segment_free(start, point):
                            continue
                        if any(not peer_motion_free(start, point, p, p,
                                                    preferred=self.trigger, physical=self.physical)
                               for p in peers):
                            continue
                        separation = min(math.dist(point[:2], p[:2]) for p in peers)
                        if separation < self.target:
                            continue
                        # Prefer follower motion, short travel, and extra wall margin.
                        score = distance + 0.015 * rank + 0.4 * max(0.0, 0.24 - self.clearance(point))
                        choices.append((score, index, point))
            if not choices:
                self.failure_reason = 'No short separating movement has wall and peer clearance'
                return None
            _, index, point = min(choices, key=lambda item: item[0])
            current[index] = point
            rows.append(tuple(current))
        self.failure_reason = 'Could not separate all close pairs with bounded local movements'
        return None

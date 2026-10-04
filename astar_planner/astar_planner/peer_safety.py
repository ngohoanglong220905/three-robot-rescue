"""Keep normal peer margins while permitting motion out of a close measured pair."""

import math

from .adaptive_structure import swept_separation


PHYSICAL_SEPARATION = 0.32  # Two 0.15 m footprints plus 0.02 m reserve.
CONTROL_SEPARATION = 0.34
ROUTING_SEPARATION = 0.38


def peer_motion_free(a0, a1, b0, b1, preferred=CONTROL_SEPARATION,
                     physical=PHYSICAL_SEPARATION):
    """A close pair may hold or separate; it may never get closer on the sweep.

    Normal approaches retain `preferred`. Recovery is possible only above the
    physical footprint limit, without dropping a global collision constraint.
    """
    initial = math.dist(a0[:2], b0[:2])
    if initial < physical - 1e-9:
        return False
    return swept_separation(a0, a1, b0, b1) >= max(physical, min(preferred, initial)) - 1e-9


def minimum_team_distance(poses):
    return min(math.dist(poses[i][:2], poses[j][:2]) for i, j in ((0, 1), (0, 2), (1, 2)))

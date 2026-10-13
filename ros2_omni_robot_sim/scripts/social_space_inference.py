"""Geometry and exclusive pair classification from measured camera directions.

Proximity alone and assumed targets do not create social regions.
"""
import math

import cv2
import numpy as np


def convex(points):
    return cv2.convexHull(np.asarray(points, dtype=np.float32)).reshape(-1, 2).astype(float)


def viewing_hull(a, b, focus):
    """Protect a target band, not a zero-width ray ending at the TV center."""
    forward = focus - (a + b) / 2
    forward = forward / max(float(np.linalg.norm(forward)), 1e-9)
    side = np.array([-forward[1], forward[0]]) * 0.55
    return convex([a, b, focus - side, focus + side])


def components(edges):
    adjacent = {}
    for a, b in edges:
        adjacent.setdefault(a, set()).add(b)
        adjacent.setdefault(b, set()).add(a)
    result = []
    while adjacent:
        pending, found = [next(iter(adjacent))], set()
        while pending:
            key = pending.pop()
            if key in found:
                continue
            found.add(key)
            pending.extend(adjacent.pop(key, ()))
        result.append(found)
    return result


def directed_attention(a, b, angle_degrees=60.0):
    """One measured heading towards another measured person; no invented heading."""
    if a.get('d') is None:
        return False
    delta = b['p'] - a['p']
    distance = float(np.linalg.norm(delta))
    return distance > 1e-6 and a['d'] @ (delta / distance) >= math.cos(math.radians(angle_degrees))


def pair_relation(a, b, nearby_distance=2.5):
    """Shared direction selects screen search; one-way attention protects a gap."""
    delta = b['p'] - a['p']
    distance = float(np.linalg.norm(delta))
    if not 0.50 <= distance <= nearby_distance:
        return 'unrelated'
    # Two viewers can be slightly staggered. Do not mistake the rear viewer's
    # forward ray for a conversation when both have a measured common heading.
    if (a.get('d') is not None and b.get('d') is not None
            and a['d'] @ b['d'] >= math.cos(math.radians(55))):
        return 'parallel'
    if directed_attention(a, b) or directed_attention(b, a):
        return 'conversation'
    return 'unrelated'

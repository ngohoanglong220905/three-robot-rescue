"""Continuous circular-footprint clearance against occupied grid cell boxes.

The D* grid stays conservatively inflated. Collision checks at measured poses
use the actual point/segment geometry, avoiding a second cell-sized rounding
of the inflation margin. The clearance radius is unchanged.
"""

import math


def point_box_distance_squared(x, y, box):
    left, bottom, right, top = box
    dx = max(left - x, 0.0, x - right)
    dy = max(bottom - y, 0.0, y - top)
    return dx * dx + dy * dy


def segment_box_distance_squared(a, b, box):
    left, bottom, right, top = box
    dx, dy = b[0] - a[0], b[1] - a[1]
    enter, leave = 0.0, 1.0
    intersects = True
    for origin, delta, lower, upper in (
        (a[0], dx, left, right), (a[1], dy, bottom, top)
    ):
        if abs(delta) < 1e-12:
            if origin < lower or origin > upper:
                intersects = False
                break
        else:
            first, last = (lower - origin) / delta, (upper - origin) / delta
            enter, leave = max(enter, min(first, last)), min(leave, max(first, last))
            if enter > leave:
                intersects = False
                break
    if intersects:
        return 0.0
    closest = min(point_box_distance_squared(*a, box),
                  point_box_distance_squared(*b, box))
    length_squared = dx * dx + dy * dy
    if length_squared > 1e-16:
        for x, y in ((left, bottom), (left, top), (right, bottom), (right, top)):
            t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length_squared))
            closest = min(closest, (a[0] + t * dx - x) ** 2 + (a[1] + t * dy - y) ** 2)
    return closest


class GridClearance:
    def __init__(self, width, height, resolution, origin, occupied, radius):
        self.width, self.height = width, height
        self.resolution, self.radius = resolution, radius
        self.origin_x, self.origin_y, yaw = origin
        self.cosine, self.sine = math.cos(yaw), math.sin(yaw)
        self.occupied = tuple(occupied)
        self.cache = {}

    def local(self, point):
        dx, dy = point[0] - self.origin_x, point[1] - self.origin_y
        return self.cosine * dx + self.sine * dy, -self.sine * dx + self.cosine * dy

    def inside(self, point):
        return (self.radius <= point[0] <= self.width * self.resolution - self.radius
                and self.radius <= point[1] <= self.height * self.resolution - self.radius)

    def boxes(self, a, b):
        r, size = self.radius, self.resolution
        bounds = (max(0, math.floor((min(a[0], b[0]) - r) / size)),
                  max(0, math.floor((min(a[1], b[1]) - r) / size)),
                  min(self.width - 1, math.floor((max(a[0], b[0]) + r) / size)),
                  min(self.height - 1, math.floor((max(a[1], b[1]) + r) / size)))
        if bounds in self.cache:
            return self.cache[bounds]
        xmin, ymin, xmax, ymax = bounds
        boxes = tuple(
            (x * size, y * size, (x + 1) * size, (y + 1) * size)
            for y in range(ymin, ymax + 1) for x in range(xmin, xmax + 1)
            if self.occupied[y * self.width + x])
        # Cache local MPC queries, not map-wide path simplification rectangles.
        if (xmax - xmin + 1) * (ymax - ymin + 1) <= 256:
            if len(self.cache) >= 2048:
                self.cache.clear()
            self.cache[bounds] = boxes
        return boxes

    def point_free(self, point):
        p = self.local(point)
        return self.inside(p) and all(
            point_box_distance_squared(*p, box) >= self.radius ** 2 - 1e-12
            for box in self.boxes(p, p))

    def segment_free(self, start, end):
        a, b = self.local(start), self.local(end)
        return self.inside(a) and self.inside(b) and all(
            segment_box_distance_squared(a, b, box) >= self.radius ** 2 - 1e-12
            for box in self.boxes(a, b))

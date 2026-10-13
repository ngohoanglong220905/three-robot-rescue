"""Anonymous geometric tracks and bounded constant-velocity forecasts.

Positions come only from RGB-D detections transformed at capture time. These
tracks do not recognize identities or infer a person's intent.
"""
from collections import deque
import math

import numpy as np


class CameraPeople:
    def __init__(self):
        self.tracks = {}
        self.next_id = 0
        self.last_stamp = -math.inf

    def observe(self, points, stamp):
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.prune(stamp)
        used = set()
        for position in points:
            p = np.asarray(position, dtype=float)
            if p.shape != (2,) or not np.all(np.isfinite(p)):
                continue
            choices = [(np.linalg.norm(t['p'] - p), key) for key, t in self.tracks.items()
                       if key not in used and stamp - t['stamp'] < 1.0]
            distance, key = min(choices, default=(math.inf, -1))
            if distance > 0.70:
                key = self.next_id
                self.next_id += 1
                self.tracks[key] = dict(p=p.copy(), stamp=stamp, since=stamp, hits=0,
                                        history=deque(), velocity=np.zeros(2))
            t = self.tracks[key]
            t['stamp'], t['hits'] = stamp, t['hits'] + 1
            t['history'].append((stamp, p.copy()))
            while stamp - t['history'][0][0] > 1.6:
                t['history'].popleft()
            t['p'] = 0.5 * t['p'] + 0.5 * p
            t['velocity'] = self.velocity(t['history'])
            used.add(key)

    @staticmethod
    def velocity(history):
        if len(history) < 4 or history[-1][0] - history[0][0] < 0.8:
            return np.zeros(2)
        times = np.array([item[0] for item in history])
        points = np.array([item[1] for item in history])
        times -= times.mean()
        centered = points - points.mean(axis=0)
        v = (times[:, None] * centered).sum(axis=0) / max(float(times @ times), 1e-9)
        residual = np.sqrt(np.mean(np.sum((centered - times[:, None] * v) ** 2, axis=1)))
        speed = float(np.linalg.norm(v))
        displacement = np.linalg.norm(points[-1] - points[0])
        if 0.20 <= speed <= 1.5 and displacement >= 0.25 and residual <= 0.12:
            return v
        return np.zeros(2)

    def prune(self, now):
        self.tracks = {key: t for key, t in self.tracks.items() if 0 <= now - t['stamp'] < 3.0}

    def regions(self, now, horizon):
        self.prune(now)
        result = {}
        for key, t in self.tracks.items():
            if t['hits'] < 3 or t['stamp'] - t['since'] < 0.4:
                continue
            age = now - t['stamp']
            v = t['velocity']
            speed = np.linalg.norm(v)
            # Bound extrapolation from the last measurement. A lost track is
            # held briefly, never extrapolated indefinitely across the map.
            start = t['p'] + v * min(age, horizon)
            end = t['p'] + v * horizon
            moving = speed >= 0.20
            polygon = np.array([start, end]) if moving else np.array([t['p']])
            margin = 0.40 + (0.10 * horizon if moving else 0.0)
            source = f'constant velocity {speed:.2f} m/s; <= {horizon:.1f} s' if moving else 'camera person; motion unconfirmed'
            result[('person', key)] = dict(polygon=polygon, margin=margin, source=source, stamp=now)
        return result

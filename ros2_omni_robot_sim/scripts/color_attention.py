"""Short memory for sequential views of people and a colourful shared target.

Capture timestamps are kept separately for position and direction. Reading
memory never renews them; stale evidence cannot keep a social region forever.
"""
import itertools
import math

import numpy as np
from social_space_inference import components, convex, directed_attention, pair_relation, viewing_hull


class ColorAttention:
    def __init__(self, person_memory=75.0, object_memory=75.0, nearby_distance=2.5, target_distance=4.5):
        self.person_memory = person_memory
        self.object_memory = object_memory
        self.nearby_distance = nearby_distance
        self.target_distance = target_distance
        self.people, self.objects = {}, {}
        self.next_person = self.next_object = 0
        self.last_object_stamp = -math.inf
        self.invalidated = set()

    def prune(self, now):
        self.people = {k: p for k, p in self.people.items()
                       if -0.1 <= now - p['stamp'] < self.person_memory}
        for person in self.people.values():
            if not -0.1 <= now - person['direction_stamp'] < self.person_memory:
                person['d'] = None
        self.objects = {k: p for k, p in self.objects.items() if -0.1 <= now - p['stamp'] < self.object_memory}

    def observe_people(self, camera_tracks, orientation_sets, now):
        """Remember camera people, including a peer whose heading is still unknown.

        No LiDAR identity quaternion and no inferred/assumed heading enters here.
        Each direction observation can belong to at most one person per update.
        """
        used_people, used_directions = set(), set()
        fresh_people = [p for p in camera_tracks.values() if -0.1 <= now - p['stamp'] < 1.0]
        # Reserve each head/body measurement for its nearest camera person
        # across all sources. A neighbour must not inherit the same person's
        # body direction after that person's head direction was already used.
        owners = {(source, ident): min(fresh_people, key=lambda p: np.linalg.norm(p['p'] - t['p']), default=None)
                  for source, measurements in enumerate(orientation_sets) for ident, t in measurements.items()}
        for observed in camera_tracks.values():
            if not -0.1 <= now - observed['stamp'] < 1.0 or observed['hits'] < 2:
                continue
            choices = [(float(np.linalg.norm(p['p'] - observed['p'])), key) for key, p in self.people.items()
                       if key not in used_people]
            distance, key = min(choices, default=(math.inf, -1))
            if np.linalg.norm(observed['velocity']) >= 0.2:
                if distance < 0.8:
                    self.people.pop(key)
                    self.invalidated.add(key)
                continue
            if observed['stamp'] - observed['since'] < 0.2:
                continue
            direction = None
            for source, measurements in enumerate(orientation_sets):
                options = [(float(np.linalg.norm(t['p'] - observed['p'])), ident, t)
                           for ident, t in measurements.items() if (source, ident) not in used_directions
                           and owners[(source, ident)] is observed
                           and -0.1 <= now - t['stamp'] < 2.0 and t['hits'] >= 2 and t['stamp'] - t['since'] >= 0.2]
                error, ident, estimate = min(options, default=(math.inf, -1, None), key=lambda item: item[0])
                if error < 0.65:
                    direction = estimate
                    used_directions.add((source, ident))
                    break
            if distance >= 0.75:
                key = self.next_person
                self.next_person += 1
                self.people[key] = dict(p=observed['p'].copy(), stamp=observed['stamp'],
                                        d=None if direction is None else direction['d'].copy(),
                                        direction_stamp=-math.inf if direction is None else direction['stamp'])
            else:
                old = self.people[key]
                if distance > 0.45:
                    self.invalidated.add(key)
                old['p'], old['stamp'] = observed['p'].copy(), observed['stamp']
                if direction is not None and direction['stamp'] >= old['direction_stamp']:
                    if old['d'] is not None and old['d'] @ direction['d'] < math.cos(math.radians(35)):
                        self.invalidated.add(key)
                    old['d'], old['direction_stamp'] = direction['d'].copy(), direction['stamp']
            used_people.add(key)

    def observe_objects(self, objects, stamp):
        if stamp <= self.last_object_stamp:
            return
        self.last_object_stamp = stamp
        used = set()
        for obj in objects:
            p = np.asarray(obj['position'], dtype=float)
            boundary = np.asarray(obj['footprint'], dtype=float)
            if (p.shape != (3,) or boundary.ndim != 2 or boundary.shape[1] != 2
                    or not 2 <= len(boundary) <= 16 or not np.all(np.isfinite(p))
                    or not np.all(np.isfinite(boundary)) or obj['hue_groups'] < 3
                    or np.max(np.linalg.norm(boundary - p[:2], axis=1)) > 3.0):
                continue
            choices = [(float(np.linalg.norm(t['p'] - p[:2])), key) for key, t in self.objects.items()
                       if t['source'] == 'colour' and key not in used and abs(t['z'] - p[2]) < 0.65
                       and 0 <= stamp - t['stamp'] < self.object_memory]
            distance, key = min(choices, default=(math.inf, -1))
            if distance > 0.9:
                key = self.next_object
                self.next_object += 1
                self.objects[key] = dict(p=p[:2].copy(), z=p[2], footprint=boundary, source='colour',
                                         stamp=stamp, since=stamp, hits=1)
            else:
                old = self.objects[key]
                combined = convex([*old['footprint'], *boundary])
                # Preserve the previously seen whole screen if a close view
                # clips it. Bound accumulation to a single modest object.
                if np.linalg.norm(np.ptp(combined, axis=0)) <= 4.0:
                    boundary = combined
                old['footprint'] = boundary
                old['p'] = (boundary.min(axis=0) + boundary.max(axis=0)) / 2
                old['z'], old['stamp'], old['hits'] = p[2], stamp, old['hits'] + 1
            used.add(key)

    def observe_tv_objects(self, tracks, now):
        """A depth-localized YOLO TV is also evidence; keep its timestamps separate.

        A TV label does not refresh colour observations or fabricate screen width.
        """
        for ident, track in tracks.items():
            if not -0.1 <= now - track['stamp'] < 1.5:
                continue
            key = ('tv', ident)
            old = self.objects.get(key)
            if old is None or track['stamp'] > old['stamp']:
                self.objects[key] = dict(p=track['p'].copy(), z=0.0, footprint=None,
                                         source='tv', stamp=track['stamp'], since=track['since'], hits=track['hits'])

    def relation(self, a, b):
        return pair_relation(a, b, self.nearby_distance)

    def viewers_face_target(self, members, target):
        if not members or any(p['d'] is None for p in members):
            return False
        if any(self.relation(a, b) != 'parallel' for a, b in itertools.combinations(members, 2)):
            return False
        for person in members:
            vector = target['p'] - person['p']
            length = np.linalg.norm(vector)
            if not 0.4 < length <= self.target_distance or person['d'] @ (vector / length) < math.cos(math.radians(60)):
                return False
        return True

    def facing_target(self, a, b, target):
        return self.viewers_face_target((a, b), target)

    def conversation_groups(self, visible):
        edges = [(ia, ib) for (ia, a), (ib, b) in itertools.combinations(self.people.items(), 2)
                 if self.relation(a, b) == 'conversation' and visible(a['p'], b['p'])]
        return components(edges)

    def parallel_pairs(self, visible):
        # A member of an observed conversation is not simultaneously a viewer
        # in another pair. Resolve conversation components before querying TV.
        chatting = set().union(*self.conversation_groups(visible))
        return [(ia, ib) for (ia, a), (ib, b) in itertools.combinations(self.people.items(), 2)
                if ia not in chatting and ib not in chatting and self.relation(a, b) == 'parallel']

    def viewer_candidates(self, visible):
        """Prefer shared viewing, but one measured heading can locate a TV.

        Conversation members keep their interaction space instead of starting
        an unrelated screen search. Unknown headings never become viewer cues.
        """
        chatting = set().union(*self.conversation_groups(visible))
        people = [(i, p) for i, p in self.people.items() if i not in chatting and p['d'] is not None]
        pairs = [(ia, ib) for (ia, a), (ib, b) in itertools.combinations(people, 2)
                 if self.relation(a, b) == 'parallel']
        return pairs + [(i,) for i, _ in people]

    def contradicted(self, key, visible):
        """Fresh motion/heading evidence removes an old cached region early."""
        if key[0] == 'facing_group':
            members = set(key[1:])
            if members & self.invalidated:
                return True
            if members <= self.people.keys():
                return not any(members <= group for group in self.conversation_groups(visible))
            return False
        members, target_id = set(key[1:-1]), key[-1]
        if members & self.invalidated:
            return True
        if any(members & group for group in self.conversation_groups(visible)):
            return True
        known = [self.people[i] for i in members if i in self.people and self.people[i]['d'] is not None]
        if any(self.relation(a, b) != 'parallel' for a, b in itertools.combinations(known, 2)):
            return True
        if target_id in self.objects:
            return any(not self.viewers_face_target((p,), self.objects[target_id]) for p in known)
        return False  # Missing data expires by time, not a claim of absence.

    def regions(self, visible, margin, group_margin=0.35, color_enabled=True):
        regions = {}
        for ids in self.conversation_groups(visible):
            members = [self.people[i] for i in sorted(ids)]
            points = [p['p'] for p in members]
            stamps = [p['stamp'] for p in members]
            # Preserve the interaction center when only two of a group's
            # faces are currently measurable; the center may be off the chord.
            for a, b in itertools.combinations(members, 2):
                if self.relation(a, b) != 'conversation' or not visible(a['p'], b['p']):
                    continue
                stamps.extend(person['direction_stamp'] for person, other in ((a, b), (b, a))
                              if directed_attention(person, other))
                center = (a['p'] + b['p']) / 2
                rays = (np.column_stack((a['d'], -b['d']))
                        if a['d'] is not None and b['d'] is not None else None)
                if rays is not None and abs(np.linalg.det(rays)) > math.sin(math.radians(12)):
                    lengths = np.linalg.solve(rays, b['p'] - a['p'])
                    if np.all((lengths > 0) & (lengths < 3.0)):
                        estimate = a['p'] + lengths[0] * a['d']
                        if np.linalg.norm(estimate - center) < 0.85:
                            center = estimate
                points.extend(center + 0.4 * np.array([math.cos(t), math.sin(t)])
                              for t in np.linspace(0, 2 * math.pi, 8, endpoint=False))
            stamp = min(stamps)
            regions[('facing_group', *sorted(ids))] = dict(
                polygon=convex(points), margin=group_margin, stamp=stamp,
                source='nearby people: at least one measured direction towards a peer; TV not required')
        if not color_enabled:
            return regions
        covered = set()
        for ids in self.viewer_candidates(visible):
            if len(ids) == 1 and ids[0] in covered:
                continue
            members = [self.people[i] for i in ids]
            center = np.mean([p['p'] for p in members], axis=0)
            choices = []
            for target_id, target in self.objects.items():
                if (target['hits'] < 2 or target['stamp'] - target['since'] < 0.2
                        or not self.viewers_face_target(members, target)
                        or not all(visible(p['p'], target['p']) for p in members)):
                    continue
                choices.append((np.linalg.norm(target['p'] - center), target_id, target))
            if not choices:
                continue
            _, target_id, target = min(choices, key=lambda item: item[0])
            is_tv = target['source'] == 'tv'
            key = ('viewing_tv' if is_tv else 'color_attention', *ids, target_id)
            # Include the measured object's width, not only its centre ray.
            polygon = (viewing_hull(members[0]['p'], members[-1]['p'], target['p']) if is_tv else
                       convex([*(p['p'] for p in members), *target['footprint']]))
            evidence_stamp = min([target['stamp']] + [p[k] for p in members for k in ('stamp', 'direction_stamp')])
            regions[key] = dict(polygon=polygon, margin=margin, stamp=evidence_stamp,
                                source=f'{len(members)} measured viewer(s) + ' + (
                                    'nearby camera TV; approximate screen band' if is_tv else
                                    'nearby RGB-D multicolour target; heuristic'))
            covered.update(ids)
        return regions

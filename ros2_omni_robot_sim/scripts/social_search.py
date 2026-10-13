"""Bounded active perception: find viewers of a screen, or a screen for viewers.

Only measured camera tracks enter this state machine. It requests rotations
from the formation coordinator and never publishes velocity commands.
"""
import itertools
import math
import uuid

import numpy as np

VIEWING_KINDS = ('color_attention', 'viewing_tv')


class SocialSearch:
    def __init__(self, emit, cooldown=30.0, viewing_length=3.2):
        self.emit = emit
        self.cooldown = max(5.0, cooldown)
        self.viewing_length = viewing_length
        self.session = uuid.uuid4().hex[:12]
        self.serial = 0
        self.current = None
        self.history = []
        self.retry_after = -math.inf
        self.priority_retry_after = -math.inf
        self.status = 'idle'

    def send(self, command, now, **fields):
        self.emit(dict(version=1, frame_id='map', stamp=now,
                       id=self.current['id'], command=command, **fields))
        self.current['sent'] = now

    def reset(self, now):
        if self.current is not None:
            previous = self.current.get('previous')
            if previous is not None:
                self.emit(dict(version=1, frame_id='map', stamp=now,
                               id=previous['id'], command='cancel'))
            self.send('cancel', now)
        self.current = None
        self.history.clear()
        self.retry_after = -math.inf
        self.priority_retry_after = -math.inf
        self.status = 'idle'

    def feedback(self, data, now):
        current = self.current
        if current is None or data.get('version') != 1:
            return
        state = data.get('state')
        if data.get('id') != current['id']:
            previous = current.get('previous')
            if (previous is not None and data.get('id') == previous['id']
                    and state in ('rejected', 'completed', 'aborted', 'cancelled')):
                current['previous'] = None
            return
        if state == 'scanning':
            current['active'] = True
            current['previous'] = None
            self.status = current['kind']
        elif state == 'focusing':
            if current['focus_pending'] is not None:
                current['focused'].append(current['focus_pending'])
            current['focus_pending'] = None
            self.status = current['kind'] + ': holding person view'
        elif state == 'focus_rejected':
            current['focus_pending'] = None
        elif state in ('rejected', 'completed', 'aborted', 'cancelled'):
            if state != 'rejected':
                # This is attempt history, not a claim that the view was clear.
                # Failed rotations must not freeze the convoy by repeating forever.
                self.remember_attempt(now, max(60.0, self.cooldown) if state == 'completed' else self.cooldown)
            self.retry_after = now + (5.0 if state == 'rejected' else self.cooldown)
            if state in ('rejected', 'aborted', 'cancelled'):
                self.priority_retry_after = now + (1.0 if state == 'rejected' else 5.0)
            self.status = str(data.get('reason', state))
            self.current = current.get('previous') if state == 'rejected' else None

    def remember_attempt(self, now, duration):
        current = self.current
        self.history.append(dict(kind=current['kind'], center=current['center'].copy(),
                                 origin=current['origin'].copy(), direction=current['direction'], until=now + duration))

    def attempted(self, kind, center, leader, now, direction=None):
        return any(entry['kind'] == kind and now < entry['until']
                   and np.linalg.norm(center - entry['center']) < 0.9
                   and np.linalg.norm(leader - entry['origin']) < 0.8
                   and (direction is None or entry['direction'] is None
                        or direction @ entry['direction'] >= math.cos(math.radians(35)))
                   for entry in self.history)

    def update(self, memory, tv_tracks, camera_tracks, regions, visible, leader, now,
               available=True, enabled=True):
        self.history = [entry for entry in self.history if now < entry['until']]
        if self.current is not None:
            # Process positive evidence before choosing a different search.
            # Otherwise a resolved TV/viewer sweep can be replaced by another
            # request and keep the convoy stopped even though a keepout exists.
            self.progress(memory, camera_tracks, regions, visible, now)
            if (self.current is None or self.current['finish_requested']
                    or self.current['kind'] == 'find_screen' or not enabled):
                return
        if not enabled:
            return
        choices = []
        viewing_sets = [set(key[1:-1]) for key in regions if key[0] in VIEWING_KINDS]
        for ids in memory.viewer_candidates(visible):
            members = [memory.people[i] for i in ids]
            center = np.mean([p['p'] for p in members], axis=0)
            if (now < self.priority_retry_after or any(set(ids) & group for group in viewing_sets)
                    or not any(now - p['stamp'] < 1.5 and now - p['direction_stamp'] < 3.0 for p in members)
                    or not 0.5 < np.linalg.norm(center - leader) < 6.0):
                continue
            direction = np.sum([p['d'] for p in members], axis=0)
            direction /= np.linalg.norm(direction)
            if self.attempted('find_screen', center, leader, now, direction):
                continue
            point = center + self.viewing_length * direction
            # A first RGB-D target sample may guide the camera before there
            # are enough samples to create the social keepout.
            candidates = [t for t in memory.objects.values() if memory.viewers_face_target(members, t)
                          and all(visible(p['p'], t['p']) for p in members)]
            if candidates:
                point = min(candidates, key=lambda t: np.linalg.norm(t['p'] - center))['p']
            choices.append((2 - len(ids), np.linalg.norm(center - leader), 'find_screen', point, center, ids, direction))
        if choices:
            # A newly observed viewer heading can redirect a generic person
            # dwell / viewer sweep now. Safety validation stays in coordinator.
            # Keep the old request until the replacement is accepted, so a
            # rejected replacement cannot orphan the running search.
            self.begin(min(choices, key=lambda item: item[:2]), leader, now, previous=self.current)
            return
        if self.current is not None:
            return
        if not available or now < self.retry_after:
            return
        matched_targets = [memory.objects[key[-1]]['p'] for key in regions
                           if key[0] in VIEWING_KINDS and key[-1] in memory.objects]
        for target in itertools.chain(memory.objects.values(), tv_tracks.values()):
            center = target['p']
            if (now - target['stamp'] > 1.5 or target['hits'] < 2
                    or target['stamp'] - target['since'] < 0.2
                    or not 0.5 < np.linalg.norm(center - leader) < 6.0
                    or any(np.linalg.norm(center - p) < 0.9 for p in matched_targets)
                    or self.attempted('find_viewers', center, leader, now)):
                continue
            choices.append((2, np.linalg.norm(center - leader), 'find_viewers', center, center, (), None))
        if not choices:
            return
        self.begin(min(choices, key=lambda item: item[:2]), leader, now)

    def begin(self, choice, leader, now, previous=None):
        _, _, kind, point, center, members, direction = choice
        self.serial += 1
        self.current = dict(id=f'{self.session}-{self.serial}', kind=kind, point=point.copy(),
                            center=center.copy(), members=members, direction=direction, origin=leader.copy(),
                            started=now, sent=now, active=False, focused=[], focus_pending=None, previous=previous,
                            finish_requested=False, completion_sent=None)
        self.status = 'requesting ' + kind
        self.send('start', now, kind=kind, point=point.tolist())

    def inspect_person(self, point, leader, now):
        """Use the same completion feedback for the initial person glance."""
        if self.current is not None or self.attempted('inspect_person', point, leader, now):
            return False
        self.begin((3, np.linalg.norm(point - leader), 'inspect_person', point, point, (), None), leader, now)
        return True

    def progress(self, memory, camera_tracks, regions, visible, now):
        current = self.current
        elapsed = now - current['started']
        if elapsed < 0 or elapsed > 15.0:
            self.send('cancel', now)
            self.remember_attempt(now, self.cooldown)
            self.status = 'unresolved: search feedback timeout'
            self.current = None
            self.retry_after = now + self.cooldown
            return
        resolved = False
        for key in regions:
            if current['kind'] in ('find_screen', 'inspect_person'):
                ids = key[1:-1] if key[0] in VIEWING_KINDS else key[1:]
                # A short track loss can change the camera identity. Match its
                # measured position too; no full-body track is required.
                resolved |= bool(set(current['members']) & set(ids)) or any(
                    i in memory.people and np.linalg.norm(memory.people[i]['p'] - current['center']) < 0.75
                    for i in ids)
            elif key[0] in VIEWING_KINDS and key[-1] in memory.objects:
                resolved |= np.linalg.norm(memory.objects[key[-1]]['p'] - current['center']) < 0.9
        current['finish_requested'] |= resolved
        if not current['active']:
            if now - current['sent'] >= 2.0:
                self.send('start', now, kind=current['kind'], point=current['point'].tolist())
            return
        if current['finish_requested']:
            if current['completion_sent'] is None or now - current['completion_sent'] >= 0.5:
                self.send('complete', now)
                current['completion_sent'] = now
                self.status = 'social evidence acquired'
            return
        if current['kind'] != 'find_viewers' or len(current['focused']) >= 1:
            return
        if current['focus_pending'] is not None:
            if now - current['sent'] < 2.0:
                return
            current['focus_pending'] = None
        if now - current['sent'] < 2.0:
            return
        for track in camera_tracks.values():
            p = track['p']
            if (now - track['stamp'] > 0.8 or track['hits'] < 2
                    or np.linalg.norm(p - current['center']) > 6.0
                    or np.linalg.norm(track['velocity']) >= 0.2
                    or any(np.linalg.norm(p - prior) < 0.9 for prior in current['focused'])
                    or any(prior['d'] is not None and np.linalg.norm(p - prior['p']) < 0.65
                           for prior in memory.people.values())):
                continue
            current['focus_pending'] = p.copy()
            self.send('focus', now, point=p.tolist())
            break

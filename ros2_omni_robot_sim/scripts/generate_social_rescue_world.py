#!/usr/bin/env python3
"""Build the social scene, occupancy map and scenario annotations together.

Run from the source tree when changing the layout. The generated assets are
installed with the package; Gazebo does not run this generator at startup.
"""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_my_maze_map import collision_shapes, rasterize


WORLD = 'social_rescue'
RESOLUTION = 0.05
ORIGIN = (-6.5, -5.5)
PIXELS = (260, 220)


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def values(numbers):
    return ' '.join(f'{v:.6f}' for v in numbers)


def pose(parent, xyz=(0, 0, 0), yaw=0):
    return element(parent, 'pose', values((*xyz, 0, 0, yaw)))


def material(visual, color, emissive=False):
    mat = element(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        element(mat, tag, values(color))
    if emissive:
        element(mat, 'emissive', values(color))


class Scene:
    def __init__(self):
        self.root = ET.Element('sdf', version='1.8')
        self.world = element(self.root, 'world', name=WORLD)
        self.people, self.obstacles, self.doors = [], [], []

    def model(self, name, xyz, yaw=0):
        model = element(self.world, 'model', name=name)
        element(model, 'static', 'true')
        pose(model, xyz, yaw)
        return element(model, 'link', name='link')

    def box(self, name, x, y, sx, sy, height, color, yaw=0, base=0,
            collision=True, annotate=False, emissive=False):
        link = self.model(name, (x, y, base + height / 2), yaw)
        for tag in (('collision', 'visual') if collision else ('visual',)):
            part = element(link, tag, name=tag)
            geometry = element(part, 'geometry')
            element(element(geometry, 'box'), 'size', values((sx, sy, height)))
            if tag == 'visual':
                material(part, color, emissive)
        if annotate:
            self.obstacles.append(dict(name=name, xy=[x, y], height=height,
                                       size=[sx, sy], yaw=yaw))

    def cylinder(self, name, x, y, radius, height, color):
        link = self.model(name, (x, y, height / 2))
        for tag in ('collision', 'visual'):
            part = element(link, tag, name=tag)
            shape = element(element(part, 'geometry'), 'cylinder')
            element(shape, 'radius', radius)
            element(shape, 'length', height)
            if tag == 'visual':
                material(part, color)
        self.obstacles.append(dict(name=name, xy=[x, y], radius=radius, height=height))

    def person(self, name, x, y, facing, activity, group, posture='standing'):
        link = self.model(name, (x, y, 0), facing)
        # The original mesh faces -Y. Rotate it so model +X is the gaze
        # direction used in the manifest and the RViz orientation arrows.
        visual = element(link, 'visual', name='textured_person')
        pose(visual, (0, 0, 0.01), math.pi / 2)
        mesh = element(element(visual, 'geometry'), 'mesh')
        mesh_name = 'seated' if posture == 'seated' else 'standing'
        element(mesh, 'uri', f'model://person_standing/meshes/{mesh_name}.dae')
        element(mesh, 'scale', '0.92 0.92 0.92')
        collision = element(link, 'collision', name='body_envelope')
        if posture == 'seated':
            pose(collision, (0.18, 0, 0.70))
            shape = element(element(collision, 'geometry'), 'box')
            element(shape, 'size', '0.88 0.62 1.40')
        else:
            pose(collision, (0, 0, 0.89))
            cylinder = element(element(collision, 'geometry'), 'cylinder')
            element(cylinder, 'radius', '0.32')
            element(cylinder, 'length', '1.78')
        self.people.append(dict(name=name, x=x, y=y, yaw=facing,
                                activity=activity, group=group, radius=0.32,
                                posture=posture, height=1.40 if posture == 'seated' else 1.78))

    def wall_with_doors(self, name, axis, fixed, start, end, doors=()):
        """Split a wall at clear opening intervals; keep map and world identical."""
        edge = start
        for index, (center, width) in enumerate(sorted(doors) + [(end, 0.0)]):
            left, right = center - width / 2, center + width / 2
            if left < edge or right > end:
                raise ValueError(f'Overlapping or out-of-bounds door in {name}')
            if left > edge:
                middle = (edge + left) / 2
                x, y = (middle, fixed) if axis == 'x' else (fixed, middle)
                sx, sy = (left - edge, 0.2) if axis == 'x' else (0.2, left - edge)
                self.box(f'{name}_{index}', x, y, sx, sy, 1.6, (0.57, 0.66, 0.70, 1))
            if width:
                xy = [center, fixed] if axis == 'x' else [fixed, center]
                self.doors.append(dict(name=f'{name}_door_{index}', center=xy, clear_width=width))
            edge = right

    def core(self):
        physics = element(self.world, 'physics', name='1ms', type='ignored')
        element(physics, 'max_step_size', '0.001')
        element(physics, 'real_time_factor', '1.0')
        for system in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Contact', 'Imu'):
            stem = {'UserCommands': 'user-commands', 'SceneBroadcaster': 'scene-broadcaster'}
            element(self.world, 'plugin', filename=f'ignition-gazebo-{stem.get(system, system.lower())}-system',
                    name=f'gz::sim::systems::{system}')
        sensors = element(self.world, 'plugin', filename='libgz-sim-sensors-system.so',
                          name='gz::sim::systems::Sensors')
        element(sensors, 'render_engine', 'ogre2')
        scene = element(self.world, 'scene')
        element(scene, 'ambient', '0.65 0.65 0.65 1')
        element(scene, 'background', '0.16 0.19 0.23 1')
        element(scene, 'shadows', 'true')
        sun = element(self.world, 'light', name='sun', type='directional')
        pose(sun, (0, 0, 12))
        element(sun, 'diffuse', '0.85 0.85 0.80 1')
        element(sun, 'specular', '0.12 0.12 0.12 1')
        element(sun, 'direction', '-0.25 0.15 -0.95')
        element(sun, 'cast_shadows', 'true')
        floor = self.model('ground_plane', (0, 0, 0))
        for tag in ('collision', 'visual'):
            part = element(floor, tag, name=tag)
            plane = element(element(part, 'geometry'), 'plane')
            element(plane, 'normal', '0 0 1')
            element(plane, 'size', '16 14')
            if tag == 'visual':
                material(part, (0.77, 0.79, 0.80, 1))

    def layout(self):
        # Compact 12 x 10 m floor plan. Robot / human sizes and door widths
        # remain in real metres; shrinking all geometry would close passages.
        wall = (0.66, 0.72, 0.77, 1)
        for x in (-6, 6):
            self.box(f'outer_x_{x}', x, 0, 0.20, 10.20, 1.8, wall)
        for y in (-5, 5):
            self.box(f'outer_y_{y}', 0, y, 12, 0.20, 1.8, wall)

        # Two northern social rooms and a 1.80 m central branch. Side doors
        # form a loop, so the team can enter a room from different directions.
        self.wall_with_doors('conversation_front', 'x', 1.0, -5.9, -1.0, [(-3.4, 1.2)])
        self.wall_with_doors('conversation_east', 'y', -1.0, 1.0, 4.9, [(2.1, 0.9)])
        self.wall_with_doors('tv_front', 'x', 1.0, 1.0, 5.9, [(2.7, 1.2)])
        self.wall_with_doors('tv_west', 'y', 1.0, 1.0, 4.9, [(3.8, 1.0)])

        # Southern technical / equipment rooms. A separate 0.80 m branch
        # connects the cross corridor to a 0.90 m return passage at the back.
        self.wall_with_doors('south_rooms_front', 'x', -1.0, -5.9, 1.4,
                             [(-4.8, 0.9), (-3.1, 0.8), (-0.2, 1.0)])
        self.wall_with_doors('narrow_branch_west', 'y', -3.6, -4.0, -1.0)
        self.wall_with_doors('narrow_branch_east', 'y', -2.6, -4.0, -1.0)
        self.wall_with_doors('equipment_east', 'y', 1.4, -4.9, -1.0, [(-3.3, 1.2)])
        self.wall_with_doors('equipment_alcove', 'y', 0.2, -4.9, -3.8)

        # Visual floor strips do not introduce low-obstacle collisions.
        self.box('cross_walkway', 0, 0, 11.8, 1.7, 0.002,
                 (0.63, 0.69, 0.71, 1), collision=False)
        self.box('north_walkway', 0, 3.0, 1.7, 3.8, 0.002,
                 (0.63, 0.69, 0.71, 1), collision=False)
        self.box('conversation_rug', -3.5, 3.0, 3.8, 3.0, 0.003,
                 (0.28, 0.49, 0.53, 1), collision=False)
        self.box('tv_rug', 3.5, 3.0, 4.0, 3.5, 0.003,
                 (0.43, 0.39, 0.55, 1), collision=False)
        self.box('assembly_pad', 3.5, -2.7, 3.0, 2.2, 0.003,
                 (0.55, 0.64, 0.66, 1), collision=False)

        sofa = (0.21, 0.29, 0.37, 1)
        self.box('tv_sofa_seat', 2.35, 3.1, 0.75, 2.40, 0.42, sofa)
        self.box('tv_sofa_back', 1.92, 3.1, 0.14, 2.40, 0.95, sofa)
        for y in (1.84, 4.36):
            self.box(f'sofa_arm_{y}', 2.35, y, 0.80, 0.12, 0.62, sofa)
        self.box('tv_console', 5.5, 3.0, 0.45, 1.60, 0.65, (0.36, 0.24, 0.16, 1))
        self.box('tv_frame', 5.45, 3.0, 0.12, 1.6, 1.05, (0.04, 0.05, 0.06, 1), base=0.76)
        self.box('tv_screen', 5.382, 3.0, 0.012, 1.45, 0.88, (0.05, 0.26, 0.44, 1),
                 base=0.84, collision=False, emissive=True)
        for k, height in enumerate((0.23, 0.40, 0.30, 0.52, 0.36)):
            self.box(f'tv_chart_{k}', 5.372, 2.5 + 0.23 * k, 0.008, 0.12, height,
                     (0.22, 0.78, 0.80, 1), base=1.01, collision=False, emissive=True)
        self.box('talk_bench', -5.5, 3.3, 0.50, 1.8, 0.46, (0.48, 0.30, 0.17, 1))
        self.box('talk_bench_back', -5.8, 3.3, 0.10, 1.8, 0.92, (0.48, 0.30, 0.17, 1))

        self.box('low_step_05cm', -0.7, -4.2, 0.70, 0.40, 0.05,
                 (0.95, 0.64, 0.10, 1), annotate=True)
        self.box('low_case_12cm', 5.2, -1.2, 0.45, 0.40, 0.12,
                 (0.90, 0.39, 0.12, 1), annotate=True)
        self.box('low_crate_20cm', 0.45, 2.6, 0.40, 0.35, 0.20,
                 (0.82, 0.45, 0.18, 1), annotate=True)
        self.box('low_pallet_10cm', -4.6, -2.0, 0.55, 0.40, 0.10,
                 (0.60, 0.41, 0.23, 1), annotate=True)
        self.box('equipment_case', 0.8, -4.4, 0.60, 0.60, 0.65,
                 (0.30, 0.43, 0.47, 1), annotate=True)
        self.box('tall_cabinet', -5.35, -3.1, 0.75, 0.65, 1.60,
                 (0.26, 0.39, 0.45, 1), annotate=True)
        self.box('tall_storage', 5.2, -4.25, 0.90, 0.65, 1.55,
                 (0.45, 0.53, 0.33, 1), annotate=True)
        self.cylinder('junction_column', 1.95, -0.3, 0.22, 1.5, (0.40, 0.49, 0.54, 1))

        # Activities are authored static arrangements, not camera classifications.
        self.person('talk_person_a', -4.5, 2.8, math.atan2(0.3, 1.0), 'conversation', 'conversation_group')
        self.person('talk_person_b', -2.5, 2.8, math.atan2(0.3, -1.0), 'conversation', 'conversation_group')
        self.person('talk_person_c', -3.5, 3.85, -math.pi / 2, 'conversation', 'conversation_group')
        self.person('tv_person_a', 2.25, 2.4, math.atan2(0.6, 3.2),
                    'watching_tv', 'tv_group', posture='seated')
        self.person('tv_person_b', 2.25, 3.65, math.atan2(-0.65, 3.2),
                    'watching_tv', 'tv_group', posture='seated')
        self.person('standing_person_west', -1.3, -2.2, math.pi / 4, 'standing', 'standing_west')
        self.person('standing_person_north', 0.0, 4.05, -math.pi / 2, 'standing', 'standing_north')

        # People in the walking route, offset to leave a physical bypass. The
        # first is ~2.4 m ahead of the default leader with an unobstructed view.
        self.person('corridor_person_start', 3.55, 0.20, -math.pi / 2,
                    'standing_in_corridor', 'corridor_start')
        self.person('corridor_person_middle', 0.45, 0.35, -math.pi / 2,
                    'standing_in_corridor', 'corridor_middle')
        self.person('corridor_person_west', -4.65, 0.35, 0.0,
                    'standing_in_corridor', 'corridor_west')

    def manifest(self):
        return dict(
            world=WORLD, frame_id='map', bounds=[-6, -5, 6, 5],
            annotation_source='authored_scenario_ground_truth_not_camera_detection',
            people=self.people, obstacles=self.obstacles,
            zones=[
                dict(name='conversation_group', activity='conversation', center=[-3.5, 3.15],
                     size=[3.2, 2.5], color=[0.10, 0.75, 0.80]),
                dict(name='tv_group', activity='watching_tv', center=[3.7, 3.0],
                     size=[3.7, 2.6], color=[0.65, 0.40, 0.90]),
                dict(name='standing_west', activity='standing', center=[-1.3, -2.2],
                     size=[1.5, 1.5], color=[0.95, 0.65, 0.15]),
                dict(name='standing_north', activity='standing', center=[0.0, 4.05],
                     size=[1.5, 1.5], color=[0.95, 0.65, 0.15]),
                dict(name='corridor_start', activity='standing_in_corridor', center=[3.55, 0.20],
                     size=[1.4, 1.4], color=[0.95, 0.45, 0.15]),
                dict(name='corridor_middle', activity='standing_in_corridor', center=[0.45, 0.35],
                     size=[1.4, 1.4], color=[0.95, 0.45, 0.15]),
                dict(name='corridor_west', activity='standing_in_corridor', center=[-4.65, 0.35],
                     size=[1.4, 1.4], color=[0.95, 0.45, 0.15]),
            ],
            passages=[dict(name='cross_corridor', center=[-2.5, 0], clear_width=1.8, length=6.8),
                      dict(name='north_branch', center=[0, 3], clear_width=1.8, length=3.8),
                      dict(name='narrow_branch', center=[-3.1, -2.5], clear_width=0.8, length=3.0),
                      dict(name='south_return', center=[-3.1, -4.45], clear_width=0.9, length=2.0),
                      dict(name='spawn_approach', center=[3.5, -1], clear_width=4.4)] + self.doors,
            rooms=[
                dict(name='conversation_room', bounds=[-5.9, 1.1, -1.1, 4.9]),
                dict(name='tv_room', bounds=[1.1, 1.1, 5.9, 4.9]),
                dict(name='technical_room', bounds=[-5.9, -4.0, -3.7, -1.1]),
                dict(name='equipment_room', bounds=[-2.5, -4.9, 1.3, -1.1]),
            ],
            spawn_poses=dict(robot1=[4.0, -2.9, 1.57], robot2=[3.0, -2.9, 1.57],
                             robot3=[3.5, -2.2, 1.57]),
        )

    def write(self, package_dir):
        self.core()
        self.layout()
        world_path = package_dir / 'worlds' / f'{WORLD}.sdf'
        map_path = package_dir / 'map' / f'{WORLD}.pgm'
        manifest_path = package_dir / 'config' / 'social_rescue_scene.json'
        for path in (world_path, map_path, manifest_path):
            path.parent.mkdir(parents=True, exist_ok=True)
        ET.indent(self.root, space='  ')
        ET.ElementTree(self.root).write(world_path, encoding='utf-8', xml_declaration=True)
        shapes = collision_shapes(self.root)
        pixels, occupied = rasterize(shapes, *PIXELS, *ORIGIN, RESOLUTION)
        map_path.write_bytes(f'P5\n{PIXELS[0]} {PIXELS[1]}\n255\n'.encode() + pixels)
        map_path.with_suffix('.yaml').write_text(
            f'image: {WORLD}.pgm\nmode: trinary\nresolution: {RESOLUTION}\n'
            f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
        manifest_path.write_text(json.dumps(self.manifest(), indent=2) + '\n', encoding='utf-8')
        self.write_layout(map_path.with_name(f'{WORLD}_layout.svg'), shapes)
        print(f'Generated {world_path}, {map_path}, map YAML and {manifest_path}; '
              f'{len(self.people)} people, {len(shapes)} collision shapes, {occupied} occupied cells.')

    def write_layout(self, path, shapes):
        """Export a coordinate-aligned overview for the scene README."""
        svg = ET.Element('svg', xmlns='http://www.w3.org/2000/svg', viewBox='0 0 1000 900')
        element(svg, 'rect', x='0', y='0', width='1000', height='900', fill='#f4f7fa')

        scale = 65

        def xy(x, y):
            return 50 + (x - ORIGIN[0]) * scale, 80 + (ORIGIN[1] + PIXELS[1] * RESOLUTION - y) * scale

        def label(x, y, text, size=13, color='#213547'):
            return element(svg, 'text', text, x=str(x), y=str(y), fill=color,
                           **{'font-family': 'DejaVu Sans, sans-serif', 'font-size': str(size)})

        label(50, 36, 'SOCIAL RESCUE  /  12 × 10 m  /  10 người', 26)
        label(50, 61, 'Bố trí kịch bản — tọa độ Gazebo = map; các vùng màu là chú thích', 14)
        for zone in self.manifest()['zones']:
            x, y = zone['center']
            sx, sy = zone['size']
            px, py = xy(x - sx / 2, y + sy / 2)
            color = '#{:02x}{:02x}{:02x}'.format(*(round(v * 255) for v in zone['color']))
            element(svg, 'rect', x=str(px), y=str(py), width=str(sx * scale), height=str(sy * scale),
                    fill=color, stroke=color, **{'fill-opacity': '0.14', 'stroke-dasharray': '6 4'})
        for kind, (x, y, yaw), sx, sy in shapes:
            if kind == 'circle':
                px, py = xy(x, y)
                element(svg, 'circle', cx=str(px), cy=str(py), r=str(sx * scale), fill='#485966')
            else:
                corners = []
                for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    dx, dy = a * sx / 2, b * sy / 2
                    px, py = xy(x + math.cos(yaw) * dx - math.sin(yaw) * dy,
                                y + math.sin(yaw) * dx + math.cos(yaw) * dy)
                    corners.append(f'{px:.2f},{py:.2f}')
                element(svg, 'polygon', points=' '.join(corners), fill='#485966')
        for person in self.people:
            px, py = xy(person['x'], person['y'])
            qx, qy = xy(person['x'] + 0.7 * math.cos(person['yaw']),
                        person['y'] + 0.7 * math.sin(person['yaw']))
            element(svg, 'circle', cx=str(px), cy=str(py), r='8', fill='#f6be57', stroke='#213547')
            element(svg, 'line', x1=str(px), y1=str(py), x2=str(qx), y2=str(qy),
                    stroke='#ce7d16', **{'stroke-width': '3'})
        for name, color in (('robot1', '#ed655e'), ('robot2', '#3294d8'), ('robot3', '#1aaf78')):
            x, y, _ = self.manifest()['spawn_poses'][name]
            px, py = xy(x, y)
            element(svg, 'circle', cx=str(px), cy=str(py), r='6', fill=color)
            label(px + 8, py - 5, name, 12, color)
        for x, y, text in ((-5.6, 4.55, 'Phòng trò chuyện'), (2.0, 4.55, 'Phòng TV'),
                           (-5.65, -1.5, 'Kỹ thuật'), (-2.2, -1.5, 'Thiết bị'),
                           (-3.45, -2.5, '0,80 m'), (-2.6, 0.15, 'Hành lang 1,80 m'),
                           (2.5, 0.72, 'Người trước leader'), (-5.6, 0.72, 'Người hành lang'),
                           (-0.8, 4.6, 'Người đứng'), (2.4, -3.9, 'Xuất phát')):
            label(*xy(x, y), text)
        label(50, 810, 'Bắc: +Y ↑     Đông: +X →      Người: ● + hướng nhìn      Vật cản: xám', 15)
        label(50, 840, 'Người và hoạt động được bố trí sẵn; sơ đồ không phải kết quả nhận diện camera.', 14)
        ET.indent(svg, space='  ')
        ET.ElementTree(svg).write(path, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    Scene().write(args.package_dir)


if __name__ == '__main__':
    main()

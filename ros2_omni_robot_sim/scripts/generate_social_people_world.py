#!/usr/bin/env python3
"""Open social experiment with talking groups and two seated TV viewers.

Only chairs and TV furniture enter the physical map; never people/social GT.
"""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_social_rescue_world import Scene, element, pose, ORIGIN, PIXELS, RESOLUTION
from generate_my_maze_map import collision_shapes, rasterize


def generate(package):
    scene = Scene()
    scene.world.set('name', 'social_people')
    scene.core()
    loops = json.loads((package / 'models/person_talking/loops.json').read_text())
    actors = [
        ('pair_a', 2.65, 0.45, math.atan2(-0.55, 0.7), 'near_pair', 'a'),
        ('pair_b', 4.05, 0.45, math.atan2(-0.55, -0.7), 'near_pair', 'b'),
        ('group_a', -3.4, 1.5, math.atan2(0.45, 1.0), 'talk_group', 'b'),
        ('group_b', -1.4, 1.5, math.atan2(0.45, -1.0), 'talk_group', 'a'),
        ('group_c', -2.4, 2.8, -math.pi / 2, 'talk_group', 'b'),
    ]
    for name, x, y, facing, group, variant in actors:
        loop = loops[variant]
        actor = element(scene.world, 'actor', name=name)
        # Frame +X is facing. Source mesh forward is -Y; trajectory is stationary.
        pose(actor, (x, y, loop['height_offset']), facing + math.pi / 2)
        filename = 'model://person_talking/' + loop['mesh']
        element(element(actor, 'skin'), 'filename', filename)
        animation = element(actor, 'animation', name='talk')
        element(animation, 'filename', filename)
        element(animation, 'interpolate_x', 'false')
        script = element(actor, 'script')
        element(script, 'loop', 'true')
        element(script, 'auto_start', 'true')
        trajectory = element(script, 'trajectory', id='0', type='talk', tension='1')
        for t in (0, loop['duration']):
            waypoint = element(trajectory, 'waypoint')
            element(waypoint, 'time', t)
            pose(waypoint)
        # Actors are visual/sensor entities. A fixed torso proxy also provides
        # physical contact, but is deliberately absent from the planning map.
        link = scene.model(name + '_contact', (x, y, 0))
        collision = element(link, 'collision', name='body_contact')
        pose(collision, (0, 0, 0.87))
        shape = element(element(collision, 'geometry'), 'cylinder')
        element(shape, 'radius', 0.30)
        element(shape, 'length', 1.74)
        scene.people.append(dict(name=name, x=x, y=y, yaw=facing, activity='scripted_conversation_gestures',
                                 group=group, radius=0.30, height=1.74, posture='standing', animation=variant))
    scene.person('standing_a', -3.5, -2.0, -0.3, 'standing', 'standing_a')
    scene.person('standing_b', 0.0, 3.5, -math.pi / 2, 'standing', 'standing_b')
    # Two separate armchairs, aimed at a single screen. Keep broad physical
    # bypasses on both sides; the floor in front of the screen remains empty.
    tv = (-0.325, -0.80)
    chair_color = (0.20, 0.34, 0.45, 1)
    for name, x in (('tv_viewer_a', -1.0), ('tv_viewer_b', 0.35)):
        y = -3.85
        heading = math.atan2(tv[1] - y, tv[0] - x)

        def chair_part(suffix, forward, lateral, sx, sy, height, base=0):
            px = x + forward * math.cos(heading) - lateral * math.sin(heading)
            py = y + forward * math.sin(heading) + lateral * math.cos(heading)
            scene.box(f'chair_{name}_{suffix}', px, py, sx, sy, height, chair_color,
                      yaw=heading, base=base, annotate=True)

        chair_part('seat', 0.03, 0, 0.65, 0.72, 0.10, 0.32)
        chair_part('back', -0.34, 0, 0.12, 0.72, 0.86, 0.10)
        for side in (-1, 1):
            chair_part(f'arm_{side}', 0.02, side * 0.37, 0.68, 0.10, 0.12, 0.55)
            for front in (-1, 1):
                chair_part(f'leg_{side}_{front}', front * 0.23, side * 0.28, 0.08, 0.08, 0.32)
        scene.person(name, x, y, heading, 'watching_tv', 'tv_group', posture='seated')
    scene.box('tv_console', *tv, 1.70, 0.45, 0.55, (0.32, 0.23, 0.16, 1), annotate=True)
    scene.box('tv_stand', *tv, 0.18, 0.16, 0.18, (0.04, 0.05, 0.06, 1), base=0.55, annotate=True)
    scene.box('tv_frame', *tv, 1.70, 0.10, 1.02, (0.025, 0.03, 0.035, 1), base=0.72, annotate=True)
    scene.box('tv_screen', tv[0], tv[1] - 0.057, 1.54, 0.014, 0.86,
              (0.06, 0.24, 0.38, 1), base=0.80, collision=False, emissive=True)
    # Simple bright programme graphic, no people painted into the screen.
    scene.box('tv_horizon', tv[0], tv[1] - 0.067, 1.46, 0.006, 0.26,
              (0.22, 0.43, 0.25, 1), base=0.85, collision=False, emissive=True)
    for k, height in enumerate((0.18, 0.30, 0.23, 0.35, 0.21)):
        scene.box(f'tv_graph_{k}', tv[0] - 0.55 + 0.26 * k, tv[1] - 0.072, 0.16, 0.004, height,
                  (0.75, 0.85, 0.55, 1), base=1.08, collision=False, emissive=True)
    world = package / 'worlds/social_people.sdf'
    ET.indent(scene.root, space='  ')
    ET.ElementTree(scene.root).write(world, encoding='utf-8', xml_declaration=True)
    pgm = package / 'map/social_people.pgm'
    physical = copy.deepcopy(scene.root)
    physical_world = physical.find('world')
    for model in list(physical_world):
        if model.tag != 'model' or not model.get('name', '').startswith(('chair_', 'tv_')):
            physical_world.remove(model)
        elif model.get('name') in ('tv_viewer_a', 'tv_viewer_b'):
            physical_world.remove(model)
    pixels, _ = rasterize(collision_shapes(physical), *PIXELS, *ORIGIN, RESOLUTION)
    pgm.write_bytes(f'P5\n{PIXELS[0]} {PIXELS[1]}\n255\n'.encode() + pixels)
    pgm.with_suffix('.yaml').write_text(
        f'image: social_people.pgm\nmode: trinary\nresolution: {RESOLUTION}\n'
        f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    manifest = dict(world='social_people', frame_id='map', bounds=[-6.5, -5.5, 6.5, 5.5],
                    annotation_source='authored_scenario_ground_truth_not_camera_detection',
                    map_contains_people=False, map_contains_social_zones=False,
                    people=scene.people, obstacles=scene.obstacles, rooms=[], passages=[],
                    attention_targets=[dict(name='tv_screen', xy=list(tv))],
                    zones=[dict(name='near_pair', activity='scripted_conversation_gestures', center=[3.35, .1],
                                size=[2.2, 1.8], color=[.1, .75, .8]),
                           dict(name='talk_group', activity='scripted_conversation_gestures', center=[-2.4, 1.95],
                                size=[2.8, 2.4], color=[.65, .4, .9]),
                           dict(name='standing_a', activity='standing', center=[-3.5, -2.0],
                                size=[1.3, 1.3], color=[.95, .65, .15]),
                           dict(name='standing_b', activity='standing', center=[0, 3.5],
                                size=[1.3, 1.3], color=[.95, .65, .15]),
                           dict(name='tv_group', activity='watching_tv', center=[-0.325, -2.325],
                                size=[2.3, 3.7], color=[.85, .3, .7])],
                    spawn_poses=dict(robot1=[4., -2.9, 1.57], robot2=[3., -2.9, 1.57], robot3=[3.5, -2.2, 1.57]))
    (package / 'config/social_people_scene.json').write_text(json.dumps(manifest, indent=2) + '\n')
    svg = ET.Element('svg', xmlns='http://www.w3.org/2000/svg', viewBox='0 0 900 810')
    element(svg, 'rect', x='0', y='0', width='900', height='810', fill='#f4f7fa')

    def xy(x, y):
        return 60 + (x + 6.5) * 60, 70 + (5.5 - y) * 60

    def label(x, y, content, size=14):
        element(svg, 'text', content, x=str(x), y=str(y), fill='#213547',
                **{'font-family': 'DejaVu Sans, sans-serif', 'font-size': str(size)})

    label(60, 30, 'SOCIAL PEOPLE — 5 trò chuyện, 2 đứng riêng, 2 xem TV', 23)
    label(60, 52, 'Không có tường. Chỉ thêm ghế và TV. Bố trí kịch bản, không phải kết quả camera.')
    for x in range(-6, 7):
        px, _ = xy(x, 0)
        element(svg, 'line', x1=str(px), y1='70', x2=str(px), y2='730', stroke='#d8e0e7')
    for y in range(-5, 6):
        _, py = xy(0, y)
        element(svg, 'line', x1='60', y1=str(py), x2='840', y2=str(py), stroke='#d8e0e7')
    for obstacle in scene.obstacles:
        ox, oy = obstacle['xy']
        px, py = xy(ox, oy)
        sx, sy = obstacle['size']
        element(svg, 'rect', x=str(px - sx * 30), y=str(py - sy * 30),
                width=str(sx * 60), height=str(sy * 60), fill='#405667',
                transform=f'rotate({-math.degrees(obstacle["yaw"])} {px} {py})')
    tx, ty = xy(*tv)
    label(tx - 12, ty - 22, 'TV', 16)
    for person in scene.people:
        px, py = xy(person['x'], person['y'])
        qx, qy = xy(person['x'] + 0.7 * math.cos(person['yaw']), person['y'] + 0.7 * math.sin(person['yaw']))
        element(svg, 'circle', cx=str(px), cy=str(py), r='13', fill='#ebaf43', stroke='#213547')
        element(svg, 'line', x1=str(px), y1=str(py), x2=str(qx), y2=str(qy),
                stroke='#ad6920', **{'stroke-width': '4'})
        label(px + 16, py - 12, person['name'], 12)
    for name, color in [('robot1', '#ed655e'), ('robot2', '#3294d8'), ('robot3', '#1aaf78')]:
        px, py = xy(*manifest['spawn_poses'][name][:2])
        element(svg, 'circle', cx=str(px), cy=str(py), r='9', fill=color)
        label(px + 12, py + 15, name, 12)
    label(60, 766, 'Bắc +Y ↑    Đông +X →     Map chỉ chứa ghế/TV; vùng xã hội do cảm biến tạo khi chạy.')
    ET.ElementTree(svg).write(package / 'map/social_people_layout.svg', encoding='utf-8', xml_declaration=True)
    print(f'Generated {world}: 5 animated + 2 standing + 2 seated viewers; map has chairs/TV only.')


if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[1])

#!/usr/bin/env python3
"""Rasterize static collision geometry from my_maze.sdf into my_maze.pgm."""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORLD_FILE = PACKAGE_DIR / 'worlds' / 'my_maze.sdf'
MAP_YAML = PACKAGE_DIR / 'map' / 'my_maze.yaml'
MAP_PGM = PACKAGE_DIR / 'map' / 'my_maze.pgm'


def read_yaml_value(pattern, text, cast):
    match = re.search(pattern, text, re.MULTILINE)
    if match is None:
        raise ValueError(f'Missing map setting matching {pattern!r}')
    return cast(match.group(1))


def read_pgm_header(path):
    data = path.read_bytes()
    offset = 0

    def token():
        nonlocal offset
        while True:
            while data[offset:offset + 1] in (b' ', b'\t', b'\r', b'\n'):
                offset += 1
            if data[offset:offset + 1] != b'#':
                break
            while data[offset:offset + 1] not in (b'\n', b''):
                offset += 1
        start = offset
        while data[offset:offset + 1] not in (b' ', b'\t', b'\r', b'\n'):
            offset += 1
        return data[start:offset]

    magic = token()
    if magic != b'P5':
        raise ValueError(f'{path} must be a binary PGM (P5)')
    width = int(token())
    height = int(token())
    max_value = int(token())
    if max_value != 255:
        raise ValueError(f'{path} must use 8-bit pixels (max value 255)')
    while data[offset:offset + 1] in (b' ', b'\t', b'\r', b'\n'):
        offset += 1
    return width, height


def parse_pose(element):
    if element is None or not element.text:
        return 0.0, 0.0, 0.0
    values = [float(value) for value in element.text.split()]
    if len(values) not in (3, 6):
        raise ValueError(f'Expected an SDF pose with 3 or 6 values: {values}')
    x, y, yaw = values[0], values[1], values[-1]
    return x, y, yaw


def compose_pose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    return (
        px + cosine * cx - sine * cy,
        py + sine * cx + cosine * cy,
        pyaw + cyaw,
    )


def collision_shapes(world_root):
    shapes = []
    for model in world_root.findall('./world/model'):
        model_pose = parse_pose(model.find('pose'))
        for link in model.findall('link'):
            link_pose = compose_pose(model_pose, parse_pose(link.find('pose')))
            for collision in link.findall('collision'):
                collision_pose = compose_pose(
                    link_pose, parse_pose(collision.find('pose'))
                )
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                box = geometry.find('box/size')
                if box is not None:
                    sx, sy, _ = (float(value) for value in box.text.split())
                    shapes.append(('box', collision_pose, sx, sy))
                    continue
                cylinder = geometry.find('cylinder')
                if cylinder is not None:
                    radius = float(cylinder.findtext('radius'))
                    shapes.append(('circle', collision_pose, radius, radius))
    return shapes


def rasterize(shapes, width, height, origin_x, origin_y, resolution):
    pixels = bytearray([255]) * (width * height)
    occupied_count = 0

    for kind, (center_x, center_y, yaw), size_x, size_y in shapes:
        if kind == 'circle':
            radius = size_x
            extent_x = extent_y = radius
        else:
            half_x = size_x * 0.5
            half_y = size_y * 0.5
            extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
            extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y

        min_x = max(0, math.floor((center_x - extent_x - origin_x) / resolution))
        max_x = min(width - 1, math.floor((center_x + extent_x - origin_x) / resolution))
        min_y = max(0, math.floor((center_y - extent_y - origin_y) / resolution))
        max_y = min(height - 1, math.floor((center_y + extent_y - origin_y) / resolution))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)

        for grid_y in range(min_y, max_y + 1):
            world_y = origin_y + (grid_y + 0.5) * resolution
            for grid_x in range(min_x, max_x + 1):
                world_x = origin_x + (grid_x + 0.5) * resolution
                dx = world_x - center_x
                dy = world_y - center_y
                if kind == 'circle':
                    inside = dx * dx + dy * dy <= size_x * size_x
                else:
                    local_x = cosine * dx + sine * dy
                    local_y = -sine * dx + cosine * dy
                    inside = (
                        abs(local_x) <= size_x * 0.5
                        and abs(local_y) <= size_y * 0.5
                    )
                if inside:
                    image_y = height - 1 - grid_y
                    pixel_index = image_y * width + grid_x
                    if pixels[pixel_index] != 0:
                        occupied_count += 1
                        pixels[pixel_index] = 0

    return pixels, occupied_count


def main():
    yaml_text = MAP_YAML.read_text(encoding='utf-8')
    resolution = read_yaml_value(r'^resolution:\s*([0-9.eE+-]+)', yaml_text, float)
    origin_match = re.search(
        r'^origin:\s*\[\s*([-0-9.eE+]+)\s*,\s*([-0-9.eE+]+)',
        yaml_text,
        re.MULTILINE,
    )
    if origin_match is None:
        raise ValueError(f'No [x, y, yaw] origin found in {MAP_YAML}')
    origin_x, origin_y = (float(value) for value in origin_match.groups())
    width, height = read_pgm_header(MAP_PGM)

    world = ET.parse(WORLD_FILE).getroot()
    shapes = collision_shapes(world)
    pixels, occupied_count = rasterize(
        shapes, width, height, origin_x, origin_y, resolution
    )
    MAP_PGM.write_bytes(
        f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels
    )
    print(
        f'Wrote {MAP_PGM}: {width} x {height} cells, {resolution:.3f} m/cell, '
        f'{occupied_count} occupied cells from {len(shapes)} SDF collisions.'
    )


if __name__ == '__main__':
    main()

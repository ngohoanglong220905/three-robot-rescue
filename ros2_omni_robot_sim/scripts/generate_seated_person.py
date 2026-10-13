#!/usr/bin/env python3
"""Bake a seated derivative of the bundled, attributed MakeHuman standing mesh.

UVs/materials/topology are retained. A smooth three-part leg deformation bends
hips and knees; upper body is lowered onto a 0.42 m seat. No runtime skeleton
or remote asset is needed by Gazebo. The original standing.dae is untouched.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def smooth(a, b, x):
    t = np.clip((x - a) / (b - a), 0, 1)
    return t * t * (3 - 2 * t)


def seated(points):
    x, y, z = points.T
    upper = points.copy()
    upper[:, 2] -= 0.45
    thigh = points.copy()
    thigh[:, 1] = -0.01 + z - 1.02
    thigh[:, 2] = 0.57 - (y + 0.01)
    lower = points.copy()
    lower[:, 1] -= 0.47
    lower[:, 2] += 0.007
    torso = smooth(0.90, 1.10, z)
    # Hands stay beside the lap; do not apply the thigh bend to lateral arms.
    torso = np.maximum(torso, smooth(0.20, 0.27, np.abs(x)) * smooth(0.8, 0.95, z))
    shin = (1 - smooth(0.46, 0.66, z)) * (1 - torso)
    return upper * torso[:, None] + lower * shin[:, None] + thigh * (1 - torso - shin)[:, None]


def generate(package_dir):
    mesh_dir = package_dir / 'models/person_standing/meshes'
    tree = ET.parse(mesh_dir / 'standing.dae')
    root = tree.getroot()
    uri = 'http://www.collada.org/2005/11/COLLADASchema'
    ET.register_namespace('', uri)
    ns = {'c': uri}
    for geometry in root.findall('.//c:geometry', ns):
        sources = {s.get('id'): s for s in geometry.findall('c:mesh/c:source', ns)}
        pos_source = next(s for name, s in sources.items() if name.endswith('-positions'))
        norm_source = next(s for name, s in sources.items() if name.endswith('-normals'))
        pos_array = pos_source.find('c:float_array', ns)
        norm_array = norm_source.find('c:float_array', ns)
        points = np.fromstring(pos_array.text, sep=' ').reshape(-1, 3)
        normals = np.fromstring(norm_array.text, sep=' ').reshape(-1, 3)
        # Derive each normal's location from face indices, including split normals.
        locations = np.zeros_like(normals)
        counts = np.zeros(len(normals))
        for primitive in geometry.find('c:mesh', ns):
            inputs = primitive.findall('c:input', ns)
            if not inputs or primitive.find('c:p', ns) is None:
                continue
            offsets = {i.get('semantic'): int(i.get('offset')) for i in inputs}
            if 'NORMAL' not in offsets or 'VERTEX' not in offsets:
                continue
            stride = max(int(i.get('offset')) for i in inputs) + 1
            for indices in primitive.findall('c:p', ns):
                data = np.fromstring(indices.text, sep=' ', dtype=int).reshape(-1, stride)
                pi, ni = data[:, offsets['VERTEX']], data[:, offsets['NORMAL']]
                np.add.at(locations, ni, points[pi])
                np.add.at(counts, ni, 1)
        locations /= np.maximum(1, counts)[:, None]
        eps = 1e-5
        jacobian = np.stack([(seated(locations + np.eye(3)[i] * eps)
                              - seated(locations - np.eye(3)[i] * eps)) / (2 * eps)
                             for i in range(3)], axis=-1)
        new_normals = np.einsum('nij,nj->ni', np.linalg.pinv(jacobian).transpose(0, 2, 1), normals)
        new_normals /= np.maximum(np.linalg.norm(new_normals, axis=1), 1e-12)[:, None]
        pos_array.text = ' '.join(f'{v:.7g}' for v in seated(points).ravel())
        norm_array.text = ' '.join(f'{v:.7g}' for v in new_normals.ravel())
    # Remove unused source rig/camera/light; this file is a static baked mesh.
    for scene in root.findall('c:library_visual_scenes/c:visual_scene', ns):
        for node in list(scene):
            if node.find('c:instance_geometry', ns) is None:
                scene.remove(node)
    output = mesh_dir / 'seated.dae'
    tree.write(output, encoding='utf-8', xml_declaration=True)
    print(f'Generated seated mesh: {output}')


if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[1])

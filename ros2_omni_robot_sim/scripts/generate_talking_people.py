#!/usr/bin/env python3
"""Make local, stationary conversation loops from Gazebo's attributed actors.

Root translation/turning is removed so authored locations and collision proxies
stay aligned. Arm, head and torso joint animation is retained. No runtime download.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def generate(package):
    folder = package / 'models/person_talking'
    uri = 'http://www.collada.org/2005/11/COLLADASchema'
    ET.register_namespace('', uri)
    ns = {'c': uri}
    metadata = {}
    for variant in ('a', 'b'):
        tree = ET.parse(folder / f'talk_{variant}.dae')
        root = tree.getroot()
        hip = root.find('.//c:node[@id="Hips"]/c:matrix', ns)
        rest = np.fromstring(hip.text, sep=' ').reshape(4, 4)
        samples = {}
        for animation in root.findall('c:library_animations/c:animation', ns):
            channel = animation.find('c:channel', ns)
            if channel is None:
                continue
            sources = {s.get('id'): s for s in animation.findall('c:source', ns)}
            sampler = animation.find('c:sampler', ns)
            inputs = {i.get('semantic'): sources[i.get('source')[1:]] for i in sampler.findall('c:input', ns)}
            times = np.fromstring(inputs['INPUT'].find('c:float_array', ns).text, sep=' ')
            matrices = np.fromstring(inputs['OUTPUT'].find('c:float_array', ns).text, sep=' ').reshape(-1, 4, 4)
            indices = np.flatnonzero(times - times[0] <= 24.0)[::2]
            times = times[indices] - times[indices[0]]
            matrices = matrices[indices].copy()
            joint = channel.get('target').split('/')[0]
            if joint == 'Hips':
                matrices[:] = rest
            else:
                # Blend the end into the first frame; orthonormalize rotations.
                for index in np.flatnonzero(times > times[-1] - 0.8):
                    alpha = (times[index] - times[-1] + 0.8) / 0.8
                    mixed = (1 - alpha) * matrices[index] + alpha * matrices[0]
                    u, _, vt = np.linalg.svd(mixed[:3, :3])
                    correction = np.diag([1, 1, np.linalg.det(u @ vt)])
                    mixed[:3, :3] = u @ correction @ vt
                    matrices[index] = mixed
            samples[joint] = matrices
            for semantic, data, stride in [('INPUT', times, 1), ('OUTPUT', matrices, 16)]:
                array = inputs[semantic].find('c:float_array', ns)
                array.set('count', str(data.size))
                array.text = ' '.join(f'{v:.7g}' for v in data.ravel())
                inputs[semantic].find('c:technique_common/c:accessor', ns).set('count', str(len(times)))
            interpolation = inputs.get('INTERPOLATION')
            if interpolation is not None:
                array = list(interpolation)[0]
                array.set('count', str(len(times)))
                array.text = ' '.join(['LINEAR'] * len(times))
                interpolation.find('c:technique_common/c:accessor', ns).set('count', str(len(times)))
        # Skinning the feet during asset generation supplies the vertical offset
        # for a stationary SDF trajectory (Gazebo zeros actor root translation).
        skin = root.find('.//c:controller/c:skin', ns)
        sources = {s.get('id'): s for s in skin.findall('c:source', ns)}
        refs = {i.get('semantic'): sources[i.get('source')[1:]] for i in skin.findall('c:joints/c:input', ns)}
        names = refs['JOINT'].find('c:Name_array', ns).text.split()
        inverse = np.fromstring(refs['INV_BIND_MATRIX'].find('c:float_array', ns).text, sep=' ').reshape(-1, 4, 4)
        weights = skin.find('c:vertex_weights', ns)
        inputs = {i.get('semantic'): i for i in weights.findall('c:input', ns)}
        weight_values = np.fromstring(sources[inputs['WEIGHT'].get('source')[1:]].find('c:float_array', ns).text, sep=' ')
        counts = np.fromstring(weights.find('c:vcount', ns).text, sep=' ', dtype=int)
        stride = max(int(i.get('offset')) for i in inputs.values()) + 1
        indices = np.fromstring(weights.find('c:v', ns).text, sep=' ', dtype=int).reshape(-1, stride)
        geometry = root.find('.//c:geometry[@id="' + skin.get('source')[1:] + '"]', ns)
        pos_id = geometry.find('c:mesh/c:vertices/c:input', ns).get('source')[1:]
        points = np.fromstring(geometry.find('c:mesh/c:source[@id="' + pos_id + '"]/c:float_array', ns).text, sep=' ').reshape(-1, 3)
        point4 = np.column_stack((points, np.ones(len(points))))
        bind = np.fromstring(skin.find('c:bind_shape_matrix', ns).text, sep=' ').reshape(4, 4)
        point4 = (bind @ point4.T).T
        vertices = np.repeat(np.arange(len(points)), counts)
        bone_ids = indices[:, int(inputs['JOINT'].get('offset'))]
        w = weight_values[indices[:, int(inputs['WEIGHT'].get('offset'))]]
        minimum = 0.0
        for frame in np.linspace(0, len(times) - 1, 25).astype(int):
            world = {}

            def visit(node, parent):
                matrix = node.find('c:matrix', ns)
                local = np.eye(4) if matrix is None else np.fromstring(matrix.text, sep=' ').reshape(4, 4)
                if node.get('id') in samples:
                    local = samples[node.get('id')][frame]
                transform = parent @ local
                world[node.get('id')] = transform
                for child in node.findall('c:node', ns):
                    visit(child, transform)

            for node in root.findall('c:library_visual_scenes/c:visual_scene/c:node', ns):
                visit(node, np.eye(4))
            transforms = np.array([world[name] @ inv for name, inv in zip(names, inverse)])
            weighted = np.einsum('nij,nj->ni', transforms[bone_ids], point4[vertices]) * w[:, None]
            posed = np.zeros_like(point4)
            np.add.at(posed, vertices, weighted)
            minimum = min(minimum, float(posed[:, 2].min()))
        output = folder / f'talk_{variant}_inplace.dae'
        tree.write(output, encoding='utf-8', xml_declaration=True)
        metadata[variant] = dict(duration=float(times[-1]), height_offset=round(-minimum + 0.01, 4),
                                 mesh=output.name, mesh_forward_yaw=-float(np.pi / 2))
        print(f'Generated {output.name}: {len(times)} frames, foot offset {-minimum + .01:.3f} m')
    (folder / 'loops.json').write_text(json.dumps(metadata, indent=2) + '\n')


if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[1])

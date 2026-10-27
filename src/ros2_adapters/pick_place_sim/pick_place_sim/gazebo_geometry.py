"""Read a complete live Gazebo SDF export; never infer absence from pose topics.

Restricted to our static single-link box fixtures, exported in world frame.
Unknown shapes/local offsets fail closed, rather than silently using YAML poses.
"""

import ast
import math
import subprocess
import xml.etree.ElementTree as ET

from pick_place_scene.static_geometry import Box


def vector(text, count):
    result = tuple(map(float, text.split()))
    if len(result) != count or not all(map(math.isfinite, result)):
        raise ValueError('invalid Gazebo numeric vector')
    return result


def read_world(world_name='empty'):
    result = subprocess.run([
        'gz', 'service', '-s', f'/world/{world_name}/generate_world_sdf',
        '--reqtype', 'gz.msgs.SdfGeneratorConfig', '--reptype', 'gz.msgs.StringMsg',
        '--timeout', '5000', '--req', 'global_entity_gen_config { expand_include_tags {data: true} }'],
        text=True, capture_output=True, check=True, timeout=10)
    data = result.stdout.strip()
    if not data.startswith('data: '):
        raise ValueError('Gazebo export is not a complete SDF StringMsg')
    return ast.literal_eval(data[6:])


def extract_obstacles(sdf, ids, offset, world_name='empty'):
    world = ET.fromstring(sdf).find('world')
    if world is None or world.get('name') != world_name:
        raise ValueError('wrong or incomplete Gazebo world')
    result = {}
    for name in ids:
        models = [m for m in world.findall('model') if m.get('name') == name]
        if not models:
            result[name] = None  # Absence proven by a complete successful export.
            continue
        if len(models) != 1:
            raise ValueError(f'duplicate entity {name}')
        model = models[0]
        if model.findtext('static', 'false').strip() not in ('true', '1'):
            raise ValueError(f'{name}: only static obstacles supported')
        if len(model.findall('link')) != 1 or model.findall('model'):
            raise ValueError(f'{name}: expected one link without nested models')
        link = model.find('link')
        poses = [model.find('pose'), link.find('pose')]
        for pose in poses:
            if pose is not None and pose.get('relative_to', ''):
                raise ValueError(f'{name}: relative poses unsupported')
        if any(vector(link.findtext('pose', '0 0 0 0 0 0'), 6)):
            raise ValueError(f'{name}: local link offset unsupported')
        sizes = []
        for kind in ('visual', 'collision'):
            shapes = link.findall(kind)
            if len(shapes) != 1:
                raise ValueError(f'{name}: expected one {kind}')
            shape = shapes[0]
            pose = shape.find('pose')
            if ((pose is not None and pose.get('relative_to', '')) or
                    any(vector(shape.findtext('pose', '0 0 0 0 0 0'), 6))):
                raise ValueError(f'{name}: local shape offset unsupported')
            geometry = shape.find('geometry')
            if geometry is None or len(geometry) != 1 or geometry.find('box') is None:
                raise ValueError(f'{name}: only box geometry supported')
            size = vector(geometry.findtext('box/size', ''), 3)
            if any(x <= 0 for x in size):
                raise ValueError(f'{name}: nonpositive size')
            sizes.append(size)
        if sizes[0] != sizes[1]:
            raise ValueError(f'{name}: visual/collision mismatch')
        pose = vector(model.findtext('pose', '0 0 0 0 0 0'), 6)
        result[name] = Box(name, sizes[0], tuple(p-t for p, t in zip(pose[:3], offset))+pose[3:])
    return result
